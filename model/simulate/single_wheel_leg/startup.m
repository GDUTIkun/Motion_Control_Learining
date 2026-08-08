% Startup values for a planar wheel-leg force-tracking demo.
%
% Coordinate convention:
%   - hip is the origin
%   - +x points right, +z poi nts up
%   - qh = 0 and qk = 0 put both links vertically downward
%   - positive qh swings the thigh toward +x
%   - positive qk bends the shank further toward +x relative to the thigh
%   - qw is the wheel spin relative to the shank/wheel fork
%   - positive qw is counterclockwise in the x-z plane

clc;
clear;

thisFile = mfilename("fullpath");
simulateDir = fileparts(thisFile);
addpath(simulateDir);

leg = struct();
leg.L1 = 0.35;
leg.L2 = 0.35;
leg.c1 = leg.L1 / 2;
leg.c2 = leg.L2 / 2;
leg.m1 = 1.20;
leg.m2 = 0.80;
leg.width = 0.04;
leg.depth = 0.04;
leg.g = 9.81;

% Uniform rectangular rods, inertia about the out-of-plane joint axis.
leg.I1 = leg.m1 * (leg.L1^2 + leg.width^2) / 12;
leg.I2 = leg.m2 * (leg.L2^2 + leg.width^2) / 12;

% Wheel parameters. The wheel spin inertia is about the wheel axle.
leg.r = 0.08;
leg.mw = 0.35;
leg.Iw = 0.5 * leg.mw * leg.r^2;

traj = struct();
traj.mode = "hip_ik";
traj.freq = 0;
traj.omega = 0;
traj.phase = 0;
traj.offset = deg2rad([7; 38]);
traj.amplitude = deg2rad([0; 0]);
traj.qw0 = 0;

phase0 = traj.phase;
q_joint0 = traj.offset + traj.amplitude * sin(phase0);
dq_joint0 = traj.amplitude * traj.omega * cos(phase0);
ddq_joint0 = -traj.amplitude * traj.omega^2 * sin(phase0);

kin0 = wheel_leg_kinematics([q_joint0; traj.qw0], [dq_joint0; 0], ...
    [ddq_joint0; 0], leg);

traj.qJoint0 = q_joint0;
traj.dqJoint0 = dq_joint0;
traj.ddqJoint0 = ddq_joint0;
traj.xO0 = kin0.pO(1);
traj.zO0 = kin0.pO(2);
traj.zORef = traj.zO0;
traj.thetaWheelBase0 = sum(q_joint0);

ctrl = struct();
ctrl.bandwidthHz = [0.2651399877; 2.928535979; 3.753740554];
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = [0.7852988453; 0.7852988453; 0.7852988453];
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 .* ctrl.zeta .* ctrl.wn);
ctrl.tauMax = [160; 160; 45];
ctrl.constraintDamping = 1e-9;
ctrl.constraintVelocityGain = 41.79564438;
ctrl.qpWqdd = [1; 1; 1];
ctrl.qpWtau = [5.946535182e-06; 5.946535182e-06; 5.946535182e-06];
ctrl.qpWFc = [0.0002433157215; 0.0002433157215];
ctrl.mu = 0.8;

hip = struct();
hip.xRef = 0;
hip.baseYOffset = 0.7;
hip.groundTopY = 0.025;
hip.yWorldRef = hip.groundTopY + leg.r - traj.zORef;
hip.yRef = hip.yWorldRef - hip.baseYOffset;
hip.dxRef = 0;
hip.dyRef = 0;

% Upper-layer surrogate: a hip position PD generates F_H,des. The hip joint
% itself remains dynamically free; F_H,des is realized by the leg torques.
hip.settleTime = 1.0;
hip.moveDuration = 6.0;
hip.holdDuration = 1.0;
hip.returnDuration = 6.0;
hip.xStep = 0.35;
hip.yStep = -0.25;
hip.virtualMass = [3.0; 3.0];
hip.Kp = diag([3000; 5000]);
hip.Kd = diag([70; 70]);
hip.forceBias = [0; 0];
hip.forceMax = [140; 140];
hip.command = @(x) hip_position_pd(x, hip);
hip.reference = @(t) hip_reference_trajectory(t, hip);

assignin("base", "hip", hip);
[q_ref0, dq_ref0, ddq_ref0] = wheel_leg_reference(0, traj, leg);

leg.q0 = q_ref0;
leg.dq0 = dq_ref0;
leg.ddq0 = ddq_ref0;

leg.M = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "M");
leg.C = @(q, dq) wheel_leg_dynamics(q, dq, leg, "C");
leg.G = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "G");
leg.dynamics = @(q, dq) wheel_leg_dynamics(q, dq, leg);
leg.kinematics = @(q, dq, ddq) wheel_leg_kinematics(q, dq, ddq, leg);
traj.reference = @(t) wheel_leg_reference(t, traj, leg);

assignin("base", "leg", leg);
assignin("base", "ctrl", ctrl);
assignin("base", "traj", traj);
assignin("base", "hip", hip);

fprintf("Loaded planar wheel-leg parameters into base workspace.\n");
fprintf("Initial q0 = [%.4f; %.4f; %.4f] rad.\n", ...
    leg.q0(1), leg.q0(2), leg.q0(3));
fprintf("Initial dq0 = [%.4f; %.4f; %.4f] rad/s.\n", ...
    leg.dq0(1), leg.dq0(2), leg.dq0(3));
fprintf("Initial hip refs: x = %.4f m, joint y = %.4f m, world y = %.4f m.\n", ...
    hip.xRef, hip.yRef, hip.yWorldRef);
fprintf("Hip reference: settle %.2f s, move %.2f s, hold %.2f s, return %.2f s.\n", ...
    hip.settleTime, hip.moveDuration, hip.holdDuration, hip.returnDuration);
fprintf("Hip reference displacement: dx = %.4f m, dy = %.4f m.\n", ...
    hip.xStep, hip.yStep);
