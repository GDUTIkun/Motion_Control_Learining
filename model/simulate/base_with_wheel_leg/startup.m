% Startup values for a planar wheel-leg floating-base LQR + QP demo.
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
% Lower the default equilibrium while keeping the wheel directly below the
% hip. Recompute the positive-knee pose instead of adding a z transient.
traj.nominalOffset = deg2rad([-19; 38]);
traj.defaultHeightReduction = 0.08;
nominalKin = wheel_leg_kinematics([traj.nominalOffset; 0], ...
    zeros(3, 1), zeros(3, 1), leg);
traj.offset = wheel_leg_inverse_kinematics( ...
    nominalKin.pO + [0; traj.defaultHeightReduction], ...
    zeros(2, 1), zeros(2, 1), leg);
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
% Scheme 1: use the final upper-layer body force to generate a bounded wheel
% equilibrium, then approach it through a stateful second-order governor.
traj.wheelPositionPlanning = true;
traj.wheelPositionForceSource = "reference_acceleration";
traj.wheelPositionForceScale = 0.20;
traj.wheelPositionKneeMin = deg2rad(25);
traj.wheelPositionFrequencyHz = 0.8;
traj.wheelPositionDamping = 1.0;
traj.wheelPositionVelocityMax = 0.4;
traj.wheelPositionAccelerationMax = 2.0;

ctrl = struct();
ctrl.Ts = 0.005;
ctrl.bandwidthHz = [0.2651399877; 2.928535979; 3.753740554];
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = [0.7852988453; 0.7852988453; 0.7852988453];
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 .* ctrl.zeta .* ctrl.wn);
ctrl.tauMax = [160; 160; 45];
ctrl.tauSign = [1; 1; 1];
ctrl.constraintDamping = 1e-9;
ctrl.constraintVelocityGain = 41.79564438;
ctrl.qpWqdd = [1; 1; 1];
% The hip torque tracks MBy_des from the floating-base LQR. Keep the knee
% and wheel torque entries as small regularizers.
ctrl.qpWtau = [1.0; 5.946535182e-06; 5.946535182e-06];
ctrl.qpWFc = [0.0002433157215; 0.0002433157215];
ctrl.qpWarmStart = true;
ctrl.qpSolver = "quadprog";
ctrl.kneeGuardEnabled = true;
ctrl.kneeGuardMin = deg2rad(10);
ctrl.kneeGuardFrequencyHz = 3.0;
ctrl.kneeGuardDamping = 1.0;
% Match the Simscape Spatial Contact Force block conservatively
% (static friction = 0.5, dynamic friction = 0.3 in source.slx).
ctrl.mu = 0.45;
% Maps desired body pitch moment MBy_des to the hip joint torque reference.
% The torque applied to the leg is the reaction of the body-side moment.
ctrl.hipMomentToTauSign = -1;
% Maps base pitch into the absolute thigh angle used by the analytic leg
% dynamics: qh_abs = qh_rel + ctrl.basePitchToAbsHipSign * thetaB.
ctrl.basePitchToAbsHipSign = 1;
% Use the upper-layer wrench to estimate floating-hip acceleration in the
% lower QP rolling constraint. This must be paired with an implicit solver
% for the compliant Simscape contact model.
ctrl.useFloatingHipAcceleration = true;
ctrl.discreteExecution = true;

hip = struct();
hip.xRef = 0;
hip.yRef = 0;
hip.baseYOffset = 0;
hip.groundTopY = 0;
hip.yWorldRef = 0;
hip.dxRef = 0;
hip.dyRef = 0;

% Legacy hip reference. The maintained upper layer is floating-base LQR;
% these values still provide x/z references for the existing Simulink wiring.
hip.settleTime = 1.0;
hip.moveDuration = 6.0;
hip.holdDuration = 1.0;
hip.returnDuration = 6.0;
hip.xStep = 0;
hip.yStep = 0;
hip.virtualMass = [3.0; 3.0];
hip.Kp = diag([3000; 5000]);
hip.Kd = diag([70; 70]);
hip.forceBias = [0; 0];
hip.forceMax = [140; 140];
hip.reference = @(t) hip_reference_trajectory(t, hip);

leg.M = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "M");
leg.C = @(q, dq) wheel_leg_dynamics(q, dq, leg, "C");
leg.G = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "G");
leg.dynamics = @(q, dq) wheel_leg_dynamics(q, dq, leg);
leg.kinematics = @(q, dq, ddq) wheel_leg_kinematics(q, dq, ddq, leg);
traj.reference = @(t) wheel_leg_reference(t, traj, leg);

base = struct();
base.g = leg.g;
base.body = struct();
base.body.mass = hip.virtualMass(1);
base.body.lengthX = 0.45;
base.body.widthY = 0.18;
base.body.heightZ = 0.32;
base.body.comPositionBody = [0; 0];
base.body.hipPositionBody = [0; -base.body.heightZ/2];
base.body.inertiaIyy = base.body.mass * ...
    (base.body.lengthX^2 + base.body.heightZ^2) / 12;
base.m = base.body.mass;
base.Iyy = base.body.inertiaIyy;
base.rHBody = base.body.hipPositionBody - base.body.comPositionBody;
base.thetaEq = 0;
base.xEq = [0; 0; 0; 0; 0; 0];
base.xRef = [0; 0; 0; 0; 0; 0];
base.x0 = zeros(6, 1);
base.Ts = ctrl.Ts;
base.controllerType = "discrete";
base.hipRef = base.xRef(1:2) + base.rHBody;
base.simscapeGroundTopY = 0.025;
base.simscapeWorldYOffset = base.simscapeGroundTopY + leg.r ...
    - (base.rHBody(2) + traj.zO0);

hip.xRef = base.hipRef(1);
hip.yRef = base.hipRef(2);
hip.yWorldRef = hip.yRef;
hip.groundTopY = hip.yWorldRef + traj.zORef - leg.r;
hip.reference = @(t) hip_reference_trajectory(t, hip);

assignin("base", "hip", hip);

base.Q = diag([25, 80, 120, 8, 16, 10]);
base.R = diag([1/80^2, 1/140^2, 1/60^2]);
base.forceMax = hip.forceMax(:);
base.momentMax = ctrl.tauMax(1);
base.thetaIntegralGain = 80;
base.thetaIntegralLimit = 0.5;
% Horizontal constant-speed comparison: forward, stop, then reverse home.
base.trajectory = struct();
base.trajectory.enabled = true;
base.trajectory.mode = "velocity_round_trip";
base.trajectory.settleTime = 1.0;
base.trajectory.cruiseVelocity = 0.5;
base.trajectory.accelDuration = 0.5;
base.trajectory.cruiseDuration = 1.5;
base.trajectory.decelDuration = 0.5;
base.trajectory.turnHoldDuration = 0.5;
base.trajectory.crouchDepth = 0;
base.trajectory.crouchDownDuration = base.trajectory.settleTime;
base.trajectory.crouchRecoverStart = 6.5;
base.trajectory.crouchRecoverDuration = 1.0;

baseLqr = floating_base_lqr_design(base);
base.command = @(x) floating_base_lqr_command(x, baseLqr);

assignin("base", "leg", leg);
assignin("base", "ctrl", ctrl);
assignin("base", "traj", traj);
assignin("base", "hip", hip);
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);

[base, leg, hip, baseLqr] = set_initial_base_state(base.x0);

if bdIsLoaded("source")
    suppress_scope_windows("source");
end

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
fprintf("Floating base body: %.2f x %.2f x %.2f m, m = %.2f kg, Iyy = %.4f kg*m^2.\n", ...
    base.body.lengthX, base.body.widthY, base.body.heightZ, ...
    base.body.mass, base.body.inertiaIyy);
fprintf("Hip in body frame: [%.4f; %.4f] m from CoM.\n", ...
    base.rHBody(1), base.rHBody(2));
fprintf("Floating-base LQR K loaded. Equilibrium [FHx; FHz; MBy] = [%.4f; %.4f; %.4f].\n", ...
    baseLqr.model.uEq(1), baseLqr.model.uEq(2), baseLqr.model.uEq(3));
fprintf("Floating-base controller: %s LQR, Ts = %.4f s.\n", ...
    baseLqr.controllerType, baseLqr.Ts);
