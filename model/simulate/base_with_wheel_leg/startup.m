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
% Lower the default equilibrium while keeping the wheel directly below the
% hip. Recompute the positive-knee pose instead of adding a z transient.
traj.nominalOffset = deg2rad([-19; 38]);
traj.defaultHeightReduction = 0.08;
nominalKin = wheel_leg_kinematics([traj.nominalOffset; 0], ...
    zeros(3, 1), zeros(3, 1), leg);
traj.offset = wheel_leg_inverse_kinematics( ...
    nominalKin.pO + [0; traj.defaultHeightReduction], ...
    zeros(2, 1), zeros(2, 1), leg);
traj.qw0 = 0;

q_joint0 = traj.offset;
dq_joint0 = zeros(2, 1);
ddq_joint0 = zeros(2, 1);

kin0 = wheel_leg_kinematics([q_joint0; traj.qw0], [dq_joint0; 0], ...
    [ddq_joint0; 0], leg);

traj.qJoint0 = q_joint0;
traj.dqJoint0 = dq_joint0;
traj.ddqJoint0 = ddq_joint0;
traj.xO0 = kin0.pO(1);
traj.zO0 = kin0.pO(2);
traj.thetaWheelBase0 = sum(q_joint0);
% Scheme 1: use the final upper-layer body force to generate a bounded wheel
% equilibrium, then approach it through a stateful second-order governor.
traj.wheelPositionPlanning = true;
traj.wheelPositionPlanner = "lqr";
traj.wheelPositionForceSource = "reference_acceleration";
traj.wheelPositionForceScale = 0.20;
traj.wheelPositionKneeMin = deg2rad(25);
traj.wheelPositionFrequencyHz = 0.4;
traj.wheelPositionDamping = 1.0;
traj.wheelPositionVelocityMax = 0.15;
traj.wheelPositionAccelerationMax = 0.5;
traj.wheelLqrQ = [4; 1];
traj.wheelLqrR = 200;

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

leg.M = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "M");
leg.C = @(q, dq) wheel_leg_dynamics(q, dq, leg, "C");
leg.G = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "G");
leg.dynamics = @(q, dq) wheel_leg_dynamics(q, dq, leg);
leg.kinematics = @(q, dq, ddq) wheel_leg_kinematics(q, dq, ddq, leg);

base = struct();
base.g = leg.g;
base.body = struct();
base.body.mass = 3.0;
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

base.Q = diag([25, 80, 120, 8, 16, 10]);
base.R = diag([1/80^2, 1/140^2, 1/60^2]);
base.forceMax = [140; 140];
base.momentMax = ctrl.tauMax(1);
base.thetaIntegralGain = 80;
base.thetaIntegralLimit = 0.5;
% Horizontal constant-speed comparison: forward, stop, then reverse home.
base.trajectory = struct();
base.trajectory.enabled = true;
base.trajectory.mode = "stand";
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
wheelLqr = wheel_position_lqr_design(base, leg, traj);

% Planar base-plus-wheel-position NMPC configuration. Solver generation is
% deliberately kept out
% of startup; use build_base_nmpc_solver when the generated S-Function is absent.
baseNmpc = struct();
baseNmpc.enabled = true;
baseNmpc.Ts = 0.005;
baseNmpc.N = 50;
baseNmpc.Q = blkdiag(base.Q, 50, 5);
baseNmpc.R = diag([0.02, 0.01, 0.02]);
baseNmpc.model = base_wheel_state_space(base, leg, traj);
terminalSystem = c2d(ss(baseNmpc.model.A, baseNmpc.model.B, ...
    baseNmpc.model.C, baseNmpc.model.D), baseNmpc.Ts, "zoh");
[~, baseNmpc.W_e] = dlqr(terminalSystem.A, terminalSystem.B, ...
    baseNmpc.Q, baseNmpc.R);
baseNmpc.uMin = [-80; 0; -40];
baseNmpc.uMax = [ 80; 100; 40];
baseNmpc.xiMin = wheelLqr.positionMin;
baseNmpc.xiMax = wheelLqr.positionMax;
% The measured relative speed contains a short contact-settling transient;
% keep the planner reference conservative without making the OCP infeasible.
baseNmpc.dxiMax = 2.0;
baseNmpc.maxSolveTime = baseNmpc.Ts;
% The reduced upper model does not include contact/leg transients. Blend its
% correction with the proven base LQR command before driving the lower QP.
baseNmpc.commandBlend = 0.2;
baseNmpc.solverName = "base_wheel_8state_nmpc";
baseNmpc.sfunName = "acados_solver_sfunction_" + baseNmpc.solverName;
tsTag = replace(string(sprintf("%.9g", baseNmpc.Ts)), ...
    [".", "-", "+"], ["p", "m", ""]);
baseNmpc.buildTag = "Ts_" + tsTag + "_N_" + string(baseNmpc.N) + "_v2";
baseNmpc.generatedDir = fullfile(simulateDir, "generated", ...
    baseNmpc.solverName, baseNmpc.buildTag);
baseNmpc.referenceSize = 11 + 11*(baseNmpc.N - 1) + 8;
baseNmpc.available = isfile(fullfile(baseNmpc.generatedDir, ...
    baseNmpc.sfunName + "." + mexext));
if baseNmpc.available
    addpath(baseNmpc.generatedDir);
end

assignin("base", "leg", leg);
assignin("base", "ctrl", ctrl);
assignin("base", "traj", traj);
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "wheelLqr", wheelLqr);
assignin("base", "baseNmpc", baseNmpc);

[base, leg, baseLqr] = set_initial_base_state(base.x0);

if bdIsLoaded("source")
    suppress_scope_windows("source");
end

fprintf("Loaded planar wheel-leg parameters into base workspace.\n");
fprintf("Initial q0 = [%.4f; %.4f; %.4f] rad.\n", ...
    leg.q0(1), leg.q0(2), leg.q0(3));
fprintf("Initial dq0 = [%.4f; %.4f; %.4f] rad/s.\n", ...
    leg.dq0(1), leg.dq0(2), leg.dq0(3));
fprintf("Floating base body: %.2f x %.2f x %.2f m, m = %.2f kg, Iyy = %.4f kg*m^2.\n", ...
    base.body.lengthX, base.body.widthY, base.body.heightZ, ...
    base.body.mass, base.body.inertiaIyy);
fprintf("Hip in body frame: [%.4f; %.4f] m from CoM.\n", ...
    base.rHBody(1), base.rHBody(2));
fprintf("Floating-base LQR K loaded. Equilibrium [FHx; FHz; MBy] = [%.4f; %.4f; %.4f].\n", ...
    baseLqr.model.uEq(1), baseLqr.model.uEq(2), baseLqr.model.uEq(3));
fprintf("Floating-base controller: %s LQR, Ts = %.4f s.\n", ...
    baseLqr.controllerType, baseLqr.Ts);
fprintf("Wheel-position planner: %s, scheduled height %.3f to %.3f m.\n", ...
    traj.wheelPositionPlanner, wheelLqr.heightGrid(1), wheelLqr.heightGrid(end));
if baseNmpc.available
    fprintf("Upper-layer 8-state NMPC S-Function ready, Ts = %.4f s, N = %d.\n", ...
        baseNmpc.Ts, baseNmpc.N);
else
    fprintf("Upper-layer NMPC S-Function is not built. Run " + ...
        "build_base_nmpc_solver(true).\n");
end
