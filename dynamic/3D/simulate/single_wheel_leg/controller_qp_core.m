function [tau, debug] = controller_qp_core(x)
%CONTROLLER_QP_CORE Shared implementation for QP inverse dynamics.

if numel(x) ~= 9 && numel(x) ~= 11
    error("controller_qp:InvalidInput", ...
        "Expected x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHy_ext].");
end

leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

x = double(x(:));
t = x(1);
q = x(2:4);
dq = x(5:7);
if numel(x) == 11
    FH_ext = x(10:11);
else
    FH_ext = x(8:9);
end

[qd, dqd, ddqd] = wheel_leg_reference(t, traj, leg);
qddCmd = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);

[M, C, G] = wheel_leg_dynamics(q, dq, leg);
kin = wheel_leg_kinematics(q, dq, [], leg);
[vH, aH] = hipMotionTerms(t);

Kc = getCtrlField(ctrl, "constraintVelocityGain", 0);
bc = -kin.dJc * dq - aH - Kc * (kin.Jc * dq + vH);

% z = [qdd; tau; Fc]
H = diag([getCtrlVec(ctrl, "qpWqdd", [1; 1; 1]); ...
          getCtrlVec(ctrl, "qpWtau", 1e-5 * [1; 1; 1]); ...
          getCtrlVec(ctrl, "qpWFc", 1e-5 * [1; 1])]);
H = H + 1e-9 * eye(8);
f = [-getCtrlVec(ctrl, "qpWqdd", [1; 1; 1]) .* qddCmd; zeros(5, 1)];

Aeq = [
    M, -eye(3), -kin.Jc';
    kin.Jc, zeros(2, 3), zeros(2, 2)
];
beq = [
    kin.JH' * FH_ext - C - G;
    bc
];

mu = getCtrlField(ctrl, "mu", 0.8);
Aineq = [
    zeros(1, 6),  1, -mu;
    zeros(1, 6), -1, -mu;
    zeros(1, 6),  0, -1
];
bineq = zeros(3, 1);

tauMax = ctrl.tauMax(:);
lb = [-inf(3, 1); -tauMax; -inf; 0];
ub = [ inf(3, 1);  tauMax;  inf; inf];

z0 = [qddCmd; zeros(3, 1); 0; max(0, sum([leg.m1, leg.m2, leg.mw]) * leg.g)];
exitflag = -999;
try
    opts = optimoptions("quadprog", "Display", "off", ...
        "Algorithm", "interior-point-convex");
    [z, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, lb, ub, z0, opts);
catch
    z = [];
end

if isempty(z) || exitflag <= 0 || any(~isfinite(z))
    tau = controller(x);
    qddSol = qddCmd;
    FcSol = zeros(2, 1);
else
    qddSol = z(1:3);
    tau = z(4:6);
    FcSol = z(7:8);
end

tau = min(max(tau(:), -tauMax), tauMax);
debug = struct();
debug.qdd = qddSol(:);
debug.Fc = FcSol(:);
debug.exitflag = exitflag;
end

function [vH, aH] = hipMotionTerms(t)
if evalin("base", "exist('hip', 'var')")
    hip = evalin("base", "hip");
    [~, dpHRef, ddpHRef] = hip_reference_trajectory(t, hip);
    vH = dpHRef(:);
    aH = ddpHRef(:);
else
    vH = zeros(2, 1);
    aH = zeros(2, 1);
end
end

function value = getCtrlField(ctrl, fieldName, defaultValue)
if isfield(ctrl, fieldName)
    value = ctrl.(fieldName);
else
    value = defaultValue;
end
end

function value = getCtrlVec(ctrl, fieldName, defaultValue)
if isfield(ctrl, fieldName)
    value = ctrl.(fieldName);
else
    value = defaultValue;
end
value = value(:);
end
