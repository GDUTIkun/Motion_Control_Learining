function [tau, debug] = controller_qp_core(x)
%CONTROLLER_QP_CORE Shared implementation for QP inverse dynamics.

if ~ismember(numel(x), [9, 10, 11, 12])
    error("controller_qp:InvalidInput", ...
        "Expected x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; optional MBy_des].");
end

leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

x = double(x(:));
t = x(1);
q = x(2:4);
dq = x(5:7);
[FH_ext, MBy_des] = parseUpperCommand(x);

[qd, dqd, ddqd] = wheel_leg_reference(t, traj, leg);
qddCmd = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);

[M, C, G] = wheel_leg_dynamics(q, dq, leg);
kin = wheel_leg_kinematics(q, dq, [], leg);
[vH, aH] = hipMotionTerms(t);

Kc = getCtrlField(ctrl, "constraintVelocityGain", 0);
bc = -kin.dJc * dq - aH - Kc * (kin.Jc * dq + vH);

% z = [qdd; tau; Fc]
wQdd = getCtrlVec(ctrl, "qpWqdd", [1; 1; 1]);
wTau = getCtrlVec(ctrl, "qpWtau", 1e-5 * [1; 1; 1]);
wFc = getCtrlVec(ctrl, "qpWFc", 1e-5 * [1; 1]);

tauRef = zeros(3, 1);
if isfinite(MBy_des)
    % MBy_des is the pure pitch moment applied by the leg to the body.
    % The hip motor torque applied to the leg is the opposite reaction.
    tauRef(1) = -MBy_des;
end

H = diag([wQdd; wTau; wFc]);
H = H + 1e-9 * eye(8);
f = [-wQdd .* qddCmd; -wTau .* tauRef; zeros(2, 1)];

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
    % Stay on the QP controller path. This fallback uses the same model and
    % upper-layer load terms without calling legacy non-QP controllers.
    tau = M * qddCmd + C + G - kin.JH' * FH_ext;
    if isfinite(MBy_des)
        tau(1) = tau(1) - MBy_des;
    end
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
debug.FH_ext = FH_ext(:);
debug.MBy_des = MBy_des;
debug.tauRef = tauRef(:);
end

function [FH_ext, MBy_des] = parseUpperCommand(x)
% Supported layouts:
%   9:  [t; q; dq; FHx_ext; FHz_ext]
%   10: [t; q; dq; FHx_ext; FHz_ext; MBy_des]
%   11: [t; q; dq; Fcx; Fcz; FHx_ext; FHz_ext]
%   12: [t; q; dq; Fcx; Fcz; FHx_ext; FHz_ext; MBy_des]
switch numel(x)
    case 9
        FH_ext = x(8:9);
        MBy_des = NaN;
    case 10
        FH_ext = x(8:9);
        MBy_des = x(10);
    case 11
        FH_ext = x(10:11);
        MBy_des = NaN;
    case 12
        FH_ext = x(10:11);
        MBy_des = x(12);
    otherwise
        error("controller_qp:InvalidInput", "Unsupported input width.");
end
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
