function [tau, debug] = controller_qp_core(x)
%CONTROLLER_QP_CORE Shared implementation for QP inverse dynamics.
persistent qpOptions zWarm

if numel(x) ~= 16
    error("controller_qp:InvalidInput", ...
        "Expected the 16D floating-base controller input.");
end

leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

x = double(x(:));
t = x(1);
[q, dq, FH_ext, MBy_des, vH, aH] = parseControllerInput(x);

[qd, dqd, ddqd] = controllerLegReference(t, x, traj, leg, aH);
qddCmd = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);
kneeQddMin = kneeAccelerationLowerBound(q, dq, ctrl);
if isfinite(kneeQddMin)
    qddCmd(2) = max(qddCmd(2), kneeQddMin);
end

[M, C, G] = wheel_leg_dynamics(q, dq, leg);
kin = wheel_leg_kinematics(q, dq, [], leg);

Kc = getCtrlField(ctrl, "constraintVelocityGain", 0);
bc = -kin.dJc * dq - aH - Kc * (kin.Jc * dq + vH);

% z = [qdd; tau; Fc]
wQdd = getCtrlVec(ctrl, "qpWqdd", [1; 1; 1]);
wTau = getCtrlVec(ctrl, "qpWtau", 1e-5 * [1; 1; 1]);
wFc = getCtrlVec(ctrl, "qpWFc", 1e-5 * [1; 1]);

tauRef = zeros(3, 1);
if isfinite(MBy_des)
    tauRef(1) = getCtrlField(ctrl, "hipMomentToTauSign", 1) * MBy_des;
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
if isfinite(kneeQddMin)
    Aineq = [Aineq; 0, -1, 0, zeros(1, 5)];
    bineq = [bineq; -kneeQddMin];
end

tauMax = ctrl.tauMax(:);
lb = [-inf(3, 1); -tauMax; -inf; 0];
ub = [ inf(3, 1);  tauMax;  inf; inf];

z0 = [qddCmd; zeros(3, 1); 0; max(0, sum([leg.m1, leg.m2, leg.mw]) * leg.g)];
if t <= 0 || isempty(zWarm) || numel(zWarm) ~= 8 || any(~isfinite(zWarm))
    zWarm = z0;
elseif getCtrlField(ctrl, "qpWarmStart", true)
    z0 = zWarm;
end
exitflag = -999;
if string(getCtrlField(ctrl, "qpSolver", "quadprog")) == "equality"
    [z, exitflag] = solveEqualityQp(H, f, Aeq, beq);
    if ~isempty(z)
        z(4:6) = min(max(z(4:6), -tauMax), tauMax);
        z(8) = max(z(8), 0);
        mu = getCtrlField(ctrl, "mu", 0.8);
        z(7) = min(max(z(7), -mu*z(8)), mu*z(8));
    end
else
    try
        if isempty(qpOptions)
            qpOptions = optimoptions("quadprog", "Display", "off", ...
                "Algorithm", "interior-point-convex");
        end
        [z, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, lb, ub, ...
            z0, qpOptions);
    catch
        z = [];
    end
end

if isempty(z) || exitflag <= 0 || any(~isfinite(z))
    % Stay on the QP controller path. This fallback uses the same model and
    % upper-layer load terms without calling legacy non-QP controllers.
    tau = M * qddCmd + C + G - kin.JH' * FH_ext;
    if isfinite(MBy_des)
        tau(1) = tau(1) + getCtrlField(ctrl, "hipMomentToTauSign", 1) * MBy_des;
    end
    qddSol = qddCmd;
    FcSol = zeros(2, 1);
else
    qddSol = z(1:3);
    tau = z(4:6);
    FcSol = z(7:8);
    zWarm = z;
end

tau = min(max(tau(:), -tauMax), tauMax);
tau = getCtrlVec(ctrl, "tauSign", [1; 1; 1]) .* tau;
debug = struct();
debug.qdd = qddSol(:);
debug.Fc = FcSol(:);
debug.exitflag = exitflag;
debug.FH_ext = FH_ext(:);
debug.MBy_des = MBy_des;
debug.tauRef = tauRef(:);
debug.kneeQddMin = kneeQddMin;
end

function [qd, dqd, ddqd] = controllerLegReference(t, x, traj, leg, aH)
base = evalin("base", "base");
[qd, dqd, ddqd] = floating_base_leg_reference(t, x(2:7), ...
    traj, leg, base, aH, x(14:15));
end

function [q, dq, FH_ext, MBy_des, vH, aH] = parseControllerInput(x)
% x = [t; xB; zB; thetaB; dxB; dzB; dthetaB;
%        qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; MBy_des]
%
% qh is relative to the base; the analytic leg model uses absolute thigh
% angle. Hip velocity and acceleration enter the rolling constraint.
ctrl = evalin("base", "ctrl");
baseState = x(2:7);
qRel = x(8:10);
dqRel = x(11:13);
FH_ext = x(14:15);
MBy_des = x(16);
pitchSign = getCtrlField(ctrl, "basePitchToAbsHipSign", 1);
q = [qRel(1) + pitchSign * baseState(3); qRel(2:3)];
dq = [dqRel(1) + pitchSign * baseState(6); dqRel(2:3)];
[vH, aH] = floatingHipMotionTerms(baseState, FH_ext, MBy_des);
end

function [vH, aH] = floatingHipMotionTerms(baseState, FH_ext, MBy_des)
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
m = base.m;
Iyy = base.Iyy;
g = base.g;

theta = baseState(3);
dtheta = baseState(6);
rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];
d2rdtheta2 = [-rH(1); -rH(2)];

vB = baseState(4:5);
vH = vB + dtheta * drdtheta;

% FH_ext is body-on-leg. The body receives the opposite force.
FBody = -FH_ext(:);
ddtheta = (rH(1)*FBody(2) - rH(2)*FBody(1) + MBy_des) / Iyy;
aB = [FBody(1)/m; FBody(2)/m - g];
aH = aB + ddtheta * drdtheta + dtheta^2 * d2rdtheta2;
if ~getCtrlField(ctrl, "useFloatingHipAcceleration", false)
    aH = zeros(2, 1);
end
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 - sin(theta)*rz0;
    sin(theta)*rx0 + cos(theta)*rz0
];
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

function qddMin = kneeAccelerationLowerBound(q, dq, ctrl)
if ~getCtrlField(ctrl, "kneeGuardEnabled", false)
    qddMin = -inf;
    return;
end

qMin = getCtrlField(ctrl, "kneeGuardMin", 0);
frequencyHz = max(0, getCtrlField(ctrl, "kneeGuardFrequencyHz", 3));
zeta = max(0, getCtrlField(ctrl, "kneeGuardDamping", 1));
omega = 2 * pi * frequencyHz;
qddMin = -2 * zeta * omega * dq(2) - omega^2 * (q(2) - qMin);
end

function [z, exitflag] = solveEqualityQp(H, f, Aeq, beq)
n = size(H, 1);
p = size(Aeq, 1);
KKT = [
    H, Aeq';
    Aeq, zeros(p, p)
];
rhs = [-f; beq];
if rcond(KKT) < 1e-12
    KKT = KKT + 1e-9 * eye(size(KKT));
end
sol = KKT \ rhs;
z = sol(1:n);
if all(isfinite(z))
    exitflag = 1;
else
    z = [];
    exitflag = -1;
end
end
