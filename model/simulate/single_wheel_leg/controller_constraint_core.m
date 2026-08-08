function [tau, qddConstraint, FcHat, vcError] = controller_constraint_core(x)
%CONTROLLER_CONSTRAINT_CORE Shared constrained inverse-dynamics logic.
%
% The contact constraints are:
%   dxH + xdot_O + r*(dqh+dqk+dqw) = 0
%   dyH + ydot_O = 0
%
% Since the controller input remains 9-dimensional for easy comparison with
% controller.m, dxH/dyH and ddxH/ddyH are taken from the hip reference.

if numel(x) ~= 9 && numel(x) ~= 11
    error("controller_constraint:InvalidInput", ...
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
qddNominal = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);

[M, C, G] = wheel_leg_dynamics(q, dq, leg);
kin = wheel_leg_kinematics(q, dq, [], leg);

[vH, aH] = hipMotionTerms(t);
vcError = kin.Jc * dq + vH;

velocityGain = getCtrlField(ctrl, "constraintVelocityGain", 0);
constraintDamping = getCtrlField(ctrl, "constraintDamping", 1e-9);

% Project the nominal tracking acceleration to the rolling/no-separation
% constraint manifold. This is the M-weighted closest feasible acceleration.
bc = -kin.dJc * dq - aH - velocityGain * vcError;
MinvJcT = M \ kin.Jc';
constraintMass = kin.Jc * MinvJcT ...
    + constraintDamping * eye(size(kin.Jc, 1));
qddConstraint = qddNominal - MinvJcT * ...
    (constraintMass \ (kin.Jc * qddNominal - bc));

% Dynamic equation:
%   M*qdd + C + G = tau + JH'*FH_ext + Jc'*Fc
%
% In an ideal rigid-contact inverse dynamics problem, tau and Fc can be
% solved together with extra actuator/objective constraints. In this
% Simscape contact model, subtracting the full estimated Jc'*Fc term makes
% the controller under-actuate the real compliant contact. Therefore this
% comparison controller uses the constraint solve to make qdd_cmd contact
% consistent, but keeps the same inverse-dynamics torque structure as the
% nominal controller. FcHat is reported only as a diagnostic estimate.
tau = M * qddConstraint + C + G - kin.JH' * FH_ext;

contactMass = kin.Jc * (M \ kin.Jc') ...
    + constraintDamping * eye(size(kin.Jc, 1));
FcHat = contactMass \ (kin.Jc * (M \ (tau + kin.JH' * FH_ext - C - G)) ...
    + kin.dJc * dq + aH + velocityGain * vcError);

tau = min(max(tau, -ctrl.tauMax), ctrl.tauMax);
tau = tau(:);
qddConstraint = qddConstraint(:);
FcHat = FcHat(:);
vcError = vcError(:);
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
