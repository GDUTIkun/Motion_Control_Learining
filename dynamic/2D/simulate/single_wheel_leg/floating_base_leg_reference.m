function [qd, dqd, ddqd] = floating_base_leg_reference(t, baseState, traj, leg, base, aH)
%FLOATING_BASE_LEG_REFERENCE Stage-1 leg reference tied to the floating base.
%
% The reference keeps the wheel center at the nominal horizontal offset from
% the hip while the vertical wheel-center reference stays on the ground:
%   pO_ref = [pH_x + xOH_nom; groundTop + r]
%
% The two-link IK receives pO_ref - pH, so qh/qk stay geometrically
% consistent with the current floating-base pose instead of fighting it.

if nargin < 3 || isempty(traj)
    traj = evalin("base", "traj");
end
if nargin < 4 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 5 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 6 || isempty(aH)
    aH = zeros(2, 1);
else
    aH = double(aH(:));
end
if numel(aH) ~= 2
    error("floating_base_leg_reference:InvalidHipAcceleration", ...
        "aH must be a 2-element vector.");
end

baseState = double(baseState(:));
if numel(baseState) ~= 6
    error("floating_base_leg_reference:InvalidInput", ...
        "baseState must be [xB; zB; thetaB; dxB; dzB; dthetaB].");
end

theta = baseState(3);
dtheta = baseState(6);

rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];

pH = baseState(1:2) + rH;
vH = baseState(4:5) + dtheta * drdtheta;

xOHNom = getFieldOrDefault(traj, "xO0", 0);
groundTop = getFieldOrDefault(hipFromBase(), "groundTopY", ...
    getFieldOrDefault(base, "simscapeGroundTopY", 0));
wheelCenterZ = groundTop + leg.r;

% Stage 1: wheel x follows hip x with the nominal offset; wheel z is fixed
% by the ground. Express that desired wheel-center motion relative to hip.
pO = [xOHNom; wheelCenterZ - pH(2)];
pO = projectToReachableAnnulus(pO, leg);
vO = [0; -vH(2)];
aO = [0; -aH(2)];

[qJoint, dqJoint, ddqJoint] = wheel_leg_inverse_kinematics(pO, vO, aO, leg);

wheelX = pH(1) + xOHNom;
wheelDx = vH(1);
[qw, dqw, ddqw] = wheelSpinReference(wheelX, wheelDx, qJoint, ...
    dqJoint, ddqJoint, aH(1), traj, leg, base);

qd = [qJoint; qw];
dqd = [dqJoint; dqw];
ddqd = [ddqJoint; ddqw];
end

function [qw, dqw, ddqw] = wheelSpinReference(wheelX, wheelDx, qJoint, ...
    dqJoint, ddqJoint, wheelDdx, traj, leg, base)
qw0 = getFieldOrDefault(traj, "qw0", 0);
thetaWheelBase0 = getFieldOrDefault(traj, "thetaWheelBase0", sum(qJoint));

thetaEq = getFieldOrDefault(base, "thetaEq", 0);
rHEq = rotatePitch2D(base.rHBody(:), thetaEq);
xEq = getFieldOrDefault(base, "xEq", zeros(6, 1));
wheelX0 = xEq(1) + rHEq(1) + getFieldOrDefault(traj, "xO0", 0);

qw = qw0 - (wheelX - wheelX0) / leg.r ...
    - (sum(qJoint) - thetaWheelBase0);
dqw = -wheelDx / leg.r - sum(dqJoint);
ddqw = -wheelDdx / leg.r - sum(ddqJoint);
end

function hip = hipFromBase()
if evalin("base", "exist('hip', 'var')")
    hip = evalin("base", "hip");
else
    hip = struct();
end
end

function p = projectToReachableAnnulus(p, leg)
reach = norm(p);
if reach < eps
    p = [0; -(abs(leg.L1 - leg.L2) + 1e-3)];
    return;
end

margin = 1e-3;
minReach = abs(leg.L1 - leg.L2) + margin;
maxReach = leg.L1 + leg.L2 - margin;
targetReach = min(max(reach, minReach), maxReach);
p = p * (targetReach / reach);
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 - sin(theta)*rz0;
    sin(theta)*rx0 + cos(theta)*rz0
];
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
