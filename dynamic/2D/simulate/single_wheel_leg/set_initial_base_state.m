function [base, leg, hip, baseLqr] = set_initial_base_state(x0Request)
%SET_INITIAL_BASE_STATE Set a consistent floating-base initial condition.
%
% x0Request = [xOffset; zOffset; thetaB; dxB; dzB; dthetaB]
%
% The x/z entries are offsets relative to the equilibrium pose. For a pitch
% disturbance, the base CoM is shifted so the hip point stays at the same
% world position. The hip joint initial angle is then adjusted so the leg's
% absolute pose does not jump when thetaB is nonzero.

if nargin < 1 || isempty(x0Request)
    x0Request = zeros(6, 1);
end

x0Request = double(x0Request(:));
if numel(x0Request) ~= 6
    error("set_initial_base_state:InvalidInput", ...
        "x0Request must be [x; z; theta; dx; dz; dtheta].");
end

base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
hip = evalin("base", "hip");

theta0 = x0Request(3);
dtheta0 = x0Request(6);

thetaEq = 0;
if isfield(base, "xEq") && numel(base.xEq) >= 3
    thetaEq = base.xEq(3);
end

rHEq = rotatePitch2D(base.rHBody(:), thetaEq);
rH0 = rotatePitch2D(base.rHBody(:), theta0);

basePosEq = zeros(2, 1);
if isfield(base, "xEq") && numel(base.xEq) >= 2
    basePosEq = base.xEq(1:2);
end

% Keep the hip point fixed while pitching the base, then add requested
% translational offsets on top.
basePos0 = basePosEq + x0Request(1:2) + rHEq - rH0;
base.x0 = [basePos0; theta0; x0Request(4:5); dtheta0];

[qAbs0, dqAbs0, ddqAbs0] = wheel_leg_reference(0, traj, leg);
basePitchToAbsHipSign = 1;
if isfield(ctrl, "basePitchToAbsHipSign")
    basePitchToAbsHipSign = ctrl.basePitchToAbsHipSign;
end
leg.q0 = qAbs0;
leg.q0(1) = qAbs0(1) - basePitchToAbsHipSign * theta0;
leg.dq0 = dqAbs0;
leg.dq0(1) = dqAbs0(1) - basePitchToAbsHipSign * dtheta0;
leg.ddq0 = ddqAbs0;

if exist("floating_base_lqr_design", "file") == 2
    baseLqr = floating_base_lqr_design(base);
    base.command = @(x) floating_base_lqr_command(x, baseLqr);
else
    baseLqr = [];
end

assignin("base", "base", base);
assignin("base", "leg", leg);
assignin("base", "hip", hip);
if ~isempty(baseLqr)
    assignin("base", "baseLqr", baseLqr);
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
