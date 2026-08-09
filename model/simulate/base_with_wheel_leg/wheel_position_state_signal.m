function y = wheel_position_state_signal(x, base, leg, ctrl)
%WHEEL_POSITION_STATE_SIGNAL Build measured planar 8-state upper state.
%
% Input:  [t; baseState(6); qRelative(3); dqRelative(3)]
% Output: [t; baseState(6); xi; dxi; height]

if nargin < 2 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 3 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 4 || isempty(ctrl)
    ctrl = evalin("base", "ctrl");
end

x = double(x(:));
if numel(x) ~= 13
    error("wheel_position_state_signal:InvalidInput", ...
        "Expected [t; baseState(6); qRelative(3); dqRelative(3)].");
end

t = x(1);
baseState = x(2:7);
qRelative = x(8:10);
dqRelative = x(11:13);
pitchSign = ctrl.basePitchToAbsHipSign;
q = [qRelative(1) + pitchSign * baseState(3); qRelative(2:3)];
dq = [dqRelative(1) + pitchSign * baseState(6); dqRelative(2:3)];
kin = wheel_leg_kinematics(q, dq, [], leg);

rH = rotatePitch2D(base.rHBody(:), baseState(3));
drH = baseState(6) * [-rH(2); rH(1)];
rWheel = rH + kin.pO;
vWheelRelative = drH + kin.vO;
height = max(1e-3, -rWheel(2));
y = [t; baseState; rWheel(1); vWheelRelative(1); height];
end

function y = rotatePitch2D(v, theta)
y = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v(:);
end
