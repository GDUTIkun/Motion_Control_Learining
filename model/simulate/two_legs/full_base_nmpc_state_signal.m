function y = full_base_nmpc_state_signal(x, base, leg, ctrl)
%FULL_BASE_NMPC_STATE_SIGNAL Build the measured 16-state 8-DoF signal.
%
% Input: [t; planarBase(6); rollYaw(4); lateral(2); qL(3); dqL(3);
%         qR(3); dqR(3)]
% Output: [t; state(16); wheelHeight]

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
if numel(x) ~= 25
    error("full_base_nmpc_state_signal:InvalidInput", ...
        "Expected the 25D full-base state input.");
end

t = x(1);
planar = x(2:7);
rollYaw = x(8:11);
lateral = x(12:13);
qRelative = [x(14:16), x(20:22)];
dqRelative = [x(17:19), x(23:25)];
qAbsolute = qRelative;
dqAbsolute = dqRelative;
qAbsolute(1, :) = qAbsolute(1, :) ...
    + ctrl.basePitchToAbsHipSign*planar(3);
dqAbsolute(1, :) = dqAbsolute(1, :) ...
    + ctrl.basePitchToAbsHipSign*planar(6);
kinLeft = wheel_leg_kinematics(qAbsolute(:, 1), dqAbsolute(:, 1), [], leg);
kinRight = wheel_leg_kinematics(qAbsolute(:, 2), dqAbsolute(:, 2), [], leg);
rH = rotatePitch2D(base.rHBody(:), planar(3));
drH = planar(6)*[-rH(2); rH(1)];
wheelPosition = [rH + kinLeft.pO, rH + kinRight.pO];
wheelVelocity = [drH + kinLeft.vO, drH + kinRight.vO];
xi = wheelPosition(1, :).';
dxi = wheelVelocity(1, :).';
height = max(1e-3, -mean(wheelPosition(2, :)));

state = [
    planar(1); lateral(1); planar(2);
    rollYaw(1); planar(3); rollYaw(2);
    planar(4); lateral(2); planar(5);
    rollYaw(3); planar(6); rollYaw(4);
    xi; dxi
];
y = [t; state; height];
end

function rWorld = rotatePitch2D(rBody, theta)
rWorld = [cos(theta)*rBody(1) - sin(theta)*rBody(2); ...
    sin(theta)*rBody(1) + cos(theta)*rBody(2)];
end
