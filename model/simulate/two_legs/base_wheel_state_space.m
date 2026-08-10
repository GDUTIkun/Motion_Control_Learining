function model = base_wheel_state_space(base, leg, traj)
%BASE_WHEEL_STATE_SPACE Linear planar base-plus-common-wheel-position model.
%
% State: [xB; zB; thetaB; dxB; dzB; dthetaB; xi; dxi]
% Input: [FHx; FHz; MBy], the total two-leg wrench applied to the base.

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 2 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 3 || isempty(traj)
    traj = evalin("base", "traj");
end

baseModel = floating_base_state_space(base);
A = zeros(8, 8);
B = zeros(8, 3);
A(1:6, 1:6) = baseModel.A;
B(1:6, :) = baseModel.B;
A(7, 8) = 1;

rollingDenominator = leg.mw * leg.r + leg.Iw / leg.r;
B(8, 1) = -1 / base.m - leg.r / (2 * rollingDenominator);
B(8, 3) = -1 / (2 * rollingDenominator);

rHEq = rotatePitch2D(base.rHBody(:), base.thetaEq);
xiEq = rHEq(1) + traj.xO0;

model = struct();
model.A = A;
model.B = B;
model.C = eye(8);
model.D = zeros(8, 3);
model.xEq = [base.xEq(:); xiEq; 0];
model.uEq = baseModel.uEq(:);
model.stateNames = [string(baseModel.stateNames(:)); "xi"; "dxi"];
model.inputNames = baseModel.inputNames;
model.rollingDenominator = rollingDenominator;
model.wheelCount = 2;
end

function y = rotatePitch2D(v, theta)
y = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v(:);
end
