function dX = floating_base_nonlinear_state(~, X, u, base)
%FLOATING_BASE_NONLINEAR_STATE Nonlinear 2D floating-base state equation.
%
% X = [x; z; theta; dx; dz; dtheta]
% u = [FHx; FHz; MBy], force/moment applied by the leg to the body.

if nargin < 4 || isempty(base)
    base = evalin("base", "base");
end

X = double(X(:));
u = double(u(:));
if numel(X) ~= 6 || numel(u) ~= 3
    error("floating_base_nonlinear_state:InvalidInput", ...
        "Expected X to have 6 elements and u to have 3 elements.");
end

m = base.m;
Iyy = base.Iyy;
g = base.g;
rH = rotatePitch2D(base.rHBody(:), X(3));

FHx = u(1);
FHz = u(2);
MBy = u(3);

dX = zeros(6, 1);
dX(1:3) = X(4:6);
dX(4) = FHx / m;
dX(5) = FHz / m - g;
dX(6) = (rH(2)*FHx - rH(1)*FHz + MBy) / Iyy;
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 + sin(theta)*rz0;
   -sin(theta)*rx0 + cos(theta)*rz0
];
end
