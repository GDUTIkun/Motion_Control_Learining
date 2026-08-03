function model = floating_base_state_space(base)
%FLOATING_BASE_STATE_SPACE Linearize a 2D floating-base wrench model.
%
% State:
%   X = [x; z; theta; dx; dz; dtheta]
%
% Input:
%   U = [FHx; FHz; MBy]
%
% FH is the force that the wheel-leg applies to the body at the hip
% interface. MBy is the pure pitch moment applied to the body by the hip
% interface. Positive theta and MBy follow the 3D pitch-axis convention used
% in the notes:
%   tau_y = r_z*FHx - r_x*FHz + MBy.

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end

m = getField(base, "m");
Iyy = getField(base, "Iyy");
g = getFieldOrDefault(base, "g", 9.81);
rHBody = getField(base, "rHBody");
thetaEq = getFieldOrDefault(base, "thetaEq", 0);

rH = rotatePitch2D(rHBody(:), thetaEq);
rx = rH(1);
rz = rH(2);

uEq = [0; m*g; rx*m*g];
xEq = getFieldOrDefault(base, "xEq", [0; 0; thetaEq; 0; 0; 0]);
xEq = xEq(:);
if numel(xEq) ~= 6
    error("floating_base_state_space:InvalidEquilibrium", ...
        "base.xEq must be a 6-element vector.");
end

A = zeros(6, 6);
A(1, 4) = 1;
A(2, 5) = 1;
A(3, 6) = 1;

% d/dtheta of tau_y = r_z(theta)*FHx - r_x(theta)*FHz + MBy.
% With r_x' = r_z and r_z' = -r_x:
%   ktheta = -r_x*FHx_eq - r_z*FHz_eq.
ktheta = -rx*uEq(1) - rz*uEq(2);
A(6, 3) = ktheta / Iyy;

B = zeros(6, 3);
B(4, 1) = 1 / m;
B(5, 2) = 1 / m;
B(6, 1) = rz / Iyy;
B(6, 2) = -rx / Iyy;
B(6, 3) = 1 / Iyy;

model = struct();
model.A = A;
model.B = B;
model.C = eye(6);
model.D = zeros(6, 3);
model.xEq = xEq;
model.uEq = uEq;
model.rHEq = rH;
model.ktheta = ktheta;
model.stateNames = ["x"; "z"; "theta"; "dx"; "dz"; "dtheta"];
model.inputNames = ["FHx"; "FHz"; "MBy"];
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 + sin(theta)*rz0;
   -sin(theta)*rx0 + cos(theta)*rz0
];
end

function value = getField(s, name)
if ~isfield(s, name)
    error("floating_base_state_space:MissingField", ...
        "Missing base.%s.", name);
end
value = s.(name);
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
