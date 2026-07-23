function [q, dq, ddq] = wheel_leg_inverse_kinematics(pO, vO, aO, leg)
%WHEEL_LEG_INVERSE_KINEMATICS Two-link IK for the wheel center.
%
% The position convention matches wheel_leg_kinematics:
%   xO = L1*sin(qh) + L2*sin(qh + qk)
%   zO = -L1*cos(qh) - L2*cos(qh + qk)

if nargin < 4 || isempty(leg)
    leg = evalin("base", "leg");
end

pO = double(pO(:));
if nargin < 2 || isempty(vO)
    vO = zeros(2, 1);
else
    vO = double(vO(:));
end
if nargin < 3 || isempty(aO)
    aO = zeros(2, 1);
else
    aO = double(aO(:));
end

if numel(pO) ~= 2 || numel(vO) ~= 2 || numel(aO) ~= 2
    error("wheel_leg_inverse_kinematics:InvalidInput", ...
        "Expected pO, vO, and aO to be 2-element vectors.");
end

L1 = leg.L1;
L2 = leg.L2;
xO = pO(1);
zO = pO(2);

cos_qk = (xO^2 + zO^2 - L1^2 - L2^2) / (2 * L1 * L2);
cos_qk = min(max(cos_qk, -1), 1);
qk = acos(cos_qk);

theta = atan2(zO, xO) - atan2(L2*sin(qk), L1 + L2*cos(qk));
qh = theta + pi / 2;
q = [qh; qk];

kin = wheel_leg_kinematics([q; 0], zeros(3, 1), zeros(3, 1), leg);
J = kin.JO(:, 1:2);
dq = J \ vO;

kin = wheel_leg_kinematics([q; 0], [dq; 0], zeros(3, 1), leg);
dJ = kin.dJO(:, 1:2);
ddq = J \ (aO - dJ * dq);
end
