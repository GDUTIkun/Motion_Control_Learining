function varargout = two_link_leg_dynamics(q, dq, leg, term)
%TWO_LINK_LEG_DYNAMICS Fixed-hip planar two-link leg dynamics.
%
% Returns M(q), C(q,dq), and G(q) for:
%   M(q)*ddq + C(q,dq) + G(q) = tau
%
% Angles are measured from the vertical-down zero direction. qk is relative
% to the thigh, so the shank absolute angle is qh + qk.

if nargin < 3 || isempty(leg)
    leg = evalin("base", "leg");
end

if nargin < 4
    term = "all";
end

q = double(q(:));
dq = double(dq(:));

qh = q(1);
qk = q(2);
dqh = dq(1);
dqk = dq(2);

L1 = leg.L1;
c1 = leg.c1;
c2 = leg.c2;
m1 = leg.m1;
m2 = leg.m2;
I1 = leg.I1;
I2 = leg.I2;
g = leg.g;

cos_qk = cos(qk);
sin_qk = sin(qk);

M11 = I1 + I2 + m1*c1^2 + m2*(L1^2 + c2^2 + 2*L1*c2*cos_qk);
M12 = I2 + m2*(c2^2 + L1*c2*cos_qk);
M22 = I2 + m2*c2^2;
M = [M11, M12; M12, M22];

h = m2 * L1 * c2 * sin_qk;
C = [
    -h * (2*dqh*dqk + dqk^2);
     h * dqh^2
];

G = [
    g*((m1*c1 + m2*L1)*sin(qh) + m2*c2*sin(qh + qk));
    g*m2*c2*sin(qh + qk)
];

switch string(term)
    case "M"
        varargout = {M};
    case "C"
        varargout = {C};
    case "G"
        varargout = {G};
    otherwise
        varargout = {M, C, G};
end
end
