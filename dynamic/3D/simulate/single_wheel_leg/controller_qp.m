function tau = controller_qp(x)
%CONTROLLER_QP Small QP inverse dynamics with contact constraints.
%
% Decision variable:
%   z = [qdd(3); tau(3); Fc(2)]
%
% Equality constraints:
%   M*qdd - tau - Jc'*Fc = JH'*FH_ext - C - G
%   Jc*qdd = -dJc*dq - aH - Kc*(Jc*dq + vH)
%
% Inequality constraints:
%   Fcz >= 0
%   |Fcx| <= mu*Fcz
%
% Input:
%   x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHy_ext]

[tau, ~] = controller_qp_core(x);
end
