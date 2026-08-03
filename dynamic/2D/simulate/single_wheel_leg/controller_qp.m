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
%   x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext]
% or:
%   x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; MBy_des]
%
% FH_ext is the body-on-leg reaction force. If the upper LQR returns the
% force applied by the leg to the body, pass its negative here.

[tau, ~] = controller_qp_core(x);
end
