function y = controller_qp_debug(x)
%CONTROLLER_QP_DEBUG Diagnostic output for QP inverse dynamics.
%
% Output:
%   y = [tau; qdd; Fc; exitflag; FH_ext; MBy_des]

[tau, debug] = controller_qp_core(x);
y = [tau; debug.qdd; debug.Fc; debug.exitflag; ...
     debug.FH_ext; debug.MBy_des];
end
