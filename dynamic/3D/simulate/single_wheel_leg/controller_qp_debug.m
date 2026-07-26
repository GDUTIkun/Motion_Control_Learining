function y = controller_qp_debug(x)
%CONTROLLER_QP_DEBUG Diagnostic output for QP inverse dynamics.
%
% Output:
%   y = [tau; qdd; Fc; exitflag]

[tau, debug] = controller_qp_core(x);
y = [tau; debug.qdd; debug.Fc; debug.exitflag];
end
