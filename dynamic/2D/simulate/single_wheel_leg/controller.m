function tau = controller(x)
%CONTROLLER Compatibility wrapper. The maintained leg controller is QP.
tau = controller_qp(x);
end
