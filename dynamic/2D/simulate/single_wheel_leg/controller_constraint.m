function tau = controller_constraint(x)
%CONTROLLER_CONSTRAINT Compatibility wrapper. The maintained controller is QP.
tau = controller_qp(x);
end
