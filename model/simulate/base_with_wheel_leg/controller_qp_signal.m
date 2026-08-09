function y = controller_qp_signal(x)
%CONTROLLER_QP_SIGNAL Simulink adapter with torque and QP wrench diagnostics.

[tau, debug] = controller_qp_core(x);
y = [tau; debug.wrenchSlack; debug.wrenchFeasible; debug.wrenchSlackNorm];
end
