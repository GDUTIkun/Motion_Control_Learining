function y = coupled_two_leg_qp_signal(x)
%COUPLED_TWO_LEG_QP_SIGNAL Simulink adapter for the shared two-leg QP.

[tau, debug] = coupled_two_leg_qp_core(x);
y = [tau; debug.wrenchSlack; debug.wrenchFeasible; ...
    debug.wrenchSlackNorm; double(debug.qpFeasible); ...
    debug.FcLeft; debug.FcRight];
end
