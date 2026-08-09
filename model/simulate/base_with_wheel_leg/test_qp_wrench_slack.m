function test_qp_wrench_slack()
%TEST_QP_WRENCH_SLACK Verify feasible wrench projection and hard torque bounds.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");

x = [0; zeros(6, 1); leg.q0; leg.dq0; 0; -base.m * base.g; 1e3];
[tau, debug] = controller_qp_core(x);

assert(debug.qpFeasible && all(isfinite(tau)));
assert(all(abs(tau) <= ctrl.tauMax + 1e-8));
assert(norm(debug.wrenchFeasible - ...
    (debug.wrenchCommand + debug.wrenchSlack), inf) < 1e-10);
assert(abs(tau(1) - ctrl.hipMomentToTauSign * ...
    debug.wrenchFeasible(3)) < 1e-7);
assert(abs(debug.wrenchSlack(3)) > 1e-6);
end
