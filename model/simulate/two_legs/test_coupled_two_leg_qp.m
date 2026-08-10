function test_coupled_two_leg_qp()
%TEST_COUPLED_TWO_LEG_QP Small regression check for the shared QP.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
wheelLqr = evalin("base", "wheelLqr");

wheelReference = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
upperCommand = [0; -base.m*base.g; 0];
x = [0; zeros(6, 1); leg.q0; leg.dq0; leg.q0; leg.dq0; ...
    upperCommand; wheelReference];
clear coupled_two_leg_qp_core
[tau, debug] = coupled_two_leg_qp_core(x);

assert(numel(tau) == 6 && all(isfinite(tau)));
assert(all(abs(tau) <= repmat(ctrl.tauMax, 2, 1) + 1e-8));
assert(debug.qpFeasible, "The nominal coupled QP is infeasible.");
assert(norm(debug.dynamicsResidual, inf) < 1e-6);
assert(norm(debug.symmetryQddError, inf) < 1e-6);
assert(norm(debug.massMatrix - debug.massMatrix', inf) < 1e-10);
assert(min(eig(debug.massMatrix)) > 0);
assert(norm(tau(1:3) - tau(4:6), inf) < 1e-7);
assert(norm(debug.FcLeft - debug.FcRight, inf) < 1e-7);
assert(norm(debug.wrenchFeasible - ...
    (debug.wrenchCommand + debug.wrenchSlack), inf) < 1e-7);

% The deployed reduced QP may use common-mode-specific task priorities, but
% it must remain symmetric, feasible, and dynamically consistent.
clear coupled_two_leg_qp_core
[tauCommon, common] = coupled_two_leg_qp_core(x, "common");
assert(common.commonMode && common.qpFeasible);
assert(norm(tauCommon(1:3) - tauCommon(4:6), inf) < 1e-10);
assert(norm(common.FcLeft - common.FcRight, inf) < 1e-10);
assert(norm(common.dynamicsResidual, inf) < 1e-6);
assert(norm(common.wrenchFeasible - ...
    (common.wrenchCommand + common.wrenchSlack), inf) < 1e-7);
assert(abs(common.wrenchSlack(3)) <= ctrl.commonModeMomentSlackMax + 1e-8);
assert(isequal(size(common.massMatrix), [6, 6]));
assert(min(eig(common.massMatrix)) > 0);
assert(norm(common.qdd(1:6), inf) < 1e-4, ...
    "The nominal common-mode state must be a static QP equilibrium.");

% With identical objective weights and inactive common-only slack bounds,
% the reduced QP is the exact symmetric restriction of the full QP.
ctrlEquivalent = ctrl;
ctrlEquivalent.commonModeQpWqdd = ctrlEquivalent.qpWqdd;
ctrlEquivalent.commonModeMomentSlackMax = inf;
restoreCtrl = onCleanup(@() assignin("base", "ctrl", ctrl));
assignin("base", "ctrl", ctrlEquivalent);
clear coupled_two_leg_qp_core
[tauFullEquivalent, fullEquivalent] = coupled_two_leg_qp_core(x, "full");
clear coupled_two_leg_qp_core
[tauCommonEquivalent, commonEquivalent] = coupled_two_leg_qp_core(x, "common");
assert(norm(tauCommonEquivalent - tauFullEquivalent, inf) < 1e-5);
assert(norm(commonEquivalent.FcLeft - fullEquivalent.FcLeft, inf) < 1e-5);
assert(norm(commonEquivalent.wrenchFeasible - ...
    fullEquivalent.wrenchFeasible, inf) < 1e-7);
assignin("base", "ctrl", ctrl);

% A differential leg perturbation must produce differential acceleration
% and torque; otherwise the independent left/right mode is uncontrolled.
xDifferential = x;
xDifferential(8) = xDifferential(8) + 0.01;
[tauDifferential, differential] = coupled_two_leg_qp_core(xDifferential);
qError = xDifferential(8:10) - xDifferential(14:16);
assert(dot(qError, differential.symmetryQddError) < 0);
assert(norm(tauDifferential(1:3) - tauDifferential(4:6), inf) > 1e-4);

upperCommand(1) = -20;
x(20:22) = upperCommand;
[~, pulse] = coupled_two_leg_qp_core(x);
assert(pulse.qpFeasible && pulse.wrenchFeasible(1) > 0);
assert(abs(pulse.wrenchSlack(1)) < abs(pulse.wrenchCommand(1)));

fprintf("Coupled two-leg floating-base QP checks passed.\n");
end
