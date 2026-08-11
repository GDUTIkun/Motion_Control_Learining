function test_coupled_two_leg_qp()
%TEST_COUPLED_TWO_LEG_QP Small regression check for the shared QP.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
wheelLqr = evalin("base", "wheelLqr");
baseNmpc = evalin("base", "baseNmpc");

wheelReference = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
upperCommand = [-baseNmpc.model.uEq(1:2); baseNmpc.model.uEq(3)];
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
assert(norm(debug.qdd, inf) < 1e-4, ...
    "The nominal full two-leg state must be a static QP equilibrium.");
assert(all(debug.frictionMargin >= -1e-8));
assert(all(debug.torqueMargin >= -1e-8));

% The deployed full path retains six base coordinates and three contact
% force components per wheel.
fullState = [0; fullBaseNmpc.model.xEq; -traj.zO0];
fullCommand = fullBaseNmpc.model.uEq;
xSpatial = [fullState; leg.q0; leg.dq0; leg.q0; leg.dq0; ...
    fullCommand; wheelReference];
clear coupled_two_leg_qp_core spatial_two_leg_qp_core
[tauSpatial, spatial] = coupled_two_leg_qp_core(xSpatial);
assert(spatial.spatialQp && isequal(size(spatial.massMatrix), [12, 12]));
assert(numel(spatial.FcLeft) == 3 && numel(spatial.FcRight) == 3);
assert(numel(spatial.wrenchFeasible) == 12);
assert(spatial.qpFeasible, "The nominal spatial QP is infeasible.");
assert(norm(spatial.dynamicsResidual, inf) < 1e-6);
assert(norm(spatial.contactResidual, inf) < 1e-6);
assert(norm(spatial.wrenchResidual, inf) < 1e-6);
assert(min(eig(spatial.massMatrix)) > 0);
assert(all(abs(tauSpatial) <= repmat(ctrl.tauMax, 2, 1) + 1e-8));
assert(all(spatial.frictionMargin >= -1e-8));

% Differential longitudinal/vertical interaction requests must remain
% distinct instead of being summed before the QP.
xSpatialDifferential = xSpatial;
xSpatialDifferential(31:42) = fullCommand;
xSpatialDifferential([31, 37]) = [2; -2];
xSpatialDifferential([33, 39]) = fullCommand([3, 9]) + [3; -3];
clear spatial_two_leg_qp_core
[~, spatialDifferential] = coupled_two_leg_qp_core(xSpatialDifferential);
assert(spatialDifferential.spatialQp);
assert(norm(spatialDifferential.wrenchCommand ...
    - xSpatialDifferential(31:42), inf) < 1e-12);

% The deployed reduced QP may use common-mode-specific task priorities, but
% it must remain symmetric, feasible, and dynamically consistent.
upperCommandCommon = [-baseNmpc.model.uEq(1:2); baseNmpc.model.uEq(3)];
xCommon = x;
xCommon(20:22) = upperCommandCommon;
clear coupled_two_leg_qp_core
[tauCommon, common] = coupled_two_leg_qp_core(xCommon, "common");
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

% A differential leg perturbation must produce differential acceleration
% and torque; otherwise the independent left/right mode is uncontrolled.
xDifferential = x;
xDifferential(8) = xDifferential(8) + 0.01;
[tauDifferential, differential] = coupled_two_leg_qp_core(xDifferential);
qError = xDifferential(8:10) - xDifferential(14:16);
assert(dot(qError, differential.symmetryQddError) < 0);
assert(norm(tauDifferential(1:3) - tauDifferential(4:6), inf) > 1e-4);
assert(differential.xiDifferential * ...
    differential.xiDifferentialAcceleration <= 0);

% Swapping left and right preserves common quantities and flips differences.
xMirrored = x;
xMirrored(14) = xMirrored(14) + 0.01;
[tauMirrored, mirrored] = coupled_two_leg_qp_core(xMirrored);
assert(norm(tauDifferential(1:3) - tauMirrored(4:6), inf) < 1e-6);
assert(norm(tauDifferential(4:6) - tauMirrored(1:3), inf) < 1e-6);
assert(norm(differential.qddCommon - mirrored.qddCommon, inf) < 1e-6);
assert(norm(differential.qddDifferential + ...
    mirrored.qddDifferential, inf) < 1e-6);
assert(norm(differential.contactForceCommon - ...
    mirrored.contactForceCommon, inf) < 1e-6);
assert(norm(differential.contactForceDifferential + ...
    mirrored.contactForceDifferential, inf) < 1e-6);

upperCommand(1) = -20;
x(20:22) = upperCommand;
[~, pulse] = coupled_two_leg_qp_core(x);
assert(pulse.qpFeasible && pulse.wrenchFeasible(1) > 0);
assert(abs(pulse.wrenchSlack(1)) < abs(pulse.wrenchCommand(1)));

xCommon(20) = 20;
[~, commonPulse] = coupled_two_leg_qp_core(xCommon, "common");
assert(commonPulse.qpFeasible && commonPulse.wrenchFeasible(1) < 0);
assert(abs(commonPulse.wrenchSlack(1)) < abs(commonPulse.wrenchCommand(1)));

fprintf("Coupled two-leg floating-base QP checks passed.\n");
end
