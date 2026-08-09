function test_force_based_wheel_position_reference()
%TEST_FORCE_BASED_WHEEL_POSITION_REFERENCE Minimal Scheme-1 governor check.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
baseLqr.trajectory.mode = "velocity_round_trip";
assignin("base", "baseLqr", baseLqr);
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");
traj = evalin("base", "traj");
baseState = zeros(6, 1);
FHSupport = base.m * base.g;
Ts = base.Ts;

nominalKin = wheel_leg_kinematics([traj.nominalOffset; 0], ...
    zeros(3, 1), zeros(3, 1), leg);
assert(abs(traj.xO0 - nominalKin.pO(1)) < 1e-12 ...
        && abs(traj.zO0 - nominalKin.pO(2) ...
        - traj.defaultHeightReduction) < 1e-12, ...
    "Default pose did not apply the requested height reduction.");
assert(abs(traj.xO0) < 1e-12 && leg.q0(2) > traj.nominalOffset(2), ...
    "Lower default pose is not on the expected positive-knee branch.");

crouchLqr = baseLqr;
crouchLqr.trajectory.crouchDepth = 0.025;
[xStart, aStart] = floating_base_reference(0, crouchLqr);
[xDownMid, ~] = floating_base_reference(0.5, crouchLqr);
[xDown, aDown] = floating_base_reference(1, crouchLqr);
[xUpMid, ~] = floating_base_reference(7, crouchLqr);
[xRecovered, aRecovered] = floating_base_reference(7.5, crouchLqr);
assert(abs(xStart(2)) < 1e-12 && abs(xStart(5)) < 1e-12 ...
        && abs(aStart(2)) < 1e-12, ...
    "Crouch reference is discontinuous at startup.");
assert(abs(xDownMid(2) + 0.0125) < 1e-12 && xDownMid(5) < 0, ...
    "Crouch descent reference is incorrect.");
assert(abs(xDown(2) + 0.025) < 1e-12 && abs(xDown(5)) < 1e-12 ...
        && abs(aDown(2)) < 1e-12, ...
    "Crouch hold reference is incorrect.");
assert(abs(xUpMid(2) + 0.0125) < 1e-12 && xUpMid(5) > 0, ...
    "Crouch recovery reference is incorrect.");
assert(abs(xRecovered(2)) < 1e-12 && abs(xRecovered(5)) < 1e-12 ...
        && abs(aRecovered(2)) < 1e-12, ...
    "Crouch reference did not recover smoothly.");

clear floating_base_leg_reference
[~, ~, ~, previous] = floating_base_leg_reference(0, baseState, ...
    traj, leg, base, zeros(2, 1), -[0; FHSupport]);

t = 0;
offsetMin = inf;
offsetMax = -inf;
for sample = 1:round(6.5 / Ts)
    t = t + Ts;
    bodyFx = 140 * (-1)^sample;
    [qd, ~, ~, plan] = floating_base_leg_reference(t, baseState, ...
        traj, leg, base, zeros(2, 1), -[bodyFx; FHSupport]);

    [~, aRef] = floating_base_reference(t, baseLqr);
    assert(abs(plan.forcePlanningX - base.m * aRef(1)) < 1e-12, ...
        "Wheel planner did not use the reference-acceleration force.");

    assert(plan.rXDes >= plan.rXLower - 1e-12 && ...
        plan.rXDes <= plan.rXUpper + 1e-12, ...
        "Wheel governor left the safe horizontal interval.");
    assert(abs(plan.drXDes) <= traj.wheelPositionVelocityMax + 1e-12, ...
        "Wheel governor exceeded its velocity limit.");
    assert(abs(plan.ddrXDes) <= traj.wheelPositionAccelerationMax + 1e-9, ...
        "Wheel governor exceeded its acceleration limit.");
    assert(abs((plan.rXDes - previous.rXDes) / Ts - plan.drXDes) < 1e-8, ...
        "Wheel position and velocity references are inconsistent.");
    if plan.geometryFeasible
        assert(qd(2) >= traj.wheelPositionKneeMin - 1e-9, ...
            "Wheel reference crossed the configured knee margin.");
    end
    offsetMin = min(offsetMin, plan.rXDes - plan.rXNeutral);
    offsetMax = max(offsetMax, plan.rXDes - plan.rXNeutral);
    previous = plan;
end

assert(offsetMin < 0 && offsetMax > 0, ...
    "Wheel reference did not respond to both acceleration directions.");

probeTime = 1.25;
[~, ~, ~, positiveFeedback] = floating_base_leg_reference(probeTime, ...
    baseState, traj, leg, base, zeros(2, 1), -[140; FHSupport], false);
[~, ~, ~, negativeFeedback] = floating_base_leg_reference(probeTime, ...
    baseState, traj, leg, base, zeros(2, 1), -[-140; FHSupport], false);
assert(abs(positiveFeedback.rXEquilibrium ...
        - negativeFeedback.rXEquilibrium) < 1e-12, ...
    "Total LQR feedback force leaked into the wheel planner.");

% Scope/reference reads must not advance the controller-owned governor.
[~, ~, ~, readOnly] = floating_base_leg_reference(t + Ts/2, baseState, ...
    traj, leg, base, zeros(2, 1), -[140; FHSupport], false);
assert(abs(readOnly.rXDes - previous.rXDes) < 1e-12, ...
    "A read-only reference call advanced the wheel governor.");

clear controller_qp_core floating_base_leg_reference
controllerInput = [0; baseState; leg.q0; leg.dq0; 0; -FHSupport; 0];
[tau, controllerDebug] = controller_qp_core(controllerInput);
assert(all(isfinite(tau)) && controllerDebug.exitflag > 0, ...
    "Constrained controller integration failed.");
assert(all(isfinite(wheel_leg_tracking_signal(controllerInput))), ...
    "Tracking-signal integration returned a non-finite reference.");

guardQ = leg.q0;
guardDq = zeros(3, 1);
guardQ(2) = ctrl.kneeGuardMin;
guardDq(2) = -0.5;
guardInput = [Ts; baseState; guardQ; guardDq; 0; -FHSupport; 0];
[~, guardDebug] = controller_qp_core(guardInput);
assert(guardDebug.exitflag > 0 && ...
    guardDebug.qdd(2) >= guardDebug.kneeQddMin - 1e-7, ...
    "QP did not enforce the knee acceleration guard.");

fprintf("Force-based wheel-position governor check passed.\n");
end
