function test_wheel_position_lqr()
%TEST_WHEEL_POSITION_LQR Check scheduled planning, state, and limits.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));

stateInput = [0; base.xEq; leg.q0; leg.dq0];
measured = wheel_position_state_signal(stateInput, base, leg, ctrl);
assert(numel(measured) == 10 && all(isfinite(measured)));
assert(abs(measured(8) - wheelLqr.neutral) < 1e-10);
assert(abs(measured(9)) < 1e-10);
assert(abs(measured(10) - wheelLqr.heightNominal) < 1e-10);

expectedBxi = [-1/base.m - leg.r/(leg.mw*leg.r + leg.Iw/leg.r), ...
    0, -1/(leg.mw*leg.r + leg.Iw/leg.r)];
assert(norm(baseNmpc.model.B(8, :) - expectedBxi, inf) < 1e-12);

clear wheel_position_lqr_reference
tProbe = base.trajectory.settleTime + 0.25 * base.trajectory.accelDuration;
[xRef, ~] = floating_base_reference(tProbe, baseLqr);
probe = [tProbe; xRef; wheelLqr.neutral; 0; wheelLqr.heightNominal];
planned = wheel_position_lqr_reference(probe, wheelLqr, baseLqr);
assert(planned(4) < wheelLqr.neutral, ...
    "Forward acceleration must plan the wheel behind the base.");

clear wheel_position_lqr_reference
previous = wheelLqr.neutral;
for sample = 0:round(2 / base.Ts)
    t = sample * base.Ts;
    [xRef, ~] = floating_base_reference(t, baseLqr);
    planned = wheel_position_lqr_reference( ...
        [t; xRef; previous; 0; wheelLqr.heightNominal], wheelLqr, baseLqr);
    assert(planned(1) >= wheelLqr.positionMin - 1e-12 ...
        && planned(1) <= wheelLqr.positionMax + 1e-12);
    assert(abs(planned(2)) <= wheelLqr.velocityMax + 1e-12);
    assert(abs(planned(3)) <= wheelLqr.accelerationMax + 1e-9);
    previous = planned(1);
end

fprintf("Wheel-position LQR checks passed.\n");
end
