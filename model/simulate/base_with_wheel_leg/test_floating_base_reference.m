function test_floating_base_reference()
%TEST_FLOATING_BASE_REFERENCE Minimal endpoint check for the base trajectory.

baseLqr.xRef = zeros(6, 1);
baseLqr.trajectory = struct( ...
    "enabled", true, ...
    "mode", "velocity", ...
    "settleTime", 1, ...
    "cruiseVelocity", 0.5, ...
    "accelDuration", 0.5, ...
    "cruiseDuration", 1.5, ...
    "decelDuration", 0.5, ...
    "turnHoldDuration", 0.5);

[xStart, aStart] = floating_base_reference(0, baseLqr);
[xForward, ~] = floating_base_reference(3.5, baseLqr);
[xReverse, aReverse] = floating_base_reference(4.5, baseLqr);
[xEnd, aEnd] = floating_base_reference(6.5, baseLqr);

assert(isequal(xStart, zeros(6, 1)) && isequal(aStart, zeros(3, 1)));
assert(xForward(1) == 1.0 && xForward(4) == 0);
assert(xReverse(4) == -0.5 && aReverse(1) == 0);
assert(isequal(xEnd, zeros(6, 1)) && isequal(aEnd, zeros(3, 1)));

baseLqr.trajectory.mode = "stand";
assert(isequal(floating_base_reference(3, baseLqr), zeros(6, 1)));

baseLqr.trajectory.mode = "z";
baseLqr.trajectory.crouchDepth = 0.025;
baseLqr.trajectory.crouchDownDuration = 1;
baseLqr.trajectory.crouchRecoverStart = 6.5;
baseLqr.trajectory.crouchRecoverDuration = 1;
xZ = floating_base_reference(1, baseLqr);
assert(abs(xZ(2) + 0.025) < 1e-12 && xZ(1) == 0);
end
