function test_turning_motion_reference()
%TEST_TURNING_MOTION_REFERENCE Validate steering profiles and sign rules.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
d = fullBaseNmpc.model.halfTrack;
trajectory = base.trajectory;
trajectory.cruiseVelocity = 0.1;
trajectory.turning.enabled = true;
trajectory.turning.mode = "single";
trajectory.turning.yawRate = 0.05;
trajectory.turning.startTime = 1;
trajectory.turning.rampDuration = 0.5;
trajectory.turning.holdDuration = 1;

before = turning_motion_reference(0.5, 0.1, trajectory, d);
steady = turning_motion_reference(1.75, 0.1, trajectory, d);
after = turning_motion_reference(3.5, 0.1, trajectory, d);
assert(norm(before([1:4, 6]) - [0; 0; 0; 0; 0.1], inf) < 1e-12);
assert(abs(steady(2) - 0.05) < 1e-12);
assert(steady(6) > steady(5), ...
    "Positive yaw must command the right wheel faster than the left.");
assert(abs(steady(6) - steady(5) - 2*d*steady(2)) < 1e-12);
assert(abs(after(2)) < 1e-12 && abs(after(3)) < 1e-12);

mirrorTrajectory = trajectory;
mirrorTrajectory.turning.yawRate = -trajectory.turning.yawRate;
mirror = turning_motion_reference(1.75, 0.1, mirrorTrajectory, d);
assert(norm(mirror(1:4) + steady(1:4), inf) < 1e-12);
assert(abs(mirror(5) - steady(6)) < 1e-12);
assert(abs(mirror(6) - steady(5)) < 1e-12);

zeroYawTrajectory = trajectory;
zeroYawTrajectory.turning.yawRate = 0;
straight = turning_motion_reference(1.75, 0.1, zeroYawTrajectory, d);
assert(abs(straight(5) - straight(6)) < 1e-12);

noSpinTrajectory = trajectory;
noSpinTrajectory.cruiseVelocity = 0;
noSpin = turning_motion_reference(1.75, 0, noSpinTrajectory, d);
assert(norm(noSpin(1:6), inf) < 1e-12, ...
    "The first steering version must not request ideal in-place rotation.");
didRejectInPlace = false;
try
    configure_turning_case(0, 0.05, "single", "source");
catch exception
    didRejectInPlace = string(exception.identifier) ...
        == "configure_turning_case:InPlaceTurnUnsupported";
end
assert(didRejectInPlace, ...
    "The public steering interface must reject ideal in-place rotation.");

sTrajectory = trajectory;
sTrajectory.turning.mode = "s";
sTrajectory.turning.holdDuration = 0.75;
sTrajectory.turning.zeroHoldDuration = 0.25;
sEndTime = sTrajectory.turning.startTime ...
    + 4*sTrajectory.turning.rampDuration ...
    + 2*sTrajectory.turning.holdDuration ...
    + sTrajectory.turning.zeroHoldDuration + 0.5;
sEnd = turning_motion_reference(sEndTime, 0.1, sTrajectory, d);
assert(abs(sEnd(1)) < 1e-12 && abs(sEnd(2)) < 1e-12);

dt = 1e-5;
tRamp = trajectory.turning.startTime + 0.25;
minus = turning_motion_reference(tRamp - dt, 0.1, trajectory, d);
plus = turning_motion_reference(tRamp + dt, 0.1, trajectory, d);
numericYawRate = (plus(1) - minus(1))/(2*dt);
numericYawAcceleration = (plus(2) - minus(2))/(2*dt);
center = turning_motion_reference(tRamp, 0.1, trajectory, d);
assert(abs(numericYawRate - center(2)) < 1e-8);
assert(abs(numericYawAcceleration - center(3)) < 1e-8);

baseLqrTurning = baseLqr;
baseLqrTurning.trajectory = trajectory;
planner = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
reference = full_base_nmpc_reference( ...
    [tRamp; planner; fullBaseNmpc.model.uEq], ...
    baseLqrTurning, fullBaseNmpc, wheelLqr);
turning = turning_motion_reference(tRamp, ...
    floatingVelocity(tRamp, baseLqrTurning), trajectory, d);
stateReference = reference(1:16);
inputReference = reference(17:28);
yawMoment = d*(inputReference(7) - inputReference(1));
assert(abs(stateReference(6) - turning(1)) < 1e-12);
assert(abs(stateReference(12) - turning(2)) < 1e-12);
assert(abs(yawMoment - fullBaseNmpc.model.inertia(3)*turning(3)) < 1e-10);

fprintf("Turning-motion reference checks passed.\n");
end

function vx = floatingVelocity(t, baseLqr)
[reference, ~] = floating_base_reference(t, baseLqr);
vx = reference(4);
end
