function configure_base_tracking_case(caseMode, plannerMode)
%CONFIGURE_BASE_TRACKING_CASE Configure stand, z, or velocity tracking.

if nargin < 1 || isempty(caseMode)
    caseMode = "velocity";
end
if nargin < 2 || isempty(plannerMode)
    plannerMode = "lqr";
end
caseMode = lower(string(caseMode));
plannerMode = lower(string(plannerMode));
if ~ismember(caseMode, ["stand", "z", "velocity"])
    error("configure_base_tracking_case:InvalidMode", ...
        "caseMode must be 'stand', 'z', or 'velocity'.");
end
if ~ismember(plannerMode, ["lqr", "qp_force"])
    error("configure_base_tracking_case:InvalidPlanner", ...
        "plannerMode must be 'lqr' or 'qp_force'.");
end

model = "source";
load_system(model);

base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
baseNmpc = evalin("base", "baseNmpc");
traj = evalin("base", "traj");
trajectory = base.trajectory;
trajectory.enabled = true;
trajectory.mode = caseMode;
trajectory.crouchDepth = 0;
stopTime = 10;
if caseMode == "stand"
    stopTime = 5;
elseif caseMode == "z"
    trajectory.crouchDepth = 0.025;
end
traj.wheelPositionPlanner = plannerMode;
baseNmpc.enabled = caseMode == "velocity" && plannerMode == "lqr";
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "baseNmpc", baseNmpc);
assignin("base", "traj", traj);

pulseBlocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");
for idx = 1:numel(pulseBlocks)
    set_param(pulseBlocks{idx}, "Amplitude", "0");
end

set_param(model, "StopTime", string(stopTime));
fprintf("Configured %g s %s case with %s wheel planning; NMPC enabled = %d.\n", ...
    stopTime, caseMode, plannerMode, baseNmpc.enabled);
end
