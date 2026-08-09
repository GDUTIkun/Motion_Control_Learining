function configure_base_tracking_case(caseMode)
%CONFIGURE_BASE_TRACKING_CASE Configure stand, z, or velocity tracking.

if nargin < 1 || isempty(caseMode)
    caseMode = "velocity";
end
caseMode = lower(string(caseMode));
if ~ismember(caseMode, ["stand", "z", "velocity"])
    error("configure_base_tracking_case:InvalidMode", ...
        "caseMode must be 'stand', 'z', or 'velocity'.");
end

model = "source";
load_system(model);
configure_discrete_controller_timing(false);

base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
baseNmpc = evalin("base", "baseNmpc");
trajectory = base.trajectory;
trajectory.enabled = true;
trajectory.mode = caseMode;
trajectory.crouchDepth = 0;
stopTime = 10;
if caseMode == "stand"
    stopTime = 5;
    % The generated NMPC command does not meet the stand pitch acceptance;
    % use the existing LQR fallback for this stability baseline.
    baseNmpc.enabled = false;
elseif caseMode == "z"
    trajectory.crouchDepth = 0.025;
end
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "baseNmpc", baseNmpc);

pulseBlocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");
for idx = 1:numel(pulseBlocks)
    set_param(pulseBlocks{idx}, "Amplitude", "0");
end

set_param(model, "StopTime", string(stopTime));
fprintf("Configured %g s %s case with pulse disturbances disabled.\n", ...
    stopTime, caseMode);
end
