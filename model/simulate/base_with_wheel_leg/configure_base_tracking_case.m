function configure_base_tracking_case()
%CONFIGURE_BASE_TRACKING_CASE Prepare a zero-disturbance base motion test.

model = "source";
load_system(model);
configure_discrete_controller_timing(false);

pulseBlocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");
for idx = 1:numel(pulseBlocks)
    set_param(pulseBlocks{idx}, "Amplitude", "0");
end

set_param(model, "StopTime", "10");
fprintf("Configured 10 s base tracking case with all pulse disturbances disabled.\n");
end
