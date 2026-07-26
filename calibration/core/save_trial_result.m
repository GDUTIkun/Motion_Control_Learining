function trialFile = save_trial_result(params, score, result, cfg)
%SAVE_TRIAL_RESULT Persist one objective evaluation.

if ~isfield(cfg, "trialsDir") || isempty(cfg.trialsDir)
    trialsDir = fullfile(cfg.outputDir, "trials");
else
    trialsDir = cfg.trialsDir;
end

if ~isfolder(trialsDir)
    mkdir(trialsDir);
end

trialIndex = nextTrialIndex(trialsDir);
trialId = sprintf("trial_%04d", trialIndex);
trialFile = fullfile(trialsDir, sprintf("%s.mat", trialId));

while isfile(trialFile)
    trialIndex = trialIndex + 1;
    trialId = sprintf("trial_%04d", trialIndex);
    trialFile = fullfile(trialsDir, sprintf("%s.mat", trialId));
end

success = isfield(result, "success") && isequal(result.success, true);
errorMessage = "";
if isfield(result, "errorMessage")
    errorMessage = string(result.errorMessage);
end

elapsedTime = NaN;
if isfield(result, "elapsedTime")
    elapsedTime = result.elapsedTime;
end

timestamp = datetime("now", "Format", "yyyy-MM-dd HH:mm:ss.SSS");

save(trialFile, ...
    "trialId", "trialIndex", "timestamp", "params", "score", ...
    "result", "success", "errorMessage", "elapsedTime");
end

function trialIndex = nextTrialIndex(trialsDir)
files = dir(fullfile(trialsDir, "trial_*.mat"));
trialIndex = numel(files) + 1;
end
