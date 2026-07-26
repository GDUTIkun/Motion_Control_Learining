function score = run_experiment_objective(x, cfg)
%RUN_EXPERIMENT_OBJECTIVE Safe objective wrapper for bayesopt trials.

trialTimer = tic;
params = struct();
result = struct("success", false, "errorMessage", "", "elapsedTime", NaN);

try
    params = table_to_params(x, cfg);

    if isfield(cfg, "objectiveFcn") && ~isempty(cfg.objectiveFcn)
        if nargout(cfg.objectiveFcn) == 2
            [score, result] = cfg.objectiveFcn(params, cfg);
        else
            score = cfg.objectiveFcn(params, cfg);
            result.success = true;
        end
    else
        result = cfg.runTrialFcn(params, cfg);
        score = cfg.scoreTrialFcn(result, params, cfg);
    end

    if ~isstruct(result)
        result = struct("success", true, "rawResult", result);
    end

    if ~isfield(result, "success") || isempty(result.success)
        result.success = true;
    end

    if ~result.success
        score = cfg.penaltyScore;
    end

    if ~isnumeric(score) || ~isscalar(score) || ~isfinite(score)
        result.success = false;
        result.errorMessage = appendMessage(result, "Score was not a finite numeric scalar.");
        score = cfg.penaltyScore;
    end
catch err
    result = struct();
    result.success = false;
    result.errorMessage = getReport(err, "extended", "hyperlinks", "off");
    result.errorIdentifier = err.identifier;
    score = cfg.penaltyScore;
end

result.elapsedTime = toc(trialTimer);

try
    save_trial_result(params, double(score), result, cfg);
catch err
    warning("run_experiment_objective:SaveFailed", ...
        "Failed to save trial result: %s", err.message);
end
end

function message = appendMessage(result, newMessage)
if isfield(result, "errorMessage") && strlength(string(result.errorMessage)) > 0
    message = string(result.errorMessage) + newline + string(newMessage);
else
    message = string(newMessage);
end
message = char(message);
end
