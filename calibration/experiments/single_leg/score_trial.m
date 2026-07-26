function score = score_trial(result, params, cfg) %#ok<INUSD>
%SCORE_TRIAL Convert a trial result into a scalar minimization score.

if ~isfield(result, "success") || ~result.success
    score = cfg.penaltyScore;
    return;
end

requiredMetrics = ["positionRms", "velocityRms", "finalPositionErrorNorm"];
for idx = 1:numel(requiredMetrics)
    if ~isfield(result.metrics, requiredMetrics(idx))
        score = cfg.penaltyScore;
        return;
    end
end

weights = cfg.scoreWeights;
torqueRms = 0;
saturationExcess = 0;
if isfield(result.metrics, "torque")
    torqueRms = result.metrics.torque.rmsRatio;
    saturationExcess = result.metrics.torque.saturationExcess;
end

score = ...
    weights.positionRms * result.metrics.positionRms + ...
    weights.velocityRms * result.metrics.velocityRms + ...
    weights.finalPosition * result.metrics.finalPositionErrorNorm + ...
    weights.torqueRms * torqueRms + ...
    weights.saturationPenalty * saturationExcess ^ 2;

if ~isfinite(score)
    score = cfg.penaltyScore;
end
end
