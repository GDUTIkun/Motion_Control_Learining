function score = score_trial(result, params, cfg) %#ok<INUSD>
%SCORE_TRIAL Convert a wheel-leg QP trial result into one minimization score.

if ~isfield(result, "success") || ~result.success
    score = cfg.penaltyScore;
    return;
end

requiredMetrics = [
    "legPositionRms"
    "wheelPositionRms"
    "legVelocityRms"
    "wheelVelocityRms"
    "finalLegPositionErrorNorm"
    "finalWheelPositionErrorAbs"
    ];
for idx = 1:numel(requiredMetrics)
    if ~isfield(result.metrics, requiredMetrics(idx))
        score = cfg.penaltyScore;
        return;
    end
end

weights = cfg.scoreWeights;
bandwidthNorm = 0;
if isfield(result.metrics, "bandwidthNorm")
    bandwidthNorm = result.metrics.bandwidthNorm;
end

score = ...
    weights.legPositionRms * result.metrics.legPositionRms + ...
    weights.wheelPositionRms * result.metrics.wheelPositionRms + ...
    weights.legVelocityRms * result.metrics.legVelocityRms + ...
    weights.wheelVelocityRms * result.metrics.wheelVelocityRms + ...
    weights.finalLegPosition * result.metrics.finalLegPositionErrorNorm + ...
    weights.finalWheelPosition * result.metrics.finalWheelPositionErrorAbs + ...
    weights.bandwidthPenalty * bandwidthNorm;

if ~isfinite(score)
    score = cfg.penaltyScore;
end
end
