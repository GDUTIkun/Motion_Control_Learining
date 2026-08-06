function score = score_trial(result, params, cfg) %#ok<INUSD>
%SCORE_TRIAL Convert QR training case metrics into one minimization score.

if ~isfield(result, "success") || ~result.success || ~isfield(result, "cases")
    score = cfg.penaltyScore;
    return;
end

weights = cfg.scoreWeights;
scoreTotal = 0;
weightTotal = 0;

for idx = 1:numel(result.cases)
    c = result.cases(idx).case;
    m = result.cases(idx).metrics;
    caseWeight = c.weight;
    caseScore = scoreOneCase(m, weights);

    if ~m.stable
        caseScore = caseScore + weights.unstablePenalty;
    end

    scoreTotal = scoreTotal + caseWeight * caseScore;
    weightTotal = weightTotal + caseWeight;
end

if weightTotal <= 0
    score = cfg.penaltyScore;
else
    score = scoreTotal / weightTotal;
end

if ~isfinite(score)
    score = cfg.penaltyScore;
end
end

function s = scoreOneCase(m, w)
s = ...
    w.thetaRms * m.thetaRms + ...
    w.finalTheta * abs(m.finalTheta) + ...
    w.maxAbsTheta * m.maxAbsTheta + ...
    w.dthetaRms * m.dthetaRms + ...
    w.finalX * abs(m.finalX) + ...
    w.maxAbsX * m.maxAbsX + ...
    w.tauSaturation * m.tauSaturationRatio + ...
    w.uLqrSaturation * m.uLqrSaturationRatio + ...
    w.legPositionRms * m.legPositionRms + ...
    w.legVelocityRms * m.legVelocityRms;
end
