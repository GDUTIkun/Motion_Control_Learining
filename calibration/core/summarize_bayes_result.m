function summary = summarize_bayes_result(bayesResults, cfg)
%SUMMARIZE_BAYES_RESULT Save CSV and objective-history plot.

objectiveTrace = bayesResults.ObjectiveTrace(:);
iteration = (1:numel(objectiveTrace)).';
bestObjective = cumulativeFiniteMin(objectiveTrace);

summary = table(iteration, objectiveTrace, bestObjective, ...
    'VariableNames', {'iteration', 'objective', 'bestObjective'});

try
    xTrace = bayesResults.XTrace;
    if height(xTrace) == height(summary)
        summary = [summary, xTrace];
    end
catch
    % Older bayesopt result shapes may not expose XTrace.
end

summaryPath = fullfile(cfg.outputDir, "summary.csv");
writetable(summary, summaryPath);

plotPath = fullfile(cfg.outputDir, "objective_history.png");
fig = figure("Visible", "off");
plot(iteration, objectiveTrace, "o-", "LineWidth", 1.0);
hold on;
plot(iteration, bestObjective, "-", "LineWidth", 2.0);
grid on;
xlabel("Iteration");
ylabel("Objective");
legend(["Objective", "Best so far"], "Location", "best");
title("Bayesian calibration objective history");
saveas(fig, plotPath);
close(fig);
end

function bestObjective = cumulativeFiniteMin(objectiveTrace)
bestObjective = nan(size(objectiveTrace));
currentBest = Inf;
for idx = 1:numel(objectiveTrace)
    value = objectiveTrace(idx);
    if isfinite(value)
        currentBest = min(currentBest, value);
    end
    bestObjective(idx) = currentBest;
end
end
