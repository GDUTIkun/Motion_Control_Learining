function output = run_bayes_calibration(experiment)
%RUN_BAYES_CALIBRATION Run a reusable Bayesian calibration experiment.
%
%   output = RUN_BAYES_CALIBRATION("example_pd_single_leg")
%   output = RUN_BAYES_CALIBRATION("calibration/experiments/name/experiment_config.m")

if nargin < 1 || strlength(string(experiment)) == 0
    experiment = "example_pd_single_leg";
end

calibrationDir = fileparts(mfilename("fullpath"));
coreDir = fullfile(calibrationDir, "core");
experimentsDir = fullfile(calibrationDir, "experiments");

addpath(calibrationDir);
addpath(coreDir);

[configPath, experimentDir] = resolveExperimentConfig(experiment, experimentsDir);
addpath(experimentDir);

cfg = loadExperimentConfig(configPath, experimentDir);
cfg = prepareConfig(cfg, calibrationDir, experimentDir);

objectiveFcn = @(x) run_experiment_objective(x, cfg);
bayesOptions = default_bayes_options(cfg);

fprintf("Running Bayesian calibration: %s\n", cfg.name);
fprintf("Output directory: %s\n", cfg.outputDir);

bayesResults = bayesopt(objectiveFcn, cfg.variables, bayesOptions{:});

bestTable = bayesResults.XAtMinObjective;
bestParams = table_to_params(bestTable, cfg);
bestObjective = bayesResults.MinObjective;

save(fullfile(cfg.outputDir, "best_params.mat"), ...
    "bestParams", "bestObjective", "bestTable");
save(fullfile(cfg.outputDir, "bayes_results.mat"), ...
    "bayesResults", "cfg");

summary = summarize_bayes_result(bayesResults, cfg);

output = struct();
output.cfg = cfg;
output.bayesResults = bayesResults;
output.bestParams = bestParams;
output.bestObjective = bestObjective;
output.summary = summary;

fprintf("Calibration complete. Best objective: %.6g\n", bestObjective);
end

function [configPath, experimentDir] = resolveExperimentConfig(experiment, experimentsDir)
experiment = string(experiment);

candidate = char(experiment);
if isfolder(candidate)
    experimentDir = candidate;
    configPath = fullfile(experimentDir, "experiment_config.m");
elseif isfile(candidate)
    configPath = candidate;
    experimentDir = fileparts(configPath);
else
    experimentDir = fullfile(experimentsDir, char(experiment));
    configPath = fullfile(experimentDir, "experiment_config.m");
end

if ~isfile(configPath)
    error("run_bayes_calibration:ConfigNotFound", ...
        "Experiment config not found: %s", configPath);
end

configPath = char(java.io.File(configPath).getCanonicalPath());
experimentDir = char(java.io.File(experimentDir).getCanonicalPath());
end

function cfg = loadExperimentConfig(configPath, experimentDir)
originalDir = pwd;
cleanup = onCleanup(@() cd(originalDir));
cd(experimentDir);

clear experiment_config;
cfg = experiment_config();

if ~isstruct(cfg)
    error("run_bayes_calibration:InvalidConfig", ...
        "%s must return a struct.", configPath);
end
end

function cfg = prepareConfig(cfg, calibrationDir, experimentDir)
cfg.calibrationDir = calibrationDir;
cfg.experimentDir = experimentDir;

requiredFields = ["name", "variables"];
for idx = 1:numel(requiredFields)
    field = requiredFields(idx);
    if ~isfield(cfg, field) || isempty(cfg.(field))
        error("run_bayes_calibration:MissingConfigField", ...
            "cfg.%s is required.", field);
    end
end

cfg.name = string(cfg.name);

if ~isa(cfg.variables, "optimizableVariable")
    error("run_bayes_calibration:InvalidVariables", ...
        "cfg.variables must be an array of optimizableVariable objects.");
end

hasObjective = isfield(cfg, "objectiveFcn") && ~isempty(cfg.objectiveFcn);
hasTrialScore = isfield(cfg, "runTrialFcn") && ~isempty(cfg.runTrialFcn) ...
    && isfield(cfg, "scoreTrialFcn") && ~isempty(cfg.scoreTrialFcn);

if ~hasObjective && ~hasTrialScore
    if exist("run_trial", "file") == 2 && exist("score_trial", "file") == 2
        cfg.runTrialFcn = @run_trial;
        cfg.scoreTrialFcn = @score_trial;
    else
        error("run_bayes_calibration:MissingObjective", ...
            "Provide cfg.objectiveFcn or cfg.runTrialFcn plus cfg.scoreTrialFcn.");
    end
end

if ~isfield(cfg, "maxObjectiveEvaluations") || isempty(cfg.maxObjectiveEvaluations)
    cfg.maxObjectiveEvaluations = 30;
end
if ~isfield(cfg, "isDeterministic") || isempty(cfg.isDeterministic)
    cfg.isDeterministic = true;
end
if ~isfield(cfg, "useParallel") || isempty(cfg.useParallel)
    cfg.useParallel = false;
end
if ~isfield(cfg, "penaltyScore") || isempty(cfg.penaltyScore)
    cfg.penaltyScore = 1e9;
end
if ~isfield(cfg, "outputDir") || strlength(string(cfg.outputDir)) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    cfg.outputDir = fullfile(calibrationDir, "results", char(cfg.name), char(stamp));
end

cfg.outputDir = char(cfg.outputDir);
cfg.trialsDir = fullfile(cfg.outputDir, "trials");
if ~isfolder(cfg.trialsDir)
    mkdir(cfg.trialsDir);
end
end
