function score = run_smoke_test()
%RUN_SMOKE_TEST Run one short QR objective evaluation for wiring checks.

calibrationDir = char(java.io.File(fullfile(fileparts(mfilename("fullpath")), "..", "..")).getCanonicalPath());
experimentDir = fileparts(mfilename("fullpath"));

addpath(calibrationDir);
addpath(fullfile(calibrationDir, "core"));
addpath(experimentDir);

cfg = experiment_config();
cfg.calibrationDir = calibrationDir;
cfg.experimentDir = experimentDir;
cfg.caseLimit = 1;
cfg.outputDir = fullfile(calibrationDir, "results", char(cfg.name), "smoke_test");
cfg.trialsDir = fullfile(cfg.outputDir, "trials");
if ~isfolder(cfg.trialsDir)
    mkdir(cfg.trialsDir);
end

variableNames = {'log_s_qtheta', 'log_s_qdtheta', 'log_s_qx', ...
    'log_s_qdx', 'log_s_rfhx', 'log_s_rfz', 'log_s_rmb'};
x = array2table(zeros(1, numel(variableNames)), ...
    'VariableNames', variableNames);

score = run_experiment_objective(x, cfg);
fprintf("Smoke-test score: %.6g\n", score);
end
