function score = run_default_qr_check()
%RUN_DEFAULT_QR_CHECK Check the current default Q/R before Bayesian tuning.

calibrationDir = char(java.io.File(fullfile(fileparts(mfilename("fullpath")), "..", "..")).getCanonicalPath());
experimentDir = fileparts(mfilename("fullpath"));

addpath(calibrationDir);
addpath(fullfile(calibrationDir, "core"));
addpath(experimentDir);

cfg = experiment_config();
cfg.calibrationDir = calibrationDir;
cfg.experimentDir = experimentDir;
cfg.outputDir = fullfile(calibrationDir, "results", ...
    "single_wheel_leg_lqr_qr", "default_qr_check");
cfg.trialsDir = fullfile(cfg.outputDir, "trials");
if ~isfolder(cfg.trialsDir)
    mkdir(cfg.trialsDir);
end

variableNames = {'log_s_qtheta', 'log_s_qdtheta', 'log_s_qx', ...
    'log_s_qdx', 'log_s_rfhx', 'log_s_rmb'};
x = array2table(zeros(1, numel(variableNames)), ...
    'VariableNames', variableNames);

score = run_experiment_objective(x, cfg);
fprintf("Default-Q/R check score: %.6g\n", score);
end
