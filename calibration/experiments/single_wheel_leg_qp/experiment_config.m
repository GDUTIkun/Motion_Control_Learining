function cfg = experiment_config()
%EXPERIMENT_CONFIG Bayesian calibration for source.slx with controller_qp.

experimentDir = fileparts(mfilename("fullpath"));
calibrationDir = fullfile(experimentDir, "..", "..");
repoRoot = fullfile(calibrationDir, "..");
simDir = fullfile(repoRoot, "model", "simulate", "single_wheel_leg");

cfg = struct();
cfg.name = "single_wheel_leg_qp";
cfg.notes = "Memory-conscious calibration for the QP/WBC single wheel-leg controller.";

cfg.variables = [
    optimizableVariable("log_bw_hz_h", log10([0.25, 3.0]))
    optimizableVariable("log_bw_hz_k", log10([0.25, 3.0]))
    optimizableVariable("log_bw_hz_w", log10([0.30, 4.0]))
    optimizableVariable("log_zeta", log10([0.60, 2.0]))
    optimizableVariable("log_constraint_gain", log10([2.0, 50.0]))
    optimizableVariable("log_qp_wtau", log10([1e-7, 1e-3]))
    optimizableVariable("log_qp_wfc", log10([1e-7, 1e-3]))
    ];

cfg.mapParamsFcn = @map_params;
cfg.runTrialFcn = @run_trial;
cfg.scoreTrialFcn = @score_trial;

cfg.maxObjectiveEvaluations = 200;
cfg.isDeterministic = true;
cfg.useParallel = false;
cfg.acquisitionFunctionName = "expected-improvement-plus";
cfg.verbose = 1;
cfg.plotFcn = [];
cfg.outputDir = "";
cfg.penaltyScore = 1e9;

cfg.simDir = char(java.io.File(simDir).getCanonicalPath());
cfg.modelName = "source";
cfg.modelPath = fullfile(cfg.simDir, "source.slx");
cfg.startupScript = fullfile(cfg.simDir, "startup.m");
cfg.stopTime = 14.0;
cfg.simulationMode = "normal";
cfg.fastRestart = true;
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.disableGraphics = true;
cfg.saveSignals = false;
cfg.maxLoggedPoints = 15000;
cfg.scopeDecimation = 2;

% The model's reference block outputs [q; dq; ddq]. Scopes 2-7 compare
% reference and actual for qh/qk/qw/dqh/dqk/dqw respectively.
cfg.scopeBlocks = [
    "source/PD_only/Scope2"
    "source/PD_only/Scope3"
    "source/PD_only/Scope4"
    "source/PD_only/Scope5"
    "source/PD_only/Scope6"
    "source/PD_only/Scope7"
    ];
cfg.scopeSaveNames = [
    "calib_qh"
    "calib_qk"
    "calib_qw"
    "calib_dqh"
    "calib_dqk"
    "calib_dqw"
    ];
cfg.signalNames = ["qh", "qk", "qw", "dqh", "dqk", "dqw"];
cfg.actualSignalIndex = 2;
cfg.referenceSignalIndex = 1;

cfg.scoreWeights = struct();
cfg.scoreWeights.legPositionRms = 90.0;
cfg.scoreWeights.wheelPositionRms = 8.0;
cfg.scoreWeights.legVelocityRms = 8.0;
cfg.scoreWeights.wheelVelocityRms = 0.8;
cfg.scoreWeights.finalLegPosition = 25.0;
cfg.scoreWeights.finalWheelPosition = 2.0;
cfg.scoreWeights.bandwidthPenalty = 0.05;
end

function params = map_params(params, cfg) %#ok<INUSD>
params.bandwidthHz = [
    10 ^ params.log_bw_hz_h
    10 ^ params.log_bw_hz_k
    10 ^ params.log_bw_hz_w
    ];
params.zeta = (10 ^ params.log_zeta) * ones(3, 1);
params.constraintVelocityGain = 10 ^ params.log_constraint_gain;
params.qpWtau = (10 ^ params.log_qp_wtau) * ones(3, 1);
params.qpWFc = (10 ^ params.log_qp_wfc) * ones(2, 1);
end
