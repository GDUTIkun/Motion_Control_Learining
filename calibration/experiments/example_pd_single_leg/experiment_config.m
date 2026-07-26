function cfg = experiment_config()
%EXPERIMENT_CONFIG Example PD/computed-torque calibration for single_leg.slx.

experimentDir = fileparts(mfilename("fullpath"));
calibrationDir = fullfile(experimentDir, "..", "..");
repoRoot = fullfile(calibrationDir, "..");
simDir = fullfile(repoRoot, "dynamic", "3D", "simulate", "single_leg");

cfg = struct();
cfg.name = "example_pd_single_leg";
cfg.notes = "Example calibration for the existing fixed-hip two-link leg Simulink model.";

cfg.variables = [
    optimizableVariable("log_bw_hz_h", log10([0.4, 5.0]))
    optimizableVariable("log_bw_hz_k", log10([0.4, 5.0]))
    optimizableVariable("log_zeta_h", log10([0.35, 2.0]))
    optimizableVariable("log_zeta_k", log10([0.35, 2.0]))
    ];

cfg.mapParamsFcn = @map_params;
cfg.runTrialFcn = @run_trial;
cfg.scoreTrialFcn = @score_trial;

cfg.maxObjectiveEvaluations = 5;
cfg.isDeterministic = true;
cfg.useParallel = false;
cfg.outputDir = "";
cfg.penaltyScore = 1e9;

cfg.simDir = char(java.io.File(simDir).getCanonicalPath());
cfg.modelName = "single_leg";
cfg.modelPath = fullfile(cfg.simDir, "single_leg.slx");
cfg.startupScript = fullfile(cfg.simDir, "startup.m");
cfg.stopTime = 4.0;
cfg.simulationMode = "normal";
cfg.fastRestart = false;
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.saveSignals = false;

cfg.scopeBlocks = [
    "single_leg/Scope"
    "single_leg/Scope1"
    "single_leg/Scope2"
    "single_leg/Scope3"
    ];
cfg.scopeSaveNames = [
    "calib_qh"
    "calib_qk"
    "calib_dqh"
    "calib_dqk"
    ];
cfg.signalNames = ["qh", "qk", "dqh", "dqk"];
cfg.controllerSignalIndex = 2;
cfg.referenceSignalIndex = 3;

cfg.scoreWeights = struct();
cfg.scoreWeights.positionRms = 100.0;
cfg.scoreWeights.velocityRms = 10.0;
cfg.scoreWeights.torqueRms = 0.10;
cfg.scoreWeights.saturationPenalty = 100.0;
cfg.scoreWeights.finalPosition = 25.0;
end

function params = map_params(params, cfg) %#ok<INUSD>
params.bandwidthHz = [
    10 ^ params.log_bw_hz_h
    10 ^ params.log_bw_hz_k
    ];
params.zeta = [
    10 ^ params.log_zeta_h
    10 ^ params.log_zeta_k
    ];
end
