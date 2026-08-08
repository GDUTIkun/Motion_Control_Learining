function cfg = experiment_config()
%EXPERIMENT_CONFIG Bayesian tuning for the 2D wheel-leg floating-base LQR Q/R.

experimentDir = fileparts(mfilename("fullpath"));
calibrationDir = fullfile(experimentDir, "..", "..");
repoRoot = fullfile(calibrationDir, "..");
simDir = fullfile(repoRoot, "model", "simulate", "base_with_wheel_leg");

cfg = struct();
cfg.name = "single_wheel_leg_lqr_qr";
cfg.notes = "Tune only upper-layer LQR Q/R multipliers; keep QP, contact, and actuator limits fixed.";

cfg.variables = [
    optimizableVariable("log_s_qtheta", [-0.4, 0.6])
    optimizableVariable("log_s_qdtheta", [-0.4, 0.7])
    optimizableVariable("log_s_qx", [-0.4, 0.6])
    optimizableVariable("log_s_qdx", [-0.4, 0.6])
    optimizableVariable("log_s_rfhx", [-0.4, 0.7])
    optimizableVariable("log_s_rfz", [-0.4, 0.7])
    optimizableVariable("log_s_rmb", [-0.7, 0.4])
    ];

cfg.mapParamsFcn = @map_params;
cfg.runTrialFcn = @run_trial;
cfg.scoreTrialFcn = @score_trial;

cfg.maxObjectiveEvaluations = 20;
cfg.isDeterministic = true;
cfg.useParallel = false;
cfg.acquisitionFunctionName = "expected-improvement-plus";
cfg.verbose = 1;
cfg.plotFcn = [];
cfg.outputDir = "";
cfg.penaltyScore = 1e6;
cfg.bayesoptOptions = {
    "InitialX", defaultInitialX()
    };

cfg.simDir = char(java.io.File(simDir).getCanonicalPath());
cfg.modelName = "source";
cfg.modelPath = fullfile(cfg.simDir, "source.slx");
cfg.startupScript = fullfile(cfg.simDir, "startup.m");
cfg.configureScript = fullfile(cfg.simDir, "configure_discrete_controller_timing.m");
cfg.saveSignals = false;
cfg.maxLoggedPoints = 20000;
cfg.scopeDecimation = 1;

cfg.baseQDiag = [25; 80; 120; 8; 16; 10];
cfg.baseRDiag = [1/80^2; 1/140^2; 1/60^2];

cfg.scoreWeights = struct();
cfg.scoreWeights.thetaRms = 80.0;
cfg.scoreWeights.finalTheta = 150.0;
cfg.scoreWeights.maxAbsTheta = 25.0;
cfg.scoreWeights.dthetaRms = 12.0;
cfg.scoreWeights.finalDtheta = 40.0;
cfg.scoreWeights.finalX = 10.0;
cfg.scoreWeights.maxAbsX = 8.0;
cfg.scoreWeights.tauSaturation = 40.0;
cfg.scoreWeights.uLqrSaturation = 20.0;
cfg.scoreWeights.legPositionRms = 8.0;
cfg.scoreWeights.legVelocityRms = 1.5;
cfg.scoreWeights.unstablePenalty = 1e5;
end

function initialX = defaultInitialX()
names = {'log_s_qtheta', 'log_s_qdtheta', 'log_s_qx', ...
    'log_s_qdx', 'log_s_rfhx', 'log_s_rfz', 'log_s_rmb'};
values = [
    0,    0,    0,    0,    0,    0,     0
    0.1,  0.3,  0,    0,    0.2,  0.3,  -0.3
    0.2,  0.5,  0.1,  0.1,  0.3,  0.4,  -0.4
    ];
initialX = array2table(values, "VariableNames", names);
end

function params = map_params(params, cfg)
params.scale = struct();
params.scale.qtheta = 10 ^ params.log_s_qtheta;
params.scale.qdtheta = 10 ^ params.log_s_qdtheta;
params.scale.qx = 10 ^ params.log_s_qx;
params.scale.qdx = 10 ^ params.log_s_qdx;
params.scale.rfhx = 10 ^ params.log_s_rfhx;
params.scale.rfz = 10 ^ params.log_s_rfz;
params.scale.rmb = 10 ^ params.log_s_rmb;

qDiag = cfg.baseQDiag(:);
rDiag = cfg.baseRDiag(:);

% X = [xB; zB; thetaB; dxB; dzB; dthetaB]
qDiag(1) = qDiag(1) * params.scale.qx;
qDiag(3) = qDiag(3) * params.scale.qtheta;
qDiag(4) = qDiag(4) * params.scale.qdx;
qDiag(6) = qDiag(6) * params.scale.qdtheta;

% u = [FHx; FHz; MBy]
rDiag(1) = rDiag(1) * params.scale.rfhx;
rDiag(2) = rDiag(2) * params.scale.rfz;
rDiag(3) = rDiag(3) * params.scale.rmb;

params.QDiag = qDiag;
params.RDiag = rDiag;
params.Q = diag(qDiag);
params.R = diag(rDiag);
end
