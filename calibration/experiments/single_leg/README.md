# single_leg computed-torque calibration

This is the production calibration experiment for:

```text
dynamic/3D/simulate/single_leg/single_leg.slx
```

Run from the repository root:

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("single_leg");
```

Results are written to:

```text
calibration/results/single_leg/<timestamp>/
```

## Tuned Parameters

Bayesian optimization searches in log space:

```text
log_bw_hz_h
log_bw_hz_k
log_zeta_h
log_zeta_k
```

`experiment_config.m` maps those to computed-torque gains:

```matlab
ctrl.wn = 2*pi*ctrl.bandwidthHz;
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);
```

The controller being calibrated is:

```matlab
v = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);
tau = M * v + C + G;
```

## Batch Simulation Defaults

The production experiment is configured for automatic tuning rather than visual inspection:

```matlab
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.disableGraphics = true;
cfg.saveSignals = false;
cfg.maxLoggedPoints = 20000;
cfg.scopeDecimation = 1;
cfg.fastRestart = true;
cfg.plotFcn = [];
```

The trial runner disables global Simulink logging, SDI recording, state/output/time saves, and Scope auto-opening. Scope data is still captured in bounded form because the current model does not yet expose scalar cost outputs directly.

## Objective

`score_trial.m` minimizes a weighted scalar cost:

```text
position RMS error
velocity RMS error
final position error
torque RMS ratio
torque saturation excess
```

Weights live in `experiment_config.m` under `cfg.scoreWeights`.

## Parameters To Confirm Before Long Runs

Before running a long calibration, confirm these defaults:

```matlab
cfg.stopTime = 4.0;
cfg.maxObjectiveEvaluations = 30;
cfg.variables = [
    optimizableVariable("log_bw_hz_h", log10([0.4, 5.0]))
    optimizableVariable("log_bw_hz_k", log10([0.4, 5.0]))
    optimizableVariable("log_zeta_h", log10([0.35, 2.0]))
    optimizableVariable("log_zeta_k", log10([0.35, 2.0]))
    ];
```

For quick smoke tests, temporarily set:

```matlab
cfg.maxObjectiveEvaluations = 3;
cfg.stopTime = 0.5;
```

