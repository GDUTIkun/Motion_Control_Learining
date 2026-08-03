# single_wheel_leg QP calibration

This experiment calibrates the QP/WBC controller in:

```text
dynamic/3D/simulate/single_wheel_leg/source.slx
```

Run from the repository root:

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("single_wheel_leg_qp");
```

Results are written to:

```text
calibration/results/single_wheel_leg_qp/<timestamp>/
```

## Tuned Parameters

Bayesian optimization searches these log-space variables:

```text
log_bw_hz_h
log_bw_hz_k
log_bw_hz_w
log_zeta
log_constraint_gain
log_qp_wtau
log_qp_wfc
```

They map to the QP controller as:

```matlab
ctrl.wn = 2*pi*params.bandwidthHz;
ctrl.zeta = params.zeta;
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);
ctrl.constraintVelocityGain = params.constraintVelocityGain;
ctrl.qpWtau = params.qpWtau;
ctrl.qpWFc = params.qpWFc;
```

`hip` position-PD parameters are fixed in this first calibration pass. Tune
them separately after the QP layer is stable.

## Memory Defaults

The trial runner follows the production `single_leg` pattern:

```matlab
cfg.useParallel = false;
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.disableGraphics = true;
cfg.saveSignals = false;
cfg.maxLoggedPoints = 15000;
cfg.scopeDecimation = 2;
cfg.fastRestart = true;
cfg.plotFcn = [];
```

Only six scopes are saved: qh/qk/qw and dqh/dqk/dqw actual/reference. The
runner does not call `save_system`; it temporarily sets the top-level
controller block to `controller_qp` and restores it after each trial.

## Smoke Test

Before a long run, reduce:

```matlab
cfg.maxObjectiveEvaluations = 3;
cfg.stopTime = 1.0;
```

For a full pass, use:

```matlab
cfg.maxObjectiveEvaluations = 30;
cfg.stopTime = 14.0;
```
