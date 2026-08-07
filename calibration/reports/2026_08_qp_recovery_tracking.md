# QP Recovery Tracking Analysis

This report checks whether lower-layer QP hip torque tracks the upper-layer pitch moment command during the post-pulse recovery window.

Tracking table:

```text
D:\Workspace\CodeWorkspace\calibration\data\processed\2026_08_qp_recovery_tracking.csv
```

Figures:

```text
D:\Workspace\CodeWorkspace\calibration\figures\studies\2026_08_lqr_disturbance_response\qp_recovery_tracking
```

## Key Metrics

| label | case | MBy sat frac | tau_h sat frac | tau_h RMS err | tau_h max err | q RMS | dq RMS | theta zero t | tau err @ zero | max theta deg | final theta deg |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| default_1x | pitch_-10deg_pulse_10N | 0.00 | 0.00 | 0.25 | 1.27 | 1.13 | 32.6 | 2.78 | -0.05 | 1797.1 | 1577.2 |
| default_1x | pitch_0deg_pulse_10N | 0.01 | 0.01 | 0.25 | 1.35 | 1.05 | 32.7 | 2.78 | -0.05 | 1182.0 | -598.4 |
| default_1x | pitch_10deg_pulse_10N | 0.02 | 0.02 | 0.33 | 1.31 | 1.55 | 45.9 | 2.79 | 0.03 | 1682.1 | -1171.9 |
| round1_best | pitch_-10deg_pulse_10N | 0.01 | 0.01 | 0.26 | 1.62 | 1.2 | 35.2 | 2.74 | -0.03 | 3596.0 | -3596.0 |
| round1_best | pitch_0deg_pulse_10N | 0.02 | 0.02 | 0.32 | 2.11 | 1.59 | 38.6 | 2.74 | -0.02 | 396.3 | -88.1 |
| round1_best | pitch_10deg_pulse_10N | 0.01 | 0.01 | 0.25 | 1.17 | 1.06 | 28.4 | 2.74 | -0.02 | 50.6 | 9.1 |
| round2_best | pitch_-10deg_pulse_10N | 0.00 | 0.00 | 0.02 | 0.02 | 0.341 | 0.504 | 2.69 | -0.02 | 66.5 | -16.1 |
| round2_best | pitch_0deg_pulse_10N | 0.00 | 0.00 | 0.02 | 0.02 | 0.362 | 0.621 | 2.69 | -0.02 | 65.1 | -44.3 |
| round2_best | pitch_10deg_pulse_10N | 0.01 | 0.01 | 0.30 | 2.17 | 1.49 | 36.8 | 2.67 | -0.02 | 652.1 | -196.0 |

## Interpretation Guide

If `MBy sat frac` is high before `tau_h sat frac`, the upper layer is asking for saturated pitch recovery before the lower layer hits its own hip limit. If `tau_h RMS err` is large while `MBy sat frac` is low, the QP is sacrificing moment tracking to satisfy dynamics, contact, or joint tracking. If both are high, the recovery request is outside the combined upper/lower authority envelope.

Use this report to decide whether the next change should be LQR wrench shaping or QP priority tuning.

## Result Interpretation

For the round-2 best validation result:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260807_182335
```

The `-10 deg / 10 N` and `0 deg / 10 N` cases do not show a meaningful
`MBy_des -> tau_h` tracking problem during the first 0.5 s recovery window:

```text
recoveryRmsTauHError ~= 0.015 N*m
recoveryMaxAbsTauHError ~= 0.023 N*m
MBy saturation fraction = 0
tau_h saturation fraction = 0
```

So for these two cases, the lower QP is not the main bottleneck for pitch
moment tracking.

However, their upper-layer command still saturates:

```text
pitch_-10deg_pulse_10N:
  max |FHx_ext| ~= 140 N
  max |FHz_ext| ~= 75 N
  max |MBy_des| ~= 3.6 N*m
  FHx saturation fraction ~= 0.61

pitch_0deg_pulse_10N:
  max |FHx_ext| ~= 140 N
  max |FHz_ext| ~= 75 N
  max |MBy_des| ~= 3.8 N*m
  FHx saturation fraction ~= 0.62
```

This means the remaining failure in these cases is dominated by upper-layer
force recovery, especially horizontal force saturation, not by failed hip
moment realization.

The `+10 deg / 10 N` case is different:

```text
max |FHx_ext| ~= 140 N
max |FHz_ext| ~= 140 N
max |MBy_des| ~= 160 N*m
recoveryRmsTauHError ~= 0.299 N*m
recoveryMaxAbsTauHError ~= 2.17 N*m
recoveryDqErrorRms ~= 36.8 rad/s
```

This case still drives all upper channels to their limits and also shows much
larger leg tracking error. It is the main remaining bad pressure case.

## Decision

The next controller experiment should prioritize upper-layer wrench shaping:

```text
rate-limit or filter FHx_ext/FHz_ext/MBy_des
start with FHx_ext because round-2 -10/0 deg 10 N failures are FHx dominated
keep MBy tracking diagnostics to ensure the QP does not become the new bottleneck
```

QP priority tuning is still relevant, but mainly for the `+10 deg / 10 N`
boundary case where all upper channels saturate and leg tracking error becomes
large.
