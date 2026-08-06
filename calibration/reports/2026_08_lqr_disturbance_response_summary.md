# 2026-08 LQR Disturbance Response Summary

## Question

For the current 2D floating-base single wheel-leg learning model, what small
disturbance envelope is acceptable without changing the controller?

## Data

Result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_195701
```

Study cases:

```text
theta0 = [-10, 0, 10] deg
pulseAmplitude = [0, 5, 10] N
pulseWindow = [2.0, 2.5] s
stopTime = 5 s
```

## Criteria

```text
max |theta| <= 15 deg
final |theta| <= 2 deg
final |dtheta| <= 0.1 rad/s
tauSaturationRatio <= 0.95
max |x| <= 0.5 m
```

## Results

```text
Total cases: 9
Stable: 6
Failed: 3
```

Stable cases:

```text
All 0 N pulse cases passed.
All 5 N pulse cases passed.
```

Failed cases:

```text
All 10 N pulse cases failed.
```

For the stable cases:

```text
max final |theta|: about 0.092 deg
final x range: about -0.054 m to -0.032 m
max tauSaturationRatio: about 0.069
max legPositionRms: about 0.050 rad
max legVelocityRms: about 0.214 rad/s
```

For 5 N pulse cases:

```text
pulseMaxAbsThetaDeg: about 5.08 to 5.36 deg
settlingTimeTheta: about 4.14 to 4.21 s
tauSaturationRatio: about 0.037 to 0.069
```

For 10 N pulse cases:

```text
failureReason:
  max theta too large
  final theta not settled
  final dtheta not settled
  tau near saturation
  x drift too large

tauSaturationRatio: 1.0
final x drift: about -3.66 m to -5.76 m
```

## Conclusion

Current stage-A working envelope:

```text
theta0 within +/-10 deg, dtheta0 = 0,
with up to a 5 N horizontal pulse lasting 0.5 s at t = 2.0 s.
```

Boundary from this coarse scan:

```text
10 N horizontal pulse for 0.5 s is outside the current controller's working
range for all tested initial pitch values.
```

Main failure mode at 10 N:

```text
large pitch excursion and x drift, followed by torque saturation.
```

## Next Step

Decision:

```text
Stop the stage-A sweep here.
```

No finer pulse-amplitude scan is planned for now. The learning-model target is
met: the current controller has a clear coarse envelope and an interpretable
failure boundary.

Recommended next work:

1. Explain the mechanism of the 5 N pass / 10 N fail split.

```text
Use representative cases:
  pitch_10deg_pulse_5N
  pitch_10deg_pulse_10N

Compare:
  theta and dtheta
  x drift
  FHx/FHz/MBy_des
  tau and saturation timing
  q/qRef behavior
  contact/rolling residuals if needed
```

2. Tune LQR/QP parameters with direction from the representative pass/fail
   cases.

```text
The model architecture is considered basically valid for the current learning
goal. The next step is not structural redesign; it is directed parameter
tuning.

Likely tuning families:
  LQR Q/R weights for x, z, theta, dx, dz, dtheta and wrench allocation
  QP weights for qdd tracking, tau/MBy tracking, contact force regularization
  controller limits only if the analysis shows saturation is a parameter-level
  bottleneck rather than an unrealistic target
```

3. Re-run the same 9-case robustness acceptance set after each meaningful
   parameter batch.

```text
Use the current result directory as the baseline:
  calibration/results/studies/2026_08_lqr_disturbance_response/20260806_195701

Compare:
  stable count
  failureReason
  final theta/dtheta
  x drift
  tau saturation
  leg tracking RMS
```

Later work, after stationary disturbance robustness is acceptable:

```text
Add horizontal base motion tests.
Add vertical base motion tests.
```
