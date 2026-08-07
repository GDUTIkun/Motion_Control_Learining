# 2026-08 LQR Q/R Round-2 Summary

## Result Directory

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811
```

Default round-2 check:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\default_qr_check
```

## Purpose

Round 2 tested whether upper-layer LQR `Q/R` tuning could reduce the
post-pulse recovery transient identified in the recovery analysis.

Fixed:

```text
plant
QP parameters
contact parameters
actuator limits
stage-1 leg reference
```

Tuned:

```text
upper-layer floating-base LQR Q/R multipliers only
```

## Training Cases

```text
pitch_0deg_pulse_0N
pitch_-10deg_pulse_5N
pitch_10deg_pulse_5N
pitch_-10deg_pulse_10N
pitch_0deg_pulse_10N
pitch_10deg_pulse_10N
```

The 10 N cases were treated as pressure tests. The 0 N and 5 N cases were
regression guardrails.

## Best Trial

Best trial:

```text
trial_0005
bestObjective = 69144.9623
```

Default round-2 objective:

```text
trial_0001 objective = 77830.6762
```

Improvement:

```text
about 11.2%
```

Stable case count:

```text
default: 3 / 6
best:    3 / 6
```

No trial in the 24-evaluation run exceeded `3 / 6` stable cases.

## Best Multipliers

Best log multipliers:

```text
log_s_qtheta =  0.117272
log_s_qdtheta = -0.024773
log_s_qx =     -0.217833
log_s_qdx =     0.530689
log_s_rfhx =   -0.142822
log_s_rfz =     1.166755
log_s_rmb =    -0.217722
```

Equivalent scale multipliers:

```text
Q_theta  ~= 1.31
Q_dtheta ~= 0.94
Q_x      ~= 0.61
Q_dx     ~= 3.39
R_FHx    ~= 0.72
R_Fz     ~= 14.68
R_MBy    ~= 0.61
```

Interpretation:

```text
Round 2 strongly discourages vertical-force use and increases horizontal
velocity penalty. It no longer pushes MBy cost extremely low.
```

This differs from round 1 and matches the recovery-transient diagnosis: the
optimizer is trying to reduce aggressive post-pulse recovery and x/dx growth,
not simply make pitch moment cheaper.

## Per-Case Result

Best trial:

```text
0 N case: passed
5 N cases: both passed
10 N cases: all failed
```

10 N comparison against default QR on the round-2 training set:

```text
pitch_-10deg_pulse_10N:
  default max |theta| ~= 1797 deg, final theta ~= 1577 deg
  best    max |theta| ~= 66.5 deg, final theta ~= -16.1 deg
  improved strongly, still failed

pitch_0deg_pulse_10N:
  default max |theta| ~= 1182 deg, final theta ~= -598 deg
  best    max |theta| ~= 65.1 deg, final theta ~= -44.3 deg
  improved strongly, still failed

pitch_10deg_pulse_10N:
  default max |theta| ~= 1682 deg, final theta ~= -1172 deg
  best    max |theta| ~= 652 deg, final theta ~= -196 deg
  improved relative to default, but worse than the round-1 best for this
  specific case
```

Recovery metrics:

```text
postPulsePeakAbsDtheta dropped strongly for -10 deg and 0 deg 10 N cases.
The +10 deg 10 N case remains the main bad pressure case.
All 10 N cases still hit tau and uLqr saturation.
```

## Conclusion

Round 2 is useful but not sufficient.

It reduced the catastrophic asymmetry from round 1 and made `-10 deg` and
`0 deg` 10 N failures much less severe. However, no tested QR setting made any
10 N pressure case pass, and every top candidate still saturated.

Working conclusion:

```text
pure upper-layer QR tuning is improving failure shape, but it is not enough to
turn 10 N into a stable working condition under the current architecture and
limits.
```

## Next Step

1. Validate `trial_0005` on the full 9-case stage-A set:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response")
out = validate_best_qr_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

2. If full validation remains `6 / 9`, do not continue blind QR expansion.

Move to recovery-shaping or QP-priority changes:

```text
add slew/rate limiting or filtering on LQR wrench
increase effective damping during post-pulse recovery
penalize rapid MBy/FHx/FHz changes
inspect QP moment tracking versus q/ddq tracking priority during recovery
```

3. Keep 10 N as a pressure test unless the controller architecture is changed.

For the current learning model, the demonstrated normal envelope remains:

```text
theta0 within +/-10 deg
5 N pulse for 0.5 s passes
10 N pulse exposes recovery-transient failure
```

## Full 9-Case Validation

Validation result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260807_182335
```

Source best-QR result:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811
```

Result:

```text
default QR:      6 / 9 stable
round-1 best QR: 6 / 9 stable
round-2 best QR: 6 / 9 stable
```

The pass/fail envelope did not expand. The same cases pass and fail:

```text
0 N cases passed
5 N cases passed
10 N cases failed
```

Round-2 0 N / 5 N guardrail status:

```text
max final |theta| ~= 0.120 deg
max tauSaturationRatio ~= 0.078
max |finalX| ~= 0.083 m
max legVelocityRms ~= 0.264 rad/s
```

Compared with the default QR, the 0 N / 5 N cases still pass but have slightly
larger x drift, torque usage, and leg velocity RMS.

10 N comparison:

```text
pitch_-10deg_pulse_10N:
  default final theta ~= 1577 deg
  round-1 final theta ~= -3596 deg
  round-2 final theta ~= -16.1 deg
  round-2 is much better, but still failed

pitch_0deg_pulse_10N:
  default final theta ~= -598 deg
  round-1 final theta ~= -88.1 deg
  round-2 final theta ~= -44.3 deg
  round-2 is best so far, but still failed

pitch_10deg_pulse_10N:
  default final theta ~= -1172 deg
  round-1 final theta ~= 9.08 deg
  round-2 final theta ~= -196 deg
  round-1 was best for this specific positive-pitch case
```

Interpretation:

```text
Round 2 improved symmetry and reduced two catastrophic 10 N failures, but it
did not make the 10 N pressure cases stable. Pure upper-layer QR tuning has
now produced useful shaping evidence but has not changed the working envelope.
```

Decision:

```text
Do not continue blind QR expansion.
Do not write round-2 best Q/R into startup.m as final default.
Move next to recovery shaping or QP-priority analysis.
```

Recommended next technical target:

```text
Add a controlled recovery-shaping experiment:
  1. keep 5 N guardrail cases
  2. use 10 N cases only as pressure tests
  3. test LQR wrench slew/rate limiting or first-order filtering
  4. inspect whether QP moment tracking is being sacrificed during recovery
```
