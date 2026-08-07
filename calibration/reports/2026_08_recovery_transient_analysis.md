# Recovery Transient Analysis

This report quantifies the post-pulse transient where the body pitch appears to snap back toward zero and then diverges.

Event table:

```text
D:\Workspace\CodeWorkspace\calibration\data\processed\2026_08_recovery_transient_events.csv
```

Figures:

```text
D:\Workspace\CodeWorkspace\calibration\figures\studies\2026_08_lqr_disturbance_response\recovery_transient
```

## Key Event Table

| label | case | theta@pulseEnd deg | dtheta@pulseEnd | thetaZeroTime | dtheta@thetaZero | x@thetaZero | tauRatio@thetaZero | uLqrRatio@thetaZero | peakAbsDtheta | maxTheta deg | finalTheta deg |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| default_1x | pitch_-10deg_pulse_10N | 11.25 | 0.316 | 2.78 | -1.76 | 0.0784 | 0.165 | 0.956 | 384 | 1797.1 | 1577.2 |
| default_1x | pitch_0deg_pulse_10N | 11.08 | 0.321 | 2.78 | -1.84 | 0.0672 | 0.136 | 0.992 | 217 | 1182.0 | -598.4 |
| default_1x | pitch_10deg_pulse_10N | 10.94 | 0.323 | 2.79 | -3.26 | 0.0495 | 0.55 | 1 | 263 | 1682.1 | -1171.9 |
| best_qr_1x | pitch_-10deg_pulse_10N | 11.28 | 0.201 | 2.74 | -1.95 | 0.0592 | 0.0566 | 1 | 319 | 3596.0 | -3596.0 |
| best_qr_1x | pitch_0deg_pulse_10N | 11.22 | 0.206 | 2.74 | -2.04 | 0.0484 | 0.065 | 1 | 81.4 | 396.3 | -88.1 |
| best_qr_1x | pitch_10deg_pulse_10N | 11.08 | 0.21 | 2.74 | -1.9 | 0.031 | 0.0572 | 1 | 48.9 | 50.6 | 9.1 |
| best_qr_2x_all_limits | pitch_-10deg_pulse_10N | 11.28 | 0.201 | 2.74 | -1.94 | 0.0592 | 0.0229 | 0.53 | 1.66e+03 | 34029.1 | -34029.1 |
| best_qr_2x_all_limits | pitch_0deg_pulse_10N | 11.22 | 0.206 | 2.74 | -1.99 | 0.0484 | 0.0232 | 0.554 | 208 | 294.2 | 150.3 |
| best_qr_2x_all_limits | pitch_10deg_pulse_10N | 11.08 | 0.21 | 2.74 | -1.88 | 0.031 | 0.022 | 0.538 | 710 | 4279.6 | 609.3 |

## Interpretation

The failure should not be judged by whether theta briefly returns near zero. The important state is the energy at that return: dtheta, x/dx, and whether the upper and lower controllers are saturated. If theta crosses zero with high angular velocity or saturated commands, the apparent recovery is a pass-through event rather than a settled recovery.

For the current 10 N cases, the extracted event table should be used to decide whether the next controller change should reduce recovery aggressiveness, add damping/rate limits, or change QP priorities.

## Event Ordering

Across the 10 N cases, the dangerous sequence is consistent:

```text
t = 2.50 s:
  pulse ends
  theta ~= 11 deg
  dtheta ~= 0.20 to 0.32 rad/s

t ~= 2.73 to 2.80 s:
  upper LQR command reaches its limit, or approaches the new doubled limit
  theta crosses back through zero

t ~= 2.84 to 2.88 s:
  lower QP torque reaches its limit

t ~= 2.87 to 2.99 s:
  theta exceeds the 15 deg failure threshold

t > 3.5 s:
  x drift becomes large in many cases
```

So the failure is not caused by a large pitch angle at pulse end. The pulse
only puts the system into a recoverable-looking state. The failure is created
by the recovery transient after the pulse.

## Mechanism

The animation observation is consistent with the data:

```text
the body leans and rolls forward
the controller drives theta rapidly back toward zero
theta crosses zero with nonzero angular velocity
upper command and then joint torques saturate
the system leaves the local recovery region
```

For `best_qr_1x`, theta crosses zero at about `2.74 s`, while the upper LQR
command is already saturated. At that instant the lower torque is not yet
saturated, which means the upper recovery command becomes aggressive before
the lower layer fully saturates.

For `best_qr_2x_all_limits`, theta crosses zero with lower normalized command
ratios because the limits were doubled, but the system later reaches the new
doubled limits and diverges harder. This means that simply allowing larger
commands increases injected recovery energy rather than producing a settled
recovery.

## Decision

Stop enlarging all limits.

The next controller change should reduce recovery aggressiveness and add
damping/shape to the post-pulse return:

```text
increase effective dtheta damping relative to theta stiffness
penalize or rate-limit sudden MBy/FHx/FHz changes
keep x/dx from accumulating during pitch recovery
then revisit QP priorities if torque saturation still follows the same order
```

This points to second-round QR/QP design rather than more limit testing.
