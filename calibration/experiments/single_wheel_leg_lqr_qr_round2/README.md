# Single Wheel-Leg LQR Q/R Round 2

This experiment is the second Bayesian tuning pass for the 2D floating-base
single wheel-leg model.

It keeps the plant, QP, contact parameters, actuator limits, and stage-1 leg
reference fixed. It only tunes upper-layer floating-base LQR `Q/R`
multipliers.

## Why Round 2 Exists

Round 1 result:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337
```

Round 1 improved some 10 N failures, especially `pitch_10deg_pulse_10N`, but
it regressed `pitch_-10deg_pulse_10N`. Full 9-case validation stayed at
`6 / 9` stable.

Recovery transient analysis showed the key failure mechanism:

```text
after the pulse, theta crosses back through zero with too much recovery energy;
upper LQR command saturates first, then QP torque saturates, then theta
diverges.
```

Round 2 therefore uses symmetric 10 N training cases and adds recovery-aware
score terms.

## Training Cases

Defined in:

```text
qr_training_cases.m
```

Cases:

```text
pitch_0deg_pulse_0N
pitch_-10deg_pulse_5N
pitch_10deg_pulse_5N
pitch_-10deg_pulse_10N
pitch_0deg_pulse_10N
pitch_10deg_pulse_10N
```

The 0 N and 5 N cases are regression guardrails. The 10 N cases are pressure
tests, not guaranteed pass requirements.

## Search Variables

Variables are log10 multipliers:

```text
log_s_qtheta
log_s_qdtheta
log_s_qx
log_s_qdx
log_s_rfhx
log_s_rfz
log_s_rmb
```

Round 2 expands the directions that Round 1 pushed against:

```text
R_Fz upper range is expanded
R_MBy lower range is expanded
Q_dtheta upper range is expanded
```

Round 1 best parameters are included as an initial point.

## Score Additions

Besides the original state, saturation, and leg-tracking metrics, Round 2
adds:

```text
postPulsePeakAbsDtheta
thetaZeroDtheta
recoveryTauSaturationRatio
recoveryULqrSaturationRatio
```

These terms penalize the failure pattern where pitch appears to recover but
crosses zero with too much angular velocity and saturated commands.

## Run Order

Smoke test:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr_round2")
score = run_smoke_test;
```

Default QR check on the round-2 training set:

```matlab
score = run_default_qr_check;
```

Full Bayesian optimization:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration")
out = run_bayes_calibration("single_wheel_leg_lqr_qr_round2");
```

Results are saved to:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\<timestamp>
```

After optimization, validate the best result on the full 9-case stage-A set
before changing any default controller parameters.
