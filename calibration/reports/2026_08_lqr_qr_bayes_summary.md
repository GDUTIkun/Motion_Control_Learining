# 2026-08 LQR Q/R Bayesian Optimization Summary

## Purpose

Tune only the upper-layer floating-base LQR `Q/R` multipliers for the current
2D single wheel-leg learning model.

Fixed during this run:

```text
QP parameters
contact parameters
actuator limits
model structure
stage-1 floating-base leg reference
```

## Result Directory

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337
```

Main files:

```text
summary.csv
best_params.mat
bayes_results.mat
trials/trial_*.mat
objective_history.png
```

## Training Cases

The optimization used 5 representative cases:

```text
pitch_0deg_pulse_0N
pitch_10deg_pulse_5N
pitch_-10deg_pulse_5N
pitch_0deg_pulse_10N
pitch_10deg_pulse_10N
```

Each case used:

```text
stopTime = 5 s
pulseWindow = [2.0, 2.5] s
```

## Best Trial

Best trial:

```text
trial_0014
bestObjective = 54903.9256
```

Default trial:

```text
trial_0001
objective = 57503.5084
```

Improvement relative to default:

```text
about 4.5%
```

This is a modest improvement, not a solved-controller result.

## Best Multipliers

Best log multipliers:

```text
log_s_qtheta =  0.005343
log_s_qdtheta = 0.057054
log_s_qx =     -0.382841
log_s_qdx =     0.084591
log_s_rfhx =   -0.165352
log_s_rfz =     0.691867
log_s_rmb =    -0.682140
```

Equivalent scale multipliers:

```text
Q_theta  ~= 1.01
Q_dtheta ~= 1.14
Q_x      ~= 0.41
Q_dx     ~= 1.21
R_FHx    ~= 0.68
R_Fz     ~= 4.92
R_MBy    ~= 0.21
```

Interpretation:

```text
The best candidate makes vertical force much more expensive and pitch moment
much cheaper. It changes theta and dtheta state weights only mildly.
```

This agrees with the earlier pass/fail mechanism analysis: the dangerous
failure mode is the post-pulse recovery transient where force commands and
joint torques hit limits, not simply a lack of small-angle theta gain.

## Per-Case Comparison

Default trial:

```text
stable cases: 3 / 5
0 N and 5 N training cases passed
10 N training cases failed
```

Best trial:

```text
stable cases: 3 / 5
0 N and 5 N training cases still passed
10 N training cases still failed
```

The best trial did not increase the number of passing training cases.

Important changes:

```text
pitch_0deg_pulse_10N:
  theta RMS improved from about 91.9 deg to 24.7 deg
  final theta improved from about -598 deg to -88.1 deg
  still failed

pitch_10deg_pulse_10N:
  theta RMS improved from about 161.9 deg to 5.47 deg
  final theta improved from about -1172 deg to 9.08 deg
  max |theta| improved from about 1682 deg to 50.6 deg
  still failed
```

Tradeoff:

```text
The no-disturbance and 5 N cases remain stable, but x drift is slightly larger
in some cases. This is acceptable for a candidate, but it must be checked on
the full 9-case acceptance set before changing default controller parameters.
```

## Ranking Pattern

Top candidates commonly used:

```text
R_Fz multiplier: about 3 to 5
R_MBy multiplier: about 0.2 to 0.45
Q_theta multiplier: near 1
```

This pattern is stronger than any single trial result. It suggests the next
search should keep exploring the wrench-allocation side of LQR, especially the
relative cost of `FHz` and `MBy`.

## Conclusion

The Bayesian run found a plausible direction but not a final setting.

Current conclusion:

```text
The controller can be improved by LQR Q/R tuning, especially by reducing
overuse of Fz and allowing more MBy. However, the current search did not make
the 10 N pulse cases pass, and all top-ranked candidates still hit saturation.
```

Do not write the best parameters into `startup.m` yet.

## Recommended Next Step

1. Validate `trial_0014` on the full 9-case stage-A acceptance set.

```text
theta0 = [-10, 0, 10] deg
pulse = [0, 5, 10] N
```

Compare against baseline:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_195701
```

Check:

```text
stable count
10 N failure severity
final theta and final dtheta
x drift
tau saturation timing
uLqr saturation timing
5 N regression
```

2. If full validation still has only 6 / 9 passes, run a second Bayesian
   optimization round with the first-run result as evidence.

Suggested changes for round 2:

```text
expand R_Fz upper range beyond 10^0.7
allow R_MBy lower range below 10^-0.7
add more weight to final theta, final dtheta, and x drift for 10 N cases
keep the 0 N and 5 N cases as regression guardrails
```

3. If saturation remains the first hard limit after round 2, switch from pure
   upper-layer QR tuning to QP priority tuning.

Candidate QP direction:

```text
make pitch moment tracking more important during recovery
inspect whether tau saturation is caused by infeasible MBy, q tracking, or
contact-force allocation
```

## Full 9-Case Validation

Validation result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_220040
```

Source best-QR result:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337
```

Result:

```text
default QR: 6 / 9 stable
best QR:    6 / 9 stable
```

The best QR did not expand the pass/fail envelope. The same pattern remains:

```text
0 N cases passed
5 N cases passed
10 N cases failed
```

However, the failure severity changed strongly and asymmetrically.

10 N comparison:

```text
pitch_-10deg_pulse_10N:
  default max |theta| ~= 1797 deg, final theta ~= 1577 deg
  best    max |theta| ~= 3596 deg, final theta ~= -3596 deg
  verdict: worse

pitch_0deg_pulse_10N:
  default max |theta| ~= 1182 deg, final theta ~= -598 deg
  best    max |theta| ~= 396 deg,  final theta ~= -88 deg
  verdict: improved but still failed

pitch_10deg_pulse_10N:
  default max |theta| ~= 1682 deg, final theta ~= -1172 deg
  best    max |theta| ~= 50.6 deg, final theta ~= 9.08 deg
  verdict: much improved but still failed
```

Regression note:

```text
All 0 N and 5 N cases still pass, but final x drift is generally larger with
the best QR candidate. This is acceptable as an optimization tradeoff only if
the 10 N recovery improves enough, which it currently does not.
```

Important lesson:

```text
The first Bayesian training set included pitch_10deg_pulse_10N but did not
include pitch_-10deg_pulse_10N. The selected QR therefore overfit the positive
pitch recovery direction and regressed the negative-pitch 10 N case.
```

Updated next step:

```text
Run a second Bayesian optimization round, but make the training set symmetric:
include pitch_-10deg_pulse_10N, pitch_0deg_pulse_10N, and
pitch_10deg_pulse_10N.
```

Suggested round-2 changes:

```text
1. Add pitch_-10deg_pulse_10N to qr_training_cases.m.
2. Keep the 0 N and 5 N cases as regression guardrails.
3. Expand R_Fz upper range beyond 10^0.7.
4. Expand R_MBy lower range below 10^-0.7.
5. Increase the scoring penalty for x drift and final dtheta in failed 10 N
   cases, because the current best candidate still relies on large recovery
   transients and saturation.
```

## 2x Limit Diagnostic

Result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_221550
```

Configuration:

```text
best QR from 20260806_203337
tauScale    = 2
forceScale  = 2
momentScale = 2
```

Result:

```text
stable count stayed at 6 / 9
0 N and 5 N cases still passed
all 10 N cases still failed
```

Important 10 N comparison against best QR with original limits:

```text
pitch_-10deg_pulse_10N:
  1x max |theta| ~= 3596 deg
  2x max |theta| ~= 34029 deg
  worse

pitch_0deg_pulse_10N:
  1x max |theta| ~= 396 deg
  2x max |theta| ~= 294 deg
  max angle improved slightly, but final theta/dtheta still failed badly

pitch_10deg_pulse_10N:
  1x max |theta| ~= 50.6 deg
  2x max |theta| ~= 4280 deg
  much worse
```

Both upper LQR command and lower QP torque hit the new doubled limits:

```text
1x max tau  = [160, 160, 45]
2x max tau  = [320, 320, 90]
1x max uLqr = [140, 140, 160]
2x max uLqr = [280, 280, 320]
```

Interpretation:

```text
Increasing all limits does not simply reveal hidden robustness. It lets the
current recovery transient inject more aggressive commands, and the system
still saturates at the new limits. This points to controller/priority/trajectory
allocation, not just insufficient numeric limits.
```

Important caveat:

```text
This test scaled both upper-layer LQR wrench limits and lower-layer QP torque
limits. It is therefore not a clean actuator-only test.
```

Recommended isolation test:

```matlab
limits = struct("tauScale", 2.0, "forceScale", 1.0, "momentScale", 1.0);
out = validate_limit_scale_9cases(limits, ...
    "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337");
```

This keeps the upper-layer LQR command envelope unchanged and tests whether
the lower QP/actuator limit alone is the bottleneck.

## Round-2 Experiment Prepared

Experiment directory:

```text
D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr_round2
```

Purpose:

```text
keep model/QP/contact/limits fixed
tune only upper-layer LQR Q/R
make 10 N training symmetric in initial pitch
penalize high-energy theta recovery through zero
```

Training cases:

```text
pitch_0deg_pulse_0N
pitch_-10deg_pulse_5N
pitch_10deg_pulse_5N
pitch_-10deg_pulse_10N
pitch_0deg_pulse_10N
pitch_10deg_pulse_10N
```

New score terms:

```text
postPulsePeakAbsDtheta
thetaZeroDtheta
recoveryTauSaturationRatio
recoveryULqrSaturationRatio
```

Search changes:

```text
expand Q_dtheta upper range
expand R_Fz upper range
expand R_MBy lower range
seed with default QR and round-1 best QR
```

Run order:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr_round2")
score = run_smoke_test;
score = run_default_qr_check;

cd("D:\Workspace\CodeWorkspace\calibration")
out = run_bayes_calibration("single_wheel_leg_lqr_qr_round2");
```

Do not skip full 9-case validation after this optimization.
