# Single Wheel Leg Floating-Base LQR + QP Engineering Log

Last updated: 2026-08-06

## Project Goal

Build a 2D Simulink learning model for a floating base plus single wheel-leg
inverted-pendulum system. The purpose is not final control performance yet.
The immediate goal is to make the layered control stack runnable and
diagnosable, so poor behavior can be attributed to one of:

- upper floating-base LQR command generation
- lower QP inverse dynamics / contact constraints
- reference geometry mismatch
- sampling / ZOH / Simscape contact timing

Overall controller objective:

```text
The floating base has 3 planar degrees of freedom: x, z, theta.
x and z motion are expected to be directly controllable.
theta is coupled through the wheel-leg/contact dynamics, so the practical
goal is disturbance recovery and robustness around the standing equilibrium,
not arbitrary direct theta tracking.
```

Current belief:

```text
The model architecture is basically valid for the learning goal.
Reasonable disturbance recovery and robustness should be achievable through
LQR/QP parameter tuning before larger structural changes are considered.
```

Development stages:

```text
1. Stationary standing disturbance recovery and robustness.
2. Horizontal base motion tests.
3. Vertical base motion tests.
4. Broader combined motion/disturbance tests if needed.
```

Model path:

```matlab
D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg\source.slx
```

## Current Control Architecture

Plant and controller chain:

```text
Simscape continuous plant
  -> measured floating-base and joint state
  -> sampled controller input
  -> floating_base_lqr_command
  -> controller_qp
  -> ZOH torque command
  -> Simscape joint torques
```

Upper layer:

```text
[t; xB; zB; thetaB; dxB; dzB; dthetaB]
  -> floating_base_lqr_command
  -> [FHx_ext; FHz_ext; MBy_des]
```

Lower layer:

```text
[t; xB; zB; thetaB; dxB; dzB; dthetaB;
 qh; qk; qw; dqh; dqk; dqw;
 FHx_ext; FHz_ext; MBy_des]
  -> controller_qp
  -> [tau_h; tau_k; tau_w]
```

Coordinate conventions:

- `thetaB > 0` is counterclockwise.
- 2D body rotation uses:

```matlab
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];
```

- Pitch moment convention:

```matlab
tau = r_x * F_z - r_z * F_x + MBy;
```

## Important Files

- `startup.m`: main parameter entry point.
- `source.slx`: Simulink model.
- `floating_base_state_space.m`: simplified floating-base linear wrench model.
- `floating_base_lqr_design.m`: continuous/discrete LQR design.
- `floating_base_lqr_command.m`: Simulink upper-layer command interface.
- `floating_base_lqr_wrench.m`: LQR wrench law, currently with pitch integral compensation.
- `controller_qp.m`: Simulink lower-layer QP interface.
- `controller_qp_core.m`: QP inverse dynamics core.
- `floating_base_leg_reference.m`: stage-1 floating-base-consistent leg reference.
- `wheel_leg_reference.m`: legacy fixed-hip / hip-reference leg trajectory generator.
- `wheel_leg_tracking_signal.m`: Scope-friendly expected relative joint signal.
- `set_initial_base_state.m`: consistent floating-base initial state setter.
- `set_initial_pitch.m`: convenience pitch disturbance setter.
- `configure_discrete_controller_timing.m`: sampled controller timing and scope/solver configuration.
- `run_diagnostic_case.m`: one-command diagnostic runner for short cases,
  summary statistics, offline QP debug sampling, and saved plots.
- `suppress_scope_windows.m`: prevents Scope windows from popping up during batch runs.

Removed during cleanup:

- `configure_model.m`: old rewiring script that could overwrite current
  manual/phase-1 Simulink connections.
- `hip_reference_signal.m`: unused legacy Simulink hip-reference wrapper.
- `wheel_leg_reference_signal.m`: unused legacy Simulink joint-reference
  wrapper. The current Scope reference path uses `wheel_leg_tracking_signal.m`.

## Current Key Parameters

From `startup.m`:

```matlab
ctrl.Ts = 0.005;
base.Ts = ctrl.Ts;
base.controllerType = "discrete";

base.Q = diag([25, 80, 120, 8, 16, 10]);
base.R = diag([1/80^2, 1/140^2, 1/60^2]);

ctrl.qpSolver = "equality";
ctrl.qpWarmStart = true;

base.thetaIntegralGain = 80;
base.thetaIntegralLimit = 0.5;
```

`ctrl.qpSolver = "equality"` is currently used to keep the model runnable.
Switching to full QP is possible but much slower:

```matlab
ctrl.qpSolver = "quadprog";
```

The pitch integral term is a temporary stabilizing patch, not the final
control design.

## Historical Findings

The older model at:

```matlab
D:\Workspace\CodeWorkspace\dynamic\3D\simulate\single_wheel_leg
```

used an upper hip-position PD to generate a 2D desired force, then used the
lower QP. After parameter tuning, that model tracked reasonably well.

This suggested that the lower wheel-leg QP and contact model are not
inherently unusable. The new problem appeared after adding floating-base
pitch and replacing the upper hip PD force command with floating-base LQR
wrench allocation.

Previously observed:

- Continuous LQR stands more easily than discrete LQR.
- Discrete LQR with the same `Q/R` gives a much weaker pitch moment channel.
- Simply increasing `Qtheta/Qdtheta` does not strongly increase `MBy`.
- Short 5 deg pitch tests could run, but the behavior was poor.
- Zero-disturbance short tests could run, but long-term behavior was not yet
  trusted.

## 2026-08-06 Diagnostic Before Stage 1 Change

Short test:

```matlab
cd D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg
startup
configure_discrete_controller_timing(false)
set_initial_pitch(deg2rad(5))
sim("source", "StopTime", "3")
```

Important result:

- `tau_h` tracked `ctrl.hipMomentToTauSign * MBy_des` very well.
- Torque saturation was not the main problem.
- QP exitflag was good.
- But `q/dq` tracking was poor.

Representative numbers:

```text
theta final: about -0.034 rad
tau_h vs hipMomentToTauSign*MBy_des RMS error: about 0.005 N*m
tau saturation ratio: 0
QP exitflag: 1

qAbs tracking RMS:
  hip   about 0.048 rad
  knee  about 0.117 rad
  wheel about 2.154 rad
```

Interpretation:

The bad behavior was not mainly caused by `MBy_des -> tau_h` failure.
Instead, the lower QP was being asked to track a leg reference that was not
geometrically consistent with the moving floating base.

## Stage 1 Reference Fix

Design decision:

First make the lower leg reference consistent with the current floating-base
geometry. Do not yet use wheel horizontal motion as a balancing degree of
freedom.

Stage 1 rule:

```text
base state -> hip world position -> wheel-center reference -> IK -> q/dq/ddq
```

Wheel-center reference:

```matlab
pO_x_ref = pH_x + traj.xO0;
pO_z_ref = groundTop + leg.r;
```

`floating_base_leg_reference.m` computes:

```matlab
pH = [xB; zB] + R(thetaB) * base.rHBody;
pO_rel = [traj.xO0; groundTop + leg.r - pH_z];
[qh_abs_ref; qk_ref] = wheel_leg_inverse_kinematics(pO_rel, ...);
```

Then wheel spin is generated from the wheel-center world horizontal motion
and no-slip rolling convention.

Important distinction:

- `wheel_leg_reference.m` is the legacy/reference generator.
- `floating_base_leg_reference.m` is now used by the 16D floating-base QP
  path.
- `wheel_leg_tracking_signal.m` also uses `floating_base_leg_reference.m`, so
  Scope reference signals match what the controller is actually tracking.

## 2026-08-06 Diagnostic After Stage 1 Change

5 deg pitch, 3 s:

```text
theta final: about 0.00048 rad
theta min/max: about [-0.0459, 0.0873] rad

qAbs error final:
  hip   about -0.034 rad
  knee  about -0.026 rad
  wheel about  0.499 rad

qAbs error RMS:
  hip   about 0.033 rad
  knee  about 0.026 rad
  wheel about 0.484 rad

tau_h vs hipMomentToTauSign*MBy_des RMS error: about 0.0012 N*m
tau saturation ratio: 0
QP exitflag: 1
```

Zero initial state, 5 s:

```text
theta final: about 0.000003 rad
theta min/max: about [-0.0336, 0.00066] rad

qAbs error final:
  hip   about -0.034 rad
  knee  about -0.026 rad
  wheel about  0.459 rad

tau_h vs hipMomentToTauSign*MBy_des RMS error: about 0.0011 N*m
```

Conclusion:

Stage 1 made the system behavior much more reasonable. A major part of the
previous poor performance was caused by a leg-reference / floating-base
geometry mismatch, not by failure to track `MBy_des` with `tau_h`.

## 2026-08-06 Residual Error Diagnosis

After adding `floating_base_leg_reference.m`, the remaining steady errors are
mostly:

```text
hip/knee: small fixed offset
wheel: about 0.5 rad accumulated offset
wheel center: about -3 cm x error, about -2.6 mm z error
```

Additional diagnostics were added to `run_diagnostic_case.m`:

- actual contact velocity residual:

```matlab
kin.Jc * dqAbs + vH
```

- reference contact velocity residual:

```matlab
kinRef.Jc * dqdAbs + vH
```

- actual/reference wheel-center world position and error.

3 s, 5 deg pitch result:

```text
reference contact velocity RMS:
  about [4e-18, 7e-19] m/s

actual contact velocity RMS:
  about [0.0040, 0.0099] m/s over the full transient

actual contact velocity after 1 s:
  RMS about [0.00043, 0.000018] m/s

wheel-center final error:
  about [-0.0308, -0.00262] m

wheel-center RMS error:
  about [0.0298, 0.00262] m
```

Interpretation:

- The stage-1 reference is self-consistent with the rolling/no-slip velocity
  constraint.
- The actual plant also satisfies the contact velocity constraint well after
  the initial transient.
- The remaining joint error is therefore closer to a wheel-center position
  error / contact compliance / structural objective conflict than to a
  rolling-velocity reference bug.

Two quick counter-tests:

1. Disable upper horizontal force by setting `baseLqr.forceMax(1)=0`.

```text
theta final after 3 s: about -0.64 rad
```

This is unstable, so the LQR currently needs `FHx` for pitch recovery.

2. Increase `ctrl.qpWqdd` by 10x and 100x.

```text
wheel-center x error remains about -3 cm
wheel angle error remains about 0.49 rad
```

This suggests the residual is not simply caused by weak `qdd` tracking weight.
The current upper LQR uses horizontal hip force as an important pitch-control
channel, while stage 1 asks the lower layer to keep the wheel horizontally
under the hip. That goal is only a soft joint-acceleration tracking objective,
not a hard position constraint.

## 2026-08-06 Design Interpretation

The remaining steady joint/wheel-center offsets should not automatically be
treated as controller failure.

The lower QP is intentionally a tradeoff layer. It should satisfy hard
constraints first, then balance multiple soft objectives:

```text
hard / high-priority:
  dynamics consistency
  contact and rolling constraints
  feasible contact force / torque limits

soft objectives:
  base wrench request from upper LQR
  joint q/dq tracking
  wheel-center geometry preference
  torque/contact-force regularization
```

Therefore, zero joint tracking error is not the real design objective. A
nonzero steady `q - qd` can be acceptable if the system stays inside a normal
working envelope and the higher-level task is achieved.

The current learning goal should shift from:

```text
make every q exactly track qd
```

to:

```text
define acceptable operating metrics and diagnose which objective is limiting
performance when those metrics are violated
```

Candidate stage-1 acceptability metrics:

- floating-base pitch settles and remains bounded
- `dthetaB` decays
- joint angles remain away from singularity and joint limits
- wheel-center error remains within an agreed tolerance
- torque commands stay below saturation with margin
- contact velocity residual remains small after transient
- contact force and friction margin remain physically plausible
- QP exitflag remains healthy
- response recovers from a selected disturbance set

This means the next decision is not "remove all steady error"; it is "choose
the performance envelope that stage 1 should satisfy." Once that envelope is
defined, further controller changes can be judged by metrics instead of by
whether each soft objective reaches zero error.

## Current Diagnosis

What is likely fixed:

- The lower QP no longer tracks a fixed-hip-style leg reference in the 16D
  floating-base path.
- The expected joint reference shown in Scope now matches the reference used
  by the controller.
- Pitch recovery from a 5 deg initial disturbance is much better.

What is still not solved:

- There is still a steady joint tracking offset.
- Wheel angle tracking has a noticeable accumulated offset.
- `qddSol` still differs from `qddCmd`, so contact/no-slip constraints and
  the tracking objective are still in conflict.
- The residual offset does not disappear by simply increasing `ctrl.qpWqdd`.
- Disabling the LQR horizontal force channel destabilizes pitch, so `FHx` is
  currently part of the balancing mechanism.
- The pitch integral term is still a temporary patch.
- This is still stage 1; the wheel is not yet being used as an intentional
  horizontal balancing degree of freedom.

Current best explanation:

```text
Main old issue:
  lower leg reference fought the floating-base geometry.

Remaining issue:
  lower QP/contact/no-slip satisfy velocity consistency, but stage-1
  wheel-under-hip position tracking remains soft and conflicts with the
  upper LQR's use of horizontal force for pitch recovery.
```

## Recommended Next Steps

Do not blindly tune LQR `Q/R` yet. First agree on the stage-1 performance
metrics and tolerances.

Next diagnostic target:

```text
stage-1 wheel-center position target vs upper LQR horizontal-force demand
```

Recommended signals:

- `thetaB`, `dthetaB`
- `xB`, `dxB`
- `pH_x`, `pH_z`
- wheel center `pO_x`, `pO_z`
- `qh_abs`, `qk`, `qw`
- `qd_abs`, `dqd_abs`
- `qAbs - qd_abs`
- `dqAbs - dqd_abs`
- `qddCmd`
- QP `debug.qdd`
- QP `debug.Fc`
- QP `exitflag`
- `MBy_des`
- `tau_h`
- `tau_h - ctrl.hipMomentToTauSign * MBy_des`
- contact velocity constraint residual `kin.Jc*dq + vH`
- wheel-center world position actual/reference/error

Decision options before running more simulations:

Option A: Define stage-1 acceptance criteria first.

- Pick tolerances for pitch settling, wheel-center error, joint range, torque
  margin, and contact residual.
- Then run a small disturbance sweep and classify pass/fail.
- This is the most conservative next step because it does not change the
  controller before defining what "good enough" means.

Option B: Strengthen the stage-1 wheel-center objective.

- Add an explicit wheel-center position objective or Baumgarte-style position
  correction in the lower QP.
- Then check whether pitch and `MBy` tracking stay acceptable.
- This is useful if "wheel under hip" is considered a required stage-1
  behavior, not just a diagnostic preference.

Option C: Move toward stage 2.

- Make wheel horizontal position or velocity a deliberate upper-layer
  command/state instead of forcing it to stay under the hip.
- This is more physically aligned with wheel inverted-pendulum balancing, but
  it expands the control problem.

Current recommendation:

```text
Option A selected: define acceptance criteria and run a small robustness
study before changing the controller.
```

Keep `FHx` enabled during future tests; disabling it made the 5 deg case
unstable.

Stage-A calibration plan:

```text
location:
  D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response

cases:
  theta0 = [-10, 0, 10] deg
  pulseAmplitude = [0, 5, 10] N
  pulseWindow = [2.0, 2.5] s
  stopTime = 5 s

first-pass criteria:
  max |theta| <= 15 deg
  final |theta| <= 2 deg
  final |dtheta| <= 0.1 rad/s
  tauSaturationRatio <= 0.95
  max |x| <= 0.5 m
```

Smoke test:

```text
2026-08-06, calibration runner smoke case pitch_-10deg_pulse_0N passed.
Result directory:
  D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_193715
Elapsed time:
  about 42.18 s for one 5 s case
```

Pulse smoke / bugfix:

```text
2026-08-06, pitch_-10deg_pulse_5N passed after fixing pulse-window metrics.
Bug: base/tau/uLqr logs can have different sample counts, so pulse metrics
must use each signal's own time vector.
Result directory:
  D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_194726
Elapsed time:
  about 40.98 s for one 5 s pulse case
```

IK robustness bugfix:

```text
2026-08-06, full sweep initially hit invalid output in
wheel_leg_tracking_signal because floating_base_leg_reference requested a
wheel-center target at/near the two-link IK singular boundary.

Fix:
  floating_base_leg_reference projects the wheel-center target into the
  reachable annulus.
  wheel_leg_inverse_kinematics uses damped least squares for velocity and
  acceleration solves near singularity.

Validation:
  pitch_10deg_pulse_10N now completes simulation and is classified unstable,
  instead of crashing reference generation.
```

Full stage-A robustness result:

```text
2026-08-06 result directory:
  D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_195701

Report:
  D:\Workspace\CodeWorkspace\calibration\reports\2026_08_lqr_disturbance_response_summary.md

Summary:
  6 / 9 cases passed.
  All 0 N and 5 N pulse cases passed for theta0 = [-10, 0, 10] deg.
  All 10 N pulse cases failed.

Working envelope from this coarse scan:
  theta0 within +/-10 deg, dtheta0 = 0, pulse <= 5 N for 0.5 s at t = 2 s.

Decision:
  Stop stage-A scanning. Do not refine pulse amplitude between 5 N and 10 N
  for now.

Next plan:
  1. Interpret representative pass/fail cases, especially
     pitch_10deg_pulse_5N vs pitch_10deg_pulse_10N.
  2. Use that interpretation to tune LQR/QP parameters, not to redesign the
     model architecture yet.
  3. Re-run the same 9-case robustness set after each meaningful parameter
     batch and compare against baseline result 20260806_195701.
  4. Only after stationary robustness is acceptable, add horizontal and
     vertical base motion tests.
```

Representative pass/fail analysis:

```text
Report:
  D:\Workspace\CodeWorkspace\calibration\reports\2026_08_pass_fail_mechanism.md

Figures:
  D:\Workspace\CodeWorkspace\calibration\figures\studies\2026_08_lqr_disturbance_response\pass_fail_20260806_195701

Key finding:
  The 10 N failure is mainly a post-pulse recovery transient problem.
  At pulse end, theta is about 10.95 deg and x is about 0.095 m.
  The first upper-command saturation appears around t = 2.755 s in FHz_ext.
  Around t = 2.8 s, dtheta is already about -3.31 rad/s and upper force
  commands are near saturation.

Initial tuning implication:
  Do not globally increase all Q weights.
  First try shifting pitch recovery effort toward earlier moment/dtheta damping
  and away from force-channel saturation.
```

Diagnostic entry point:

```matlab
run_diagnostic_case.m
```

Current behavior:

- `startup`
- optional `set_initial_pitch(deg2rad(5))`
- `configure_discrete_controller_timing(false)`
- `sim("source")`
- extract `logsout`
- compute absolute and relative joint errors
- compute torque tracking and saturation
- compute offline QP debug samples
- save plots and print a short summary

Examples:

```matlab
run_diagnostic_case
run_diagnostic_case(deg2rad(5), 3)
run_diagnostic_case(zeros(6,1), 5, false)
```

## Common Commands

5 deg pitch short run:

```matlab
cd D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg
startup
configure_discrete_controller_timing(false)
set_initial_pitch(deg2rad(5))
sim("source", "StopTime", "3")
```

Zero initial short run:

```matlab
cd D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg
startup
configure_discrete_controller_timing(false)
set_initial_base_state(zeros(6,1))
sim("source", "StopTime", "5")
```

Persist timing changes to the model only when intentional:

```matlab
startup
configure_discrete_controller_timing(true)
```

Important:

- `startup` clears the workspace, so set initial pitch after `startup`.
- Use `set_initial_pitch` or `set_initial_base_state`; do not hand-edit
  `base.x0`.
- Use `configure_discrete_controller_timing.m` for timing updates. The old
  `configure_model.m` rewiring script has been removed.

## Handoff Notes For Future Agents

Start by reading this file, then inspect:

```matlab
startup.m
controller_qp_core.m
floating_base_leg_reference.m
wheel_leg_tracking_signal.m
```

Current development posture:

- Do not refactor first.
- Do not tune `Q/R` first.
- Preserve the stage-1 geometry reference while diagnosing residual QP/contact
  inconsistency.
- Treat the model as a learning prototype; the current goal is clear
  attribution of failure modes, not final performance.

Calibration workspace boundary:

- Keep the model, dynamics, and controller implementation in this
  `dynamic/.../single_wheel_leg` directory.
- Put experiment design, batch simulation entry points, data extraction,
  summary tables, plots, and analysis notes under:

```text
D:\Workspace\CodeWorkspace\calibration
```

- Follow `calibration/README.md`.
- Current study path for the stage-A +/-10 deg disturbance acceptance work:

```text
D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
```

- The study runner has been updated to use
  `configure_discrete_controller_timing(false)`, `logsout` extraction, and the
  stage-1 `floating_base_leg_reference` metric convention.

## 2026-08-06 LQR Q/R Bayesian Run

Result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337
```

Summary report:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_lqr_qr_bayes_summary.md
```

Key result:

- Best trial: `trial_0014`
- Objective improved from about `57503.5` to `54903.9`, about `4.5%`.
- Stable training cases remained `3 / 5`: 0 N and 5 N cases passed, 10 N
  cases still failed.
- Best parameter trend: make `FHz` more expensive and `MBy` cheaper in the
  upper-layer LQR `R`; do not treat this as final tuning yet.

Next action:

- Validate `trial_0014` on the full 9-case stage-A acceptance set before
  changing default `startup.m` parameters.
- Validation helper added:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
out = validate_best_qr_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337");
```

This calls `run_cases(cases, bestParams)`, so the saved LQR `Q/R` is reapplied
after each per-case `startup`.

Validation completed:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_220040
```

Validation conclusion:

- Full 9-case result stayed at `6 / 9` stable.
- All `0 N` and `5 N` cases still passed.
- All `10 N` cases still failed.
- `pitch_10deg_pulse_10N` improved strongly, but
  `pitch_-10deg_pulse_10N` regressed badly.

Updated next action:

- Do not apply `trial_0014` to `startup.m`.
- Run a second QR Bayesian round with symmetric 10 N training cases, including
  `pitch_-10deg_pulse_10N`, and expanded `R_Fz` / `R_MBy` search ranges.

## 2026-08-06 Limit-Scale Diagnostic

Reason:

- The 10 N pulse cases all reached `tauSaturationRatio = 1.0`.
- Before treating 10 N as purely a QR/QP tuning problem, test whether the
  current failure is mainly a control-authority limit.

Helper:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
```

Best QR with 2x upper/LQR and lower/QP limits:

```matlab
out = validate_limit_scale_9cases(2.0, "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337");
```

Default QR with 2x limits, useful as an isolation check:

```matlab
out = validate_limit_scale_9cases(2.0);
```

This helper applies, after each per-case `startup`:

```text
ctrl.tauMax     *= limitScale
base.forceMax   *= limitScale
hip.forceMax    *= limitScale
base.momentMax  *= limitScale
```

It does not edit `startup.m` defaults.

Completed best QR + 2x all-limits run:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_221550
```

Conclusion:

- Stable count stayed at `6 / 9`.
- All `10 N` cases still failed.
- The 2x run still hit the new doubled LQR and torque limits.
- `pitch_-10deg_pulse_10N` and `pitch_10deg_pulse_10N` became much worse.

Interpretation:

- Scaling all limits is not a clean actuator-only test, because it also lets
  the upper LQR command become more aggressive.
- The next cleaner diagnostic is to scale only lower torque limits:

```matlab
limits = struct("tauScale", 2.0, "forceScale", 1.0, "momentScale", 1.0);
out = validate_limit_scale_9cases(limits, "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\20260806_203337");
```

## 2026-08-06 Recovery Transient Analysis

User observation from animation:

```text
after the disturbance, the inverted pendulum leans and rolls forward;
then pitch suddenly returns toward zero;
after that the system diverges.
```

Offline analysis script:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
analysis = analyze_recovery_transient();
```

Outputs:

```text
D:\Workspace\CodeWorkspace\calibration\data\processed\2026_08_recovery_transient_events.csv
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_recovery_transient_analysis.md
D:\Workspace\CodeWorkspace\calibration\figures\studies\2026_08_lqr_disturbance_response\recovery_transient
```

Key result:

- At pulse end (`t = 2.5 s`), theta is only about `11 deg` and dtheta is about
  `0.20` to `0.32 rad/s`.
- The dangerous phase is post-pulse recovery: upper LQR command reaches its
  limit around `2.73` to `2.80 s`, theta crosses zero around `2.74` to
  `2.79 s`, lower torque saturates around `2.84` to `2.88 s`, and then theta
  exceeds the `15 deg` failure threshold.
- The apparent pitch correction is a pass-through event with too much recovery
  energy, not a settled recovery.

Decision:

- Stop enlarging all limits.
- Next tuning should reduce recovery aggressiveness and add damping/shape:
  increase effective dtheta damping relative to theta stiffness, constrain
  sudden LQR wrench changes, keep x/dx from accumulating, then revisit QP
  priorities if torque saturation still follows.

## 2026-08-06 Round-2 QR Experiment

Prepared experiment:

```text
D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr_round2
```

Purpose:

- Keep the plant, QP, contact, limits, and stage-1 reference fixed.
- Tune only upper-layer floating-base LQR `Q/R`.
- Use symmetric 10 N pressure cases so the optimizer cannot improve positive
  pitch recovery while regressing negative pitch recovery.
- Add recovery-aware scoring terms so theta returning through zero with high
  angular velocity is penalized.

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

Run order:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr_round2
score = run_smoke_test;
score = run_default_qr_check;

cd D:\Workspace\CodeWorkspace\calibration
out = run_bayes_calibration("single_wheel_leg_lqr_qr_round2");
```

Static check:

```text
experiment_config.m, qr_training_cases.m, run_trial.m, score_trial.m,
run_smoke_test.m, and run_default_qr_check.m all pass MATLAB checkcode.
```

Round-2 result:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811
```

Summary report:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_lqr_qr_round2_summary.md
```

Key outcome:

- Best trial: `trial_0005`
- Objective improved from about `77830.7` to `69145.0`, about `11.2%`.
- Stable training cases stayed at `3 / 6`; no 10 N case passed in any of the
  24 trials.
- Best multipliers: `R_Fz ~= 14.7x`, `Q_dx ~= 3.4x`, `R_MBy ~= 0.61x`.
- Round 2 greatly reduced the `-10 deg` and `0 deg` 10 N failure severity, but
  `+10 deg` 10 N remains poor.

Next action:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
out = validate_best_qr_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

If full validation remains `6 / 9`, stop blind QR expansion and move to
recovery shaping or QP priority tuning.

Full 9-case validation completed:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260807_182335
```

Validation result:

- Round-2 best stayed at `6 / 9` stable.
- All `0 N` and `5 N` cases passed.
- All `10 N` cases failed.
- 0 N / 5 N guardrails remain acceptable, with mild increases in x drift,
  torque usage, and leg velocity RMS compared with the default QR.
- `pitch_-10deg_pulse_10N` and `pitch_0deg_pulse_10N` are much improved.
- `pitch_10deg_pulse_10N` is worse than round-1 best, though still better than
  default.

Decision:

- Do not continue blind QR expansion.
- Do not write round-2 best Q/R into `startup.m` as a final default.
- Move next to recovery shaping or QP-priority analysis:

```text
LQR wrench slew/rate limiting or filtering
post-pulse effective damping
QP moment tracking versus q/ddq tracking priority during recovery
```

## 2026-08-07 QP Recovery Tracking Analysis

Offline analysis script:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
analysis = analyze_qp_recovery_tracking();
```

Outputs:

```text
D:\Workspace\CodeWorkspace\calibration\data\processed\2026_08_qp_recovery_tracking.csv
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_qp_recovery_tracking.md
D:\Workspace\CodeWorkspace\calibration\figures\studies\2026_08_lqr_disturbance_response\qp_recovery_tracking
```

Key result:

- For round-2 best, `pitch_-10deg_pulse_10N` and
  `pitch_0deg_pulse_10N` have very small `MBy_des -> tau_h` tracking error
  during the 0.5 s post-pulse recovery window:
  `recoveryRmsTauHError ~= 0.015 N*m`.
- In those cases, `MBy_des` and `tau_h` do not saturate; the remaining upper
  saturation is mainly `FHx_ext`.
- For round-2 best:

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

pitch_10deg_pulse_10N:
  max |FHx_ext| ~= 140 N
  max |FHz_ext| ~= 140 N
  max |MBy_des| ~= 160 N*m
  recoveryDqErrorRms ~= 36.8 rad/s
```

Decision:

- Do not treat lower QP pitch moment tracking as the primary bottleneck for
  the improved `-10 deg` and `0 deg` 10 N cases.
- Next experiment should prioritize upper-layer wrench shaping, especially
  `FHx_ext` rate limiting/filtering.
- Keep QP priority tuning as a follow-up for `pitch_10deg_pulse_10N`, where all
  upper channels saturate and leg tracking error is large.

## 2026-08-07 Qx/Qdx Rollback Diagnostic

Reason:

- Round-2 best makes `FHx_ext` mainly driven by `dxB`.
- Round-2 best has `Q_dx ~= 3.39x`, which can make horizontal velocity recovery
  too aggressive.
- Before adding `FHx_ext` filtering/rate limiting, test whether simply reducing
  `Q_x/Q_dx` is enough.

Helper:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
batch = validate_qx_qdx_rollback_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Default variants:

```text
qdx_2p0:
  Q_x  = 0.60557x
  Q_dx = 2.0x

qdx_1p0:
  Q_x  = 0.60557x
  Q_dx = 1.0x

qx_0p3_qdx_1p0:
  Q_x  = 0.3x
  Q_dx = 1.0x
```

All variants keep the other round-2 best `Q/R` entries unchanged and run the
full 9-case stage-A validation. The batch writes a `rollback_index.csv` with
stable count, 5 N guardrail metrics, and 10 N pressure-test severity.

Rollback result:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\qx_qdx_rollback_20260807_190201
```

Summary report:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_qx_qdx_rollback_summary.md
```

Key outcome:

- All variants stayed at `6 / 9` stable.
- All `0 N` and `5 N` guardrail cases still passed.
- All `10 N` pressure cases still failed.
- `Q_dx = 2.0x` improved `pitch_10deg_pulse_10N` relative to round-2 best but
  degraded `pitch_-10deg_pulse_10N` and `pitch_0deg_pulse_10N`.
- `Q_dx = 1.0x` is too low and causes severe 10 N failures.
- Lowering `Q_dx` reduces `FHx` saturation fraction but shifts recovery burden
  into `FHz/MBy` or lets horizontal velocity drift into a worse region.

Decision:

- Do not use `Q_dx = 1.0x`.
- Do not continue coarse `Q_x/Q_dx` rollback alone.
- Next useful test is a narrower combined design:

```text
Q_dx between 2.5x and 3.4x
plus mild FHx_ext output shaping
```

## 2026-08-07 FHx Output Shaping Diagnostic

Implementation:

- `floating_base_lqr_command.m` now supports optional output shaping through
  `baseLqr.commandShaping`.
- Default behavior is unchanged because shaping is disabled unless the
  validation script passes `lqrParams.commandShaping`.
- Current shaping is applied only to `FHx_ext`; `FHz_ext` and `MBy_des` are
  left unchanged.

Supported shaping parameters:

```text
commandShaping.enabled
commandShaping.channels    % [FHx; FHz; MBy]
commandShaping.filterTau   % first-order filter time constant, seconds
commandShaping.rateLimit   % N/s for FHx when channels(1)=true
```

Helper:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
batch = validate_fhx_shaping_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Default variants:

```text
fhx_filter_30ms:
  round-2 best Q/R
  FHx_ext first-order filter, tau = 0.030 s

fhx_rate_2000:
  round-2 best Q/R
  FHx_ext rate limit = 2000 N/s

qdx_2p5_fhx_filter_30ms:
  Q_dx = 2.5x
  FHx_ext first-order filter, tau = 0.030 s
```

The batch writes `fhx_shaping_index.csv` plus one full 9-case result directory
per variant. Use it to compare 5 N guardrail stability and 10 N pressure-test
severity against round-2 best and the Qx/Qdx rollback runs.

First FHx shaping batch:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\fhx_shaping_20260807_191943
```

Status:

- Invalid for judging shaping.
- `fhx_filter_30ms` and `fhx_rate_2000` produced exactly the same `uLqr` signal
  as round-2 best.
- Cause: `set_initial_base_state` rebuilt `baseLqr` per case, and
  `floating_base_lqr_design` did not preserve `base.commandShaping`.

Fix:

- `floating_base_lqr_design.m` now copies `base.commandShaping` into
  `baseLqr.commandShaping`.
- Verified that shaping survives `set_initial_base_state`.

Report:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_fhx_shaping_batch_note.md
```

Action:

- Re-run `validate_fhx_shaping_9cases(...)`.
- Only the new post-fix batch should be used to judge FHx filtering/rate
  limiting.

Post-fix FHx shaping batch:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\fhx_shaping_20260807_195550
```

Outcome:

- All three variants remain `6 / 9` stable.
- All `0 N` and `5 N` guardrail cases still pass.
- All `10 N` pressure cases still fail.
- `fhx_filter_30ms` improves the positive-pitch 10 N case relative to round-2
  best, but it worsens the negative-pitch and zero-pitch 10 N cases.
- `fhx_rate_2000` helps the zero-pitch 10 N case slightly, but worsens the
  negative-pitch and positive-pitch 10 N cases.
- `qdx_2p5_fhx_filter_30ms` is not useful; 10 N failures become severe.

Recovery-event comparison:

```text
round2_best:
  dtheta at theta zero ~= -1.7 to -2.0 rad/s

fhx_filter_30ms:
  dtheta at theta zero ~= -3.0 rad/s

qdx_2p5_fhx_filter_30ms:
  dtheta at theta zero ~= -2.6 to -2.7 rad/s
```

Decision:

- Do not adopt fixed FHx filtering/rate limiting as the next baseline.
- Do not continue coarse `Q_dx` rollback plus fixed FHx filtering.
- Next diagnostic should test smaller upper `Fx/Fz` limits. The normal
  `0 N / 5 N` envelope does not need the current large force authority, and the
  10 N failures look like excessive recovery energy rather than insufficient
  command smoothness.

Prepared helper:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
batch = validate_force_limit_grid_9cases("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Default variants:

```text
forceScale = 0.8, tauScale = 1.0, momentScale = 1.0
forceScale = 0.6, tauScale = 1.0, momentScale = 1.0
forceScale = 0.4, tauScale = 1.0, momentScale = 1.0
forceScale = 0.6, tauScale = 0.8, momentScale = 1.0
```

Note:

- Smaller `Fx/Fz` limits reduce overly aggressive or infeasible force requests,
  but they do not guarantee perfect lower-layer realization. QP realization
  still depends on leg configuration, contact, joint torque limits, and the
  unchanged pitch-moment target.

## 2026-08-08 Boundary Failure: Leg Branch / Singularity

User observation:

- `8 N` pulse can recover.
- `8.5 N` is near the boundary: pitch oscillates heavily and may eventually
  settle, but the leg can end on the opposite side.
- `9 N` loses balance.
- Animation shows pitch rapidly returning upright, then the thigh/shank are
  dragged toward a near-singular or opposite-branch configuration.

Interpretation:

- The current failure is no longer just "upper LQR output too large" or
  "lower QP cannot track moment".
- The dangerous event is recovery-phase coupling:

```text
pitch recovery -> large hip/thigh motion -> leg approaches singular/branch
change -> QP/contact cannot preserve the intended stage-1 leg geometry
```

- Stage-1 IK asks the wheel center to remain near the hip vertical line, but
  the actual dynamics do not contain a hard constraint that keeps the leg on
  the same IK branch.
- The two-link wheel-center Jacobian determinant is proportional to
  `sin(qk)`, so `qk ~= 0 deg` or `qk ~= 180 deg` is singular. The nominal
  stance is `qk ~= 38 deg`; large recovery transients can push the actual leg
  too close to the singular set or to an unintended branch.

Prepared diagnostics:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
analysis = analyze_leg_branch_geometry(resultDirs, labels, caseNames);
```

This reads existing `.mat` files and extracts:

```text
minAbsSinQkPost
minAbsDetJPost
minQkDegPost / maxQkDegPost / finalQkDeg
pOxMinPost / pOxMaxPost / pOxFinal
post-pulse q/dq tracking errors
```

Prepared boundary experiment:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response
batch = validate_boundary_guard_grid("D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Default cases:

```text
theta0 = [-10, 0, 10] deg
pulse  = [8, 8.5, 9] N
stop   = 8 s
```

Default variants:

```text
round2_baseline
force_0p8
wqdd_leg_3x
force_0p8_wqdd_leg_3x
force_0p8_pd_1p5_wqdd_leg_3x
force_0p8_tau_1p2_wqdd_leg_3x
```

The batch writes `boundary_guard_grid_index.csv`. The summary now includes
leg-geometry metrics such as `legBranchOk`, `minAbsSinQkPost`,
`maxAbsPOxPost`, and post-pulse max q/dq error.

Post-run analysis helper:

```matlab
analysis = analyze_boundary_guard_batch(batch.batchDir);
```

Decision logic:

- A candidate is useful only if it preserves the original stability criteria
  and keeps `legBranchOk` true.
- If stronger QP leg tracking improves branch health, the next controller
  change should be a formal QP posture/branch priority.
- If reducing `Fx/Fz` helps but branch health is still poor, use force authority
  reduction as a guardrail but still add leg configuration protection.
- If mild extra torque helps only when paired with stronger QP tracking, the
  previous torque limit was constraining the lower layer during recovery.

Batch robustness note:

- A boundary case may terminate Simulink with a solver/min-step or non-finite
  derivative error near the end of the 8 s run. One observed error occurred at
  about `t = 7.9718 s` in `source/PD_only/Planar Joint` for state `Px.v`.
- This is a valid unstable failure mode for the boundary study, not a reason
  to discard the full batch.
- `run_cases.m` now catches per-case simulation errors, records that case as
  `stable = false` with `failureReason = "simulation error: ..."`, saves the
  case result, and continues with the remaining cases.

Boundary batch result:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\boundary_guard_grid_20260808_073645
```

Post-run summary:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_boundary_guard_grid_summary.md
```

Outcome:

```text
round2_baseline:                       stable 0 / 9, branchOk 0 / 9
force_0p8:                             stable 0 / 9, branchOk 0 / 9
wqdd_leg_3x:                           stable 0 / 9, branchOk 0 / 9
force_0p8_wqdd_leg_3x:                 stable 0 / 9, branchOk 0 / 9
force_0p8_pd_1p5_wqdd_leg_3x:          stable 0 / 9, branchOk 0 / 9
force_0p8_tau_1p2_wqdd_leg_3x:         stable 0 / 9, branchOk 0 / 9
```

Common geometric signature:

```text
maxAbsPOxPost    ~= 0.70 m
minAbsSinQkPost  ~= 0
```

Since `L1 + L2 = 0.70 m`, the actual leg is being pulled close to full
horizontal extension relative to the hip. Since the two-link wheel-center
Jacobian determinant is proportional to `sin(qk)`, the actual leg is also
passing close to a singular configuration.

Baseline `0 deg / 8 N` event order:

```text
t ~= 2.725 s: |pO_x| > 0.25 m
t ~= 2.795 s: |theta| > 15 deg
t ~= 2.820 s: qk branch/singularity guard trips
```

Decision:

- Stop this line of soft parameter-only tests.
- `QR`, `Fx/Fz` limit scaling, QP qdd soft-weight scaling, and small torque
  scaling did not prevent branch loss.
- Next useful change is explicit leg-configuration protection:

```text
qk lower/upper guard
relative wheel-center x guard
or physical Simscape joint limits/contact-safe stop
```

Important implementation note:

- The current `ctrl.qpSolver = "equality"` path cannot enforce inequality
  branch/joint guards. A formal QP guard requires `quadprog` or a new small QP
  solver that handles inequalities.
- A faster learning-model alternative is a pre-QP reference projection/command
  governor or Simscape joint limits.

## 2026-08-08 Pause Summary

Stage summary:

```text
D:\Workspace\CodeWorkspace\calibration\reports\2026_08_single_wheel_leg_stage_summary.md
```

Current engineering conclusion:

- The plant and basic LQR + QP chain are usable for small disturbances.
- The main failure is not simply "QP untuned" or "LQR wrong".
- The missing piece is coordination between upper floating-base recovery and
  lower leg workspace feasibility.
- The upper LQR can command a pitch recovery that is reasonable for the
  simplified base model but unsafe for the current leg configuration.
- The lower fast equality-QP has no hard guard for `qk`, `pO_x`, or IK branch
  safety, so it cannot guarantee avoidance of leg singularity/branch flip.

Recommended when returning:

```text
1. reduce upper recovery aggressiveness near pitch zero crossing
2. add explicit leg-configuration protection
3. stop broad QR/limit/soft-weight scans until those guards exist
```

## 2026-08-08 Stage 2: First Vertical Tracking Case

Decision:

- Keep the current LQR/QP gains and the small-disturbance controller baseline.
- Stop parameter scanning while the motion-control flow is being connected.
- First test only a smooth floating-base `z` trajectory with zero external
  disturbance. Horizontal and combined motion remain disabled.

Implemented reference:

```text
0 to 1 s: hold the initial height
1 to 4 s: minimum-jerk rise by 0.02 m
4 to 6 s: hold the raised height
6 to 9 s: minimum-jerk return to the initial height
9 to 10 s: hold the initial height
```

Control changes:

- `floating_base_reference.m` generates consistent `z_ref`, `dz_ref`, and
  `ddz_ref` without changing the existing Simulink signal widths.
- `floating_base_lqr_wrench.m` tracks the time-varying reference and adds
  rigid-body acceleration feedforward.
- The lower floating-base leg reference now receives the estimated hip
  acceleration, so its `ddqd` remains consistent with a grounded wheel center
  during vertical base motion.
- `configure_base_tracking_case.m` prepares the existing model in memory, sets a
  10 s stop time, and sets every Pulse Generator amplitude to zero. It does
  not save or overwrite `source.slx`.

Run entry point:

```matlab
startup
configure_base_tracking_case
sim("source", "StopTime", "10")
```

Status:

```text
Implementation and static inspection complete.
Simulation intentionally left for manual observation.
```

### Aggressive z-step update

Manual observation of the first smooth case:

- The base broadly followed the commanded vertical motion.
- The original 2 cm / 3 s minimum-jerk profile was too mild for the next
  diagnostic.

The next case keeps the same controller gains and changes only the reference:

```text
0 to 1 s: hold the initial height
at 1 s:   step down by 0.06 m
1 to 4 s: hold the lowered height
at 4 s:   step back to the initial height
4 to 10 s: observe recovery
```

This deliberately uses a position step. Reference velocity and acceleration
remain zero away from the two discontinuities; the LQR feedback supplies the
transient vertical wrench. All Pulse Generator disturbances remain disabled
by `configure_base_tracking_case.m`.

## 2026-08-08 Stage 3: Forward/Reverse Velocity Tracking

The aggressive vertical step completed with good manual behavior. The next
case keeps the base-height reference fixed and compares symmetric forward and
reverse constant-speed motion:

```text
0 to 1.0 s: hold the initial position
1.0 to 1.5 s: accelerate to +0.5 m/s
1.5 to 3.0 s: move forward at +0.5 m/s
3.0 to 3.5 s: decelerate to rest at x_ref = +1.0 m
3.5 to 4.0 s: hold the forward position
4.0 to 4.5 s: accelerate to -0.5 m/s
4.5 to 6.0 s: return at -0.5 m/s
6.0 to 6.5 s: decelerate to rest at the initial position
6.5 to 10 s: observe recovery
```

The velocity ramps use the same minimum-jerk polynomial, integrated
analytically so `x_ref`, `dx_ref`, and `ddx_ref` remain mutually consistent.
The rigid-body feedforward supports `FHx = m*ddx_ref`, and the grounded-wheel
reference uses the estimated hip horizontal acceleration for wheel-spin
acceleration consistency. Vertical motion and all external pulse disturbances
are disabled for this case.

Data capture entry point:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_base_motion_tracking
out = run_velocity_round_trip;
```

Each run writes a timestamped directory under:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_base_motion_tracking
```

The result keeps both the complete `SimulationOutput/logsout` and a compact
analysis-ready structure containing base state, joint state, LQR wrench,
joint torque, the generated reference trajectory, and parameter metadata.

## 2026-08-08 Stage 4: Force-Based Wheel-Position Planning (Scheme 1)

Purpose:

- Keep the existing floating-base LQR as the upper desired-wrench generator.
- Use its final desired horizontal body force to determine the wheel-position
  direction without introducing a second outer LQR.
- Preserve the lower inverse-dynamics QP interface.

Force convention:

```text
F_H_des = force applied by the leg to the body
FH_ext  = -F_H_des = body-on-leg force passed to the lower QP
FBody   = -FH_ext
```

The first implementation used the WIPM equilibrium relation:

```text
r_x_raw = r_x_eq - H/g * a_Bx_des
a_Bx_des = FBody_x / m_B
```

where `r_x` is the wheel-center position relative to the floating-base CoM.
The target was projected onto the positive-knee IK branch before conversion
to `qd`, `dqd`, and `ddqd`.

Implementation entry points:

```text
startup.m
floating_base_leg_reference.m
controller_qp_core.m
wheel_leg_tracking_signal.m
run_diagnostic_case.m
test_force_based_wheel_position_reference.m
```

Static interface/sign check:

```matlab
cd D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg
test_force_based_wheel_position_reference
```

This check passed. It verifies the instantaneous sign relation, finite
controller outputs, and the reference knee guard at the nominal base state.
It does not verify temporal continuity or closed-loop feasibility.

### Recorded zero-disturbance work condition

The first closed-loop diagnostic reused the Stage 3 forward/reverse velocity
tracking case:

```text
duration:              10 s
controller sample:     0.005 s
initial base state:    zeros(6,1)
external pulse inputs: disabled
0.0 to 1.0 s:          hold
1.0 to 1.5 s:          accelerate to +0.5 m/s
1.5 to 3.0 s:          forward cruise
3.0 to 3.5 s:          decelerate to rest
3.5 to 4.0 s:          hold
4.0 to 4.5 s:          accelerate to -0.5 m/s
4.5 to 6.0 s:          reverse cruise toward the initial position
6.0 to 6.5 s:          decelerate to rest
6.5 to 10.0 s:         recovery observation
```

Run setup:

```matlab
startup
configure_base_tracking_case
summary = run_diagnostic_case(zeros(6,1), 10, false);
```

One-off diagnostic timestamp:

```text
20260808_213358
```

Measured result:

```text
theta final                         0.012463 rad
FHx body-force range               [-140, 140] N
FHx RMS                             85.9203 N
CoM-to-wheel height H              [0.7513, 0.8542] m
unconstrained r_x_raw range         [-4.0608, 4.0560] m
reference projection/clamp ratio   89.3 %
FHx sign changes                    68
r_x_ref max step per 5 ms           0.411922 m
r_x_ref steps larger than 5 cm      197
wheel q_ref max step per 5 ms       5.84495 rad
finite-difference wheel q_ref rate  180.105 rad/s RMS
commanded wheel dqd                 3.54559 rad/s RMS
reference knee minimum              12.469 deg
actual knee minimum                -63.0427 deg
wheel-center relative error RMS     0.213619 m
joint tracking RMS [hip knee wheel] [0.59062 0.67439 3.0386] rad
QP exitflags                        [1]
torque saturation ratio             [0 0.00065369 0.00065369]
```

Observed behavior:

- The floating base remained stable despite large internal wheel/leg motion.
- The wheel-position target repeatedly switched near the reachable-workspace
  boundary.
- The actual knee crossed the straight-leg singularity and entered the
  opposite IK branch even though the reference remained mostly positive.

Diagnosis:

1. The WIPM relation produces an instantaneous equilibrium position, not an
   executable position trajectory. The complete upper feedback force changes
   too quickly to be used directly as a lower position reference.
2. The current geometry maps the saturated `140 N` upper force to roughly
   `4 m`, far outside the leg workspace, so the reference spends most of the
   run on a geometric clamp.
3. `qd` responds to the changing wheel target, while `dqd` and `ddqd` omit the
   corresponding wheel-target velocity and acceleration. The lower PD/QP
   therefore receives mutually inconsistent reference derivatives.
4. A horizontal reference projection cannot guarantee the requested knee
   margin when the vertical hip-to-wheel distance alone exceeds the safe
   reach for that knee angle.
5. `ctrl.qpSolver = "equality"` cannot enforce actual knee-angle or IK-branch
   inequalities. A reference-only guard cannot prevent the real mechanism
   from crossing the singularity.

Why the body can remain stable:

- Base stabilization, rolling/contact consistency, and wheel/leg posture are
  different objectives.
- The upper LQR continues to regulate the base while the soft lower reference
  is sacrificed, producing stable external motion with poor internal motion.

### Decision for the next implementation

Keep Scheme 1 only as a force-conditioned equilibrium generator. Do not pass
its instantaneous output directly to IK. Insert a stateful wheel-position
command governor:

```text
FHx_des
  -> bounded feasible wheel equilibrium r_x_eq
  -> second-order command governor
  -> consistent r_x_des, dr_x_des, ddr_x_des
  -> IK
  -> consistent qd, dqd, ddqd
  -> constrained QP with actual knee/branch protection
```

Recommended initial governor limits:

```text
natural frequency:       0.8 Hz
damping ratio:           1.0
relative wheel speed:    0.4 m/s
relative wheel accel:    2.0 m/s^2
reference knee margin:   25 deg
actual hard knee guard:  10 deg
```

The actual-state guard requires an inequality-capable QP path such as
`quadprog`; changing only `wheelPositionForceScale` is a diagnostic, not the
final correction.

### Data-analysis boundary

This one-off run was generated before the storage boundary was restated. From
this stage onward:

```text
dynamic/2D/simulate/single_wheel_leg
  = model, dynamics, controller implementation, engineering history

calibration/studies/2026_08_force_based_wheel_position_planning
  = work-condition definitions and analysis code

calibration/results/studies/2026_08_force_based_wheel_position_planning
  = raw runs, processed signals, metrics, and automatic plots

calibration/figures/force_based_wheel_position_planning
  = selected long-term figures

calibration/reports
  = stage conclusions intended for reuse or citation
```

Do not add new diagnostic datasets or batch-analysis outputs under the dynamic
model directory.

### Governed Scheme 1 implementation

Implemented after the first closed-loop failure analysis:

- The raw force inversion is replaced by a bounded `tanh` wheel equilibrium
  that preserves the WIPM small-signal slope.
- A stateful critically damped second-order governor produces consistent
  relative wheel position, velocity, and acceleration commands.
- Initial settings are `0.8 Hz`, damping `1.0`, `0.4 m/s`, and `2.0 m/s^2`.
- The reference knee margin is raised to `25 deg`.
- Wheel-world velocity and acceleration now include the governed relative
  wheel derivatives before velocity/acceleration IK and rolling conversion.
- The lower QP now uses `quadprog` and includes a `10 deg` knee acceleration
  guard with a `3 Hz`, critically damped CBF-style inequality.
- The Scope reference path reads the latest governor state without advancing
  it. Only `controller_qp_core` updates the state at the `0.005 s` controller
  sample time.

Data-recording entry point:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true);
```

Each run writes `raw_simulation.mat`, `tracking_data.mat`, and `summary.txt`
under the matching timestamped `calibration/results/studies/...` directory.

Static check:

```matlab
test_force_based_wheel_position_reference
```

Result:

```text
Force-based wheel-position governor check passed.
```

The static check covers governor bounds and derivative consistency, read-only
Scope behavior, finite constrained-QP output, and the knee acceleration guard.
Closed-loop Simulink execution remains pending manual observation and recorded
data from `run_case`.

### Next isolation after the first governed run

The first governed full-motion run still showed an approximately `8.2 cm`
peak-to-peak, `1 Hz` wheel motion during nominal standing. The complete upper
LQR force had `5.09 N` RMS while the bounded map's characteristic force was
only about `6.1 to 7.6 N`, so the force-to-wheel interface remained too
sensitive near equilibrium.

The default force interface scale is reduced from `1.0` to `0.03`. Before
another motion test, run the isolated 5 s standing case:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true, "standing");
```

This change only tests the standing wheel oscillation. No new physical knee
limit is added in the same step, because the first full run also showed a
separate QP-to-Simscape acceleration mismatch that must be isolated after the
force-map sensitivity is verified.

### Standing result and lower-controller isolation

The recorded 5 s standing run `20260808_224537_126` passed the wheel-motion
screen after setting `wheelPositionForceScale = 0.03`:

```text
steady actual wheel-position peak-to-peak:     4.70 mm
steady reference peak-to-peak:                 2.21 mm
maximum absolute base pitch:                 0.0335 rad
actual knee range:                            36.9 to 40.5 deg
geometry infeasibility / torque saturation:   none / none
```

The next run isolates the lower QP-to-plant path. It retains the original
10 s velocity round trip, `quadprog`, and the knee guard, while disabling only
the force-based wheel-position planner:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true, "velocity_round_trip_no_planner");
```

If this case remains stable, the previous moving-case failure is specific to
the added wheel-position task or its coupling with lower tracking. If it still
fails, the lower QP/model mismatch must be corrected before reconnecting the
planner.

### Coupled-run feedback diagnosis and force-source separation

The coupled run `20260808_230501_519` remained bounded but visibly amplified
the recovery motion relative to `20260808_225504_029`, where the planner was
disabled:

```text
recovery pitch RMS:             0.0353 -> 0.0559 rad  (+58.5 %)
recovery pitch peak-to-peak:    0.1696 -> 0.2353 rad  (+38.7 %)
recovery horizontal-force RMS:  8.27   -> 13.63 N     (+64.8 %)
wheel-reference peak-to-peak:   0      -> 60.4 mm
actual wheel peak-to-peak:      172.2  -> 249.0 mm    (+44.6 %)
```

The effect is not dominated by the direct pitch-to-horizontal-force LQR
coefficient. At the maximum pitch, the total body force was `-57.30 N`; the
position and velocity error terms contributed `-25.99 N` and `-29.84 N`,
while the direct pitch term contributed only `-1.47 N`. Pitch, horizontal
motion, and wheel motion are dynamically coupled, so the complete LQR force
still carries the fast stabilization response.

Using that complete force as a wheel-position target creates the additional
delayed loop:

```text
base errors -> total LQR Fx -> wheel governor -> leg/contact motion
            -> base errors
```

Decision: keep the complete LQR wrench in the fast lower-QP path, but replace
the wheel-planning input with the task-level feedforward force:

```text
Fx_plan = m_B * ddx_ref
```

The implementation sets
`traj.wheelPositionForceSource = "reference_acceleration"`. The old
`"total_lqr_force"` source remains available only for reproducing the failed
comparison. The existing `wheelPositionForceScale = 0.03` is intentionally
unchanged so the first validation isolates force-source separation from
amplitude retuning.

### Acceleration knee margin and crouch isolation

The feedforward-force run `20260808_231843_996` reduced the maximum pitch to
`0.1102 rad` and the wheel-reference motion to `4.23 mm` peak-to-peak, but the
actual knee still reached `12.58 deg` at `1.410 s`. This is almost identical
to the planner-disabled baseline, so the remaining knee-margin loss is not
caused primarily by wheel-reference feedback.

At the critical sample, the hip-to-wheel displacement was approximately
`[-195.8, -667.7] mm`. Its length was `695.8 mm`, close to the two-link
maximum reach of `700 mm`. With wheel contact and base height constrained,
horizontal motion therefore drives the knee toward the straight-leg branch.
At the same horizontal displacement, the geometric height reductions needed
for knee angles of `25 deg` and `30 deg` are approximately `12.9 mm` and
`20.5 mm`, respectively.

The next isolation adds a `25 mm` base crouch without changing controller or
wheel-planner parameters:

```text
0.0 to 1.0 s:   quintic smooth descent to -25 mm
1.0 to 6.5 s:   hold the lower base height
6.5 to 7.5 s:   quintic smooth recovery to nominal height
7.5 to 10.0 s:  nominal-height recovery observation
```

Run command:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true, "velocity_round_trip_crouch");
```

### Low default pose and force-scale restoration

The `25 mm` crouch run `20260808_233707_701` raised the actual minimum knee
angle from `12.58 deg` to `28.87 deg` without increasing maximum pitch,
vertical-force variation, or base-height tracking error. This confirms that
lowering the equilibrium pose is an effective way to preserve the
positive-knee workspace during acceleration.

For the next run, the default equilibrium is lowered by `80 mm` relative to
the original `[-19 deg, 38 deg]` stand pose. Startup recomputes the initial
positive-knee joint angles through inverse kinematics and adjusts the
Simscape world offset consistently; the LQR state still starts at its zero
equilibrium, so no artificial z-reference transient is introduced.

At the same time, `wheelPositionForceScale` is increased from `0.03` to
`0.20`. The wheel planner continues to use only
`Fx_plan = m_B * ddx_ref`; the complete LQR feedback force remains confined
to the fast QP path.

Run the ordinary motion case, without the additional crouch profile:

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true, "velocity_round_trip");
```
