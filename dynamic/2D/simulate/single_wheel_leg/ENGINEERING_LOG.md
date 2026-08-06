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
