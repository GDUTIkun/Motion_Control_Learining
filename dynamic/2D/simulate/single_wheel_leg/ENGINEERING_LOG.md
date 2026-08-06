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
- The pitch integral term is still a temporary patch.
- This is still stage 1; the wheel is not yet being used as an intentional
  horizontal balancing degree of freedom.

Current best explanation:

```text
Main old issue:
  lower leg reference fought the floating-base geometry.

Remaining issue:
  lower QP/contact/no-slip/wheel-spin reference still have a residual
  consistency problem, especially in the wheel coordinate.
```

## Recommended Next Steps

Do not blindly tune LQR `Q/R` yet.

Next diagnostic target:

```text
wheel spin reference vs rolling contact constraint
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
