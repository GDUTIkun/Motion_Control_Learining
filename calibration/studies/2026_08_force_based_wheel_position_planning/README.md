# 2026-08 Force-Based Wheel-Position Planning

## Question

Can the final horizontal body force from the existing floating-base LQR be
converted into a feasible wheel-position reference for the 2D single
wheel-leg model without adding another outer LQR?

The current revision separates the planner input from the fast stabilization
feedback. The lower QP still receives the complete LQR wrench, while wheel
placement uses only the task-level feedforward force
`Fx_plan = m_B * ddx_ref`.

Current default validation settings use an initial equilibrium height `80 mm`
below the original `[-19 deg, 38 deg]` pose and
`wheelPositionForceScale = 0.20`. The ordinary `"velocity_round_trip"` case
starts directly from this lower pose; it does not add a vertical transient.

The system under test remains in:

```text
D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg
```

This study owns the work conditions, data extraction, metrics, comparisons,
and tuning related to force-based wheel-position planning.

## Run

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_force_based_wheel_position_planning
out = run_case(true, "standing");
```

Pass `true` to open the visual model during the recorded run. Use
`out = run_case(false, "standing")` for a headless run. After the standing
case is accepted, first isolate the lower controller with the original motion
trajectory but no force-based wheel-position planning:

```matlab
out = run_case(true, "velocity_round_trip_no_planner");
```

This keeps `quadprog`, the knee guard, and the upper base trajectory enabled.
Only `traj.wheelPositionPlanning` is disabled. Do not run the fully coupled
`"velocity_round_trip"` case until this isolation result has been analyzed.

Send `out.resultDir` for analysis after the run. Each run writes:

```text
calibration/results/studies/2026_08_force_based_wheel_position_planning/<timestamp>/
  raw_simulation.mat  complete SimulationOutput and logsout
  tracking_data.mat   controller samples, reconstructed wheel plan, and metrics
  summary.txt         compact run summary
```

The reconstructed wheel data includes the actual, equilibrium, governed,
velocity, and acceleration references together with workspace bounds,
geometry feasibility, and actual/reference joint states.

To isolate additional knee workspace from a lower base height, run the
`25 mm` crouch case:

```matlab
out = run_case(true, "velocity_round_trip_crouch");
```

It descends smoothly during `0-1 s`, holds the lower height through the
horizontal motion, and recovers during `6.5-7.5 s`. All controller and wheel
planner settings remain unchanged.

## Initial Scheme

The first implementation used the instantaneous WIPM equilibrium relation:

```text
FBody_x = -FH_ext_x
a_Bx_des = FBody_x / m_B
r_x_raw = r_x_eq - H/g * a_Bx_des
```

The result was projected onto a reference-level positive-knee workspace and
converted to joint references through inverse kinematics.

## Recorded Work Condition

```text
duration:              10 s
sample time:           0.005 s
initial base state:    zeros(6,1)
external disturbances: disabled
trajectory:            +0.5 m/s forward, stop, -0.5 m/s return
```

Trajectory phases:

```text
0.0 to 1.0 s: hold
1.0 to 1.5 s: accelerate forward
1.5 to 3.0 s: forward cruise
3.0 to 3.5 s: stop
3.5 to 4.0 s: hold
4.0 to 4.5 s: accelerate in reverse
4.5 to 6.0 s: reverse cruise
6.0 to 6.5 s: stop at the initial position
6.5 to 10.0 s: recovery
```

## First Closed-Loop Result

One-off run timestamp:

```text
20260808_213358
```

Key measurements:

```text
final pitch                         0.012463 rad
body FHx range                     [-140, 140] N
CoM-to-wheel height                [0.7513, 0.8542] m
raw wheel-equilibrium range        [-4.0608, 4.0560] m
projection/clamp ratio             89.3 %
max wheel-reference step           0.411922 m per 5 ms
max wheel-joint reference step     5.84495 rad per 5 ms
wheel-reference rate from position 180.105 rad/s RMS
commanded wheel dqd                3.54559 rad/s RMS
minimum reference knee angle       12.469 deg
minimum actual knee angle         -63.0427 deg
wheel-center relative RMS error    0.213619 m
QP exitflags                       [1]
```

The base remained stable, but the wheel target repeatedly hit the workspace
boundary and the real knee crossed the straight-leg singularity.

## Current Conclusion

The force relation is useful as an instantaneous wheel-equilibrium direction,
but its raw output is not an executable lower-layer reference in the current
architecture. The missing pieces are:

1. bounded force-to-equilibrium shaping;
2. a stateful wheel-position command governor;
3. consistent position, velocity, and acceleration references;
4. a vertical-geometry feasibility check;
5. an actual knee/IK-branch inequality in the lower QP.

The next implementation should keep the existing upper LQR and insert:

```text
FHx_des
  -> feasible r_x_eq
  -> second-order governor
  -> r_x_des, dr_x_des, ddr_x_des
  -> IK and constrained QP
```

Suggested starting values:

```text
governor natural frequency: 0.8 Hz
damping ratio:              1.0
relative speed limit:       0.4 m/s
relative acceleration:      2.0 m/s^2
reference knee margin:      25 deg
actual knee guard:          10 deg
```

## Storage Rule

All subsequent analysis for this topic goes under `calibration`:

```text
calibration/studies/2026_08_force_based_wheel_position_planning/
  reproducible run and analysis code

calibration/results/studies/2026_08_force_based_wheel_position_planning/<timestamp>/
  raw_simulation.mat
  tracking_data.mat
  metrics or summaries
  automatically generated plots

calibration/figures/force_based_wheel_position_planning/
  selected figures worth keeping
```

Do not place new datasets, plots, or batch-analysis outputs in the dynamic
model directory. Reusable experiment machinery belongs in
`calibration/experiments`, while reusable extraction or plotting functions
belong in `calibration/tools`.
