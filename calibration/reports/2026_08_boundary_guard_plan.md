# 2026-08 Boundary Guard Plan

## Current Observation

The disturbance boundary is now roughly:

```text
8 N    pulse: recovers
8.5 N  pulse: near boundary; large pitch oscillation, possible leg reversal
9 N    pulse: loses balance
```

The visual failure mode is not a simple fall. Pitch rapidly returns toward
upright, then the thigh/shank are dragged into a near-singular or opposite-side
configuration.

## Interpretation

The current bottleneck is likely recovery-phase coupling:

```text
upper LQR pitch recovery
  -> aggressive hip/thigh motion
  -> actual leg approaches IK singularity or changes branch
  -> lower QP/contact can no longer preserve the intended stage-1 geometry
```

For the two-link wheel-leg, the wheel-center Jacobian determinant is
proportional to:

```text
sin(qk)
```

So `qk` near `0 deg` or `180 deg` is singular. The nominal stance is
approximately:

```text
qh = -19 deg
qk =  38 deg
```

## Prepared Experiments

Run the boundary guard grid:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response")
batch = validate_boundary_guard_grid( ...
    "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Cases:

```text
theta0 = [-10, 0, 10] deg
pulse  = [8, 8.5, 9] N
stop   = 8 s
```

Variants:

```text
round2_baseline
force_0p8
wqdd_leg_3x
force_0p8_wqdd_leg_3x
force_0p8_pd_1p5_wqdd_leg_3x
force_0p8_tau_1p2_wqdd_leg_3x
```

## Post-Run Analysis

After the batch finishes:

```matlab
analysis = analyze_boundary_guard_batch(batch.batchDir);
```

Optional geometry diagnostic on selected result directories:

```matlab
analysis = analyze_leg_branch_geometry(resultDirs, labels, caseNames);
```

## What To Look For

A candidate is useful only if both conditions hold:

```text
stable == true
legBranchOk == true
```

Key columns:

```text
legBranchOk
minAbsSinQkPost
minQkDegPost / maxQkDegPost / finalQkDeg
maxAbsPOxPost / finalPOx
maxLegQErrorPost / maxLegDqErrorPost
maxAbsThetaDeg / finalThetaDeg
tauSaturationRatio
```

Decision:

- If `wqdd_leg_3x` or its combined variants improve branch health, make leg
  configuration tracking a formal QP priority.
- If `force_0p8` helps but does not prevent branch loss, keep force authority
  reduction as a guardrail and add explicit leg branch protection.
- If `tau_1p2` helps only together with stronger QP tracking, recovery was
  torque-limited at the lower layer.
