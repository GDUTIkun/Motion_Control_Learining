# 2026-08 Boundary Guard Grid Summary

Batch:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\boundary_guard_grid_20260808_073645
```

Decision table:

```text
D:\Workspace\CodeWorkspace\calibration\data\processed\2026_08_boundary_guard_grid_summary.csv
```

A case is counted as good only when both the original stability criteria pass and `legBranchOk` remains true.

Summary:

```text
                variant                goodCount    stableCount    branchOkCount    good8N    good8p5N    good9N    maxTheta8p5N    maxTheta9N    maxAbsPOx    minAbsSinQk    maxLegDqErr    maxTauSat                                                    resultDir                                                
    _______________________________    _________    ___________    _____________    ______    ________    ______    ____________    __________    _________    ___________    ___________    _________    _________________________________________________________________________________________________________

    "force_0p8_pd_1p5_wqdd_leg_3x"         0             0               0            0          0          0           32557         15015        0.69995     0.00024896        21804           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_075300"
    "wqdd_leg_3x"                          0             0               0            0          0          0           31952         28679        0.69996     0.00031547        29667           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_074047"
    "force_0p8_wqdd_leg_3x"                0             0               0            0          0          0           24166         45130        0.69992      0.0001552        37061           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_074702"
    "round2_baseline"                      0             0               0            0          0          0           51567         45964        0.69995     1.7524e-05        50291           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_073645"
    "force_0p8"                            0             0               0            0          0          0          7111.4         53410        0.69998     3.6525e-06        16089           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_073850"
    "force_0p8_tau_1p2_wqdd_leg_3x"        0             0               0            0          0          0           15019           NaN         0.6998      0.0002906        28535           1        "D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260808_075922"

```

## Result Interpretation

All six variants failed the boundary set:

```text
stableCount    = 0 / 9
branchOkCount  = 0 / 9
goodCount      = 0 / 9
```

This is a stronger result than the original 5 s 10 N pressure-test summary.
With the 8 s stop time and the added leg-branch metrics, even the `8 N`
boundary cases eventually expose the same failure mechanism.

The common geometric signature is:

```text
maxAbsPOxPost    ~= 0.70 m
minAbsSinQkPost  ~= 0
```

Since `L1 + L2 = 0.70 m`, `maxAbsPOxPost ~= 0.70 m` means the leg is being
dragged close to full horizontal extension relative to the hip. Since the
two-link Jacobian determinant is proportional to `sin(qk)`, `minAbsSinQkPost`
near zero means the leg passes near a singular configuration.

For the baseline `0 deg / 8 N` case, the event order is:

```text
t ~= 2.725 s: |pO_x| > 0.25 m
t ~= 2.795 s: |theta| > 15 deg
t ~= 2.820 s: qk branch/singularity guard trips
```

So the leg-geometry failure begins immediately after the pulse recovery starts,
not only at the end of the 8 s simulation.

## Decision

Do not continue the current parameter-only grid:

```text
QR tuning
Fx/Fz limit scaling
QP qdd soft-weight scaling
small torque-limit scaling
```

These soft changes did not prevent branch loss. The next model/controller
change should add explicit leg-configuration protection:

```text
qk lower/upper guard
relative wheel-center x guard
or physical joint limits/contact-safe stop
```

The current equality-QP path cannot enforce inequality guards, so a proper
solution likely needs either:

```text
1. switch boundary experiments to full quadprog with joint/branch inequalities
2. add a pre-QP reference projection / command governor that keeps qd feasible
3. add physical Simscape joint limits for the learning model
```
