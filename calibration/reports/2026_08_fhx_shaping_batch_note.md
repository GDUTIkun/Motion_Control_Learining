# 2026-08 FHx Shaping Batch Note

## Batch

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\fhx_shaping_20260807_191943
```

## Result

The batch is not valid for judging `FHx_ext` shaping.

Observed:

```text
fhx_filter_30ms summary == round-2 best summary
fhx_rate_2000 summary   == round-2 best summary
```

Signal check confirmed:

```text
max abs uLqr diff round2_best vs fhx_filter_30ms = 0
max abs uLqr diff round2_best vs fhx_rate_2000   = 0
```

So the first two variants did not actually apply shaping.

The third variant:

```text
qdx_2p5_fhx_filter_30ms
```

changed behavior mainly because `Q_dx` was changed to `2.5x`; the `FHx_ext`
filtering part should not be credited from this batch.

## Cause

`run_cases` applied `lqrParams.commandShaping`, but each case then called
`set_initial_base_state`, which rebuilt `baseLqr` using
`floating_base_lqr_design(base)`.

Before the fix, `floating_base_lqr_design` did not copy
`base.commandShaping` into `baseLqr`, so shaping was lost before simulation.

## Fix

`floating_base_lqr_design.m` now preserves:

```matlab
baseLqr.commandShaping = getFieldOrDefault(base, "commandShaping", struct());
```

Verification:

```text
After startup + base.commandShaping + set_initial_base_state,
baseLqr.commandShaping.enabled remains true.
```

## Action

Re-run:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response")
batch = validate_fhx_shaping_9cases( ...
    "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Only the new batch after this fix should be used to decide whether
`FHx_ext` filtering/rate limiting helps.

## Post-Fix Batch

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\fhx_shaping_20260807_195550
```

Source controller:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811
```

Result:

```text
fhx_filter_30ms:          6 / 9 stable
fhx_rate_2000:            6 / 9 stable
qdx_2p5_fhx_filter_30ms:  6 / 9 stable
```

The normal 0 N / 5 N guardrail cases still pass, so mild FHx shaping does not
break the current nominal working envelope.

10 N pressure-test comparison against round-2 best:

```text
round2_best:
  -10 deg: max |theta| ~= 66.5 deg,  final theta ~= -16.1 deg
   0 deg:  max |theta| ~= 65.1 deg,  final theta ~= -44.3 deg
  +10 deg: max |theta| ~= 652 deg,   final theta ~= -196 deg

fhx_filter_30ms:
  -10 deg: max |theta| ~= 197 deg,   final theta ~= -89.9 deg
   0 deg:  max |theta| ~= 405 deg,   final theta ~= -318 deg
  +10 deg: max |theta| ~= 174 deg,   final theta ~= 151 deg

fhx_rate_2000:
  -10 deg: max |theta| ~= 1496 deg,  final theta ~= -1470 deg
   0 deg:  max |theta| ~= 62.3 deg,  final theta ~= 11.6 deg
  +10 deg: max |theta| ~= 581 deg,   final theta ~= -395 deg

qdx_2p5_fhx_filter_30ms:
  -10 deg: max |theta| ~= 1872 deg,  final theta ~= -1602 deg
   0 deg:  max |theta| ~= 2630 deg,  final theta ~= 49.6 deg
  +10 deg: max |theta| ~= 426 deg,   final theta ~= -303 deg
```

Recovery-event check:

```text
round2_best:
  theta at pulse end ~= 10 deg
  dtheta at theta zero ~= -1.7 to -2.0 rad/s

fhx_filter_30ms:
  theta at pulse end ~= 10.1 to 10.4 deg
  dtheta at theta zero ~= -3.0 rad/s

qdx_2p5_fhx_filter_30ms:
  dtheta at theta zero ~= -2.6 to -2.7 rad/s
```

Interpretation:

- Fixed FHx filtering/rate limiting is not a stable improvement.
- It can improve one 10 N direction while making the opposite direction much
  worse.
- The likely issue is not only command smoothness; it is recovery-phase energy
  management and feasible wrench allocation.

Decision:

- Do not keep the current FHx shaping variants as the next baseline.
- Do not continue coarse `Q_dx` rollback plus fixed FHx filtering.
- Next experiment should test reducing the upper `Fx/Fz` authority, because the
  normal 0 N / 5 N envelope does not require such large forces and the 10 N
  failures show too much recovery energy after pitch passes upright.

Prepared helper:

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response")
batch = validate_force_limit_grid_9cases( ...
    "D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811");
```

Default variants:

```text
forceScale = 0.8, tauScale = 1.0, momentScale = 1.0
forceScale = 0.6, tauScale = 1.0, momentScale = 1.0
forceScale = 0.4, tauScale = 1.0, momentScale = 1.0
forceScale = 0.6, tauScale = 0.8, momentScale = 1.0
```

This does not mathematically guarantee lower-layer feasibility, because QP
feasibility also depends on leg configuration, contact, torque limits, and the
unchanged pitch-moment target. It should reduce infeasible or overly aggressive
force requests and make the recovery transient easier to diagnose.
