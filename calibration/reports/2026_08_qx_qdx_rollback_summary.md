# 2026-08 Qx/Qdx Rollback Summary

## Result Batch

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\qx_qdx_rollback_20260807_190201
```

Source controller:

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr_round2\20260806_224811
```

The experiment keeps all round-2 best LQR `Q/R` entries unchanged except
`Q_x` and `Q_dx`.

## Variants

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

Reference:

```text
round-2 best:
  Q_x  ~= 0.60557x
  Q_dx ~= 3.39x
```

## Pass/Fail Result

All variants stayed at:

```text
6 / 9 stable
```

The working envelope did not expand:

```text
0 N cases passed
5 N cases passed
10 N cases failed
```

## 5 N Guardrails

The 5 N guardrails remain acceptable for all rollback variants.

Compared with round-2 best:

```text
round2_best:
  max final |theta| ~= 0.120 deg
  max |finalX| ~= 0.083 m
  max tau ratio ~= 0.078
  max legVelocityRms ~= 0.264 rad/s

qdx_2p0:
  max final |theta| ~= 0.104 deg
  max |finalX| ~= 0.094 m
  max tau ratio ~= 0.078
  max legVelocityRms ~= 0.249 rad/s

qdx_1p0:
  max final |theta| ~= 0.083 deg
  max |finalX| ~= 0.088 m
  max tau ratio ~= 0.078
  max legVelocityRms ~= 0.228 rad/s

qx_0p3_qdx_1p0:
  max final |theta| ~= 0.081 deg
  max |finalX| ~= 0.113 m
  max tau ratio ~= 0.076
  max legVelocityRms ~= 0.208 rad/s
```

So normal 0 N / 5 N standing is not very sensitive to this rollback.

## 10 N Pressure Cases

10 N comparison:

```text
round2_best:
  -10 deg: max |theta| ~= 66.5 deg,  final theta ~= -16.1 deg
   0 deg:  max |theta| ~= 65.1 deg,  final theta ~= -44.3 deg
  +10 deg: max |theta| ~= 652 deg,   final theta ~= -196 deg

qdx_2p0:
  -10 deg: max |theta| ~= 224 deg,   final theta ~= -194 deg
   0 deg:  max |theta| ~= 576 deg,   final theta ~= 113 deg
  +10 deg: max |theta| ~= 126 deg,   final theta ~= -126 deg

qdx_1p0:
  -10 deg: max |theta| ~= 461 deg,   final theta ~= 418 deg
   0 deg:  max |theta| ~= 8024 deg,  final theta ~= 7342 deg
  +10 deg: max |theta| ~= 162 deg,   final theta ~= 71.2 deg

qx_0p3_qdx_1p0:
  -10 deg: max |theta| ~= 1785 deg,  final theta ~= -4.5 deg
   0 deg:  max |theta| ~= 2380 deg,  final theta ~= -2380 deg
  +10 deg: max |theta| ~= 3377 deg,  final theta ~= -3377 deg
```

`qdx_2p0` improves the `+10 deg / 10 N` pressure case relative to round-2 best,
but it degrades the `-10 deg` and `0 deg` pressure cases. Lowering `Q_dx` all
the way to `1.0x` is clearly too weak for 10 N recovery.

## Channel Interpretation

Lowering `Q_dx` does reduce `FHx` saturation fraction somewhat, but it does not
produce a cleaner recovery. Instead, the controller shifts burden into `FHz`
and sometimes `MBy`:

```text
round2_best, -10/0 deg 10 N:
  FHx saturated about 0.61 to 0.62 of the recovery window
  FHz and MBy not saturated

qdx_2p0:
  FHx saturated about 0.56 to 0.59
  FHz starts saturating about 0.09 to 0.16
  MBy remains below saturation

qdx_1p0:
  FHx saturated about 0.46 to 0.50
  FHz saturation increases
  MBy reaches saturation in some samples
```

Interpretation:

```text
Q_dx was high for a reason: it helps keep horizontal velocity from accumulating
too much during recovery. Reducing it softens FHx demand, but it can let the
state drift into a worse recovery region and push the controller into other
channels.
```

## Decision

Do not use the `Q_dx = 1.0x` rollback.

`Q_dx = 2.0x` is a useful diagnostic point, but not clearly better than
round-2 best because it trades one bad 10 N case for two worse 10 N cases.

Recommended next step:

```text
Do not continue coarse Q_x/Q_dx rollback.
Try a narrower combined design:
  Q_dx between 2.5x and 3.4x
  plus mild FHx_ext rate limiting/filtering
```

Reason:

```text
Purely reducing Q_dx is not enough. The controller still hits saturation, and
the energy moves into FHz/MBy or into delayed horizontal drift. A mild output
shaping layer can target the abrupt FHx recovery without removing too much
horizontal damping from the LQR.
```
