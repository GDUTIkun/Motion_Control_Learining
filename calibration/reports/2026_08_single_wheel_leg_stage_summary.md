# 2026-08 Single Wheel-Leg Stage Summary

## Current Goal

The current learning model is a 2D floating-base + single wheel-leg inverted
pendulum:

```text
upper layer: floating-base LQR -> [FHx_ext; FHz_ext; MBy_des]
lower layer: QP inverse dynamics -> [tau_h; tau_k; tau_w]
plant:       Simscape continuous contact model
```

The active stage is stationary disturbance recovery. The practical target is
not final performance, but understanding whether poor behavior comes from the
upper LQR, the lower QP, contact/timing, or missing constraints.

## Main Finding

The plant and basic controller chain are not completely broken. Small
disturbances and some manually observed `8 N / 8.5 N` cases show that the
system can recover.

The current bottleneck is the missing coordination between:

```text
upper-layer floating-base recovery
lower-layer leg workspace / IK branch feasibility
```

The upper LQR only sees base `x/z/theta`. It can command a pitch recovery that
is reasonable for the simplified floating-base model but unsafe for the
current leg configuration.

The lower QP executes the wrench and joint tracking tradeoff, but the current
fast equality-QP path has no hard guard for:

```text
qk range
wheel-center position relative to hip, pO_x
IK branch / singularity avoidance
```

So the failure should not be classified as just "QP not tuned" or just "LQR
bad". The control architecture is missing feasibility/workspace protection at
the interface between the two layers.

## Observed Failure Mechanism

The common failure chain is:

```text
disturbance increases pitch error
-> LQR rapidly pulls pitch back toward upright
-> pitch crosses upright with too much recovery energy
-> thigh/shank are dragged through a large motion
-> qk approaches singularity or the leg changes branch
-> contact and joint dynamics become stiff/unstable
-> system loses balance
```

This matches the visual observation: pitch snaps back, then the leg is pulled
to the other side or near full extension.

## Evidence From Experiments

Several parameter-only tests were tried:

```text
Bayesian QR tuning
larger force/torque limits
Qx/Qdx rollback
FHx output shaping
smaller Fx/Fz limits
stronger QP soft qdd tracking
small torque-limit increase
```

Results:

```text
small cases: still generally pass
10 N pressure cases: still fail
boundary 8/8.5/9 N grid with strict branch metrics: no soft variant prevented branch loss
```

The boundary grid showed a repeated geometric signature:

```text
maxAbsPOxPost    ~= 0.70 m
minAbsSinQkPost  ~= 0
```

Since `L1 + L2 = 0.70 m`, the leg is being pulled close to full extension.
Since the wheel-center Jacobian determinant is proportional to `sin(qk)`, the
leg is also passing close to a singular configuration.

For one representative baseline case:

```text
0 deg / 8 N:
  t ~= 2.725 s: |pO_x| > 0.25 m
  t ~= 2.795 s: |theta| > 15 deg
  t ~= 2.820 s: qk branch/singularity guard trips
```

The leg-geometry problem starts soon after pulse recovery begins, not only at
the end of simulation.

## Correct Interpretation

Current answer to "why does it not stabilize?":

```text
primary trigger: upper LQR recovery is too aggressive near pitch return
primary missing protection: lower layer has no hard leg-configuration guard
root issue: upper wrench command is not constrained by lower feasible workspace
```

The QP is not merely badly tuned. Its problem formulation is incomplete for
this architecture because the fast equality solver cannot enforce the
inequality guards that matter here.

The LQR is not merely "wrong" either. It is doing what the simplified
floating-base model asks, but that model does not include leg branch or contact
workspace limits.

## Pause Decision

Stop broad simulation scans for now. The next round should not continue blind:

```text
QR tuning
Fx/Fz limit scaling
soft QP weight scaling
small torque-limit scaling
```

These can change symptoms but do not guarantee branch safety.

## Recommended Next Design Work

When returning to this problem, prioritize two design changes.

First, reduce upper-layer recovery aggressiveness:

```text
add recovery energy shaping near pitch zero crossing
limit/rate-limit FHx_ext and/or MBy_des during recovery
increase R for aggressive wrench channels if needed
consider a recovery mode that emphasizes damping before position recentering
```

Second, add explicit leg-configuration protection:

```text
protect qk away from 0 deg and 180 deg
limit wheel-center horizontal offset relative to hip, pO_x
keep the IK branch consistent
add Simscape joint limits for the learning model if needed
```

Implementation options, from light to formal:

```text
1. reference projection / command governor before QP
2. physical Simscape joint limits
3. switch boundary experiments to full quadprog with inequality guards
4. later, formulate a simplified constrained NMPC once the safe workspace is clear
```

NMPC may eventually fit this problem better, but it should come after the safe
workspace and constraints are defined. Otherwise it will hide the same issue
inside a larger optimizer.
