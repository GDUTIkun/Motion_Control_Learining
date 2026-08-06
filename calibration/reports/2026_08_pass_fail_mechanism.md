# Pass/Fail Mechanism Analysis

Result directory:

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\20260806_195701
```

## Passing representative

- Case: `pitch_10deg_pulse_5N`
- Stable: 1
- Failure reason: ok
- max |theta|: 10 deg
- final theta: 0.0923 deg
- final dtheta: 0.00222 rad/s
- final x: -0.054 m
- tau saturation ratio: 0.0691
- max |FHx|: 37.5 N
- max |FHz|: 60.6 N
- max |MBy|: 11.1 N*m
- first theta > 15 deg: NaN s
- first |x| > 0.5 m: NaN s
- first tau saturation: NaN s

## Failing representative

- Case: `pitch_10deg_pulse_10N`
- Stable: 0
- Failure reason: max theta too large; final theta not settled; final dtheta not settled; tau near saturation; x drift too large
- max |theta|: 1.68e+03 deg
- final theta: -1.17e+03 deg
- final dtheta: 76 rad/s
- final x: -3.66 m
- tau saturation ratio: 1
- max |FHx|: 140 N
- max |FHz|: 140 N
- max |MBy|: 160 N*m
- first theta > 15 deg: 2.98 s
- first |x| > 0.5 m: 3.5 s
- first tau saturation: 2.8 s

## Comparison

- Delta max |theta|: 1.67e+03 deg
- Delta final x: -3.6 m
- Delta tau saturation ratio: 0.931
- First detected failing limit: tau saturation

## Interpretation

The 10 N pulse case crosses the accepted pitch/x envelope and later reaches torque saturation. The 5 N pulse case remains inside the envelope with large torque margin. This suggests the first tuning pass should compare LQR horizontal-force use, x-drift penalty, and pitch damping before changing architecture.

More detailed timing for `pitch_10deg_pulse_10N`:

```text
t = 2.50 s, pulse end:
  theta  ~= 10.95 deg
  dtheta ~= 0.323 rad/s
  x      ~= 0.095 m
  uLqr   ~= [105.65, -91.60, 0.75]

t ~= 2.755 s:
  first upper-command saturation appears in FHz_ext

t ~= 2.80 s:
  theta  ~= -1.32 deg
  dtheta ~= -3.31 rad/s
  x      ~= 0.044 m
  uLqr   ~= [-140, -140, 35.65]
  wheel torque reaches its limit in the logged data around this interval

t = 3.00 s:
  theta  ~= -6.86 deg
  dtheta ~= 7.40 rad/s
  knee torque is saturated
```

This means the failed case is not simply "too much pitch angle at pulse end."
The dangerous part is the recovery transient after the pulse: angular velocity
reverses hard, upper force commands hit limits, and QP/actuator saturation
follows.

## Tuning Hypotheses

1. Do not globally increase all LQR state weights. The failure already involves
   force/torque saturation.
2. Try shifting pitch recovery effort toward the moment channel earlier:
   lower the LQR `R` weight for `MBy` and/or increase `Qdtheta` moderately.
   The goal is more pitch damping before the force channels saturate.
3. Reduce overuse of force channels if needed:
   increase the LQR `R` weight for `FHx/FHz`, especially the force channel that
   saturates first, and watch whether x drift and theta recovery improve or
   worsen.
4. If QP torque saturation remains the first hard limit, adjust QP priorities
   so pitch moment tracking and feasible torque usage are balanced rather than
   trying to eliminate all joint tracking error.
5. Keep the fixed 9-case acceptance set as the regression baseline.
