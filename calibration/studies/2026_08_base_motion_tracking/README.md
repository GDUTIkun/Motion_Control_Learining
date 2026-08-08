# 2026-08 Base Motion Tracking

## Question

Record a reproducible zero-disturbance run of the single wheel-leg floating
base moving forward and backward at `0.5 m/s`.

## Run

```matlab
cd D:\Workspace\CodeWorkspace\calibration\studies\2026_08_base_motion_tracking
out = run_velocity_round_trip;
```

The runner loads the model startup parameters, applies
`configure_base_tracking_case`, runs the 10 s simulation, and creates:

```text
calibration/results/studies/2026_08_base_motion_tracking/<timestamp>/
  raw_simulation.mat  complete SimulationOutput and logsout
  tracking_data.mat   analysis-ready signals, references, and metadata
```

`tracking_data.mat` contains `runData`:

```text
runData.metadata
runData.signals.base
runData.signals.qpInput
runData.signals.uLqr
runData.signals.tau
runData.signals.X
runData.signals.qRel
runData.signals.dqRel
runData.reference.X
runData.reference.acceleration
runData.columns
```

The raw file is intentionally retained so later analysis can re-extract any
signal already present in `logsout` without rerunning the simulation.
