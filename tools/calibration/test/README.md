# Calibration tests

Regression test for the calibration tools, run in CI
(`.github/workflows/calibration-verify.yml`) in a plain Python environment — no
Isaac Sim, no robot.

## Files

- `Rizon4_calibrated_kinematics.example.yaml` — an example calibrated kinematics
  YAML pulled from a real Rizon4 via RDK (`Model.SyncKinematicsYAML`).
- `verify_calibration_against_urdf.py` — compares a calibrated USD against a URDF
  by forward-kinematics world poses.
- `run_ci_verification.py` — the CI entry point.

## What CI checks

1. **Always:** validates the example YAML (structure, joints, numeric fields).
2. **When a reference URDF is present:** applies the example calibration to a base
   USD and cross-checks the result against the URDF.

Step 2 is **skipped** until two inputs are provided:

- `Rizon4_calibrated.urdf` in this folder — a real calibrated URDF for the same
  robot as the example YAML (obtained from RDK `Model.SyncURDF`).
- A base Rizon4 USD to apply onto, via the `CALIBRATION_TEST_BASE_USD` env var
  (the repo does not ship USD assets).

Once both exist, the comparison runs automatically and fails the job on a
mm-scale mismatch.

## Run locally

```
python run_ci_verification.py
# or with Isaac Sim's Python and a base USD to exercise the full path:
CALIBRATION_TEST_BASE_USD=/path/to/Rizon4/Rizon4.usda \
    ~/isaacsim/kit/python/bin/python3 run_ci_verification.py
```
