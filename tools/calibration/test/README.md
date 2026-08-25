# Calibration tests

Regression test for the calibration tools, run in CI
(`.github/workflows/calibration-verify.yml`) in a plain Python environment — no
Isaac Sim, no robot.

## Files

Both example files come from the same real Rizon4 (serial A02LS-P2), pulled via
two RDK paths, so they carry the same calibration:

- `Rizon4_calibrated_kinematics.example.yaml` — from `Model.SyncKinematicsYAML`.
- `Rizon4_calibrated.example.urdf` — from `Model.SyncURDF`; the reference the
  applied USD is checked against.
- `verify_calibration_against_urdf.py` — compares a calibrated USD against a URDF
  by forward-kinematics world poses.
- `run_ci_verification.py` — the CI entry point.

## What CI checks

1. **Always:** validates the example YAML (structure, joints, numeric fields).
2. **When a base USD is available:** applies the example calibration to it and
   cross-checks the result against the example URDF.

Step 2 needs a base Rizon4 USD to apply onto, via the `CALIBRATION_TEST_BASE_USD`
env var (the repo does not ship USD assets, so it is skipped in CI until one is
provided). When it runs, the job fails on a mm-scale mismatch.

## Run locally

```
python run_ci_verification.py
# or with Isaac Sim's Python and a base USD to exercise the full path:
CALIBRATION_TEST_BASE_USD=/path/to/Rizon4/Rizon4.usda \
    ~/isaacsim/kit/python/bin/python3 run_ci_verification.py
```
