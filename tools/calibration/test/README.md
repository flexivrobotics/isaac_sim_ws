# Calibration tests

Regression test for the calibration tools, run in CI
(`.github/workflows/calibration-verify.yml`) in a plain Python environment — no
Isaac Sim, no robot.

## Files

Both example files come from the same real robot (a Rizon4s, serial A02LS-P2),
pulled via two RDK paths, so they carry the same calibration:

- `Rizon4_calibrated_kinematics.example.yaml` — from `Model.SyncKinematicsYAML`.
- `Rizon4_calibrated.example.urdf` — from `Model.SyncURDF`; the reference the
  applied USD is checked against.
- `verify_calibration_against_urdf.py` — compares a calibrated USD against a URDF
  by forward-kinematics world poses.
- `run_ci_verification.py` — the CI entry point.

## What CI checks

1. Validates the example YAML (structure, joints, numeric fields).
2. Applies the example calibration to the in-repo Rizon4s asset
   (`exts/.../data/flexiv/Rizon4s`) and cross-checks the result against the
   example URDF, failing on a mm-scale mismatch.

Override the base asset with the `CALIBRATION_TEST_BASE_USD` env var if needed.

## Run locally

```
python run_ci_verification.py
# or with Isaac Sim's Python and a base USD to exercise the full path:
CALIBRATION_TEST_BASE_USD=/path/to/Rizon4/Rizon4.usda \
    ~/isaacsim/kit/python/bin/python3 run_ci_verification.py
```
