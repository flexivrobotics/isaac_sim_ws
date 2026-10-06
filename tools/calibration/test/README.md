# Calibration tests

Regression test for the calibration tools, run in CI
(`.github/workflows/calibration-verify.yml`) in a plain Python environment — no
Isaac Sim, no robot.

## Files

The Rizon 4s example files come from one real robot (a Rizon 4s, hardware type
A02LS-P2), pulled via two RDK paths, so they carry the same calibration:

- `Rizon4_calibrated_kinematics.example.yaml` — from `Model.SyncKinematicsYAML`.
- `Rizon4_calibrated.example.urdf` — from `Model.SyncURDF`; the reference the
  applied USD is checked against.

The Enlight LL example files come from one simulated Enlight LL in Elements
Studio, whose arm adapters place the arms at y = ±0.2 m:

- `EnlightLL_calibrated_kinematics.example.yaml` — from `Model.SyncKinematicsYAML`
  on the Enlight-LL template with the `EXT_AXIS` arm adapters added.
- `EnlightLL_calibrated.example.urdf` — the URDF its controller generated, with
  the arm adapters applied; the reference the applied USD is checked against.

Scripts:

- `verify_calibration_against_urdf.py` — compares a calibrated USD against a URDF
  by forward-kinematics world poses.
- `run_ci_verification.py` — the CI entry point.

## What CI checks

For the Rizon 4s and then the Enlight LL:

1. Validates the example YAML (structure, joints, numeric fields).
2. Applies the example calibration to the in-repo asset
   (`exts/.../data/flexiv/rizon_4s` or `enlight_ll`) and cross-checks the result
   against the example URDF, failing on a mm-scale mismatch. For the Enlight LL
   this checks both arms, and so where each arm adapter mounts its arm.
3. Checks that every joint of the calibrated USD is anchored where its links are,
   since the link poses alone do not show a wrong joint anchor.

The Enlight LL is checked a second time with rotated arm adapters, written into a
temporary copy of both example files, since the example's adapters only
translate.

Override the base assets with the `CALIBRATION_TEST_BASE_USD` (Rizon 4s) and
`CALIBRATION_TEST_LL_BASE_USD` (Enlight LL) env vars if needed.

## Run locally

```
python run_ci_verification.py
# or with Isaac Sim's Python, against the installed assets instead of the repo's:
CALIBRATION_TEST_BASE_USD=/path/to/rizon_4s/rizon_4s.usda \
CALIBRATION_TEST_LL_BASE_USD=/path/to/enlight_ll/enlight_ll.usda \
    ~/isaacsim/kit/python/bin/python3 run_ci_verification.py
```
