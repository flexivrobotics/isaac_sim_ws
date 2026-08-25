#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# CI entry point for the calibration verification. Runs in a plain Python env
# (no Isaac Sim, no robot) with pip-installed usd-core / numpy / pyyaml.
#
# What it does:
#   1. Validate the example calibrated kinematics YAML parses and has the
#      expected joint set.
#   2. Apply the example YAML to the in-repo Rizon4s asset, then run
#      verify_calibration_against_urdf.py to confirm the calibrated USD matches
#      the reference URDF. Skipped (with a clear message) only if the reference
#      URDF or base USD is missing -- see REFERENCE_URDF / DEFAULT_BASE_USD.
#
# Exit codes: 0 = pass (or skipped), non-zero = failure.

import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

# The example calibrated kinematics pulled from a real robot via RDK
# (Model.SyncKinematicsYAML). Structure/joint names are asserted below.
EXAMPLE_YAML = os.path.join(HERE, "Rizon4_calibrated_kinematics.example.yaml")

# The reference calibrated URDF for the example robot (RDK Model.SyncURDF).
REFERENCE_URDF = os.path.join(HERE, "Rizon4_calibrated.example.urdf")

# Base USD to apply the calibration onto. The example robot (serial A02LS-P2) is
# a Rizon4s, so the calibration is applied onto the in-repo Rizon4s asset (which
# is tracked, so it is available in CI). Override with BASE_USD_ENV if needed.
BASE_USD_ENV = "CALIBRATION_TEST_BASE_USD"
_REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_BASE_USD = os.path.join(
    _REPO_ROOT,
    "exts",
    "isaacsim.robot.manipulators.examples",
    "data",
    "flexiv",
    "Rizon4s",
    "Rizon4s.usda",
)

EXPECTED_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "link7_to_flange",
]
REQUIRED_FIELDS = {"x", "y", "z", "roll", "pitch", "yaw"}


def check_example_yaml():
    """Validate the example calibrated kinematics YAML. Raises on any problem."""
    print(f"[ci] Validating example YAML: {EXAMPLE_YAML}")
    with open(EXAMPLE_YAML) as f:
        doc = yaml.safe_load(f)
    kine = (doc or {}).get("kinematics")
    if not kine:
        raise ValueError("example YAML has no top-level 'kinematics' node")
    missing = [j for j in EXPECTED_JOINTS if j not in kine]
    if missing:
        raise ValueError(f"example YAML missing joints: {missing}")
    for j in EXPECTED_JOINTS:
        have = set(kine[j])
        if not REQUIRED_FIELDS.issubset(have):
            raise ValueError(
                f"joint [{j}] missing fields: {REQUIRED_FIELDS - have}"
            )
        for f in REQUIRED_FIELDS:
            float(kine[j][f])  # must be numeric
    print(f"[ci] OK: {len(EXPECTED_JOINTS)} joints, all fields present & numeric.")


def run_full_verification():
    """Apply the example YAML to a base USD and verify against the reference URDF.

    Returns True if it ran and passed, False if it was skipped for missing inputs.
    Raises (non-zero exit) if it ran and FAILED.
    """
    if not os.path.isfile(REFERENCE_URDF):
        print(f"[ci] SKIP full verification: no reference URDF at [{REFERENCE_URDF}].")
        return False

    base_usd = os.environ.get(BASE_USD_ENV) or DEFAULT_BASE_USD
    if not os.path.isfile(base_usd):
        print(
            f"[ci] SKIP full verification: base USD not found at [{base_usd}] "
            f"(override with ${BASE_USD_ENV})."
        )
        return False

    # Apply the example YAML to a per-robot copy of the base USD, then verify
    # against the reference URDF. The applier is imported as a module and its
    # functions called directly, since the example YAML is already a template and
    # needs no flexiv_description resolution. Imported lazily so the YAML
    # validation above still runs without usd-core.
    import importlib.util
    import shutil
    import subprocess
    import tempfile

    calib_dir = os.path.dirname(HERE)  # .../calibration
    spec = importlib.util.spec_from_file_location(
        "calibrate_usd_from_rdk",
        os.path.join(calib_dir, "calibrate_usd_from_rdk.py"),
    )
    cal = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cal)

    # Stage the base asset in a temp dir so materialize_per_robot_usd writes its
    # sibling copy there, never into the repo's asset tree.
    workdir = tempfile.mkdtemp(prefix="calib_ci_")
    src_model_dir = os.path.dirname(os.path.abspath(base_usd))
    tmp_model_dir = os.path.join(workdir, os.path.basename(src_model_dir))
    shutil.copytree(src_model_dir, tmp_model_dir)
    tmp_base_usd = os.path.join(tmp_model_dir, os.path.basename(base_usd))

    print(f"[ci] Applying example calibration to a copy of [{base_usd}] ...")
    out_usd = cal.materialize_per_robot_usd(tmp_base_usd, "calibration-ci-test")
    cal.apply_calibration_to_usd(out_usd, EXAMPLE_YAML)

    print("[ci] Verifying calibrated USD against reference URDF ...")
    verify_cmd = [
        sys.executable,
        os.path.join(HERE, "verify_calibration_against_urdf.py"),
        "--usd",
        out_usd,
        "--from-urdf",
        REFERENCE_URDF,
    ]
    subprocess.run(verify_cmd, check=True)  # non-zero exit propagates as failure
    print("[ci] Full verification PASSED.")
    return True


def main():
    check_example_yaml()
    ran = run_full_verification()
    if not ran:
        print(
            "[ci] Verification step skipped (a required input was missing); "
            "example data validated."
        )
    print("[ci] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
