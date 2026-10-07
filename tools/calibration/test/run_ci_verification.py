#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# CI entry point for the calibration verification. Runs in a plain Python env
# (no Isaac Sim, no robot) with pip-installed usd-core / numpy / pyyaml.
#
# What it does:
#   1. Validate the example calibrated kinematics YAML parses and has the
#      expected joint set.
#   2. Apply the example YAML to the in-repo asset, then run
#      verify_calibration_against_urdf.py to confirm the calibrated USD matches
#      the reference URDF. Skipped (with a clear message) only if the reference
#      URDF or base USD is missing.
#   3. Calibrate one of two robots referencing the asset on a stage, as the
#      bridge app does (apply_calibration_to_stage), verify it against the
#      reference URDF, and check that the other robot keeps the nominal kinematics.
#
# All steps run for each robot in CASES: a Rizon 4s (single arm) and an
# Enlight LL (dual arm, with calibrated arm adapters), then for a variant of the
# Enlight LL with rotated arm adapters (make_rotated_ll_example).
#
# Exit codes: 0 = pass (or skipped), non-zero = failure.

import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

# The flexiv_isaac package, at the root of this repo
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..")))

# The example calibrated kinematics pulled from a real robot via RDK
# (Model.SyncKinematicsYAML). Structure/joint names are asserted below.
EXAMPLE_YAML = os.path.join(HERE, "Rizon4_calibrated_kinematics.example.yaml")

# The reference calibrated URDF for the example robot (RDK Model.SyncURDF).
REFERENCE_URDF = os.path.join(HERE, "Rizon4_calibrated.example.urdf")

# Base USD to apply the calibration onto. The example robot (hardware type A02LS-P2) is
# a Rizon4s, so the calibration is applied onto the in-repo rizon_4s asset (which
# is tracked, so it is available in CI). Override with BASE_USD_ENV if needed.
BASE_USD_ENV = "CALIBRATION_TEST_BASE_USD"
_REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_BASE_USD = os.path.join(_REPO_ROOT, "assets", "rizon_4s", "rizon_4s.usda")

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

# The Enlight LL example: a simulated robot's calibrated kinematics (synced with
# the EXT_AXIS arm adapters) and the URDF its controller generated, applied onto
# the in-repo enlight_ll asset.
LL_EXAMPLE_YAML = os.path.join(HERE, "EnlightLL_calibrated_kinematics.example.yaml")
LL_REFERENCE_URDF = os.path.join(HERE, "EnlightLL_calibrated.example.urdf")
LL_BASE_USD_ENV = "CALIBRATION_TEST_LL_BASE_USD"
LL_DEFAULT_BASE_USD = os.path.join(_REPO_ROOT, "assets", "enlight_ll", "enlight_ll.usda")
LL_EXPECTED_JOINTS = [("EXT_AXIS", "left_arm_adapter"), ("EXT_AXIS", "right_arm_adapter")] + [
    (arm, j) for arm in ("ARM_1", "ARM_2") for j in EXPECTED_JOINTS
]

# The example Enlight LL's arm adapters are pure translations, so a variant of it
# with rotated (and offset) adapters, written into both the YAML and the
# reference URDF, covers how a mount's rotation is composed into the arm.
# {adapter: (x, y, z, roll, pitch, yaw)}
ROTATED_LL_ADAPTERS = {
    "left_arm_adapter": (0.05, 0.25, 0.3, -1.5707963, 0.1, 0.2),
    "right_arm_adapter": (-0.05, -0.25, 0.3, 1.5707963, -0.1, 3.1415927),
}


def make_rotated_ll_example(workdir):
    """Write the Enlight LL example with ROTATED_LL_ADAPTERS into workdir and
    return (example YAML, reference URDF)."""
    import xml.etree.ElementTree as ET

    with open(LL_EXAMPLE_YAML) as f:
        doc = yaml.safe_load(f)
    tree = ET.parse(LL_REFERENCE_URDF)
    joints = {j.get("name"): j for j in tree.getroot().findall("joint")}
    for adapter, (x, y, z, roll, pitch, yaw) in ROTATED_LL_ADAPTERS.items():
        doc["kinematics"]["EXT_AXIS"][adapter] = {
            "x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw
        }
        matches = [j for n, j in joints.items() if n.endswith("." + adapter)]
        if len(matches) != 1:
            raise KeyError(f"reference URDF has {len(matches)} joints named [*{adapter}]")
        origin = matches[0].find("origin")
        origin.set("xyz", f"{x} {y} {z}")
        origin.set("rpy", f"{roll} {pitch} {yaw}")
    out_yaml = os.path.join(workdir, "EnlightLL_rotated_mounts.yaml")
    out_urdf = os.path.join(workdir, "EnlightLL_rotated_mounts.urdf")
    with open(out_yaml, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    tree.write(out_urdf)
    return out_yaml, out_urdf


# (name, example YAML, expected joints, reference URDF, base USD env var, default base USD)
CASES = [
    ("Rizon 4s", EXAMPLE_YAML, EXPECTED_JOINTS, REFERENCE_URDF, BASE_USD_ENV, DEFAULT_BASE_USD),
    ("Enlight LL", LL_EXAMPLE_YAML, LL_EXPECTED_JOINTS, LL_REFERENCE_URDF, LL_BASE_USD_ENV, LL_DEFAULT_BASE_USD),
]


def _entry(kine, joint):
    """Template entry of a joint name, or of a (section, name) path. Mirrors the
    applier's lookup on purpose rather than importing it, so the check stays
    independent of the code it verifies."""
    if isinstance(joint, str):
        return kine.get(joint)
    node = kine
    for part in joint:
        node = node.get(part) if isinstance(node, dict) else None
    return node


def check_example_yaml(example_yaml, expected_joints):
    """Validate an example calibrated kinematics YAML. Raises on any problem."""
    print(f"[ci] Validating example YAML: {example_yaml}")
    with open(example_yaml) as f:
        doc = yaml.safe_load(f)
    kine = (doc or {}).get("kinematics")
    if not kine:
        raise ValueError("example YAML has no top-level 'kinematics' node")
    missing = [j for j in expected_joints if _entry(kine, j) is None]
    if missing:
        raise ValueError(f"example YAML missing joints: {missing}")
    for j in expected_joints:
        have = set(_entry(kine, j))
        if not REQUIRED_FIELDS.issubset(have):
            raise ValueError(
                f"joint [{j}] missing fields: {REQUIRED_FIELDS - have}"
            )
        for f in REQUIRED_FIELDS:
            float(_entry(kine, j)[f])  # must be numeric
    print(f"[ci] OK: {len(expected_joints)} joints, all fields present & numeric.")


def run_full_verification(example_yaml, reference_urdf, base_usd_env, default_base_usd):
    """Apply the example YAML to a base USD and verify against the reference URDF.

    Returns True if it ran and passed, False if it was skipped for missing inputs.
    Raises (non-zero exit) if it ran and FAILED.
    """
    if not os.path.isfile(reference_urdf):
        print(f"[ci] SKIP full verification: no reference URDF at [{reference_urdf}].")
        return False

    base_usd = os.environ.get(base_usd_env) or default_base_usd
    if not os.path.isfile(base_usd):
        print(
            f"[ci] SKIP full verification: base USD not found at [{base_usd}] "
            f"(override with ${base_usd_env})."
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
        "export_calibrated_usd",
        os.path.join(calib_dir, "export_calibrated_usd.py"),
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

    print(f"[ci] Applying example calibration to a copy of [{base_usd}] ...", flush=True)
    out_usd = cal.materialize_per_robot_usd(tmp_base_usd, "calibration-ci-test")
    cal.apply_calibration_to_usd(out_usd, example_yaml)

    print("[ci] Verifying calibrated USD against reference URDF ...")
    verify_cmd = [
        sys.executable,
        os.path.join(HERE, "verify_calibration_against_urdf.py"),
        "--usd",
        out_usd,
        "--from-urdf",
        reference_urdf,
    ]
    subprocess.run(verify_cmd, check=True)  # non-zero exit propagates as failure
    print("[ci] Full verification PASSED.")

    run_stage_verification(cal, example_yaml, reference_urdf, base_usd, workdir)
    return True


def run_stage_verification(cal, example_yaml, reference_urdf, base_usd, workdir):
    """Calibrate one of two robots referencing the base USD on a stage, the way the
    bridge app does, then verify it against the reference URDF and check that the
    other robot keeps the nominal kinematics. Raises if either check fails."""
    import subprocess

    from pxr import Usd, UsdGeom

    from flexiv_isaac.calibration import apply_calibration_to_stage
    from flexiv_isaac.kinematics_sync import load_kinematics

    stage_path = os.path.join(workdir, "two_robots.usda")
    stage = Usd.Stage.CreateNew(stage_path)
    for name in ("calibrated_robot", "nominal_robot"):
        stage.DefinePrim(f"/{name}").GetReferences().AddReference(os.path.abspath(base_usd))
    stage.SetDefaultPrim(stage.GetPrimAtPath("/calibrated_robot"))
    print(f"[ci] Calibrating one of two robots referencing [{base_usd}] on a stage ...")
    n = apply_calibration_to_stage(
        stage, "/calibrated_robot", base_usd, load_kinematics(example_yaml)
    )
    stage.GetRootLayer().Save()
    print(f"[ci] {n} attributes overridden.", flush=True)

    asset = Usd.Stage.Open(base_usd)
    subprocess.run(
        [
            sys.executable,
            os.path.join(HERE, "verify_calibration_against_urdf.py"),
            "--usd",
            stage_path,
            "--from-urdf",
            reference_urdf,
            "--model",
            asset.GetDefaultPrim().GetName(),
        ],
        check=True,
    )

    # The other robot must still match the asset, link by link.
    asset_root = asset.GetDefaultPrim().GetPath()
    xc_asset, xc_stage = UsdGeom.XformCache(), UsdGeom.XformCache()
    for prim in Usd.PrimRange(asset.GetDefaultPrim()):
        if not prim.IsA(UsdGeom.Xformable):
            continue
        other = stage.GetPrimAtPath(prim.GetPath().ReplacePrefix(asset_root, "/nominal_robot"))
        a = xc_asset.GetLocalToWorldTransform(prim)
        b = xc_stage.GetLocalToWorldTransform(other)
        d = max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4))
        if d > 1e-9:
            raise AssertionError(f"[{other.GetPath()}] moved by {d:.3e}, but was not calibrated")
    print("[ci] Stage verification PASSED: the other robot keeps the nominal kinematics.")


def main():
    import tempfile

    rotated_yaml, rotated_urdf = make_rotated_ll_example(tempfile.mkdtemp(prefix="calib_ci_"))
    cases = CASES + [
        ("Enlight LL, rotated arm mounts", rotated_yaml, LL_EXPECTED_JOINTS, rotated_urdf,
         LL_BASE_USD_ENV, LL_DEFAULT_BASE_USD),
    ]
    for name, example_yaml, expected_joints, reference_urdf, base_usd_env, default_base_usd in cases:
        print(f"[ci] ===== {name} =====")
        check_example_yaml(example_yaml, expected_joints)
        ran = run_full_verification(example_yaml, reference_urdf, base_usd_env, default_base_usd)
        if not ran:
            print(
                "[ci] Verification step skipped (a required input was missing); "
                "example data validated."
            )
    print("[ci] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
