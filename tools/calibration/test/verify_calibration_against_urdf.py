#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Verification test for calibrate_usd_from_rdk.py. Used by the CI runner
# (run_ci_verification.py); can also be run by hand to debug a suspicious
# calibration.
#
# Purpose:
#   Independently confirm that the calibration was applied to the USD correctly
#   -- WITHOUT reusing any of the applier's own code paths. Ground truth comes
#   from a DIFFERENT RDK entry point (Model.SyncURDF) and is compared against the
#   calibrated USD:
#     * calibrate_usd_from_rdk.py: Model.SyncKinematicsYAML() -> YAML -> USD.
#     * this script: URDF -> forward kinematics -> world poses, compared to the
#       USD's world poses (via UsdGeom.XformCache).
#   If the two agree per-link to sub-micron, the applier is correct. A full
#   URDF->USD conversion (Isaac's URDF importer) would reproduce exactly the
#   per-link frames computed here, but requires booting Isaac Sim; this FK check
#   is the same comparison, self-contained.
#
# Inputs (--usd is the calibrated USD to check; ground truth is one of):
#   --robot-sn      : pull the URDF live via SyncURDF (needs a robot).
#   --from-urdf     : compare against an already-synced URDF file (no robot).
# Example:
#   verify_calibration_against_urdf.py --from-urdf Rizon4_calibrated.urdf \
#       --usd .../Rizon4-000001/Rizon4-000001.usda

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Usd, UsdGeom

# Same chain as the applier, but declared here independently so this check does
# not import anything from calibrate_usd_from_rdk.py. Maps the URDF joint name
# to the USD child-link prim path under <defaultPrim>/Geometry.
JOINT_TO_LINK = [
    ("joint1", "base_link/link1"),
    ("joint2", "base_link/link1/link2"),
    ("joint3", "base_link/link1/link2/link3"),
    ("joint4", "base_link/link1/link2/link3/link4"),
    ("joint5", "base_link/link1/link2/link3/link4/link5"),
    ("joint6", "base_link/link1/link2/link3/link4/link5/link6"),
    ("joint7", "base_link/link1/link2/link3/link4/link5/link6/link7"),
    ("link7_to_flange", "base_link/link1/link2/link3/link4/link5/link6/link7/flange"),
]

# Pass/fail threshold on the max abs element-wise difference of a link's 4x4
# world matrix. 1e-5 m / rad comfortably clears fp32 quantization in the USD
# while still catching any real applier mistake (mm-scale and up).
TOL = 1e-5


def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis RPY -> 3x3 rotation, R = Rz @ Ry @ Rx (numpy, independent
    of the applier's Gf-based implementation)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def joint_origin_matrix(xyz, rpy):
    """Column-vector homogeneous 4x4 (p' = M @ p) for a URDF joint <origin>."""
    M = np.eye(4)
    M[:3, :3] = rpy_to_matrix(*rpy)
    M[:3, 3] = xyz
    return M


def sync_urdf_from_robot(robot_sn, out_path, network_whitelist):
    """Pull the robot's actual URDF into out_path via Model.SyncURDF()."""
    import flexivrdk

    # SyncURDF() updates a TEMPLATE urdf in place. Ship/point it at a template
    # generated from flexiv_description; here we require the caller to have one,
    # or we seed a minimal template is NOT possible (SyncURDF needs the full
    # template structure). So we expect out_path to already be a template.
    if not os.path.isfile(out_path):
        raise FileNotFoundError(
            f"SyncURDF needs an existing template URDF at [{out_path}]. Generate "
            f"one from flexiv_description first (see the RDK docs)."
        )
    print(f"[sync] Connecting to robot [{robot_sn}] ...")
    robot = flexivrdk.Robot(robot_sn, network_whitelist)
    model = flexivrdk.Model(robot)
    print(f"[sync] Syncing URDF into [{out_path}] ...")
    model.SyncURDF(out_path)
    print("[sync] URDF synced.")
    return out_path


def parse_urdf_joint_origins(urdf_path):
    """Return {joint_name: (xyz(3,), rpy(3,))} from a URDF file."""
    root = ET.parse(urdf_path).getroot()
    origins = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        if origin is not None:
            if origin.get("xyz"):
                xyz = np.array([float(v) for v in origin.get("xyz").split()])
            if origin.get("rpy"):
                rpy = np.array([float(v) for v in origin.get("rpy").split()])
        origins[name] = (xyz, rpy)
    return origins


def urdf_world_poses(origins):
    """Forward-kinematics world pose of each link, at the zero configuration.

    We compose the joint <origin> transforms down the chain in JOINT_TO_LINK
    order. Zero configuration is correct here because the USD stores its links at
    the zero pose too (the joint variable is applied at runtime, not baked)."""
    world = np.eye(4)
    poses = {}
    for joint_name, link_rel in JOINT_TO_LINK:
        if joint_name not in origins:
            raise KeyError(f"URDF is missing joint [{joint_name}]")
        xyz, rpy = origins[joint_name]
        world = world @ joint_origin_matrix(xyz, rpy)
        poses[link_rel] = world.copy()
    return poses


def usd_world_poses(usd_path, robot_name="Rizon4"):
    """World pose (4x4, column-vector convention) of each link in the USD."""
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise FileNotFoundError(f"Could not open USD [{usd_path}]")
    default = stage.GetDefaultPrim()
    if default:
        robot_name = default.GetName()
    xc = UsdGeom.XformCache()
    poses = {}
    for _joint_name, link_rel in JOINT_TO_LINK:
        prim = stage.GetPrimAtPath(f"/{robot_name}/Geometry/{link_rel}")
        if not prim.IsValid():
            raise KeyError(f"USD missing link prim for [{link_rel}]")
        gfm = xc.GetLocalToWorldTransform(prim)
        # Gf is row-vector (p' = p @ M); transpose to the column-vector 4x4 used
        # by our numpy FK so the two are directly comparable.
        M = np.array([[gfm[i][j] for j in range(4)] for i in range(4)]).T
        poses[link_rel] = M
    return poses


def compare(urdf_poses, usd_poses):
    """Print a per-link comparison and return the overall max abs difference."""
    print(f"\n{'link':10s} {'max|Δpos| [m]':>16s} {'max|Δmatrix|':>16s}  result")
    print("-" * 56)
    overall = 0.0
    for _joint_name, link_rel in JOINT_TO_LINK:
        name = link_rel.split("/")[-1]
        A = urdf_poses[link_rel]
        B = usd_poses[link_rel]
        dpos = float(np.max(np.abs(A[:3, 3] - B[:3, 3])))
        dmat = float(np.max(np.abs(A - B)))
        overall = max(overall, dmat)
        ok = "ok" if dmat < TOL else "MISMATCH"
        print(f"{name:10s} {dpos:16.3e} {dmat:16.3e}  {ok}")
    print("-" * 56)
    print(f"{'OVERALL':10s} {'':16s} {overall:16.3e}")
    return overall


def main():
    p = argparse.ArgumentParser(
        description="Verify a calibrated USD against the robot's URDF (via RDK "
        "SyncURDF) by comparing forward-kinematics world poses."
    )
    p.add_argument("--usd", required=True, help="Calibrated robot USD to check.")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--robot-sn",
        help="Robot serial number; pulls the URDF live via SyncURDF into "
        "--urdf-template (which must already be a valid template URDF).",
    )
    src.add_argument(
        "--from-urdf",
        help="Skip the robot: compare against this already-synced URDF file.",
    )
    p.add_argument(
        "--urdf-template",
        help="Template URDF that SyncURDF updates in place (required with "
        "--robot-sn). Generate from flexiv_description.",
    )
    p.add_argument(
        "--network-interface",
        action="append",
        default=[],
        help="Whitelist a network interface for the robot connection.",
    )
    p.add_argument(
        "--dump-urdf",
        help="Also copy the synced URDF here for manual inspection / running "
        "through the Isaac URDF importer.",
    )
    args = p.parse_args()

    if args.robot_sn:
        if not args.urdf_template:
            p.error("--urdf-template is required with --robot-sn")
        urdf_path = sync_urdf_from_robot(
            args.robot_sn, args.urdf_template, args.network_interface
        )
    else:
        urdf_path = args.from_urdf

    if args.dump_urdf and urdf_path != args.dump_urdf:
        import shutil

        shutil.copyfile(urdf_path, args.dump_urdf)
        print(f"[dump] Copied synced URDF to [{args.dump_urdf}]")

    print(f"[verify] URDF : {urdf_path}")
    print(f"[verify] USD  : {args.usd}")

    origins = parse_urdf_joint_origins(urdf_path)
    urdf_poses = urdf_world_poses(origins)
    usd_poses = usd_world_poses(args.usd)
    overall = compare(urdf_poses, usd_poses)

    if overall < TOL:
        print(f"\nPASS: calibrated USD matches the robot URDF (max Δ {overall:.2e} "
              f"< tol {TOL:.0e}).")
        return 0
    print(f"\nFAIL: USD deviates from URDF by {overall:.2e} (>= tol {TOL:.0e}). "
          f"The calibration was not applied correctly.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
