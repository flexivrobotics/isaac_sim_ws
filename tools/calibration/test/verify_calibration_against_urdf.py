#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Verification test for the calibration code in flexiv_isaac/calibration.py. Used by the CI runner
# (run_ci_verification.py); can also be run by hand to debug a suspicious
# calibration.
#
# It checks the calibrated USD against a separate source of truth: the robot's
# URDF, e.g. from Model.SyncURDF, a different RDK path than the applier uses
# (Model.SyncKinematicsYAML). It computes each link's world pose from the URDF by
# forward kinematics and compares it to the USD's world pose (via
# UsdGeom.XformCache); agreement per-link to sub-micron means the calibration was
# applied correctly.
#
# Inputs (--usd is the calibrated USD to check; ground truth is one of):
#   --robot-sn      : pull the URDF live via SyncURDF (needs a robot).
#   --from-urdf     : compare against an already-synced URDF file (no robot).
# Example:
#   verify_calibration_against_urdf.py --from-urdf Rizon4_calibrated.urdf \
#       --usd .../Rizon4_000001/Rizon4_000001.usda

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# Maps the (collapsed) URDF joint name to the USD child-link prim path under
# <defaultPrim>/Geometry. Kept independent of flexiv_isaac/calibration.py so this
# check does not share code with the applier it verifies.
#
# The URDF is always the collapsed chain (a single link7_to_flange). The USD link
# it maps to differs by model: on rizon_4s the FT sensor splits link7 into
# link7_proximal/link7_distal, so joint7 lands on link7_proximal and the flange
# on link7_distal/flange. The comparison checks that the mapped links' world
# poses agree, so the split is transparent as long as the paths are right.
_CHAIN = "base_link/link1/link2/link3/link4/link5/link6"
JOINT_TO_LINK_BY_MODEL = {
    "rizon_4": [
        ("joint1", "base_link/link1"),
        ("joint2", "base_link/link1/link2"),
        ("joint3", "base_link/link1/link2/link3"),
        ("joint4", "base_link/link1/link2/link3/link4"),
        ("joint5", "base_link/link1/link2/link3/link4/link5"),
        ("joint6", _CHAIN),
        ("joint7", f"{_CHAIN}/link7"),
        ("link7_to_flange", f"{_CHAIN}/link7/flange"),
    ],
    "rizon_4s": [
        ("joint1", "base_link/link1"),
        ("joint2", "base_link/link1/link2"),
        ("joint3", "base_link/link1/link2/link3"),
        ("joint4", "base_link/link1/link2/link3/link4"),
        ("joint5", "base_link/link1/link2/link3/link4/link5"),
        ("joint6", _CHAIN),
        ("joint7", f"{_CHAIN}/link7_proximal"),
        ("link7_to_flange", f"{_CHAIN}/link7_distal/flange"),
    ],
}


# Robots verified by walking the URDF tree instead of one chain: each entry maps a
# URDF link, by the end of its name (the controller prefixes link names with the
# robot, e.g. "generated_robot_LL01-D1.arm1.link3"), to the USD link prim name.
# The walk from a link up to the root takes in every joint on the way, so the
# arm adapters of a dual-arm robot are checked through the arm links.
URDF_LINK_TO_USD_LINK_BY_MODEL = {
    "enlight_ll": [
        (f"{arm}.{link}", f"system1_{side}_arm_{link}")
        for arm, side in (("arm1", "left"), ("arm2", "right"))
        for link in [f"link{i}" for i in range(1, 8)] + ["flange"]
    ],
}


def joint_to_link_for(model):
    """Return the URDF-joint -> USD-link mapping for a model or asset name.

    Accepts a model (e.g. "Rizon4s") or an asset / defaultPrim name (e.g.
    "rizon_4s"); both are normalized to the snake_case asset name.
    """
    asset = re.sub(r"(?<=[A-Za-z])(?=\d)", "_", model).lower()
    if asset in JOINT_TO_LINK_BY_MODEL:
        return JOINT_TO_LINK_BY_MODEL[asset]
    return JOINT_TO_LINK_BY_MODEL["rizon_4s" if asset.endswith("s") else "rizon_4"]

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


def sync_urdf_from_robot(robot_sn, out_path):
    """Pull the robot's actual URDF into out_path via Model.SyncURDF()."""
    import flexivrdk

    # SyncURDF() updates a template URDF in place and needs the full template
    # structure, so out_path must already be a valid template generated from
    # flexiv_description.
    if not os.path.isfile(out_path):
        raise FileNotFoundError(
            f"SyncURDF needs an existing template URDF at [{out_path}]. Generate "
            f"one from flexiv_description first (see the RDK docs)."
        )
    print(f"[sync] Connecting to robot [{robot_sn}] ...")
    robot = flexivrdk.Robot(robot_sn)
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


def urdf_world_poses(origins, mapping):
    """Forward-kinematics world pose of each link, at the zero configuration.

    Composes the joint <origin> transforms down the chain in `mapping` order.
    Zero configuration is correct here because the USD stores its links at the
    zero pose too (the joint variable is applied at runtime, not baked)."""
    world = np.eye(4)
    poses = {}
    for joint_name, link_rel in mapping:
        if joint_name not in origins:
            raise KeyError(f"URDF is missing joint [{joint_name}]")
        xyz, rpy = origins[joint_name]
        world = world @ joint_origin_matrix(xyz, rpy)
        poses[link_rel] = world.copy()
    return poses


def urdf_tree_world_poses(urdf_path, mapping):
    """Forward-kinematics world pose of each mapped link, at the zero
    configuration, by walking the URDF from the link up to its root.

    `mapping` is a URDF_LINK_TO_USD_LINK_BY_MODEL entry; poses are keyed by the
    USD link name, like usd_world_poses()."""
    root = ET.parse(urdf_path).getroot()
    joint_by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    def origin(joint):
        o = joint.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
        return joint_origin_matrix(xyz, rpy)

    def world(link):
        joint = joint_by_child.get(link)
        if joint is None:
            return np.eye(4)
        return world(joint.find("parent").get("link")) @ origin(joint)

    links = [l.get("name") for l in root.findall("link")]
    poses = {}
    for urdf_suffix, usd_link in mapping:
        matches = [l for l in links if l == urdf_suffix or l.endswith("." + urdf_suffix)]
        if len(matches) != 1:
            raise KeyError(f"URDF has {len(matches)} links named [*{urdf_suffix}], expected 1")
        poses[usd_link] = world(matches[0])
    return poses


def usd_world_poses(usd_path, mapping, robot_name="rizon_4"):
    """World pose (4x4, column-vector convention) of each link in the USD."""
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise FileNotFoundError(f"Could not open USD [{usd_path}]")
    default = stage.GetDefaultPrim()
    if default:
        robot_name = default.GetName()
    xc = UsdGeom.XformCache()
    poses = {}
    for _joint_name, link_rel in mapping:
        prim = stage.GetPrimAtPath(f"/{robot_name}/Geometry/{link_rel}")
        if not prim.IsValid():
            # The newer SimReady assets nest each link under its parent (e.g.
            # link7_distal under link7_proximal), and a tree mapping gives bare
            # link names, so find the link by its (unique) name.
            name = link_rel.rsplit("/", 1)[-1]
            geometry = stage.GetPrimAtPath(f"/{robot_name}/Geometry")
            matches = [p for p in Usd.PrimRange(geometry) if p.GetName() == name] if geometry else []
            if len(matches) != 1:
                raise KeyError(f"USD missing link prim for [{link_rel}]")
            prim = matches[0]
        gfm = xc.GetLocalToWorldTransform(prim)
        # Gf is row-vector (p' = p @ M); transpose to the column-vector 4x4 used
        # by our numpy FK so the two are directly comparable.
        M = np.array([[gfm[i][j] for j in range(4)] for i in range(4)]).T
        poses[link_rel] = M
    return poses


def joint_frame_mismatch(usd_path):
    """Largest disagreement, over all joints in the USD, between a joint's frame as
    placed on its parent body (localPos0/localRot0) and on its child body
    (localPos1/localRot1), as a max abs difference of the two 4x4 world frames.

    The link poses alone do not show a wrong joint anchor, which is what the
    physics uses, so this checks the anchors against the link poses. The USD's
    joint positions are zero, so both frames must coincide (SimReady DJ.003).
    Returns (mismatch, joint path)."""
    stage = Usd.Stage.Open(usd_path)
    xc = UsdGeom.XformCache()

    def frame(joint, index):
        body = (joint.GetBody0Rel() if index == 0 else joint.GetBody1Rel()).GetTargets()
        world = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(body[0])) if body else Gf.Matrix4d(1.0)
        pos = (joint.GetLocalPos0Attr() if index == 0 else joint.GetLocalPos1Attr()).Get()
        rot = (joint.GetLocalRot0Attr() if index == 0 else joint.GetLocalRot1Attr()).Get()
        local = Gf.Matrix4d(1.0)
        local.SetRotateOnly(Gf.Quatd(rot if rot is not None else Gf.Quatf(1.0)))
        local.SetTranslateOnly(Gf.Vec3d(pos if pos is not None else Gf.Vec3f(0.0)))
        return local * world  # Gf is row-vector: the frame in its body, then the body in world

    worst, worst_path = 0.0, None
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        a, b = frame(joint, 0), frame(joint, 1)
        d = max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4))
        if d > worst:
            worst, worst_path = d, prim.GetPath()
    return worst, worst_path


def compare(urdf_poses, usd_poses, mapping):
    """Print a per-link comparison and return the overall max abs difference."""
    print(f"\n{'link':24s} {'max|Δpos| [m]':>16s} {'max|Δmatrix|':>16s}  result")
    print("-" * 70)
    overall = 0.0
    for _joint_name, link_rel in mapping:
        name = link_rel.split("/")[-1]
        A = urdf_poses[link_rel]
        B = usd_poses[link_rel]
        dpos = float(np.max(np.abs(A[:3, 3] - B[:3, 3])))
        dmat = float(np.max(np.abs(A - B)))
        overall = max(overall, dmat)
        ok = "ok" if dmat < TOL else "MISMATCH"
        print(f"{name:24s} {dpos:16.3e} {dmat:16.3e}  {ok}")
    print("-" * 70)
    print(f"{'OVERALL':24s} {'':16s} {overall:16.3e}")
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
        "--model",
        help="Asset name of the robot, e.g. rizon_4s, which selects the joint to link "
        "mapping. Defaults to the name of the USD's defaultPrim, the robot.",
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
        urdf_path = sync_urdf_from_robot(args.robot_sn, args.urdf_template)
    else:
        urdf_path = args.from_urdf

    if args.dump_urdf and urdf_path != args.dump_urdf:
        import shutil

        shutil.copyfile(urdf_path, args.dump_urdf)
        print(f"[dump] Copied synced URDF to [{args.dump_urdf}]")

    # Pick the joint->link mapping from the USD's model (its defaultPrim name),
    # so the FK comparison uses the right tail geometry (e.g. rizon_4s split link7).
    stage = Usd.Stage.Open(args.usd)
    model = args.model or (stage.GetDefaultPrim().GetName() if stage else "rizon_4")
    print(f"[verify] URDF : {urdf_path}")
    print(f"[verify] USD  : {args.usd}  (model {model})")

    if model in URDF_LINK_TO_USD_LINK_BY_MODEL:
        mapping = URDF_LINK_TO_USD_LINK_BY_MODEL[model]
        urdf_poses = urdf_tree_world_poses(urdf_path, mapping)
    else:
        mapping = joint_to_link_for(model)
        origins = parse_urdf_joint_origins(urdf_path)
        urdf_poses = urdf_world_poses(origins, mapping)
    usd_poses = usd_world_poses(args.usd, mapping)
    overall = compare(urdf_poses, usd_poses, mapping)
    anchors, anchor_path = joint_frame_mismatch(args.usd)
    print(f"{'JOINT ANCHORS':24s} {'':16s} {anchors:16.3e}  "
          f"{'ok' if anchors < TOL else f'MISMATCH at {anchor_path}'}")

    if overall < TOL and anchors < TOL:
        print(f"\nPASS: calibrated USD matches the robot URDF (max Δ {overall:.2e} "
              f"< tol {TOL:.0e}), and its joint anchors match its links.")
        return 0
    if overall >= TOL:
        print(f"\nFAIL: USD deviates from URDF by {overall:.2e} (>= tol {TOL:.0e}). "
              f"The calibration was not applied correctly.")
    if anchors >= TOL:
        print(f"\nFAIL: joint [{anchor_path}] is anchored {anchors:.2e} away from where "
              f"its links are (>= tol {TOL:.0e}). The joint anchors were not updated "
              f"with the links.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
