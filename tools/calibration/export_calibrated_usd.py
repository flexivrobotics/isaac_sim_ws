#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Export a calibrated copy of a robot's SimReady USD: pull the robot's
# calibrated kinematics through Flexiv RDK and save them into a per-robot copy
# of the USD. The bridge app does not need this, since it calibrates each robot
# itself once connected. Use it for a USD that carries a robot's calibration
# outside the bridge app, e.g. in your own Isaac Sim scenes. Steps:
#
#   1. The user provides a robot serial number. The source USD is the bundled
#      asset for that model in this repo's assets/, or an explicit --usd.
#   2. Create a per-robot copy of the source USD (a sibling dir named after the
#      serial). The shared meshes (geometries.usd, ~90% of the asset) are reused
#      via a relative reference rather than duplicated, so each copy is ~100 KB.
#      The source asset is never modified.
#   3. Pull the robot's calibrated kinematics through RDK (before step 2, so a
#      failed pull leaves an earlier copy as it is), and keep them in the copy as
#      <model>_synced_kinematics.yaml (flexiv_isaac.kinematics_sync).
#   4. Write them into the copy's link transforms (base.usda) and joint anchors
#      (physics.usda) (flexiv_isaac.calibration), producing the calibrated USD at
#      <assets>/<robot-sn>/<robot-sn>.usda.
#
# Needs flexivrdk, usd-core (pxr) and PyYAML, but not Isaac Sim, e.g. from the
# root of this repo:
#   python3 tools/calibration/export_calibrated_usd.py --robot-sn "Rizon4-000001"

import argparse
import logging
import os
import shutil
import sys
import tempfile

from pxr import Sdf

# Root of this repo, which holds the flexiv_isaac package and the assets/ folder.
_REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, _REPO_DIR)

from flexiv_isaac.calibration import apply_calibration, chains_for_model, find_layers  # noqa: E402
from flexiv_isaac.kinematics_sync import (  # noqa: E402
    asset_name_for_model,
    load_kinematics,
    model_from_serial,
    sync_kinematics_yaml,
)

_ASSETS_DIR = os.path.join(_REPO_DIR, "assets")


def usd_for_model(model):
    """The bundled source USD of a model, <repo>/assets/<asset>/<asset>.usda,
    where <asset> is the snake_case asset name (e.g. rizon_4s for Rizon4s).
    Used when --usd is omitted, so the common case is just --robot-sn."""
    asset = asset_name_for_model(model)
    path = os.path.join(_ASSETS_DIR, asset, f"{asset}.usda")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"No bundled USD for model [{model}] at [{path}]. Pass --usd explicitly."
        )
    return path


# The SimReady asset is a tree of relatively-referenced layers. The meshes live
# in geometries.usd (the bulk of the bytes); everything else is small. To produce
# a per-robot calibrated USD without duplicating meshes, the root and all the
# small layers are copied into the per-robot dir, and geometries.usd is shared by
# repointing the references in the copied instances.usda back to the original.
_SMALL_PAYLOADS = ["base.usda", "instances.usda", "materials.usda", "robot.usda"]
_SMALL_PHYSICS = ["physics.usda", "physx.usda", "mujoco.usda"]
_SHARED_MESHES = "geometries.usd"  # NOT copied; referenced from the source tree


def _repoint_shared_meshes(instances_layer, shared_geometries_abspath):
    """Repoint every geometries.usd reference in a copied instances.usda layer to
    the shared (source-tree) geometries.usd, so meshes are not duplicated."""

    def walk(prim_spec):
        n = 0
        rl = prim_spec.referenceList
        for field in (
            "prependedItems",
            "appendedItems",
            "explicitItems",
            "orderedItems",
            "addedItems",
        ):
            items = list(getattr(rl, field))
            changed = False
            new = []
            for it in items:
                if it.assetPath.endswith(_SHARED_MESHES):
                    it = Sdf.Reference(
                        shared_geometries_abspath,
                        it.primPath,
                        it.layerOffset,
                        it.customData,
                    )
                    changed = True
                    n += 1
                new.append(it)
            if changed:
                setattr(rl, field, new)
        for c in prim_spec.nameChildren:
            n += walk(c)
        return n

    total = 0
    for p in instances_layer.rootPrims:
        total += walk(p)
    return total


def materialize_per_robot_usd(src_usd_path, robot_sn):
    """Create a per-robot copy of the USD asset that reuses the shared meshes.

    The copy is a SIBLING of the source model dir, with an identical internal
    structure at the same directory depth:

        <assets>/<asset>/<asset>.usda        (source, unchanged; keeps the meshes)
        <assets>/<robot_sn>/<robot_sn>.usda  (this copy, same payloads/ layout)

    Every small layer is copied; geometries.usd (the bulk of the bytes) is NOT --
    the copied instances.usda is repointed to the source model's geometries.usd
    with a RELATIVE path (../<asset>/payloads/geometries.usd), so the whole
    assets/ tree stays relocatable as a unit. Returns the path to the new
    per-robot root USD (which is what then gets calibrated).
    """
    src_dir = os.path.dirname(os.path.abspath(src_usd_path))  # <assets>/<asset>
    src_model = os.path.basename(src_dir)
    assets_dir = os.path.dirname(src_dir)  # <assets>
    src_payloads = os.path.join(src_dir, "payloads")
    if not os.path.isdir(src_payloads):
        raise FileNotFoundError(
            f"Expected a payloads/ dir next to [{src_usd_path}] (SimReady layout)"
        )

    out_dir = os.path.join(assets_dir, robot_sn)  # sibling of <asset>
    # Never let the output land on the source dir (would happen if the
    # output name equals the model, e.g. --robot-sn rizon_4 on a "rizon_4" asset).
    if os.path.abspath(out_dir) == os.path.abspath(src_dir):
        raise ValueError(
            f"Refusing to write the per-robot copy onto the source model dir "
            f"[{src_dir}]. Pass a distinct --robot-sn so the output is a sibling "
            f"like <assets>/<robot-sn>/."
        )
    out_payloads = os.path.join(out_dir, "payloads")
    os.makedirs(os.path.join(out_payloads, "Physics"), exist_ok=True)

    # Root: copy verbatim; its ./payloads/... refs still resolve inside the copy.
    out_root = os.path.join(out_dir, f"{robot_sn}.usda")
    shutil.copyfile(src_usd_path, out_root)

    # Small layers: copy, preserving the ./payloads/ structure so their relative
    # references stay valid within the per-robot tree.
    for rel in _SMALL_PAYLOADS:
        shutil.copyfile(
            os.path.join(src_payloads, rel), os.path.join(out_payloads, rel)
        )
    for rel in _SMALL_PHYSICS:
        shutil.copyfile(
            os.path.join(src_payloads, "Physics", rel),
            os.path.join(out_payloads, "Physics", rel),
        )

    # Share the meshes: repoint the copied instances.usda to the source model's
    # geometries.usd via a RELATIVE path. From <robot_sn>/payloads/instances.usda
    # up to <assets>/ is ../../, then down into <asset>/payloads/geometries.usd.
    rel_geo = os.path.join("..", "..", src_model, "payloads", _SHARED_MESHES)
    inst_layer = Sdf.Layer.FindOrOpen(os.path.join(out_payloads, "instances.usda"))
    n = _repoint_shared_meshes(inst_layer, rel_geo)
    inst_layer.Save()

    print(f"[copy] Per-robot USD : {out_root}")
    print(
        f"[copy] Shared meshes : {rel_geo} (relative; {n} references repointed)"
    )
    return out_root


def apply_calibration_to_usd(usd_path, kinematics_yaml):
    """Write a kinematics YAML into a SimReady USD's base.usda and physics.usda,
    and save them. Returns the number of joints written."""
    robot_name, base_layer, physics_layer = find_layers(usd_path)
    n = apply_calibration(base_layer, physics_layer, robot_name, load_kinematics(kinematics_yaml))
    base_layer.Save()
    physics_layer.Save()
    print(f"[apply] Wrote {n} joints into {base_layer.realPath} and {physics_layer.realPath}")
    return n


def main():
    p = argparse.ArgumentParser(
        description="Export a calibrated copy of a Flexiv robot's SimReady USD: pull the "
        "connected robot's calibration through RDK and save it into a per-robot copy."
    )
    p.add_argument(
        "--robot-sn",
        required=True,
        help="Robot serial number, e.g. 'Rizon4-000001'. Its calibration is "
        "synced in and written to a per-robot USD named after it.",
    )
    p.add_argument(
        "--usd",
        help="Path to the root robot USD to calibrate. Optional: when omitted, "
        "the bundled USD for the robot's model is used from this repo's assets/.",
    )
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")

    model = model_from_serial(args.robot_sn)

    # Fail fast on an unsupported model, before copying the USD. chains_for_model()
    # raises for a model it does not describe.
    chains_for_model(model)

    source_usd = args.usd or usd_for_model(model)
    if not args.usd:
        print(f"[usd] No --usd; using bundled asset [{source_usd}]")

    # Pull the calibration first, so a failed pull leaves any earlier copy as it is.
    print(f"[sync] Pulling the calibration of [{args.robot_sn}] through RDK ...")
    with tempfile.TemporaryDirectory() as tmp:
        synced_yaml = os.path.join(tmp, "kinematics.yaml")
        n = sync_kinematics_yaml(args.robot_sn, synced_yaml)
        print(f"[sync] Synced {n} joints")

        # A per-robot copy of the USD (reusing the shared meshes), so the source asset
        # is never modified and robots don't collide. The synced YAML is kept in it.
        per_robot_usd = materialize_per_robot_usd(source_usd, args.robot_sn)
        kinematics_yaml = os.path.join(
            os.path.dirname(per_robot_usd), f"{model}_synced_kinematics.yaml"
        )
        shutil.copyfile(synced_yaml, kinematics_yaml)
    apply_calibration_to_usd(per_robot_usd, kinematics_yaml)
    print("[done]")


if __name__ == "__main__":
    sys.exit(main())
