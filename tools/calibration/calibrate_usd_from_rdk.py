#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Calibrate a SimReady Flexiv robot USD using the per-robot kinematic
# calibration pulled from the real robot via Flexiv RDK.
#
# Why this approach (SyncKinematicsYAML, not SyncURDF -> convert):
#   The shipped USD assets are already SimReady-verified (physics, articulation,
#   materials, validation metadata). Regenerating a USD from a freshly converted
#   URDF would throw all of that away. Instead we pull ONLY the kinematic
#   parameters (per-joint origin: translation + RPY) into a small YAML via
#   Model.SyncKinematicsYAML(), then surgically write those origins into the
#   existing USD's link transforms and joint anchors. The edit is modular,
#   scoped, and preserves every other property of the asset.
#
# What "applying the calibration" means, concretely:
#   A Flexiv arm USD stores each joint's origin (the child link frame expressed
#   in its parent link frame) in TWO consistent places, both of which we update:
#     1. base.usda   : link<N> Xform  -> xformOp:transform  (a full 4x4 matrix)
#     2. physics.usda: joint<N>       -> physics:localPos0 + physics:localRot0
#
#   IMPORTANT -- two different frames:
#   * The kinematic links in base.usda are authored with
#       xformOpOrder = ["!resetXformStack!", "xformOp:transform"]
#     The "!resetXformStack!" discards the inherited parent transform, so each
#     link's xformOp:transform is its pose in the ROBOT ROOT frame (accumulated
#     down the chain), NOT relative to its parent. (These prims also carry
#     leftover xformOp:translate/orient attributes, but xformOpOrder does not
#     reference them, so USD ignores them -- writing those alone would be a
#     silent no-op.) We therefore compose the per-joint origins forward
#     (W_i = W_{i-1} * L_i) and write each link's ROOT-relative matrix.
#   * The physics joints in physics.usda ARE parent-relative: localPos0/localRot0
#     is the joint frame on the parent body, which equals the child link's LOCAL
#     origin L_i. localPos1/localRot1 stay identity (joint frame == child origin)
#     and are intentionally left untouched.
#
# Flow:
#   With --robot-sn : connect via RDK, sync the robot's actual calibration into a
#                     working-copy YAML, then write it into a per-robot USD copy.
#   Without --robot-sn : apply the nominal flexiv_description template only (no
#                     robot); the output is named after the model.
#
# Output: the source asset is never modified. A per-robot copy is written
# to <src_dir>/calibrated/<robot-sn>/<robot-sn>.usda (named after the model when
# no serial is given), so different robots never overwrite each other. The copy
# reuses the shared meshes (geometries.usd) from the source tree rather than
# duplicating them -- see materialize_per_robot_usd().
#
# The nominal template comes from flexiv_description (config/<Model>/
# default_kinematics.yaml), resolved at runtime -- see resolve_working_template().
# By default it is fetched from GitHub; pass --flexiv-description for a local
# checkout. It is copied to a WORKING COPY next to the per-robot USD before any
# sync, so the flexiv_description source is never modified. The model is taken
# from the robot serial, or from the USD's defaultPrim when no serial is given.
#
# Interpreter: run with Isaac Sim's bundled Python so both `flexivrdk` and `pxr`
# (usd-core) are importable, e.g.
#   ~/isaacsim/kit/python/bin/python3 calibrate_usd_from_rdk.py \
#       --robot-sn "Rizon4-000001" \
#       --usd ~/isaacsim/extsDeprecated/.../data/flexiv/Rizon4/Rizon4.usda

import argparse
import math
import os
import shutil
import sys
import urllib.request

import yaml
from pxr import Gf, Sdf, Usd

# Canonical nominal templates live in flexiv_description on GitHub, one per model
# at config/<Model>/default_kinematics.yaml. RDK's own docstring points here. We
# never keep a hand-maintained copy (it silently drifts); instead we resolve the
# template from a local checkout or fetch the single file from GitHub.
FLEXIV_DESCRIPTION_REPO = "flexivrobotics/flexiv_description"
FLEXIV_DESCRIPTION_DEFAULT_REF = "humble"  # the repo's default branch

# Joints, in kinematic-chain order, mapping the YAML/URDF joint name to the USD
# child-link prim path (relative to the robot default prim's Geometry scope) and
# the USD joint prim name (under the Physics scope). This is the single source of
# truth tying the three representations (YAML, base.usda links, physics.usda
# joints) together.
#
#   yaml_name        key in the `kinematics` YAML node and the URDF joint name
#   link_rel_path    child link Xform path under <defaultPrim>/Geometry
#   joint_name       joint prim name under <defaultPrim>/Physics
JOINTS = [
    ("joint1", "base_link/link1", "joint1"),
    ("joint2", "base_link/link1/link2", "joint2"),
    ("joint3", "base_link/link1/link2/link3", "joint3"),
    ("joint4", "base_link/link1/link2/link3/link4", "joint4"),
    ("joint5", "base_link/link1/link2/link3/link4/link5", "joint5"),
    ("joint6", "base_link/link1/link2/link3/link4/link5/link6", "joint6"),
    ("joint7", "base_link/link1/link2/link3/link4/link5/link6/link7", "joint7"),
    (
        "link7_to_flange",
        "base_link/link1/link2/link3/link4/link5/link6/link7/flange",
        "link7_to_flange",
    ),
]


def rpy_to_quatf(roll, pitch, yaw):
    """URDF fixed-axis RPY (R = Rz * Ry * Rx) -> Gf.Quatf(w, (x, y, z)).

    This matches the convention the URDF->USD converter used to author the
    existing orient values, so re-authoring from the synced RPY is exact.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Gf.Quatf(w, Gf.Vec3f(x, y, z))


def joint_local_matrix(xyz, quatf):
    """Build the parent->child local transform L_i as a Gf.Matrix4d.

    USD matrices are row-vector / row-major (a point is p*M), so the rotation
    occupies the upper-left 3x3 and the translation is the bottom row -- exactly
    the layout seen in the authored xformOp:transform values.
    """
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Quatd(quatf))  # widen quatf -> quatd for the matrix
    m.SetTranslateOnly(Gf.Vec3d(xyz[0], xyz[1], xyz[2]))
    return m


# -------------------------- template resolution ----------------------------- #


def model_from_serial(robot_sn):
    """Derive the flexiv_description model dir from a robot serial number.

    The model is the token before the first '-', with any internal spaces
    removed, e.g. "Rizon 4s-123456" -> "Rizon4s", "Rizon4-00001" -> "Rizon4".
    This mirrors how the bridge app maps serials to models.
    """
    return robot_sn.split("-")[0].strip().replace(" ", "")


def model_from_usd(usd_path):
    """Derive the model from the USD's defaultPrim (e.g. "Rizon4").

    Flexiv arm USDs name their default prim after the model, which is also the
    flexiv_description config dir name. Used as the model source when no
    --robot-sn is given (offline / dry-run), so --model is not needed.
    """
    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        raise FileNotFoundError(f"Could not open USD [{usd_path}]")
    if not layer.defaultPrim:
        raise ValueError(
            f"USD [{usd_path}] has no defaultPrim to derive the model from; "
            f"pass --robot-sn."
        )
    return layer.defaultPrim


def _nominal_from_local_checkout(fd_path, model):
    """Read config/<model>/default_kinematics.yaml from a flexiv_description dir."""
    src = os.path.join(fd_path, "config", model, "default_kinematics.yaml")
    if not os.path.isfile(src):
        raise FileNotFoundError(
            f"No template at [{src}] -- is [{fd_path}] a flexiv_description "
            f"checkout, and is [{model}] a valid model dir under config/?"
        )
    with open(src) as f:
        return f.read(), src


def _nominal_from_github(model, ref):
    """Fetch config/<model>/default_kinematics.yaml from flexiv_description raw."""
    url = (
        f"https://raw.githubusercontent.com/{FLEXIV_DESCRIPTION_REPO}/{ref}/"
        f"config/{model}/default_kinematics.yaml"
    )
    print(f"[template] Fetching nominal template from {url}")
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read().decode("utf-8"), url
    except Exception as e:  # noqa: BLE001 -- surface a clear, actionable message
        raise RuntimeError(
            f"Failed to fetch template for model [{model}] at ref [{ref}] from "
            f"GitHub ({e}). Check the model name / --fd-ref, or pass a local "
            f"--flexiv-description checkout, or an explicit --template."
        ) from None


def resolve_working_template(usd_path, model, flexiv_description, fd_ref):
    """Resolve the nominal template and stage it as a per-robot WORKING COPY.

    The template always comes from flexiv_description:
      * --flexiv-description : read config/<model>/default_kinematics.yaml from a
                               local checkout (offline).
      * GitHub (default)     : fetch that same file from the repo at --fd-ref.

    Either way the nominal content is copied to a working file next to the USD
    (<usd_dir>/<model>_synced_kinematics.yaml) so SyncKinematicsYAML writes into
    that copy and NEVER mutates the flexiv_description source. Returns the working
    path.
    """
    if flexiv_description:
        content, origin = _nominal_from_local_checkout(flexiv_description, model)
    else:
        content, origin = _nominal_from_github(model, fd_ref)

    usd_dir = os.path.dirname(os.path.abspath(usd_path)) if usd_path else os.getcwd()
    working = os.path.join(usd_dir, f"{model}_synced_kinematics.yaml")
    header = (
        f"# WORKING COPY -- do not treat as source of truth.\n"
        f"# Nominal template copied from: {origin}\n"
        f"# SyncKinematicsYAML overwrites the values below with the connected "
        f"robot's actual calibration.\n"
    )
    with open(working, "w") as f:
        f.write(header)
        f.write(content)
    print(f"[template] Staged working copy at [{working}] (from {origin})")
    return working


# ------------------------------- sync phase --------------------------------- #


def sync_yaml_from_robot(robot_sn, template_path, network_whitelist):
    """Pull the robot's actual kinematics into the template YAML in place.

    Returns the number of joints synced (from Model.SyncKinematicsYAML).
    """
    import flexivrdk  # imported lazily so `apply` mode never needs the robot lib

    print(f"[sync] Connecting to robot [{robot_sn}] ...")
    robot = flexivrdk.Robot(robot_sn, network_whitelist)
    # A model handle is all we need; it lazily talks to the robot for the sync.
    model = flexivrdk.Model(robot)

    print(f"[sync] Syncing kinematics into [{template_path}] ...")
    n = model.SyncKinematicsYAML(template_path)
    print(f"[sync] Synced {n} joints from the robot into the template YAML.")
    return n


# ------------------------------- apply phase -------------------------------- #


def _load_kinematics(template_path):
    with open(template_path) as f:
        doc = yaml.safe_load(f)
    kine = (doc or {}).get("kinematics")
    if not kine:
        raise ValueError(
            f"Template [{template_path}] has no top-level 'kinematics' node"
        )
    return kine


def _find_layer(root_layer, sublayer_basename):
    """Return the opened Sdf.Layer whose identifier ends with sublayer_basename.

    We edit the specific authoring sublayer directly (not a composed stage) so
    the edits land in the right file and nothing gets flattened.
    """
    root_dir = os.path.dirname(root_layer.realPath)
    # The Flexiv SimReady layout keeps these under payloads/ next to the root.
    candidates = {
        "base.usda": os.path.join(root_dir, "payloads", "base.usda"),
        "physics.usda": os.path.join(root_dir, "payloads", "Physics", "physics.usda"),
    }
    path = candidates[sublayer_basename]
    layer = Sdf.Layer.FindOrOpen(path)
    if layer is None:
        raise FileNotFoundError(f"Could not open sublayer [{path}]")
    return layer


def _default_prim_name(layer):
    if not layer.defaultPrim:
        raise ValueError(f"Layer [{layer.identifier}] has no defaultPrim")
    return layer.defaultPrim


def _set_link_world_xform(base_layer, robot_name, link_rel_path, world_matrix):
    """Write the root-relative xformOp:transform 4x4 on a link Xform.

    The link is authored with xformOpOrder ["!resetXformStack!",
    "xformOp:transform"], so this matrix IS the link's pose in the robot root
    frame. We also refresh the (USD-ignored but human-readable) translate/orient
    attributes to the decomposed local values via _refresh_link_srt so the file
    stays self-consistent when someone reads it; those are cosmetic only.
    """
    prim_path = f"/{robot_name}/Geometry/{link_rel_path}"
    spec = base_layer.GetPrimAtPath(prim_path)
    if spec is None:
        raise KeyError(f"[base.usda] missing link prim [{prim_path}]")

    x = spec.properties.get("xformOp:transform")
    if x is None:
        raise KeyError(
            f"[base.usda] {prim_path} has no xformOp:transform "
            f"(unexpected xform layout)"
        )
    x.default = world_matrix


def _refresh_link_srt(base_layer, robot_name, link_rel_path, xyz, quat):
    """Refresh the ignored-but-readable xformOp:translate/orient to local S/R/T.

    xformOpOrder does not reference these, so they do not affect the composed
    pose. We keep them in sync with the local origin purely so the .usda reads
    consistently. Missing attributes are tolerated silently.
    """
    prim_path = f"/{robot_name}/Geometry/{link_rel_path}"
    spec = base_layer.GetPrimAtPath(prim_path)
    if spec is None:
        return
    t = spec.properties.get("xformOp:translate")
    if t is not None:
        t.default = Gf.Vec3d(xyz[0], xyz[1], xyz[2])
    o = spec.properties.get("xformOp:orient")
    if o is not None:
        o.default = quat


def _set_joint_anchor(physics_layer, robot_name, joint_name, xyz, quat):
    """Write physics:localPos0 + physics:localRot0 on a joint in physics.usda.

    localPos0/localRot0 are the joint frame on the PARENT body, which for these
    assets equals the child link's origin. localPos1/localRot1 stay identity
    (joint frame == child link origin) and are intentionally left untouched.
    """
    prim_path = f"/{robot_name}/Physics/{joint_name}"
    spec = physics_layer.GetPrimAtPath(prim_path)
    if spec is None:
        raise KeyError(f"[physics.usda] missing joint prim [{prim_path}]")

    p0 = spec.properties.get("physics:localPos0")
    if p0 is None:
        raise KeyError(f"[physics.usda] {prim_path} has no physics:localPos0")
    p0.default = Gf.Vec3f(xyz[0], xyz[1], xyz[2])

    r0 = spec.properties.get("physics:localRot0")
    if r0 is None:
        raise KeyError(f"[physics.usda] {prim_path} has no physics:localRot0")
    r0.default = quat


# The SimReady asset is a tree of relatively-referenced layers. The meshes live
# in geometries.usd (the bulk of the bytes); everything else is small. To produce
# a per-robot calibrated USD without duplicating meshes, we copy the root + all
# the small layers into <src>/calibrated/<robot-sn>/ and SHARE geometries.usd by
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

        <flexiv>/<Model>/<Model>.usda        (source, unchanged; keeps the meshes)
        <flexiv>/<robot_sn>/<robot_sn>.usda  (this copy, same payloads/ layout)

    Every small layer is copied; geometries.usd (the bulk of the bytes) is NOT --
    the copied instances.usda is repointed to the source model's geometries.usd
    with a RELATIVE path (../<Model>/payloads/geometries.usd), so the whole
    data/flexiv tree stays relocatable as a unit. Returns the path to the new
    per-robot root USD (which is what then gets calibrated).
    """
    src_dir = os.path.dirname(os.path.abspath(src_usd_path))  # <flexiv>/<Model>
    src_model = os.path.basename(src_dir)
    flexiv_dir = os.path.dirname(src_dir)  # <flexiv>
    src_payloads = os.path.join(src_dir, "payloads")
    if not os.path.isdir(src_payloads):
        raise FileNotFoundError(
            f"Expected a payloads/ dir next to [{src_usd_path}] (SimReady layout)"
        )

    out_dir = os.path.join(flexiv_dir, robot_sn)  # sibling of <Model>
    # Never let the output land on the source dir (would happen if the
    # output name equals the model, e.g. the no-serial case on a "Rizon4" asset).
    if os.path.abspath(out_dir) == os.path.abspath(src_dir):
        raise ValueError(
            f"Refusing to write the per-robot copy onto the source model dir "
            f"[{src_dir}]. Pass a distinct --robot-sn so the output is a sibling "
            f"like <flexiv>/<robot-sn>/."
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
    # up to <flexiv>/ is ../../, then down into <Model>/payloads/geometries.usd.
    rel_geo = os.path.join("..", "..", src_model, "payloads", _SHARED_MESHES)
    inst_layer = Sdf.Layer.FindOrOpen(os.path.join(out_payloads, "instances.usda"))
    n = _repoint_shared_meshes(inst_layer, rel_geo)
    inst_layer.Save()

    print(f"[copy] Per-robot USD : {out_root}")
    print(
        f"[copy] Shared meshes : {rel_geo} (relative; {n} references repointed)"
    )
    return out_root


def apply_calibration_to_usd(usd_path, template_path):
    """Write the YAML kinematics into the SimReady USD's links and joints."""
    kine = _load_kinematics(template_path)

    root_layer = Sdf.Layer.FindOrOpen(usd_path)
    if root_layer is None:
        raise FileNotFoundError(f"Could not open root USD [{usd_path}]")
    robot_name = _default_prim_name(root_layer)

    base_layer = _find_layer(root_layer, "base.usda")
    physics_layer = _find_layer(root_layer, "physics.usda")

    print(f"[apply] Robot prim         : /{robot_name}")
    print(f"[apply] Editing link xforms: {base_layer.realPath}")
    print(f"[apply] Editing joint anchors: {physics_layer.realPath}")

    # Compose forward down the chain. base_link is fixed to the robot root at the
    # origin (root_joint), so the running world transform starts at identity and
    # W_i = W_{i-1} * L_i, where L_i is joint i's local origin. Each JOINTS entry
    # is in chain order, so we can accumulate as we iterate.
    world = Gf.Matrix4d(1.0)
    updated = 0
    for yaml_name, link_rel_path, joint_name in JOINTS:
        j = kine.get(yaml_name)
        if j is None:
            print(f"[apply]  - {yaml_name}: not in template, skipping")
            continue
        xyz = (float(j["x"]), float(j["y"]), float(j["z"]))
        quat = rpy_to_quatf(float(j["roll"]), float(j["pitch"]), float(j["yaw"]))
        local = joint_local_matrix(xyz, quat)

        # Accumulate this joint's local origin onto the running world transform,
        # then write the resulting ROOT-relative matrix to the link.
        world = local * world
        _set_link_world_xform(base_layer, robot_name, link_rel_path, world)
        _refresh_link_srt(base_layer, robot_name, link_rel_path, xyz, quat)

        # The physics joint anchor stays PARENT-relative (the local origin).
        _set_joint_anchor(physics_layer, robot_name, joint_name, xyz, quat)

        wt = world.ExtractTranslation()
        print(
            f"[apply]  + {yaml_name}: local xyz=({xyz[0]:.6g}, {xyz[1]:.6g}, "
            f"{xyz[2]:.6g}) -> world xyz=({wt[0]:.6g}, {wt[1]:.6g}, {wt[2]:.6g})"
        )
        updated += 1

    base_layer.Save()
    physics_layer.Save()
    print(f"[apply] Wrote {updated} joints into base.usda + physics.usda.")
    return updated


# ---------------------------------- main ------------------------------------ #


def main():
    p = argparse.ArgumentParser(
        description="Calibrate a SimReady Flexiv USD from RDK kinematics. "
        "Pulls the connected robot's calibration and writes it into the USD."
    )
    p.add_argument(
        "--usd",
        required=True,
        help="Path to the root robot USD to calibrate, e.g. "
        ".../Rizon4/Rizon4.usda.",
    )
    p.add_argument(
        "--robot-sn",
        help="Robot serial number, e.g. 'Rizon4-000001'. When given, the robot's "
        "actual calibration is synced in before applying, and the per-robot "
        "output is named after it. Omit to apply the nominal flexiv_description "
        "template only, named after the model (no robot sync).",
    )
    p.add_argument(
        "--flexiv-description",
        help="Path to a local flexiv_description checkout; the nominal template "
        "is read from config/<Model>/default_kinematics.yaml (never modified). "
        "Omit to fetch that file from GitHub instead.",
    )
    p.add_argument(
        "--flexiv-description-branch",
        default=FLEXIV_DESCRIPTION_DEFAULT_REF,
        help=f"flexiv_description branch/tag to fetch the template from when no "
        f"local --flexiv-description is given (default: "
        f"{FLEXIV_DESCRIPTION_DEFAULT_REF}).",
    )
    p.add_argument(
        "--network-interface",
        action="append",
        default=[],
        help="Whitelist a network interface for the robot connection "
        "(repeatable). Passed to flexivrdk.Robot.",
    )
    args = p.parse_args()

    # The template's model comes from the robot serial when connecting, otherwise
    # from the USD's defaultPrim (Flexiv arm USDs name it after the model). The
    # per-robot output dir is a sibling named after the serial when present. With
    # no serial we suffix "-nominal" so the output can't collide with the source
    # model dir (whose name IS the model) and reads as "not robot-calibrated".
    if args.robot_sn:
        model = model_from_serial(args.robot_sn)
        out_name = args.robot_sn
    else:
        model = model_from_usd(args.usd)
        out_name = f"{model}-nominal"
        print(
            f"[template] No --robot-sn; deriving model [{model}] from the USD "
            f"and applying the nominal template only (no robot sync)."
        )

    # Materialize a per-robot copy of the USD (reusing the shared meshes) so the
    # source asset is never modified and robots don't collide.
    per_robot_usd = materialize_per_robot_usd(args.usd, out_name)

    # Stage a working copy of the nominal template next to the per-robot USD, so a
    # robot sync (if any) never mutates the flexiv_description source.
    working_template = resolve_working_template(
        usd_path=per_robot_usd,
        model=model,
        flexiv_description=args.flexiv_description,
        fd_ref=args.flexiv_description_branch,
    )

    # With a robot, refresh the working copy with the actual calibration first.
    if args.robot_sn:
        sync_yaml_from_robot(
            args.robot_sn, working_template, args.network_interface
        )

    apply_calibration_to_usd(per_robot_usd, working_template)

    print("[done]")


if __name__ == "__main__":
    sys.exit(main())
