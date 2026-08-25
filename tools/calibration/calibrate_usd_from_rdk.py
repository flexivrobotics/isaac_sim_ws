#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Apply a physical robot's kinematic calibration (pulled via Flexiv RDK) to its
# SimReady USD, so the simulated arm matches the real one. Steps:
#
#   1. The user provides a robot serial number. The source USD is the bundled
#      asset for that model in the Isaac Sim install, or an explicit --usd.
#   2. Create a per-robot copy of the source USD (a sibling dir named after the
#      serial). The shared meshes (geometries.usd, ~90% of the asset) are reused
#      via a relative reference rather than duplicated, so each copy is ~100 KB.
#      The source asset is never modified.
#   3. Fetch the nominal kinematics template from flexiv_description (GitHub) and
#      stage a working copy next to the per-robot USD.
#   4. Connect to the robot via Flexiv RDK and overwrite the working template
#      with the robot's actual kinematics (Model.SyncKinematicsYAML()).
#   5. Write the calibrated values into the copy's link transforms (base.usda)
#      and joint anchors (physics.usda), producing the calibrated USD at
#      <flexiv>/<robot-sn>/<robot-sn>.usda.
#
# (How base.usda and physics.usda store the calibration -- the root-relative
# xformOp:transform vs. parent-relative joint anchors -- is documented at
# apply_calibration_to_usd() and its helpers, where it matters.)
#
# Works with flexivrdk 1.9.x and 2.x (2.1, 2.2): the Robot / Model /
# SyncKinematicsYAML APIs the script uses are the same across these versions.
#
# Run with Isaac Sim's bundled Python so both flexivrdk and pxr (usd-core) are
# importable, e.g.
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

# The nominal kinematics template is fetched from flexiv_description on GitHub,
# one per model at config/<Model>/default_kinematics.yaml. The Rizon templates are
# identical across the repo's branches (for the fields this tool uses), so the
# branch is pinned rather than exposed as an option.
FLEXIV_DESCRIPTION_REPO = "flexivrobotics/flexiv_description"
FLEXIV_DESCRIPTION_BRANCH = "humble"

# Per-model joint mapping, in kinematic-chain order. Each entry is
# (yaml_name, link_rel_path, joint_name, fixed_pre):
#   yaml_name      key in the `kinematics` YAML / URDF joint name
#   link_rel_path  child link Xform path under <defaultPrim>/Geometry
#   joint_name     joint prim name under <defaultPrim>/Physics
#   fixed_pre      optional constant (x, y, z) offset composed before this entry,
#                  for a rigid USD segment the YAML does not model (else None)
#
# Rizon4s ("s" variant) carries a wrist FT sensor that splits link7 into
# link7_proximal/link7_distal, with a fixed sensor segment between them. RDK
# reports the collapsed chain (single link7_to_flange), so it maps onto the
# distal->flange joint with the 0.094 m sensor segment as fixed_pre.
_RIZON4_CHAIN = "base_link/link1/link2/link3/link4/link5/link6"
JOINTS_BY_MODEL = {
    "Rizon4": [
        ("joint1", "base_link/link1", "joint1", None),
        ("joint2", "base_link/link1/link2", "joint2", None),
        ("joint3", "base_link/link1/link2/link3", "joint3", None),
        ("joint4", "base_link/link1/link2/link3/link4", "joint4", None),
        ("joint5", "base_link/link1/link2/link3/link4/link5", "joint5", None),
        ("joint6", _RIZON4_CHAIN, "joint6", None),
        ("joint7", f"{_RIZON4_CHAIN}/link7", "joint7", None),
        ("link7_to_flange", f"{_RIZON4_CHAIN}/link7/flange", "link7_to_flange", None),
    ],
    "Rizon4s": [
        ("joint1", "base_link/link1", "joint1", None),
        ("joint2", "base_link/link1/link2", "joint2", None),
        ("joint3", "base_link/link1/link2/link3", "joint3", None),
        ("joint4", "base_link/link1/link2/link3/link4", "joint4", None),
        ("joint5", "base_link/link1/link2/link3/link4/link5", "joint5", None),
        ("joint6", _RIZON4_CHAIN, "joint6", None),
        ("joint7", f"{_RIZON4_CHAIN}/link7_proximal", "joint7", None),
        (
            "link7_to_flange",
            f"{_RIZON4_CHAIN}/link7_distal/flange",
            "link7_distal_to_flange",
            (0.0, 0.0, 0.094),
        ),
    ],
}


def joints_for_model(model):
    """Return the joint mapping table for a model name (e.g. 'Rizon4s').

    Only the 7-DoF Rizon series is supported. A Rizon variant reuses the base
    layout, or the split-link7 layout for an 's' (FT-sensor) variant. Any
    non-Rizon model (Enlight, MICO, ...) raises -- its USD has a different prim
    chain that these tables do not describe.
    """
    if model in JOINTS_BY_MODEL:
        return JOINTS_BY_MODEL[model]
    if not model.startswith("Rizon"):
        raise ValueError(
            f"Model [{model}] is not supported. This tool handles the 7-DoF "
            f"Rizon series only (Rizon4, Rizon4s, Rizon10, ...); Enlight/MICO and "
            f"other models have a different USD structure. Supported explicitly: "
            f"{sorted(JOINTS_BY_MODEL)}."
        )
    # Rizon variant not explicitly listed: reuse the split-link7 layout for an
    # 's' (FT-sensor) model, otherwise the base Rizon layout.
    return JOINTS_BY_MODEL["Rizon4s" if model.endswith("s") else "Rizon4"]


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


# Bundled Flexiv assets live under the Isaac Sim install, at this path relative to
# the Isaac root -- the same location the bridge app's config points at.
_FLEXIV_DATA_REL = os.path.join(
    "extsDeprecated",
    "isaacsim.robot.manipulators.examples",
    "data",
    "flexiv",
)


def _isaac_root():
    """Best-effort Isaac Sim install root: $ISAAC_PATH, else inferred from the
    running interpreter (.../isaacsim/kit/python/bin/python3 -> .../isaacsim)."""
    env = os.environ.get("ISAAC_PATH")
    if env:
        return env
    # sys.executable is <root>/kit/python/bin/python3 under Isaac's bundled Python.
    return os.path.abspath(os.path.join(os.path.dirname(sys.executable), *[".."] * 3))


def usd_for_model(model):
    """Locate the bundled source USD for a model in the Isaac Sim install.

    Returns <isaac_root>/extsDeprecated/.../data/flexiv/<model>/<model>.usda.
    Used when --usd is omitted, so the common case is just --robot-sn.
    """
    path = os.path.join(_isaac_root(), _FLEXIV_DATA_REL, model, f"{model}.usda")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"No bundled USD for model [{model}] at [{path}]. Pass --usd "
            f"explicitly, or set $ISAAC_PATH to the Isaac Sim install root."
        )
    return path


def _nominal_from_github(model):
    """Fetch config/<model>/default_kinematics.yaml from flexiv_description raw."""
    url = (
        f"https://raw.githubusercontent.com/{FLEXIV_DESCRIPTION_REPO}/"
        f"{FLEXIV_DESCRIPTION_BRANCH}/config/{model}/default_kinematics.yaml"
    )
    print(f"[template] Fetching nominal template from {url}")
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read().decode("utf-8"), url
    except Exception as e:  # noqa: BLE001 -- surface a clear, actionable message
        raise RuntimeError(
            f"Failed to fetch the nominal template for model [{model}] from "
            f"GitHub ({e}). Check network access and that [{model}] is a valid "
            f"model under config/ in flexiv_description."
        ) from None


def resolve_working_template(usd_path, model):
    """Fetch the nominal template and stage it as a per-robot WORKING COPY.

    The template is fetched from flexiv_description on GitHub and copied to a
    working file next to the USD (<usd_dir>/<model>_synced_kinematics.yaml), so
    SyncKinematicsYAML writes into that copy. Returns the working path.
    """
    content, origin = _nominal_from_github(model)

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


def sync_yaml_from_robot(robot_sn, template_path):
    """Pull the robot's actual kinematics into the template YAML in place.

    Returns the number of joints synced (from Model.SyncKinematicsYAML).
    """
    import flexivrdk  # imported lazily so `apply` mode never needs the robot lib

    print(f"[sync] Connecting to robot [{robot_sn}] ...")
    robot = flexivrdk.Robot(robot_sn)
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
    # W_i = W_{i-1} * L_i, where L_i is joint i's local origin. Each entry is in
    # chain order, so we can accumulate as we iterate.
    # The USD defaultPrim name is the model (e.g. "Rizon4s"), which selects the
    # joint mapping (base layout vs. the FT-sensor split-link7 layout).
    joints = joints_for_model(robot_name)
    world = Gf.Matrix4d(1.0)
    updated = 0
    for yaml_name, link_rel_path, joint_name, fixed_pre in joints:
        j = kine.get(yaml_name)
        if j is None:
            print(f"[apply]  - {yaml_name}: not in template, skipping")
            continue

        xyz = (float(j["x"]), float(j["y"]), float(j["z"]))
        quat = rpy_to_quatf(float(j["roll"]), float(j["pitch"]), float(j["yaw"]))
        local = joint_local_matrix(xyz, quat)  # child pose relative to parent

        if fixed_pre is None:
            # Normal joint: accumulate its local origin onto the running world
            # transform and write the resulting ROOT-relative matrix.
            world = local * world
            _set_link_world_xform(base_layer, robot_name, link_rel_path, world)
            _refresh_link_srt(base_layer, robot_name, link_rel_path, xyz, quat)
            _set_joint_anchor(physics_layer, robot_name, joint_name, xyz, quat)
            wt = world.ExtractTranslation()
            print(
                f"[apply]  + {yaml_name}: local xyz=({xyz[0]:.6g}, {xyz[1]:.6g}, "
                f"{xyz[2]:.6g}) -> world xyz=({wt[0]:.6g}, {wt[1]:.6g}, {wt[2]:.6g})"
            )
            updated += 1
            continue

        # The YAML value is the COLLAPSED offset (e.g. Rizon4s reports one
        # link7_to_flange), but the USD splits it across the fixed segment
        # (fixed_pre) and this joint. Place the flange at the collapsed offset and
        # the intermediate link at the fixed segment; this joint's parent-relative
        # local is then the remainder = fixed_pre^-1 * local.
        world_flange = local * world
        pre = joint_local_matrix(fixed_pre, Gf.Quatf(1.0))
        world_inter = pre * world
        inter_rel = link_rel_path.rsplit("/", 1)[0]  # e.g. .../link7_distal
        _set_link_world_xform(base_layer, robot_name, inter_rel, world_inter)
        _set_link_world_xform(base_layer, robot_name, link_rel_path, world_flange)

        remainder = pre.GetInverse() * local
        rt = remainder.ExtractTranslation()
        rq = remainder.ExtractRotationQuat()
        rquat = Gf.Quatf(rq.GetReal(), Gf.Vec3f(*[float(x) for x in rq.GetImaginary()]))
        _refresh_link_srt(
            base_layer, robot_name, link_rel_path,
            (rt[0], rt[1], rt[2]), rquat,
        )
        _set_joint_anchor(
            physics_layer, robot_name, joint_name, (rt[0], rt[1], rt[2]), rquat
        )
        world = world_flange
        wt = world.ExtractTranslation()
        print(
            f"[apply]  + {yaml_name}: collapsed local xyz=({xyz[0]:.6g}, "
            f"{xyz[1]:.6g}, {xyz[2]:.6g}); fixed_pre={fixed_pre} -> flange world "
            f"xyz=({wt[0]:.6g}, {wt[1]:.6g}, {wt[2]:.6g})"
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
        "--robot-sn",
        required=True,
        help="Robot serial number, e.g. 'Rizon4-000001'. Its calibration is "
        "synced in and written to a per-robot USD named after it.",
    )
    p.add_argument(
        "--usd",
        help="Path to the root robot USD to calibrate. Optional: when omitted, "
        "the bundled USD for the robot's model is used from the Isaac Sim install "
        "($ISAAC_PATH).",
    )
    args = p.parse_args()

    model = model_from_serial(args.robot_sn)

    # Fail fast on an unsupported model, before copying the USD or fetching a
    # template. joints_for_model() raises for anything but the Rizon series.
    joints_for_model(model)

    # Source USD: use --usd if given, else the bundled asset for this model.
    source_usd = args.usd or usd_for_model(model)
    if not args.usd:
        print(f"[usd] No --usd; using bundled asset [{source_usd}]")

    # Materialize a per-robot copy of the USD (reusing the shared meshes) so the
    # source asset is never modified and robots don't collide.
    per_robot_usd = materialize_per_robot_usd(source_usd, args.robot_sn)

    # Stage a working copy of the nominal template next to the per-robot USD, then
    # overwrite it with the robot's actual calibration and write it into the USD.
    working_template = resolve_working_template(per_robot_usd, model)
    sync_yaml_from_robot(args.robot_sn, working_template)
    apply_calibration_to_usd(per_robot_usd, working_template)

    print("[done]")


if __name__ == "__main__":
    sys.exit(main())
