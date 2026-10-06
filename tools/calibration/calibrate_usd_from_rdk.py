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
#       --usd ~/isaacsim/extsDeprecated/.../data/flexiv/rizon_4/rizon_4.usda

import argparse
import math
import os
import re
import shutil
import sys
import urllib.request

import yaml
from pxr import Gf, Sdf

# The nominal kinematics template is fetched from flexiv_description on GitHub,
# one per model at config/<Model>/default_kinematics.yaml. The templates are
# identical across the repo's branches (for the fields this tool uses), so the
# branch is pinned rather than exposed as an option.
FLEXIV_DESCRIPTION_REPO = "flexivrobotics/flexiv_description"
FLEXIV_DESCRIPTION_BRANCH = "humble"

# Per-model joint mapping, in kinematic-chain order. Each entry is
# (yaml_name, link_rel_path, joint_name, fixed_pre):
#   yaml_name      key in the `kinematics` YAML / URDF joint name
#   link_rel_path  child link Xform path under <defaultPrim>/Geometry
#   joint_name     joint prim name under <defaultPrim>/Physics
#   fixed_pre      optional rigid USD segment composed before this entry, which
#                  the YAML does not model (else None): the name of the fixed
#                  joint under <defaultPrim>/Physics whose parent-relative pose
#                  (localPos0/localRot0) is the segment
#
# rizon_4s ("s" variant) carries a wrist FT sensor that splits link7 into
# link7_proximal/link7_distal, with the fixed link7_ft_sensor joint between them
# (0.094 m along link7 z). RDK reports the collapsed chain (single
# link7_to_flange), so it maps onto the distal->flange pose, with the sensor
# segment as fixed_pre. The segment is read from the asset, because its rotation
# differs between assets: the newer ones orient link7_distal like the flange.
#
# The flange is a rigid body on a fixed joint (link7_to_flange /
# link7_distal_to_flange) in the older assets, and a site (a plain frame, no
# physics and no joint) in the newer SimReady assets. The joint anchor is only
# written when the joint exists.
_RIZON4_CHAIN = "base_link/link1/link2/link3/link4/link5/link6"
JOINTS_BY_MODEL = {
    "rizon_4": [
        ("joint1", "base_link/link1", "joint1", None),
        ("joint2", "base_link/link1/link2", "joint2", None),
        ("joint3", "base_link/link1/link2/link3", "joint3", None),
        ("joint4", "base_link/link1/link2/link3/link4", "joint4", None),
        ("joint5", "base_link/link1/link2/link3/link4/link5", "joint5", None),
        ("joint6", _RIZON4_CHAIN, "joint6", None),
        ("joint7", f"{_RIZON4_CHAIN}/link7", "joint7", None),
        ("link7_to_flange", f"{_RIZON4_CHAIN}/link7/flange", "link7_to_flange", None),
    ],
    "rizon_4s": [
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
            "link7_ft_sensor",
        ),
    ],
}


def _dual_arm_chain(arm, section):
    """Joint mapping of one arm of a dual-arm robot, in the same form as
    JOINTS_BY_MODEL. The YAML nests the arm's joints under `section` (ARM_1 /
    ARM_2), and the USD prefixes its links and joints with `arm`. Each link is
    found by its (unique) name, and the flange is a site with no joint."""
    chain = [
        ((section, f"joint{i}"), f"{arm}_link{i}", f"{arm}_joint{i}", None)
        for i in range(1, 8)
    ]
    chain.append(((section, "link7_to_flange"), f"{arm}_flange", f"{arm}_link7_to_flange", None))
    return chain


# Dual-arm robots: each arm is a chain like a Rizon's, mounted on the body by an
# arm adapter whose pose is part of the robot's calibration (where the arms are
# installed). Each chain is (adapter YAML key, joint mapping); the adapter is
# composed before the arm's joint1. The YAML's ARM_1 is the arm on the left
# adapter, ARM_2 the one on the right adapter.
CHAINS_BY_MODEL = {
    "enlight_ll": [
        (("EXT_AXIS", "left_arm_adapter"), _dual_arm_chain("system1_left_arm", "ARM_1")),
        (("EXT_AXIS", "right_arm_adapter"), _dual_arm_chain("system1_right_arm", "ARM_2")),
    ],
}

# Template entries this tool needs synced beyond what a model's template lists,
# by asset name. SyncKinematicsYAML fills in only the entries the template lists,
# and the robot has its arm adapters' calibration, but the Enlight-LL template
# lists only the arms (ARM_1 / ARM_2). The adapters are added to the working copy
# at their nominal pose (identity, as in the model) so the sync fills them in
# too; entries the template already lists are left as they are.
_NOMINAL_ADAPTER = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
TEMPLATE_ADDITIONS_BY_MODEL = {
    "enlight_ll": {
        "EXT_AXIS": {
            "left_arm_adapter": dict(_NOMINAL_ADAPTER),
            "right_arm_adapter": dict(_NOMINAL_ADAPTER),
        },
    },
}


def chains_for_model(model):
    """Return the kinematic chains of a model or asset name, as a list of
    (mount YAML key or None, joint mapping).

    Accepts either the model from a serial number (e.g. 'Rizon4s', 'EnlightLL')
    or the SimReady asset / USD defaultPrim name (e.g. 'rizon_4s'); both are
    normalized to the asset name.

    Supported: the 7-DoF Rizon series, as one chain from the robot root, and the
    dual-arm Enlight LL, as one chain per arm. A Rizon variant reuses the base
    layout, or the split-link7 layout for an 's' (FT-sensor) variant. Any other
    model raises -- its USD has a prim layout these tables do not describe.
    """
    asset = asset_name_for_model(model)
    if asset in CHAINS_BY_MODEL:
        return CHAINS_BY_MODEL[asset]
    if asset in JOINTS_BY_MODEL:
        return [(None, JOINTS_BY_MODEL[asset])]
    if not asset.startswith("rizon"):
        raise ValueError(
            f"Model [{model}] is not supported. This tool handles the 7-DoF "
            f"Rizon series (Rizon4, Rizon4s, Rizon10, ...) and the Enlight LL; "
            f"other models have a different USD structure. Supported explicitly: "
            f"{sorted(set(JOINTS_BY_MODEL) | set(CHAINS_BY_MODEL))}."
        )
    # Rizon variant not explicitly listed: reuse the split-link7 layout for an
    # 's' (FT-sensor) model, otherwise the base Rizon layout.
    return [(None, JOINTS_BY_MODEL["rizon_4s" if asset.endswith("s") else "rizon_4"])]


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


# Models whose SimReady asset name and flexiv_description config dir do not
# follow from the model by the default rules: (asset name, config dir).
_MODEL_NAMES = {
    "EnlightLL": ("enlight_ll", "Enlight-LL"),
}


def asset_name_for_model(model):
    """SimReady asset name of a model: snake_case, e.g. "Rizon4s" -> "rizon_4s"."""
    if model in _MODEL_NAMES:
        return _MODEL_NAMES[model][0]
    return re.sub(r"(?<=[A-Za-z])(?=\d)", "_", model).lower()


def description_dir_for_model(model):
    """flexiv_description config dir of a model, e.g. "Rizon4s" -> "Rizon4s"."""
    return _MODEL_NAMES[model][1] if model in _MODEL_NAMES else model


def usd_for_model(model):
    """Locate the bundled source USD for a model in the Isaac Sim install.

    Returns <isaac_root>/extsDeprecated/.../data/flexiv/<asset>/<asset>.usda,
    where <asset> is the snake_case asset name (e.g. rizon_4s for Rizon4s).
    Used when --usd is omitted, so the common case is just --robot-sn.
    """
    asset = asset_name_for_model(model)
    path = os.path.join(_isaac_root(), _FLEXIV_DATA_REL, asset, f"{asset}.usda")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"No bundled USD for model [{model}] at [{path}]. Pass --usd "
            f"explicitly, or set $ISAAC_PATH to the Isaac Sim install root."
        )
    return path


def _nominal_from_github(model):
    """Fetch config/<model dir>/default_kinematics.yaml from flexiv_description raw."""
    url = (
        f"https://raw.githubusercontent.com/{FLEXIV_DESCRIPTION_REPO}/"
        f"{FLEXIV_DESCRIPTION_BRANCH}/config/{description_dir_for_model(model)}/"
        f"default_kinematics.yaml"
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
    SyncKinematicsYAML writes into that copy. Sections the tool needs but the
    template does not list (TEMPLATE_ADDITIONS_BY_MODEL) are added at their
    nominal values first, so they are synced too. Returns the working path.
    """
    content, origin = _nominal_from_github(model)
    additions = TEMPLATE_ADDITIONS_BY_MODEL.get(asset_name_for_model(model), {})
    if additions:
        doc = yaml.safe_load(content) or {}
        kine = doc.get("kinematics") or {}
        added = []
        for section, entries in additions.items():
            missing = {k: v for k, v in entries.items() if k not in (kine.get(section) or {})}
            if not missing:
                continue
            added += [f"{section}.{k}" for k in missing]
            if section in kine:
                kine[section] = {**(kine[section] or {}), **missing}
            else:
                # Prepend, as in the templates that carry it (e.g. MICO-Core's EXT_AXIS)
                kine = {section: missing, **kine}
        if added:
            doc["kinematics"] = kine
            content = yaml.safe_dump(doc, sort_keys=False)
            print(f"[template] Added {added} to the template, at nominal values")

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


def _link_prim_path(base_layer, robot_name, link_rel_path):
    """Path of a link prim under <robot_name>/Geometry.

    The link tables give the older assets' paths. The newer SimReady assets nest
    each link under its parent (link7_distal under link7_proximal, rather than
    beside it), so when the exact path is missing, find the link by its name,
    which is unique in the robot.
    """
    prim_path = f"/{robot_name}/Geometry/{link_rel_path}"
    if base_layer.GetPrimAtPath(prim_path) is not None:
        return prim_path
    name = link_rel_path.rsplit("/", 1)[-1]
    found = []

    def visit(path):
        if path.IsPrimPath() and path.name == name and path.HasPrefix(f"/{robot_name}/Geometry"):
            found.append(path)

    base_layer.Traverse(Sdf.Path(f"/{robot_name}/Geometry"), visit)
    return str(found[0]) if len(found) == 1 else prim_path


def _is_site(base_layer, robot_name, link_rel_path):
    """True if the link is a site: a plain frame with no rigid body, like the
    flange of the newer SimReady assets.

    A rigid-body link is authored with xformOpOrder ["!resetXformStack!",
    "xformOp:transform"], a root-relative pose. A site keeps the converter's
    parent-relative xformOp:translate/orient, and has no xformOp:transform.
    """
    prim_path = _link_prim_path(base_layer, robot_name, link_rel_path)
    spec = base_layer.GetPrimAtPath(prim_path)
    if spec is None:
        raise KeyError(f"[base.usda] missing link prim [{prim_path}]")
    return spec.properties.get("xformOp:transform") is None


def _set_site_local_xform(base_layer, robot_name, link_rel_path, xyz, quat):
    """Write the parent-relative xformOp:translate/orient of a site."""
    prim_path = _link_prim_path(base_layer, robot_name, link_rel_path)
    spec = base_layer.GetPrimAtPath(prim_path)
    t = spec.properties.get("xformOp:translate")
    o = spec.properties.get("xformOp:orient")
    if t is None or o is None:
        raise KeyError(f"[base.usda] site {prim_path} has no xformOp:translate/orient")
    t.default = Gf.Vec3d(xyz[0], xyz[1], xyz[2])
    # orient is a quatd or a quatf, depending on the converter
    o.default = type(o.default)(quat.GetReal(), *quat.GetImaginary()) if o.default is not None else quat


def _set_link_world_xform(base_layer, robot_name, link_rel_path, world_matrix):
    """Write the root-relative xformOp:transform 4x4 on a link Xform.

    The link is authored with xformOpOrder ["!resetXformStack!",
    "xformOp:transform"], so this matrix IS the link's pose in the robot root
    frame. We also refresh the (USD-ignored but human-readable) translate/orient
    attributes to the decomposed local values via _refresh_link_srt so the file
    stays self-consistent when someone reads it; those are cosmetic only.
    """
    prim_path = _link_prim_path(base_layer, robot_name, link_rel_path)
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


def _get_joint_anchor(physics_layer, robot_name, joint_name):
    """Return the parent-relative pose (localPos0/localRot0) of a joint, as a matrix."""
    prim_path = f"/{robot_name}/Physics/{joint_name}"
    spec = physics_layer.GetPrimAtPath(prim_path)
    if spec is None:
        raise KeyError(f"[physics.usda] missing joint prim [{prim_path}]")
    p0 = spec.properties.get("physics:localPos0")
    r0 = spec.properties.get("physics:localRot0")
    xyz = p0.default if p0 is not None else Gf.Vec3f(0.0)
    quat = r0.default if r0 is not None else Gf.Quatf(1.0)
    return joint_local_matrix(xyz, quat)


def _has_joint(physics_layer, robot_name, joint_name):
    return physics_layer.GetPrimAtPath(f"/{robot_name}/Physics/{joint_name}") is not None


def _refresh_link_srt(base_layer, robot_name, link_rel_path, xyz, quat):
    """Refresh the ignored-but-readable xformOp:translate/orient to local S/R/T.

    xformOpOrder does not reference these, so they do not affect the composed
    pose. We keep them in sync with the local origin purely so the .usda reads
    consistently. Missing attributes are tolerated silently.
    """
    prim_path = _link_prim_path(base_layer, robot_name, link_rel_path)
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

        <flexiv>/<asset>/<asset>.usda        (source, unchanged; keeps the meshes)
        <flexiv>/<robot_sn>/<robot_sn>.usda  (this copy, same payloads/ layout)

    Every small layer is copied; geometries.usd (the bulk of the bytes) is NOT --
    the copied instances.usda is repointed to the source model's geometries.usd
    with a RELATIVE path (../<asset>/payloads/geometries.usd), so the whole
    data/flexiv tree stays relocatable as a unit. Returns the path to the new
    per-robot root USD (which is what then gets calibrated).
    """
    src_dir = os.path.dirname(os.path.abspath(src_usd_path))  # <flexiv>/<asset>
    src_model = os.path.basename(src_dir)
    flexiv_dir = os.path.dirname(src_dir)  # <flexiv>
    src_payloads = os.path.join(src_dir, "payloads")
    if not os.path.isdir(src_payloads):
        raise FileNotFoundError(
            f"Expected a payloads/ dir next to [{src_usd_path}] (SimReady layout)"
        )

    out_dir = os.path.join(flexiv_dir, robot_sn)  # sibling of <asset>
    # Never let the output land on the source dir (would happen if the
    # output name equals the model, e.g. --robot-sn rizon_4 on a "rizon_4" asset).
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
    # up to <flexiv>/ is ../../, then down into <asset>/payloads/geometries.usd.
    rel_geo = os.path.join("..", "..", src_model, "payloads", _SHARED_MESHES)
    inst_layer = Sdf.Layer.FindOrOpen(os.path.join(out_payloads, "instances.usda"))
    n = _repoint_shared_meshes(inst_layer, rel_geo)
    inst_layer.Save()

    print(f"[copy] Per-robot USD : {out_root}")
    print(
        f"[copy] Shared meshes : {rel_geo} (relative; {n} references repointed)"
    )
    return out_root


def _kine_entry(kine, key):
    """Look up a template entry by its key: a joint name, or a (section, name)
    path for a template that groups joints, e.g. ("ARM_1", "joint1")."""
    if isinstance(key, str):
        return kine.get(key)
    node = kine
    for part in key:
        node = node.get(part) if isinstance(node, dict) else None
    return node


def _kine_key_name(key):
    return key if isinstance(key, str) else ".".join(key)


def _matrix_to_xyz_quatf(matrix):
    """Split a rigid transform into (xyz, Gf.Quatf)."""
    t = matrix.ExtractTranslation()
    q = matrix.ExtractRotationQuat()
    return (t[0], t[1], t[2]), Gf.Quatf(q.GetReal(), Gf.Vec3f(*[float(x) for x in q.GetImaginary()]))


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

    # Compose forward down each chain. A single arm's base_link is fixed to the
    # robot root at the origin (root_joint), so its running world transform starts
    # at identity and W_i = W_{i-1} * L_i, where L_i is joint i's local origin. Each
    # arm of a dual-arm robot is its own chain, starting at its calibrated arm
    # adapter on the root body instead. Entries are in chain order, so we can
    # accumulate as we iterate.
    # The USD defaultPrim name is the asset name (e.g. "rizon_4s"), which selects the
    # chains (base layout, the FT-sensor split-link7 layout, or one per arm).
    updated = 0
    for mount_key, joints in chains_for_model(robot_name):
        world = Gf.Matrix4d(1.0)
        if mount_key is not None:
            m = _kine_entry(kine, mount_key)
            if m is None:
                raise KeyError(
                    f"Template [{template_path}] has no [{_kine_key_name(mount_key)}], "
                    f"the arm mount this robot's calibration needs"
                )
            mount_xyz = (float(m["x"]), float(m["y"]), float(m["z"]))
            world = joint_local_matrix(
                mount_xyz, rpy_to_quatf(float(m["roll"]), float(m["pitch"]), float(m["yaw"]))
            )
            print(
                f"[apply]  = {_kine_key_name(mount_key)}: xyz=({mount_xyz[0]:.6g}, "
                f"{mount_xyz[1]:.6g}, {mount_xyz[2]:.6g})"
            )
        first = True
        for yaml_name, link_rel_path, joint_name, fixed_pre in joints:
            j = _kine_entry(kine, yaml_name)
            name = _kine_key_name(yaml_name)
            if j is None:
                if mount_key is not None:
                    # Every later link of a mounted chain is placed from this one,
                    # so skipping it would misplace the rest of the arm silently
                    raise KeyError(f"Template [{template_path}] has no [{name}]")
                print(f"[apply]  - {name}: not in template, skipping")
                continue

            xyz = (float(j["x"]), float(j["y"]), float(j["z"]))
            quat = rpy_to_quatf(float(j["roll"]), float(j["pitch"]), float(j["yaw"]))
            local = joint_local_matrix(xyz, quat)  # child pose relative to parent

            if fixed_pre is None:
                # Normal joint: accumulate its local origin onto the running world
                # transform and write the resulting ROOT-relative matrix. A site
                # (the flange of the SimReady assets) takes its parent-relative
                # origin instead, and has no joint.
                world = local * world
                # The joint anchor is relative to the parent body: the previous
                # link, so the local origin, except for the first joint of a
                # mounted chain, whose parent is the body the arms are mounted on
                # and whose anchor takes the mount too. That body is fixed to the
                # robot root at identity (system1_system_mount_joint), so the
                # chain's world transform is also its pose on that body.
                anchor_xyz, anchor_quat = xyz, quat
                if first and mount_key is not None:
                    anchor_xyz, anchor_quat = _matrix_to_xyz_quatf(world)
                first = False
                if _is_site(base_layer, robot_name, link_rel_path):
                    _set_site_local_xform(base_layer, robot_name, link_rel_path, xyz, quat)
                else:
                    _set_link_world_xform(base_layer, robot_name, link_rel_path, world)
                    _refresh_link_srt(base_layer, robot_name, link_rel_path, anchor_xyz, anchor_quat)
                if _has_joint(physics_layer, robot_name, joint_name):
                    _set_joint_anchor(physics_layer, robot_name, joint_name, anchor_xyz, anchor_quat)
                wt = world.ExtractTranslation()
                print(
                    f"[apply]  + {name}: local xyz=({xyz[0]:.6g}, {xyz[1]:.6g}, "
                    f"{xyz[2]:.6g}) -> world xyz=({wt[0]:.6g}, {wt[1]:.6g}, {wt[2]:.6g})"
                )
                updated += 1
                continue

            # The YAML value is the COLLAPSED offset (e.g. rizon_4s reports one
            # link7_to_flange), but the USD splits it across the fixed segment
            # (fixed_pre) and this joint. Place the flange at the collapsed offset and
            # the intermediate link at the fixed segment; this joint's parent-relative
            # local is then the remainder = fixed_pre^-1 * local.
            world_flange = local * world
            pre = _get_joint_anchor(physics_layer, robot_name, fixed_pre)
            world_inter = pre * world
            inter_rel = link_rel_path.rsplit("/", 1)[0]  # e.g. .../link7_distal
            _set_link_world_xform(base_layer, robot_name, inter_rel, world_inter)

            remainder = local * pre.GetInverse()
            rt = remainder.ExtractTranslation()
            rq = remainder.ExtractRotationQuat()
            rquat = Gf.Quatf(rq.GetReal(), Gf.Vec3f(*[float(x) for x in rq.GetImaginary()]))
            if _is_site(base_layer, robot_name, link_rel_path):
                _set_site_local_xform(
                    base_layer, robot_name, link_rel_path, (rt[0], rt[1], rt[2]), rquat
                )
            else:
                _set_link_world_xform(base_layer, robot_name, link_rel_path, world_flange)
                _refresh_link_srt(
                    base_layer, robot_name, link_rel_path,
                    (rt[0], rt[1], rt[2]), rquat,
                )
            if _has_joint(physics_layer, robot_name, joint_name):
                _set_joint_anchor(
                    physics_layer, robot_name, joint_name, (rt[0], rt[1], rt[2]), rquat
                )
            world = world_flange
            wt = world.ExtractTranslation()
            print(
                f"[apply]  + {name}: collapsed local xyz=({xyz[0]:.6g}, "
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
    # template. chains_for_model() raises for a model it does not describe.
    chains_for_model(model)

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
