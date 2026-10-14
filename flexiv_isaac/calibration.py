# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Applies a robot's calibrated kinematics (see kinematics_sync.py) to its
# SimReady USD, so the simulated arm matches the robot:
#   apply_calibration_to_stage()  overrides one robot's prims on a stage, leaving
#                                 the asset and any other robot using it unchanged
#   apply_calibration()           edits the asset's layers, to save a calibrated copy
#
# A calibration sets each joint's origin: the link poses in base.usda (the
# root-relative xformOp:transform of a rigid body, or the parent-relative
# translate/orient of a site) and the joint anchors in physics.usda (the
# parent-relative physics:localPos0/localRot0). How each is computed is
# documented at apply_calibration() and its helpers.

import logging
import math
import os

from pxr import Gf, Sdf

from flexiv_isaac.kinematics_sync import asset_name_for_model

_logger = logging.getLogger(__name__)

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
#
# The Enlight L has the Rizon 4's link and joint names; it is added below.
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


JOINTS_BY_MODEL["enlight_l"] = JOINTS_BY_MODEL["rizon_4"]


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

def chains_for_model(model):
    """Return the kinematic chains of a model or asset name, as a list of
    (mount YAML key or None, joint mapping).

    Accepts either the model from a serial number (e.g. 'Rizon4s', 'EnlightLL')
    or the SimReady asset / USD defaultPrim name (e.g. 'rizon_4s'); both are
    normalized to the asset name.

    Supported: the 7-DoF Rizon series and the Enlight L, as one chain from the
    robot root, and the dual-arm Enlight LL, as one chain per arm. A Rizon variant reuses the base
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
            f"Model [{model}] is not supported. Calibration handles the 7-DoF "
            f"Rizon series (Rizon4, Rizon4s, Rizon10, ...), the Enlight L and the Enlight LL; "
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


def _find_layer(root_layer, sublayer_basename):
    """Return the opened Sdf.Layer whose identifier ends with sublayer_basename.

    The calibration is written into these authoring sublayers directly (not a
    composed stage), so the edits land in the right file and nothing gets flattened.
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


def find_layers(usd_path):
    """(robot name, base.usda layer, physics.usda layer) of a SimReady USD. The
    robot name is the USD's defaultPrim, which is the asset name, e.g. "rizon_4s"."""
    root_layer = Sdf.Layer.FindOrOpen(usd_path)
    if root_layer is None:
        raise FileNotFoundError(f"Could not open root USD [{usd_path}]")
    return (
        _default_prim_name(root_layer),
        _find_layer(root_layer, "base.usda"),
        _find_layer(root_layer, "physics.usda"),
    )


def apply_calibration(base_layer, physics_layer, robot_name, kine):
    """Write the calibrated kinematics (the `kinematics` node of a kinematics
    YAML) into a SimReady robot's base.usda and physics.usda layers, without
    saving them. Returns the number of joints written."""
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
                    f"Kinematics have no [{_kine_key_name(mount_key)}], "
                    f"the arm mount this robot's calibration needs"
                )
            mount_xyz = (float(m["x"]), float(m["y"]), float(m["z"]))
            world = joint_local_matrix(
                mount_xyz, rpy_to_quatf(float(m["roll"]), float(m["pitch"]), float(m["yaw"]))
            )
            _logger.debug(
                f"{_kine_key_name(mount_key)}: xyz=({mount_xyz[0]:.6g}, "
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
                    raise KeyError(f"Kinematics have no [{name}]")
                _logger.info(f"{name}: not in the kinematics, skipping")
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
                _logger.debug(
                    f"{name}: local xyz=({xyz[0]:.6g}, {xyz[1]:.6g}, "
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
            _logger.debug(
                f"{name}: collapsed local xyz=({xyz[0]:.6g}, "
                f"{xyz[1]:.6g}, {xyz[2]:.6g}); fixed_pre={fixed_pre} -> flange world "
                f"xyz=({wt[0]:.6g}, {wt[1]:.6g}, {wt[2]:.6g})"
            )
            updated += 1

    return updated




def apply_calibration_to_stage(stage, prim_path, usd_path, kine):
    """Calibrate one robot on a stage: the robot that references the SimReady USD
    usd_path at prim_path. Returns the number of attributes overridden.

    The calibration is applied to in-memory copies of the asset's layers, and
    every value it changes is authored on the stage's edit target, at the same
    path under prim_path. The asset itself, and any other robot referencing it,
    keep the nominal kinematics.
    """
    robot_name, base_layer, physics_layer = find_layers(usd_path)
    pairs = []
    for layer in (base_layer, physics_layer):
        copy = Sdf.Layer.CreateAnonymous(".usda")
        copy.TransferContent(layer)
        pairs.append((layer, copy))
    apply_calibration(pairs[0][1], pairs[1][1], robot_name, kine)

    asset_root = Sdf.Path(f"/{robot_name}")
    robot_root = Sdf.Path(prim_path)
    overridden = 0
    for layer, copy in pairs:
        changed = []

        def visit(path):
            if not path.IsPropertyPath():
                return
            new = copy.GetAttributeAtPath(path)
            old = layer.GetAttributeAtPath(path)
            if new is not None and old is not None and new.default != old.default:
                changed.append((path, new.default))

        copy.Traverse(Sdf.Path.absoluteRootPath, visit)
        for path, value in changed:
            target = path.ReplacePrefix(asset_root, robot_root)
            attr = stage.GetPrimAtPath(target.GetPrimPath()).GetAttribute(target.name)
            if not attr:
                raise KeyError(f"Robot [{prim_path}] has no attribute [{target}]")
            attr.Set(value)
            overridden += 1
    return overridden
