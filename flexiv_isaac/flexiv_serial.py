# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import logging
from typing import Optional, List
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class SiteEndEffector:
    """
    End effector on a site: a plain frame with no physics, like the flange of the SimReady assets,
    which sits under link7 (or link7_distal) with a fixed offset.

    Physics does not move a site: its USD pose stays at rest, and wrapping it as a rigid or xform
    prim would write poses onto it. So this wraps the site's rigid-body parent link, which physics
    moves, and composes the fixed offset onto the parent's pose. Anything else is forwarded to the
    parent rigid prim.

    Params:
        prim_path (str): Path of the site prim.
        name (str): Name of this end effector.
    """

    def __init__(self, prim_path: str, name: str) -> None:
        stage = get_current_stage()
        site = stage.GetPrimAtPath(prim_path)
        parent = site.GetParent()
        while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            parent = parent.GetParent()
        if not parent:
            raise RuntimeError(f"Site [{prim_path}] has no rigid-body ancestor")

        # fixed offset of the site in its parent link, from the rest poses (Gf row-vector convention)
        xform_cache = UsdGeom.XformCache()
        offset = xform_cache.GetLocalToWorldTransform(site) * xform_cache.GetLocalToWorldTransform(parent).GetInverse()
        self._offset_pos = np.array(offset.ExtractTranslation())
        rotation = offset.ExtractRotationQuat()
        self._offset_quat = np.array([rotation.GetReal(), *rotation.GetImaginary()])  # w, x, y, z

        self._prim_path = prim_path
        self._name = name
        self._link = SingleRigidPrim(prim_path=parent.GetPath().pathString, name=name + "_link")

    @property
    def prim_path(self) -> str:
        return self._prim_path

    @property
    def name(self) -> str:
        return self._name

    def initialize(self, physics_sim_view=None) -> None:
        self._link.initialize(physics_sim_view)

    def post_reset(self) -> None:
        self._link.post_reset()

    def get_world_pose(self):
        """
        Return:
            (np.ndarray, np.ndarray): Site position [m] and orientation (quaternion w, x, y, z) in world.
        """
        link_pos, link_quat = self._link.get_world_pose()
        link_pos, link_quat = np.asarray(link_pos, dtype=float), np.asarray(link_quat, dtype=float)
        position = link_pos + _rotate(link_quat, self._offset_pos)
        return position, _quat_multiply(link_quat, self._offset_quat)

    def __getattr__(self, attr):
        return getattr(self._link, attr)


def _quat_multiply(a, b):
    """Hamilton product of quaternions (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def _rotate(quat, vector):
    """Rotate a vector by a quaternion (w, x, y, z)."""
    w, xyz = quat[0], np.asarray(quat[1:])
    return vector + 2.0 * np.cross(xyz, np.cross(xyz, vector) + w * vector)


def controller_joint_order(prim_path: str, exclude_prim_paths: Optional[List[str]] = None) -> List[str]:
    """
    Actuated joint names of a robot in the order Flexiv's controller uses for the sim messages.

    Flexiv's simulated controller builds its joint order with a depth-first walk of the robot URDF
    from the root link, collecting every revolute/prismatic joint. urdfdom lists each link's child
    joints sorted by name, so the walk visits siblings in name order. The SimReady USDs are
    converted from the same URDF, so the same walk over the USD joint tree gives the same order,
    e.g. joint1..joint7 for a Rizon, and system1_body_joint1, system1_body_joint2,
    system1_left_arm_joint1..7, system1_right_arm_joint1..7 for a MICO Plus. SimRobotStates and SimRobotCommands carry no joint names,
    so this order is the only thing that ties a value to a joint.

    Params:
        prim_path (str): Prim path of the robot articulation root.
        exclude_prim_paths (Optional[List[str]]): Subtrees to leave out, e.g. attached tools, whose
            joints are driven by the bridge instead of the controller.

    Return:
        List[str]: Actuated joint prim names in controller order.
    """
    stage = get_current_stage()
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        raise RuntimeError(f"Robot prim [{prim_path}] not found")
    excluded = [Sdf.Path(p) for p in (exclude_prim_paths or [])]

    # Parent link -> [(joint name, child link, actuated)]
    children = {}
    child_links = set()
    for prim in Usd.PrimRange(root):
        if any(prim.GetPath().HasPrefix(e) for e in excluded):
            continue
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        if not body1:
            continue
        parent = body0[0] if body0 else None
        actuated = prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)
        children.setdefault(parent, []).append((prim.GetName(), body1[0], actuated))
        child_links.add(body1[0])

    order = []

    def visit(link):
        for joint_name, child, actuated in sorted(children.get(link, []), key=lambda c: c[0]):
            if actuated:
                order.append(joint_name)
            visit(child)

    # Roots: links (or the world, None) that are never a joint's child
    for parent in sorted((p for p in children if p not in child_links), key=str):
        visit(parent)
    return order


def usd_home_joint_positions(prim_path: str, joint_names: List[str]) -> List[Optional[float]]:
    """
    Home joint positions of a robot, as authored in its USD.

    The SimReady USDs carry each joint's home pose, from the "home" group state of the robot's SRDF,
    as the joint's drive target position (drive:<angular|linear>:physics:targetPosition). It is the
    same group state the simulated controller starts from, so starting the robot here keeps the two
    in agreement.

    Params:
        prim_path (str): Prim path of the robot articulation root.
        joint_names (List[str]): Joint prim names to read, e.g. from controller_joint_order().

    Return:
        List[Optional[float]]: Home position of each joint [rad or m], in the order of joint_names,
            or None for a joint with no authored drive target position.
    """
    stage = get_current_stage()
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        raise RuntimeError(f"Robot prim [{prim_path}] not found")
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(root)
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)
    }

    positions = []
    for name in joint_names:
        prim = joints.get(name)
        if prim is None:
            raise RuntimeError(f"Joint [{name}] not found under [{prim_path}]")
        revolute = prim.IsA(UsdPhysics.RevoluteJoint)
        attr = prim.GetAttribute(f"drive:{'angular' if revolute else 'linear'}:physics:targetPosition")
        value = attr.Get() if attr and attr.HasAuthoredValue() else None
        if value is not None and revolute:
            # UsdPhysics angular values are in degrees
            value = np.deg2rad(value)
        positions.append(None if value is None else float(value))
    return positions


class FlexivSerial(Robot):
    """
    Control interface for one Flexiv robot: a single arm, or a dual arm (e.g. MICO) with any
    external robot axes (e.g. the waist), all controlled as one articulation by one controller.

    Params:
        prim_path (str): Primitive path of this robot articulation in the stage. E.g. /World/Flexiv
        name (str): name of this robot articulation.
        end_effector_prim_name (str): name of the primitive in the robot articulation to be used as
            end effector. A leaf name also matches a prefixed prim, e.g. "flange" matches
            "system1_left_arm_flange".
        arm_dof (Optional[int]): number of robot joints, gripper excluded. Checked against the joint
            order when given; derived from the USD when None.
        pos_in_world (Optional[List[float]]): position (x, y, z) of the robot in world [m].
        ori_in_world (Optional[List[float]]): orientation (quaternion w, x, y, z) of the robot in world [].
        gripper (Optional[Gripper]): Constructed gripper instance.
        grippers (Optional[List[Gripper]]): More constructed gripper instances, e.g. one per arm.
        joint_names (Optional[List[str]]): Robot joint names in controller order. Derived from the
            USD with controller_joint_order() when None.
        tool_prim_paths (Optional[List[str]]): Prim paths of attached tools, whose joints are left
            out of the robot joints.
        has_ft_sensor (bool): Whether this model carries a wrist 6-DoF force-torque sensor (the "s"
            variants, e.g. Rizon 4s / Rizon 10s). When True, wrist_wrench reports a simulated reading.
    """

    # Fixed sensing joint of the wrist force-torque sensor, present only in the "s" model USDs.
    # link7 is split into link7_proximal and link7_distal at the sensor plane, joined by this
    # fixed joint; everything distal to it (link7_distal + flange + any tool) is exactly what the
    # physical wrist sensor measures. Isaac reports the reaction wrench in the child (link7_distal)
    # frame, which IS the sensor frame, so no extra frame offset is needed here.
    _FT_JOINT_NAME = "link7_ft_sensor"

    def __init__(
        self,
        prim_path: str,
        name: str,
        end_effector_prim_name: str,
        arm_dof: Optional[int] = None,
        pos_in_world: Optional[List[float]] = None,
        ori_in_world: Optional[List[float]] = None,
        gripper: Optional[Gripper] = None,
        has_ft_sensor: bool = False,
        grippers: Optional[List[Gripper]] = None,
        joint_names: Optional[List[str]] = None,
        tool_prim_paths: Optional[List[str]] = None,
    ) -> None:
        self._grippers = ([gripper] if gripper is not None else []) + list(grippers or [])
        self._default_kps = None
        self._default_kds = None
        self._end_effector = None
        self._logger = logging.getLogger("flexiv." + name)

        # Robot joints in controller order, which is the order of q, dq and the target drives
        if joint_names is None:
            joint_names = controller_joint_order(prim_path, tool_prim_paths)
        self._joint_names = list(joint_names)
        if not self._joint_names:
            raise RuntimeError(f"No actuated joints found under [{prim_path}]")
        if arm_dof is not None and arm_dof != len(self._joint_names):
            raise ValueError(
                f"arm_dof={arm_dof}, but the robot has {len(self._joint_names)} joints: "
                f"{self._joint_names}"
            )
        self._arm_dof = len(self._joint_names)
        # Articulation DoF index of each robot joint, resolved in initialize()
        self._joint_indices = None
        self._logger.info(f"Robot joints ({self._arm_dof}) in controller order: {self._joint_names}")

        # Resolve the end-effector prim path. Two USD layouts are in use:
        #   * the older "flat" layout, where the end-effector prim (e.g. "flange")
        #     is a direct child of the robot root, so prim_path + "/" + name is
        #     the correct full path; and
        #   * the newer SimReady layout, where "flange" is nested deep under
        #     Geometry/base_link/link1/.../link7 (or, after the FT-sensor split,
        #     under link7_distal), so the direct-child path does not exist.
        # Prefer the direct path when it already exists (exact old behavior);
        # otherwise search the robot subtree for a prim whose name matches the
        # LAST component of end_effector_prim_name and use that full path. This
        # also handles the gripper cases (whose names are already nested, e.g.
        # "Grav_gripper/right_finger_tip") by matching their leaf name.
        self._end_effector_prim_path = self._resolve_end_effector_prim_path(
            prim_path, end_effector_prim_name
        )
        self._has_ft_sensor = has_ft_sensor
        # Row index into get_measured_joint_forces() output for the wrist sensor joint. Resolved in
        # initialize() once the articulation metadata is available.
        self._ft_force_row = None

        # Construct base
        super().__init__(
            prim_path=prim_path,
            name=name,
            position=pos_in_world,
            orientation=ori_in_world,
        )
        return

    def _resolve_end_effector_prim_path(
        self, prim_path: str, end_effector_prim_name: str
    ) -> str:
        """Resolve the end-effector prim path across the two robot USD layouts.

        The direct path ``prim_path + "/" + end_effector_prim_name`` is correct
        for the flat layout (end-effector a direct child of the robot root). In
        the SimReady layout the end-effector (e.g. the flange) is nested many
        levels deep, so the direct path does not exist; in that case we search
        the robot subtree (restricted to ``prim_path``) for a prim whose name
        equals the LAST component of ``end_effector_prim_name`` and return its
        full path.

        Behavior is unchanged whenever the direct path already exists. If the
        stage is not available yet, or no match is found, we fall back to the
        direct path so downstream initialization surfaces the original error.

        Params:
            prim_path (str): robot articulation root prim path.
            end_effector_prim_name (str): configured end-effector name, possibly
                itself a nested path (e.g. "Grav_gripper/right_finger_tip").

        Return:
            str: resolved full prim path of the end effector.
        """
        direct_path = prim_path + "/" + end_effector_prim_name
        stage = get_current_stage()
        if stage is None:
            return direct_path

        # Exact old behavior: if the direct path already resolves, use it as-is.
        if stage.GetPrimAtPath(direct_path).IsValid():
            return direct_path

        # Otherwise search the robot subtree for the end-effector leaf name.
        leaf_name = end_effector_prim_name.rsplit("/", 1)[-1]
        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            return direct_path

        matches = [
            p.GetPath().pathString
            for p in Usd.PrimRange(root_prim)
            if p.GetName() == leaf_name
        ]
        # Multi-arm assets prefix the names per arm, e.g. "system1_left_arm_flange"
        per_arm = not matches
        if per_arm:
            matches = [
                p.GetPath().pathString
                for p in Usd.PrimRange(root_prim)
                if p.GetName().endswith("_" + leaf_name)
            ]
        if not matches:
            self._logger.warning(
                f"End-effector prim [{leaf_name}] not found under [{prim_path}]; "
                f"falling back to [{direct_path}]"
            )
            return direct_path

        # PrimRange is a depth-first pre-order walk, so matches[0] is the first
        # (shallowest, left-most) hit under prim_path. Prefer it and log if the
        # search was ambiguous. One prefixed match per arm is the expected layout of a
        # multi-arm asset, so the first arm's is used without a warning.
        resolved = matches[0]
        if len(matches) > 1 and per_arm:
            self._logger.info(
                f"Resolved end-effector [{leaf_name}] to the first arm's [{resolved}]; "
                f"name another arm's, e.g. [{matches[1].rsplit('/', 1)[-1]}], to use it instead"
            )
        elif len(matches) > 1:
            self._logger.warning(
                f"Multiple prims named [{leaf_name}] under [{prim_path}]: "
                f"{matches}; using [{resolved}]"
            )
        else:
            self._logger.info(
                f"Resolved nested end-effector [{leaf_name}] to [{resolved}]"
            )
        return resolved

    def switch_control_mode(self, mode: str) -> None:
        """
        Switch control mode for all robot joints, gripper excluded.

        Params:
            mode (str): Desired control mode, options are "position", "velocity", and "effort".
        """
        # Reset default gains for articulation view
        if self._default_kps is not None and self._default_kds is not None:
            self._articulation_view.set_gains(
                kps=self._default_kps, kds=self._default_kds
            )

        # Set control mode for all robot joints, but leave gripper joints unchanged
        self._articulation_view.switch_control_mode(
            mode, joint_indices=self._joint_indices
        )

        self._logger.info(f"Control mode switched to [{mode}]")
        return

    @property
    def end_effector(self):
        """
        Access reference to the _end_effector member.

        Return:
            SingleRigidPrim or SiteEndEffector: Reference to the end-effector instance: a
            SingleRigidPrim when the end effector is a rigid body, else a SiteEndEffector (a site,
            like the flange of the SimReady assets).
        """
        return self._end_effector

    @property
    def gripper(self) -> Optional[Gripper]:
        """
        Access reference to the first gripper.

        Return:
            Gripper: Reference to the first gripper instance, or None if there is no gripper.
        """
        return self._grippers[0] if self._grippers else None

    @property
    def grippers(self) -> List[Gripper]:
        """
        Access references to all grippers, e.g. one per arm.

        Return:
            List[Gripper]: Gripper instances.
        """
        return self._grippers

    @property
    def arm_dof(self) -> int:
        """
        Number of robot joints (arms and external robot axes), gripper excluded.

        Return:
            int: Number of robot joints.
        """
        return self._arm_dof

    @property
    def joint_names(self) -> List[str]:
        """
        Robot joint names in controller order, the order of q, dq, tau and the target drives.

        Return:
            List[str]: Robot joint names.
        """
        return self._joint_names

    @property
    def q(self) -> List[float]:
        """
        Get current positions of all robot joints, gripper excluded.

        Return:
            List[float]: Joint positions [rad].
        """
        if self._joint_indices is not None and self._articulation_view.is_physics_handle_valid():
            return self.get_joint_positions(
                joint_indices=self._joint_indices
            ).tolist()
        else:
            return np.zeros(self._arm_dof).tolist()

    @property
    def dq(self) -> List[float]:
        """
        Get current velocities of all robot joints, gripper excluded.

        Return:
            List[float]: Joint velocities [rad/s].
        """
        if self._joint_indices is not None and self._articulation_view.is_physics_handle_valid():
            return self.get_joint_velocities(
                joint_indices=self._joint_indices
            ).tolist()
        else:
            return np.zeros(self._arm_dof).tolist()

    @property
    def tau(self) -> List[float]:
        """
        Get current torques of all robot joints, gripper excluded.

        Return:
            List[float]: Joint torques [Nm].
        """
        if self._joint_indices is not None and self._articulation_view.is_physics_handle_valid():
            return self.get_measured_joint_efforts(
                joint_indices=self._joint_indices
            ).tolist()
        else:
            return np.zeros(self._arm_dof).tolist()

    @property
    def has_ft_sensor(self) -> bool:
        """
        Whether this model carries a wrist 6-DoF force-torque sensor.

        Return:
            bool: True for the "s" variants (Rizon 4s / Rizon 10s).
        """
        return self._has_ft_sensor

    @property
    def wrist_wrench(self) -> (List[float], List[float]):
        """
        Get the simulated wrist 6-DoF force-torque sensor reading, expressed in the sensor frame
        (the link7_ft_sensor sensing joint / link7_distal frame) and reported as the force/torque
        the robot applies ON the environment (Flexiv convention), matching the real Rizon wrist
        sensor.

        The reading is the reaction wrench measured at the link7_ft_sensor sensing joint, which
        captures exactly what is distal to the sensor: link7_distal, the flange frame, and any
        attached tool (gravity, inertia, and contact). Isaac reports that reaction in the child
        (link7_distal) frame -- which is the sensor frame, so no frame shift is needed. Negating it
        yields the force the wrist exerts on the environment.

        Return:
            (List[float], List[float]): (wrist_force [f_x, f_y, f_z] in N,
            wrist_torque [m_x, m_y, m_z] in Nm). Returns zeros before the physics handle is valid.
        """
        if not self._has_ft_sensor or self._ft_force_row is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        if not self._articulation_view.is_physics_handle_valid():
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        # Reaction wrench at the sensing joint, reported in the link7_distal (sensor) frame as
        # [f_x, f_y, f_z, m_x, m_y, m_z].
        wrench = self.get_measured_joint_forces(
            joint_indices=np.array([self._ft_force_row])
        )[0]

        # Negate: measured value is the reaction (proximal-on-distal); the real sensor reports the
        # force/torque the robot applies on the environment (distal-on-proximal equivalent).
        force = -wrench[0:3]
        torque = -wrench[3:6]

        return force.tolist(), torque.tolist()

    def apply_torques(self, tau_d: List[float]) -> None:
        """
        Apply desired torques to all robot joints, gripper excluded.

        Params:
            tau_d (List[float]): Desired joint torques.
        """
        # Apply only to robot joints, leave out gripper joints, which are controlled by gripper controller
        self.set_joint_efforts(tau_d, joint_indices=self._joint_indices)
        return

    def teleport_to(self, q_d: List[float]) -> None:
        """
        Instantly teleport all robot joints to desired positions. This is usually used to set robot to initial positions.
        No control is involved in the process. Call apply_torques() immediately after to kick in the joint controls.

        Params:
            q_d (List[float]): Desired joint positions.
        """
        self.set_joint_positions(q_d, joint_indices=self._joint_indices)
        return

    def _resolve_ft_force_row(self) -> int:
        """
        Resolve the row index into get_measured_joint_forces() for the sensing joint.

        The force array has one row per articulation joint plus a leading base-link row, so the row
        for joint J is (joint order index of J) + 1. The sensing joint is a fixed joint, so it is
        not in the actuated-DoF map; look it up by name in the articulation joint ordering. The
        physics metadata exposes this either as a name->index dict (joint_indices) or a name list
        (joint_names), depending on the backend, so handle both.

        Return:
            int: row index into the get_measured_joint_forces() output.
        """
        meta = self._articulation_view._metadata
        indices = getattr(meta, "joint_indices", None)
        if isinstance(indices, dict) and self._FT_JOINT_NAME in indices:
            return indices[self._FT_JOINT_NAME] + 1
        names = getattr(meta, "joint_names", None)
        if names is not None and self._FT_JOINT_NAME in list(names):
            return list(names).index(self._FT_JOINT_NAME) + 1
        raise KeyError(
            f"joint [{self._FT_JOINT_NAME}] not found in articulation metadata "
            f"(joint_indices/joint_names)"
        )

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize the articulation interface, set up torque drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)

        # Map the robot joints to articulation DoF indices. The articulation orders its DoFs by its
        # own traversal, which for a multi-branch robot differs from the controller order.
        missing = [n for n in self._joint_names if n not in self.dof_names]
        if missing:
            raise KeyError(f"Joints {missing} not found in articulation DoFs {self.dof_names}")
        self._joint_indices = np.array([self.get_dof_index(n) for n in self._joint_names])

        # Resolve the row index into get_measured_joint_forces() for the wrist sensor joint. That
        # call returns one row per articulation joint (row 0 is the base link's incoming joint), so
        # the row for a given joint is joint_index + 1. The sensing joint (link7_ft_sensor) is a
        # FIXED joint with no DoF, so it must be looked up by name in the articulation metadata's
        # joint_indices map rather than via get_dof_index (which only covers actuated DoFs).
        if self._has_ft_sensor:
            try:
                self._ft_force_row = self._resolve_ft_force_row()
            except Exception as e:
                self._ft_force_row = None
                self._logger.error(
                    f"Failed to resolve force-torque sensor joint [{self._FT_JOINT_NAME}], "
                    f"wrist wrench will report zeros: {e}"
                )

        # Initialize end-effector. In the older assets the flange is a rigid body (a 1 g link on a
        # link7_to_flange fixed joint); in the newer SimReady assets it is a site, a plain frame under
        # link7 (or link7_distal) with no physics, see SiteEndEffector. A gripper end effector is a
        # rigid body.
        end_effector_prim = get_current_stage().GetPrimAtPath(self._end_effector_prim_path)
        if end_effector_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            end_effector_class = SingleRigidPrim
        else:
            end_effector_class = SiteEndEffector
        self._end_effector = end_effector_class(
            prim_path=self._end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)

        # Initialize grippers if any
        for gripper in self._grippers:
            if isinstance(gripper, ParallelGripper):
                gripper.initialize(
                    physics_sim_view=physics_sim_view,
                    articulation_apply_action_func=self.apply_action,
                    get_joint_positions_func=self.get_joint_positions,
                    set_joint_positions_func=self.set_joint_positions,
                    dof_names=self._gripper_dof_names(gripper),
                )
            elif isinstance(gripper, SurfaceGripper):
                gripper.initialize(
                    physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof
                )

        # Joints output torque instead of acceleration
        self.get_articulation_controller().set_effort_modes("force")

        # Save default gains because calling _articulation_view.switch_control_mode() will change _articulation_view._default_kps,
        # which makes switching control mode from "effort" back to "position" not possible
        self._default_kps, self._default_kds = self._articulation_view.get_gains()
        return

    def _gripper_dof_names(self, gripper: Gripper) -> List[str]:
        """
        Articulation DoF names as seen by one gripper: its own DoFs by joint prim name, the other
        grippers' DoFs masked out.

        ParallelGripper finds its joints by name, but two copies of the same gripper (e.g. one per
        arm) have the same joint prim names, which the articulation makes unique by suffixing the
        later copies ("finger_joint_0"), so the gripper profile names would only match the first
        copy. A DoF belongs to a gripper when its joint prim is under the gripper's mount prim, the
        child of the robot prim that holds the gripper's end effector.

        Params:
            gripper (Gripper): Gripper to resolve DoF names for.

        Return:
            List[str]: DoF names, with "" for DoFs of other grippers.
        """
        names = list(self.dof_names)
        if len(self._grippers) < 2:
            return names
        dof_paths = getattr(self._articulation_view, "_dof_paths", None)
        if dof_paths is None:
            self._logger.warning("Cannot tell the grippers' DoFs apart; same-named joints may clash")
            return names

        def mount_of(g):
            path = getattr(g, "_end_effector_prim_path", "") or ""
            if not path.startswith(self.prim_path + "/"):
                return None
            return self.prim_path + "/" + path[len(self.prim_path) + 1 :].split("/")[0] + "/"

        own = mount_of(gripper)
        others = [m for m in (mount_of(g) for g in self._grippers) if m and m != own]
        for i, path in enumerate(dof_paths[0]):
            if own and path.startswith(own):
                names[i] = path.rsplit("/", 1)[-1]
            elif any(path.startswith(m) for m in others):
                names[i] = ""
        return names

    def post_reset(self) -> None:
        """
        Post reset articulation
        """
        super().post_reset()
        self._end_effector.post_reset()
        for gripper in self._grippers:
            gripper.post_reset()
        return
