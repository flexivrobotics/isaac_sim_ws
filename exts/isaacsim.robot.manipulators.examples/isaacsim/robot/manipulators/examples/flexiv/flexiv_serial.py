# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import spdlog
from typing import Optional, List
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class FlexivSerial(Robot):
    """
    Control interface for one Flexiv serial robot.

    Params:
        prim_path (str): Primitive path of this robot articulation in the stage. E.g. /World/Flexiv
        name (str): name of this robot articulation.
        end_effector_prim_name (str): name of the primitive in the robot articulation to be used as end effector.
        arm_dof (int): degrees of freedom of the robotic arm, excluding any gripper DoF.
        pos_in_world (Optional[List[float]]): position (x, y, z) of the robot in world [m].
        ori_in_world (Optional[List[float]]): orientation (quaternion w, x, y, z) of the robot in world [].
        gripper (Optional[Gripper]): Constructed gripper instance.
        has_ft_sensor (bool): Whether this model carries a wrist 6-DoF force-torque sensor (the "s"
            variants, e.g. Rizon 4s / Rizon 10s). When True, wrist_wrench reports a simulated reading.
    """

    # Name of the last actuated arm joint. Everything distal to it (link7 + flange + any tool) is
    # what the physical wrist force-torque sensor measures, so its reaction wrench is our sensor source.
    _FT_JOINT_NAME = "joint7"

    # Hard-coded offset [m] along link7's +z from the joint7 (link7) frame to the sensor frame,
    # placing the sensor origin halfway between the joint7 and flange frames. Measured from the USD,
    # the flange origin sits 0.124 m from the link7/joint7 origin along +z with zero x/y (identical for
    # Rizon 4s and 10s), so halfway is 0.062 m. Refine against the real sensor mounting if needed.
    _FT_SENSOR_Z_OFFSET = 0.062

    def __init__(
        self,
        prim_path: str,
        name: str,
        end_effector_prim_name: str,
        arm_dof: int = 7,
        pos_in_world: Optional[List[float]] = None,
        ori_in_world: Optional[List[float]] = None,
        gripper: Optional[Gripper] = None,
        has_ft_sensor: bool = False,
    ) -> None:
        self._arm_dof = arm_dof
        self._gripper = gripper
        self._default_kps = None
        self._default_kds = None
        self._end_effector = None
        self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        self._has_ft_sensor = has_ft_sensor
        # Row index into get_measured_joint_forces() output for the wrist sensor joint. Resolved in
        # initialize() once the articulation metadata is available.
        self._ft_force_row = None
        self._logger = spdlog.ConsoleLogger("flexiv::" + name)

        # Construct base
        super().__init__(
            prim_path=prim_path,
            name=name,
            position=pos_in_world,
            orientation=ori_in_world,
        )
        return

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
            mode, joint_indices=np.arange(0, self._arm_dof)
        )

        self._logger.info(f"Control mode switched to [{mode}]")
        return

    @property
    def end_effector(self) -> SingleRigidPrim:
        """
        Access reference to the _end_effector member.

        Return:
            SingleRigidPrim: Reference to the end-effector instance.
        """
        return self._end_effector

    @property
    def gripper(self) -> Gripper:
        """
        Access reference to the _gripper member.

        Return:
            Gripper: Reference to the gripper instance.
        """
        return self._gripper

    @property
    def q(self) -> List[float]:
        """
        Get current positions of all robot joints, gripper excluded.

        Return:
            List[float]: Joint positions [rad].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_joint_positions(
                joint_indices=np.arange(0, self._arm_dof)
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
        if self._articulation_view.is_physics_handle_valid():
            return self.get_joint_velocities(
                joint_indices=np.arange(0, self._arm_dof)
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
        if self._articulation_view.is_physics_handle_valid():
            return self.get_measured_joint_efforts(
                joint_indices=np.arange(0, self._arm_dof)
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
        (halfway between the joint7 and flange frames) and reported as the force/torque the robot
        applies ON the environment (Flexiv convention), matching the real Rizon wrist sensor.

        The reading is derived from the reaction wrench measured at joint7, which captures everything
        distal to the sensor: link7, the flange frame, and any attached tool (gravity, inertia, and
        contact). Isaac reports that reaction in the child link (link7) frame as the force/torque of
        the parent side on link7; negating it yields the force the wrist exerts on the environment.

        Return:
            (List[float], List[float]): (wrist_force [f_x, f_y, f_z] in N,
            wrist_torque [m_x, m_y, m_z] in Nm). Returns zeros before the physics handle is valid.
        """
        if not self._has_ft_sensor or self._ft_force_row is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        if not self._articulation_view.is_physics_handle_valid():
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        # Reaction wrench at joint7, reported in the link7 frame as [f_x, f_y, f_z, m_x, m_y, m_z].
        wrench = self.get_measured_joint_forces(
            joint_indices=np.array([self._ft_force_row])
        )[0]

        # Negate: measured value is the reaction (parent-on-link7); the real sensor reports the
        # force/torque the robot applies on the environment (link7-on-parent equivalent).
        force = -wrench[0:3]
        torque = -wrench[3:6]

        # Shift the torque reference point from the link7 (joint7) frame origin to the sensor origin,
        # a pure translation of _FT_SENSOR_Z_OFFSET along link7's +z. The two frames are aligned in
        # orientation, so the force is unchanged and only the moment arm r x f is removed:
        #   tau_sensor = tau_link7 - r x f,  with r = (0, 0, offset).
        # r x f = (-offset*f_y, offset*f_x, 0).
        offset = self._FT_SENSOR_Z_OFFSET
        torque = torque - np.array([-offset * force[1], offset * force[0], 0.0])

        return force.tolist(), torque.tolist()

    def apply_torques(self, tau_d: List[float]) -> None:
        """
        Apply desired torques to all robot joints, gripper excluded.

        Params:
            tau_d (List[float]): Desired joint torques.
        """
        # Apply only to robot joints, leave out gripper joints, which are controlled by gripper controller
        self.set_joint_efforts(tau_d, joint_indices=np.arange(0, self._arm_dof))
        return

    def teleport_to(self, q_d: List[float]) -> None:
        """
        Instantly teleport all robot joints to desired positions. This is usually used to set robot to initial positions.
        No control is involved in the process. Call apply_torques() immediately after to kick in the joint controls.

        Params:
            q_d (List[float]): Desired joint positions.
        """
        self.set_joint_positions(q_d, joint_indices=np.arange(0, self._arm_dof))
        return

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize the articulation interface, set up torque drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)

        # Resolve the row index into get_measured_joint_forces() for the wrist sensor joint. That
        # call returns one row per link's incoming joint, offset by one from the DoF index (row 0 is
        # the base link), so the wrist joint's row is get_dof_index(joint7) + 1.
        if self._has_ft_sensor:
            try:
                self._ft_force_row = self.get_dof_index(self._FT_JOINT_NAME) + 1
            except Exception as e:
                self._ft_force_row = None
                self._logger.error(
                    f"Failed to resolve force-torque sensor joint [{self._FT_JOINT_NAME}], "
                    f"wrist wrench will report zeros: {e}"
                )

        # Initialize end-effector
        self._end_effector = SingleRigidPrim(
            prim_path=self._end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)

        # Initialize gripper if any
        if isinstance(self._gripper, ParallelGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        elif isinstance(self._gripper, SurfaceGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof
            )

        # Joints output torque instead of acceleration
        self.get_articulation_controller().set_effort_modes("force")

        # Save default gains because calling _articulation_view.switch_control_mode() will change _articulation_view._default_kps,
        # which makes switching control mode from "effort" back to "position" not possible
        self._default_kps, self._default_kds = self._articulation_view.get_gains()
        return

    def post_reset(self) -> None:
        """
        Post reset articulation
        """
        super().post_reset()
        self._end_effector.post_reset()
        if self._gripper is not None:
            self._gripper.post_reset()
        return
