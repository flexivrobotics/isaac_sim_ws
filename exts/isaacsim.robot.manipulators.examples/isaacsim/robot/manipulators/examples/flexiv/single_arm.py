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
from typing import Sequence, Optional, Union, List
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class FlexivSingleArm(Robot):
    """
    Control interface of a single Flexiv robotic arm.

    Params:
        prim_path (str): Primitive path of this robot articulation in the stage. E.g. /World/Flexiv
        name (str): name of this robot articulation.
        end_effector_prim_name (str): name of the primitive in the robot articulation to be used as end effector.
        arm_dof (int): degrees of freedom of the robotic arm, excluding any gripper DoF.
        pos_in_world (Optional[Sequence[float]]): position (x, y, z) of the robot in world [m].
        ori_in_world (Optional[Sequence[float]]): orientation (quaternion w, x, y, z) of the robot in world [].
        gripper (Optional[Gripper]): Constructed gripper instance.
    """

    def __init__(
        self,
        prim_path: str,
        name: str,
        end_effector_prim_name: str,
        arm_dof: int,
        pos_in_world: Optional[Sequence[float]] = None,
        ori_in_world: Optional[Sequence[float]] = None,
        gripper: Optional[Gripper] = None,
    ) -> None:
        self._arm_dof = arm_dof
        self._gripper = gripper
        self._end_effector = None
        self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
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
        Switch control mode for all robot joints.

        Params:
            mode (str): Desired control mode, options are "position", "velocity", and "effort".
        """
        # Reset default gains for articulation view
        self._articulation_view.set_gains(kps=self._default_kps, kds=self._default_kds)

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
    def q(self) -> np.ndarray:
        """
        Get current joint positions.

        Return:
            np.ndarray: Joint positions [rad].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_joint_positions(joint_indices=np.arange(0, self._arm_dof))
        else:
            return np.zeros(self._arm_dof)

    @property
    def dq(self) -> np.ndarray:
        """
        Get current joint velocities.

        Return:
            np.ndarray: Joint velocities [rad/s].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_joint_velocities(joint_indices=np.arange(0, self._arm_dof))
        else:
            return np.zeros(self._arm_dof)

    @property
    def tau(self) -> np.ndarray:
        """
        Get current joint torques.

        Return:
            np.ndarray: Joint torques [Nm].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_measured_joint_efforts(
                joint_indices=np.arange(0, self._arm_dof)
            )
        else:
            return np.zeros(self._arm_dof)

    def apply_torques(self, tau_d: Union[List, np.ndarray]) -> None:
        """
        Apply desired joint torques to the robot articulation.

        Params:
            tau_d (Union[List, np.ndarray]): Desired joint torques.
        """
        # Apply only to robot joints, leave out gripper joints, which are controlled by gripper controller
        self.set_joint_efforts(tau_d, joint_indices=np.arange(0, self._arm_dof))
        return

    def teleport_to(self, q_d: Union[List, np.ndarray]) -> None:
        """
        Instantly teleport robot joints to desired positions. This is usually used to set robot to initial positions.
        No control is involved in the process. Call apply_torques() immediately after to kick in the joint controls.

        Params:
            q_d (Union[List, np.ndarray]): Desired joint positions.
        """
        self.set_joint_positions(q_d, joint_indices=np.arange(0, self._arm_dof))
        return

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize the articulation interface, set up torque drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)

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

        # Save default gains becaues calling _articulation_view.switch_control_mode() will change _articulation_view._default_kps,
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
