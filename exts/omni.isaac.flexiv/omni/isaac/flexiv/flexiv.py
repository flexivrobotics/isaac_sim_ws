# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import spdlog
from typing import Union, List, Optional
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper


class Flexiv(Robot):
    """
    Flexiv 7-DOF robot control interface.

    Params:
        prim_path (str): Primitive path of this robot articulation in the stage. E.g. /World/Flexiv
        name (str): name of this robot articulation.
        end_effector_prim_name (Optional[str]): name of the primitive in the robot articulation to be used as end effector.
        usd_path (Optional[str]): path to robot usd file. If not specified, flexiv_rizon4.usd from Isaac Sim assets library will be used.
        pos_in_world (Optional[Union[np.ndarray, List[float]]]): position (x, y, z) of the robot in world [m].
        ori_in_world (Optional[Union[np.ndarray, List[float]]]): orientation (quaternion w, x, y, z) of the robot in world [].
        gripper_joint_names (Optional[List[str]], optional): names of the gripper's actuation joints.
        gripper_opened_joint_positions (Optional[np.ndarray], optional): joint positions of the left finger joint and the right finger joint respectively when opened.
        gripper_closed_joint_positions (Optional[np.ndarray], optional): joint positions of the left finger joint and the right finger joint respectively when closed.
    """

    def __init__(
        self,
        prim_path: str,
        name: str,
        end_effector_prim_name: Optional[str] = None,
        usd_path: Optional[str] = None,
        pos_in_world: Optional[Union[np.ndarray, List[float]]] = None,
        ori_in_world: Optional[Union[np.ndarray, List[float]]] = None,
        gripper_joint_names: Optional[List[str]] = None,
        gripper_opened_joint_positions: Optional[np.ndarray] = None,
        gripper_closed_joint_positions: Optional[np.ndarray] = None,
    ) -> None:
        # Initialize logger
        self._logger = spdlog.ConsoleLogger("flexiv::" + name)

        # Add robot primitive to the given path if not already exists
        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            # Load usd from provided path
            if usd_path:
                self._logger.info(
                    f"Mounting provided robot usd [{usd_path}] at prim path [{prim_path}]"
                )
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            # Load usd from assets library
            else:
                default_usd_path = "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"
                self._logger.warn(
                    f"Robot usd is not provided, using default usd from Isaac Sim asset library"
                )
                self._logger.info(
                    f"Mounting default robot usd [{default_usd_path}] at prim path [{prim_path}]"
                )
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    self._logger.error("Cannot find Isaac Sim assets folder")
                usd_path = assets_root_path + default_usd_path
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

            # Set which primitive in the robot articulation to be used as end effector
            if end_effector_prim_name is None:
                end_effector_prim_name = "flange"
            self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name

        # Init base
        super().__init__(
            prim_path=prim_path,
            name=name,
            position=pos_in_world,
            orientation=ori_in_world,
        )

        # Initialize gripper
        if gripper_joint_names is not None:
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_joint_names,
                joint_opened_positions=gripper_opened_joint_positions,
                joint_closed_positions=gripper_closed_joint_positions,
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
        self._articulation_view.switch_control_mode(mode, joint_indices=np.arange(0, 7))

        self._logger.info(f"Control mode switched to [{mode}]")
        return

    @property
    def end_effector(self) -> RigidPrim:
        """
        Access reference to the _end_effector member.

        Return:
            RigidPrim: Reference to the end-effector instance.
        """
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        """
        Access reference to the _gripper member.

        Return:
            ParallelGripper: Reference to the gripper instance.
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
            return self.get_joint_positions(joint_indices=np.arange(0, 7))
        else:
            return np.zeros(7)

    @property
    def dq(self) -> np.ndarray:
        """
        Get current joint velocities.

        Return:
            np.ndarray: Joint velocities [rad/s].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_joint_velocities(joint_indices=np.arange(0, 7))
        else:
            return np.zeros(7)

    @property
    def tau(self) -> np.ndarray:
        """
        Get current joint torques.

        Return:
            np.ndarray: Joint torques [Nm].
        """
        if self._articulation_view.is_physics_handle_valid():
            return self.get_measured_joint_efforts(joint_indices=np.arange(0, 7))
        else:
            return np.zeros(7)

    def apply_torques(self, tau_d: Union[List, np.ndarray]) -> None:
        """
        Apply desired joint torques to the robot articulation.

        Params:
            tau_d (Union[List, np.ndarray]): Desired joint torques.
        """
        # Apply only to robot joints, leave out gripper joints, which are controlled by gripper controller
        self.set_joint_efforts(tau_d, joint_indices=np.arange(0, 7))
        return

    def teleport_to(self, q_d: Union[List, np.ndarray]) -> None:
        """
        Instantly teleport robot joints to desired positions. This is usually used to set robot to initial positions.
        No control is involved in the process. Call apply_torques() immediately after to kick in the joint controls.

        Params:
            q_d (Union[List, np.ndarray]): Desired joint positions.
        """
        self.set_joint_positions(q_d, joint_indices=np.arange(0, 7))
        return

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize the articulation interface, set up torque drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)

        # Initialize end-effector
        self._end_effector = RigidPrim(
            prim_path=self._end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)

        # Joints output torque instead of acceleration
        self.get_articulation_controller().set_effort_modes("force")

        # Save default gains becaues calling _articulation_view.switch_control_mode() will change _articulation_view._default_kps,
        # which makes switching control mode from "effort" back to "position" not possible
        self._default_kps, self._default_kds = self._articulation_view.get_gains()

        # Initialize gripper if added
        if hasattr(self, "_gripper"):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        return

    def post_reset(self) -> None:
        """
        Post reset articulation
        """
        super().post_reset()
        if hasattr(self, "_gripper"):
            self._gripper.post_reset()
        return
