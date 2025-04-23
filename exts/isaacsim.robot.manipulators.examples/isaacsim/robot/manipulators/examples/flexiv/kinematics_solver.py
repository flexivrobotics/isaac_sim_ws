# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional

import omni.isaac.motion_generation.interface_config_loader as interface_config_loader
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation.articulation_kinematics_solver import (
    ArticulationKinematicsSolver,
)
from omni.isaac.motion_generation.lula.kinematics import LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Flexiv robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (Articulation): An initialized Articulation object representing this Flexiv
        end_effector_frame_name (Optional[str]): The name of the Flexiv end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
    """

    def __init__(
        self,
        robot_articulation: Articulation,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        kinematics_config = (
            interface_config_loader.load_supported_lula_kinematics_solver_config(
                "Rizon4"
            )
        )
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        if end_effector_frame_name is None:
            end_effector_frame_name = "flange"

        ArticulationKinematicsSolver.__init__(
            self, robot_articulation, self._kinematics, end_effector_frame_name
        )

        return
