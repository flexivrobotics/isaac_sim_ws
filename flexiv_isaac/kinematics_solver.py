# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional

from isaacsim.core.prims import SingleArticulation
from flexiv_isaac import motion_config
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver

class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Flexiv robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (SingleArticulation): An initialized Articulation object representing this Flexiv
        end_effector_frame_name (Optional[str]): The name of the Flexiv end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
    """

    def __init__(
        self,
        robot_articulation: SingleArticulation,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        # The Enlight L's kinematics, shipped in assets/enlight_l/rmpflow
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=motion_config.ROBOT_DESCRIPTION_PATH,
            urdf_path=motion_config.URDF_PATH,
        )

        if end_effector_frame_name is None:
            end_effector_frame_name = motion_config.END_EFFECTOR_FRAME

        ArticulationKinematicsSolver.__init__(
            self, robot_articulation, self._kinematics, end_effector_frame_name
        )

        return
