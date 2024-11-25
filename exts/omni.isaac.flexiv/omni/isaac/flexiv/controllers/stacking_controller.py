# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import List

import omni.isaac.manipulators.controllers as manipulators_controllers
from omni.isaac.core.articulations import Articulation
from omni.isaac.flexiv.controllers.pick_place_controller import PickPlaceController
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper


class StackingController(manipulators_controllers.StackingController):
    """[summary]

    Args:
        name (str): [description]
        gripper (ParallelGripper): [description]
        robot_prim_path (str): [description]
        picking_order_cube_names (List[str]): [description]
        robot_observation_name (str): [description]
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: Articulation,
        picking_order_cube_names: List[str],
        robot_observation_name: str,
    ) -> None:
        manipulators_controllers.StackingController.__init__(
            self,
            name=name,
            pick_place_controller=PickPlaceController(
                name=name + "_pick_place_controller",
                gripper=gripper,
                robot_articulation=robot_articulation,
            ),
            picking_order_cube_names=picking_order_cube_names,
            robot_observation_name=robot_observation_name,
        )
        return
