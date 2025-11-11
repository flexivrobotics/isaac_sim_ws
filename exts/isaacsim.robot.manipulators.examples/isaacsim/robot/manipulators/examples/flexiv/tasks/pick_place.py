# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial


class PickPlace(tasks.PickPlace):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "flexiv_pick_place".
        cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "flexiv_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> FlexivSerial:
        """[summary]

        Returns:
            FlexivSerial: [description]
        """
        flexiv_prim_path = find_unique_string_name(
            initial_name="/World/Flexiv",
            is_unique_fn=lambda x: not is_prim_path_valid(x),
        )
        flexiv_robot_name = find_unique_string_name(
            initial_name="Rizon4",
            is_unique_fn=lambda x: not self.scene.object_exists(x),
        )
        return FlexivSerial(prim_path=flexiv_prim_path, name=flexiv_robot_name)
