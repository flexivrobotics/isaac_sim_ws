# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional

import numpy as np
import omni.isaac.core.tasks as tasks
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.flexiv import Flexiv


class FollowTarget(tasks.FollowTarget):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "flexiv_follow_target".
        target_prim_path (Optional[str], optional): [description]. Defaults to None.
        target_name (Optional[str], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
        flexiv_prim_path (Optional[str], optional): [description]. Defaults to None.
        flexiv_robot_name (Optional[str], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "flexiv_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        flexiv_prim_path: Optional[str] = None,
        flexiv_robot_name: Optional[str] = None,
        usd_path: Optional[str] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._flexiv_prim_path = flexiv_prim_path
        self._flexiv_robot_name = flexiv_robot_name
        self._usd_path = usd_path
        return

    def set_robot(self) -> Flexiv:
        """[summary]

        Returns:
            Flexiv: [description]
        """
        if self._flexiv_prim_path is None:
            self._flexiv_prim_path = find_unique_string_name(
                initial_name="/World/Flexiv",
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )
        if self._flexiv_robot_name is None:
            self._flexiv_robot_name = find_unique_string_name(
                initial_name="Rizon4",
                is_unique_fn=lambda x: not self.scene.object_exists(x),
            )
        return Flexiv(
            prim_path=self._flexiv_prim_path,
            name=self._flexiv_robot_name,
            usd_path=self._usd_path,
        )
