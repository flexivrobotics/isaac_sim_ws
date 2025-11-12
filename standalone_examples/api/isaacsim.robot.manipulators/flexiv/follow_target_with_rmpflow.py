# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# Start simulation
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.flexiv.controllers.rmpflow_controller import (
    RMPFlowController,
)
from isaacsim.robot.manipulators.examples.flexiv.tasks import FollowTarget
from isaacsim.storage.native import get_assets_root_path

# Use Rizon4 usd stored in Isaac Sim assets
usd_path = get_assets_root_path() + "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"

my_world = World(stage_units_in_meters=1.0)
my_task = FollowTarget(name="flexiv_follow_target", usd_path=usd_path)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("flexiv_follow_target").get_params()
flexiv_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_flexiv = my_world.scene.get_object(flexiv_name)
my_controller = RMPFlowController(
    name="target_follower_controller", robot_articulation=my_flexiv
)
articulation_controller = my_flexiv.get_articulation_controller()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)

simulation_app.close()
