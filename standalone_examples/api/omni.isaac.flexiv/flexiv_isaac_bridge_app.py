# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import spdlog
import numpy as np
from typing import List, Optional
from argparse import ArgumentParser
from dataclasses import dataclass
from isaacsim import SimulationApp

from flexivisaacbridge import IsaacNode

# Parse program arguments
argparser = ArgumentParser()

argparser.add_argument(
    "--robot",
    action="append",
    nargs=5,
    help="Add one or more robots specifying [serial_number usd_path pos_x pos_y pos_z], e.g. --robot Rizon4-GYdBow omniverse://localhost/Library/Rizon4.usd -0.7 0.31 0.7 --robot Rizon4s-TPqXaI omniverse://localhost/Library/Rizon4s_with_Grav.usd -0.67 -0.46 0.7",
    required=True,
)
argparser.add_argument(
    "--env",
    help="Path to the usd of the environment to load. If not provided, an empty environment with default ground plane will be used.",
    required=False,
)
args = argparser.parse_args()


# Start simulation main window
simulation_app = SimulationApp({"headless": False})

# Import isaac modules after SimulationApp is started
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.flexiv import Flexiv

# Physics and render loop period [sec]
PHYSICS_FREQ = 2000.0
RENDER_FREQ = 60.0

# lcm topic prefix for robot state and command messages */
STATES_TOPIC_PREFIX = "flexiv_isaac_bridge/robot_states/"
COMMANDS_TOPIC_PREFIX = "flexiv_isaac_bridge/robot_commands/"


class RcaBridgeRunner(object):
    """
    Set up world and run joint impedance control.

    Params:
        physics_dt (float): Physics loop period of the scene [sec].
        render_dt (float): Render loop period of the scene [sec].
        robots (List[List[str]]): 3d list with size n by 3. n robots, each contains a string list [serial_number, usd_path, pos_in_world].
        initial_q (np.ndarray, optional): Initial joint positions [rad].
    """

    # Data struct for a single robot
    @dataclass
    class SingleRobotData:
        name: str
        instance: Flexiv
        isaac_node: IsaacNode
        last_connected: bool
        is_gripper_opened: bool

    # Robot degrees of freedom
    ROBOT_DOF = 7

    def __init__(
        self,
        physics_dt,
        render_dt,
        robots: List[List[str]],
        initial_q: np.ndarray = np.zeros(ROBOT_DOF),
    ) -> None:
        # Save initial q
        self._initial_q = initial_q

        # Initialize logger
        self._logger = spdlog.ConsoleLogger("flexiv_isaac_bridge_app")

        # Create world
        self._world = World(
            stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt
        )
        # Load environment and reset world
        if args.env is None:
            # Add empty environment to stage
            self._world.scene.add_default_ground_plane()
        else:
            # Add user-provided environment to stage
            add_reference_to_stage(usd_path=args.env, prim_path="/World")
        self._world.reset()

        # Create data struct for all robots and add them to stage
        self._robots = []
        for r in robots:
            # Parse arguments
            serial_num = r[0]
            usd_path = r[1]
            pos_in_world = [float(r[2]), float(r[3]), float(r[4])]

            # Replace dash with underscore in serial number to avoid prim path error
            serial_num = serial_num.replace("-", "_")

            # Configure gripper if the usd name suggests a gripper exists in the model
            gripper_joint_names = None
            gripper_opened_joint_positions = None
            gripper_closed_joint_positions = None
            end_effector_prim_name = None
            if "Grav" in usd_path:
                # This gripper only has one actuation joint, but the API requires two,
                # thus providing a non-actuation joint (gains = 0) as a place holder
                gripper_joint_names = ["finger_joint", "right_outer_knuckle_joint"]
                gripper_opened_joint_positions = np.array([45.0, 0])
                gripper_closed_joint_positions = np.array([-8.88, 0])
                end_effector_prim_name = "Grav_gripper/right_finger_tip"
                self._logger.info(
                    "The usd name suggests a Grav gripper exists in the model, gripper control will be enabled"
                )
            elif "Robotiq" in usd_path:
                # This gripper only has one actuation joint, but the API requires two,
                # thus providing a non-actuation joint (gains = 0) as a place holder
                gripper_joint_names = ["finger_joint", "right_inner_finger_joint"]
                gripper_opened_joint_positions = np.array([0, 0])
                gripper_closed_joint_positions = np.array([45, 0])
                end_effector_prim_name = (
                    "Robotiq_2F_85_flattened/Robotiq_2F_85/right_inner_finger"
                )
                self._logger.info(
                    "The usd name suggests a Robotiq gripper exists in the model, gripper control will be enabled"
                )
            else:
                self._logger.info("Gripper control is not enabled")

            # Add robot to stage
            robot = self._world.scene.add(
                Flexiv(
                    prim_path="/World/Flexiv/" + serial_num,
                    name=serial_num,
                    end_effector_prim_name=end_effector_prim_name,
                    usd_path=usd_path,
                    pos_in_world=pos_in_world,
                    gripper_joint_names=gripper_joint_names,
                    gripper_opened_joint_positions=gripper_opened_joint_positions,
                    gripper_closed_joint_positions=gripper_closed_joint_positions,
                )
            )
            self._logger.info(
                f"Created robot [/World/Flexiv/{serial_num}] located at {pos_in_world} in world"
            )

            # Append single robot data struct
            self._robots.append(
                self.SingleRobotData(
                    name=serial_num,
                    instance=robot,
                    isaac_node=IsaacNode(serial_num),
                    last_connected=False,
                    is_gripper_opened=True,
                )
            )

        # Add physics callback
        self._world.add_physics_callback("robot_step", callback_fn=self.on_physics_step)

        # Reset world once
        self._world.reset()

        # Initialize other members
        self._reset_needed = False
        self._servo_cycle = 0

        # Put robot to initial pose
        for robot in self._robots:
            robot.instance.teleport_to(self._initial_q)

    def on_physics_step(self, dt) -> None:
        """
        Physics call back to host the real-time joint control loop. The loop period of this callback is provided by [dt].

        Params:
            dt (float): Loop period, same as [physics_dt] [sec].
        """
        for robot in self._robots:
            # Publish fresh robot states to all Flexiv Nodes before doing anything else
            robot.isaac_node.PublishRobotStates(
                self._servo_cycle, robot.instance.q.tolist(), robot.instance.dq.tolist()
            )

        for robot in self._robots:
            if robot.isaac_node.connected():
                # Upon reconnection, set joint torque control mode
                if not robot.last_connected:
                    self._logger.info(f"Connected to robot [{robot.name}]")
                    robot.instance.switch_control_mode("effort")

                # Wait for new commands to arrive before proceeding current cycle
                timeout_ms = 100
                if not robot.isaac_node.WaitForNewRobotCommands(timeout_ms):
                    self._logger.warn(f"Missed 1 message from [{robot.name}]")

                # Apply joint torques
                robot.instance.apply_torques(robot.isaac_node.target_torques())

                # Gripper control based on digital output signal
                dout_list = list(
                    robot.isaac_node.digital_outputs()
                )  # Convert map to list
                if dout_list:
                    # DOUT[0] high = open gripper
                    if dout_list[0]:
                        # Ignore if already opened
                        if not robot.is_gripper_opened:
                            self._logger.info("Opening gripper")
                            robot.instance.gripper.open()
                            robot.is_gripper_opened = True
                    # DOUT[1] high = close gripper
                    elif dout_list[1]:
                        # Ignore if already closed
                        if robot.is_gripper_opened:
                            self._logger.info("Closing gripper")
                            robot.instance.gripper.close()
                            robot.is_gripper_opened = False

                # Set last connected status
                robot.last_connected = True

            else:
                if robot.last_connected:
                    # Upon disconnection, transit this robot from torque control to position control to hold its current pose
                    self._logger.error(f"Disconnected from robot [{robot.name}]")
                    robot.instance.teleport_to(robot.instance.q)
                    robot.instance.switch_control_mode("position")

                # Set last connected status
                robot.last_connected = False

        # Increment server cycle
        self._servo_cycle += 1

    def run(self) -> None:
        """
        Poll world step, which will step physics and rendering with specified physics_dt and render_dt.
        """
        while simulation_app.is_running():
            self._world.step(render=True)
            # Reset world if needed
            if self._world.is_stopped() and not self._reset_needed:
                self._reset_needed = True
            if self._world.is_playing():
                if self._reset_needed:
                    self._world.reset()
                    self._reset_needed = False
                    # Put robot to initial pose
                    for robot in self._robots:
                        robot.instance.teleport_to(self._initial_q)


def main():
    # Create runner to handle everything
    runner = RcaBridgeRunner(
        physics_dt=1.0 / PHYSICS_FREQ,
        render_dt=1.0 / RENDER_FREQ,
        robots=args.robot,
        initial_q=np.array([0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0]),
    )
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()