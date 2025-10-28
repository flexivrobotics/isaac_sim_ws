# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# App version
APP_VERSION = "1.3"

# Compatible flexivsimplugin version
COMPATIBLE_SIM_PLUGIN_VER = "1.2.0"

import yaml
import spdlog
import numpy as np
from typing import List, Dict
from enum import Enum
from argparse import ArgumentParser
from dataclasses import dataclass
from isaacsim import SimulationApp

# Middleware plugin for connecting to Flexiv Elements Studio
import flexivsimplugin

# Check version
if flexivsimplugin.__version__ != COMPATIBLE_SIM_PLUGIN_VER:
    raise ImportError(
        f"flexivsimplugin=={COMPATIBLE_SIM_PLUGIN_VER} is required, but found {flexivsimplugin.__version__}"
    )


# Load config file
argparser = ArgumentParser()
argparser.add_argument("--config", required=True, help="Path to YAML config file")
args = argparser.parse_args()

# Start simulation main window
simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

# Import isaac modules after SimulationApp is started
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

# Physics and render loop period [sec]
RENDER_FREQ = 60.0
PHYSICS_FREQ = 2000.0


# Gripper status
class GripperStatus(Enum):
    INIT = 0
    OPENED = 1
    CLOSED = 2


class BridgeRunner(object):
    """
    Set up world and run joint impedance control.

    Params:
        physics_dt (float): Physics loop period of the scene [sec].
        render_dt (float): Render loop period of the scene [sec].
        config (Dict): Configurations parsed from the config file.
        initial_q (List[float], optional): Initial joint positions [rad].
    """

    # Data struct for a single robot
    @dataclass
    class SingleRobotData:
        name: str
        instance: FlexivSerial
        sim_plugin: flexivsimplugin.UserNode
        last_connected: bool
        gripper_status: GripperStatus

    # Robot degrees of freedom
    ROBOT_DOF = 7

    def __init__(
        self,
        physics_dt,
        render_dt,
        config: Dict,
        initial_q: List[float] = [0.0] * ROBOT_DOF,
    ) -> None:
        # Initialize logger
        self._logger = spdlog.ConsoleLogger("Flexiv-Isaac Bridge App")

        # fmt: off
        self._logger.info("——————————————————————————————————————————————————————————")
        self._logger.info(f"———            Flexiv-Isaac Bridge App v{APP_VERSION}            ———")
        self._logger.info("——————————————————————————————————————————————————————————")
        # fmt: on

        # Save initial q
        self._initial_q = initial_q

        # Create world
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
            set_defaults=False,
        )

        # Enable GPU dynamics if specified
        if config.get("gpu_dynamics", False):
            self._world.get_physics_context().enable_gpu_dynamics(True)

        # Load environment and reset world
        env_usd = config.get("env_usd", "")
        if env_usd:
            # Add user-provided environment to stage
            add_reference_to_stage(usd_path=env_usd, prim_path="/World")
        else:
            # Add empty environment to stage
            self._world.scene.add_default_ground_plane()
        self._world.reset()

        # Add cameras
        self._cameras = []
        for c in config.get("cameras", []):
            cam_name = c["name"]
            pos_in_world = [float(c["position"][i]) for i in ["x", "y", "z"]]
            ori_in_world = [float(c["orientation"][i]) for i in ["w", "x", "y", "z"]]
            camera = Camera(
                prim_path="/World/" + cam_name,
                frequency=c["fps"],
                resolution=tuple(c["resolution"]),
                position=pos_in_world,
                orientation=ori_in_world,
            )
            camera.set_focal_length(c["focal_length"])
            # Use the same camera axes as the Isaac Sim UI
            camera.set_world_pose(
                position=pos_in_world, orientation=ori_in_world, camera_axes="usd"
            )
            self._cameras.append(camera)
            self._logger.info(
                f"Added camera [/World/{cam_name}] located at {pos_in_world} {ori_in_world} in world"
            )

        # Create data struct for all robots and add them to stage
        self._robots = []
        for r in config.get("robots", []):
            # Parse config of this robot
            serial_num = r["serial_number"]
            usd_path = r["usd"]
            pos_in_world = [float(r["position"][i]) for i in ["x", "y", "z"]]
            ori_in_world = [float(r["orientation"][i]) for i in ["w", "x", "y", "z"]]

            # Replace dash with underscore in serial number to avoid prim path error
            serial_num = serial_num.replace("-", "_")

            # Add this robot to stage
            prim_path = "/World/Flexiv/" + serial_num
            self._logger.info(
                f"Adding robot usd [{usd_path}] to stage at prim path [{prim_path}]"
            )
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

            # Configure gripper if the usd name suggests a gripper exists in the model
            gripper = None
            end_effector_prim_name = "flange"
            if "Grav" in usd_path:
                # This gripper has only one actuation joint, but the API requires two,
                # thus providing a non-actuation joint (gains = 0) as a place holder
                end_effector_prim_name = "Grav_gripper/right_finger_tip"
                gripper = ParallelGripper(
                    end_effector_prim_path=prim_path + "/" + end_effector_prim_name,
                    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
                    joint_opened_positions=np.array([45.0, 0]),
                    joint_closed_positions=np.array([-8.88, 0]),
                )
                self._logger.info(
                    "The usd name suggests a Grav gripper exists in the model, gripper control will be enabled"
                )
            elif "Robotiq" in usd_path:
                # This gripper has only one actuation joint, but the API requires two,
                # thus providing a non-actuation joint (gains = 0) as a place holder
                end_effector_prim_name = (
                    "Robotiq_2F_85_flattened/Robotiq_2F_85/right_inner_finger"
                )
                gripper = ParallelGripper(
                    end_effector_prim_path=prim_path + "/" + end_effector_prim_name,
                    joint_prim_names=["finger_joint", "right_inner_finger_joint"],
                    joint_opened_positions=np.array([0, 0]),
                    joint_closed_positions=np.array([45, 0]),
                )
                self._logger.info(
                    "The usd name suggests a Robotiq gripper exists in the model, gripper control will be enabled"
                )
            else:
                self._logger.info("Gripper control is not enabled")

            # Add robot to stage
            robot = self._world.scene.add(
                FlexivSerial(
                    prim_path=prim_path,
                    name=serial_num,
                    end_effector_prim_name=end_effector_prim_name,
                    arm_dof=BridgeRunner.ROBOT_DOF,
                    pos_in_world=pos_in_world,
                    ori_in_world=ori_in_world,
                    gripper=gripper,
                )
            )
            self._logger.info(
                f"Added robot [/World/Flexiv/{serial_num}] located at {pos_in_world} {ori_in_world} in world"
            )

            # Append single robot data struct
            self._robots.append(
                self.SingleRobotData(
                    name=serial_num,
                    instance=robot,
                    sim_plugin=flexivsimplugin.UserNode(serial_num),
                    last_connected=False,
                    gripper_status=GripperStatus.INIT,
                )
            )

        # Add physics callback
        self._world.add_physics_callback("robot_step", callback_fn=self.on_physics_step)

        # Reset world once
        self._world.reset()

        # Initialize cameras
        for cam in self._cameras:
            cam.initialize()

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
            robot.sim_plugin.SendRobotStates(
                flexivsimplugin.SimRobotStates(
                    self._servo_cycle,
                    robot.instance.q,
                    robot.instance.dq,
                )
            )

        for robot in self._robots:
            if robot.sim_plugin.connected():
                # Upon reconnection, set joint torque control mode
                if not robot.last_connected:
                    self._logger.info(f"Connected to robot [{robot.name}]")
                    robot.instance.switch_control_mode("effort")

                # Wait for new commands to arrive before proceeding current cycle
                timeout_ms = 100
                if robot.sim_plugin.WaitForRobotCommands(timeout_ms):
                    # Apply joint torques
                    robot.instance.apply_torques(
                        robot.sim_plugin.robot_commands().target_drives
                    )
                else:
                    self._logger.warn(f"Missed 1 message from [{robot.name}]")

                # Gripper control based on digital output signal
                dout_list = list(
                    robot.sim_plugin.robot_commands().digital_outputs
                )  # Convert map to list
                if dout_list:
                    # DOUT[0] high = open gripper
                    if dout_list[0]:
                        # Ignore if already opened
                        if robot.gripper_status != GripperStatus.OPENED:
                            self._logger.info("Opening gripper")
                            robot.instance.gripper.open()
                            robot.gripper_status = GripperStatus.OPENED
                    # DOUT[1] high = close gripper
                    if dout_list[1]:
                        # Ignore if already closed
                        if robot.gripper_status != GripperStatus.CLOSED:
                            self._logger.info("Closing gripper")
                            robot.instance.gripper.close()
                            robot.gripper_status = GripperStatus.CLOSED

                # Set last connected status
                robot.last_connected = True

            else:
                if robot.last_connected:
                    # Upon disconnection, transit this robot from torque control to position control to hold its current pose
                    self._logger.error(f"Disconnected from robot [{robot.name}]")
                    robot.instance.switch_control_mode("position")
                    robot.instance.teleport_to(robot.instance.q)
                    robot.gripper_status = GripperStatus.INIT

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
                for robot in self._robots:
                    robot.last_connected = False
            if self._world.is_playing():
                if self._reset_needed:
                    self._world.reset()
                    self._reset_needed = False
                    # Put robot to initial pose
                    for robot in self._robots:
                        robot.instance.switch_control_mode("position")
                        robot.instance.teleport_to(self._initial_q)


def main():
    # Create runner to handle everything
    runner = BridgeRunner(
        physics_dt=1.0 / PHYSICS_FREQ,
        render_dt=1.0 / RENDER_FREQ,
        config=yaml.safe_load(open(args.config)),
        initial_q=[0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0],
    )
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
