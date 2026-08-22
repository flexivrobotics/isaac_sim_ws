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
COMPATIBLE_SIM_PLUGIN_VER = "1.3.0"

import os
import sys
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

# Check version. This app is developed against flexivsimplugin
# COMPATIBLE_SIM_PLUGIN_VER; a mismatch is warned about rather than fatal so the
# app can run against in-development plugin builds. Tighten to a hard failure
# once the plugin version is stable.
if flexivsimplugin.__version__ != COMPATIBLE_SIM_PLUGIN_VER:
    print(
        f"WARNING: this app targets flexivsimplugin=={COMPATIBLE_SIM_PLUGIN_VER}, "
        f"but found {flexivsimplugin.__version__}. Continuing anyway; behavior may "
        f"differ if the plugin API has changed.",
        file=sys.stderr,
    )


# Load config file
argparser = ArgumentParser()
argparser.add_argument("--config", required=True, help="Path to YAML config file")
args = argparser.parse_args()

# Start simulation main window
simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

# Import isaac modules after SimulationApp is started
# The Flexiv examples live in the isaacsim.robot.manipulators.examples extension,
# which Isaac Sim 6.x ships as deprecated and does not enable by default. Enable
# it so its Python modules (imported below) become importable.
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot.manipulators.examples")

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.sensors.camera import Camera
from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from pxr import Usd, UsdPhysics, Sdf, Gf

# Physics and render loop period [sec]
RENDER_FREQ = 60.0
PHYSICS_FREQ = 2000.0


# Gripper status
class GripperStatus(Enum):
    INIT = 0
    OPENED = 1
    CLOSED = 2


# Built-in gripper profiles, keyed by the tool's mount prim name (the `prim_name`
# in a robot's `tool` config block). A tool USD is referenced onto the arm under
# `<robot_prim>/<prim_name>`, then a fixed joint mounts it to the arm flange. Each
# profile describes how to drive that gripper as an Isaac ParallelGripper:
#   ee            : end-effector prim, RELATIVE to the mount prim
#   mount_body    : the gripper's base rigid body to fix to the flange, RELATIVE
#                   to the mount prim
#   joints        : the two joint_prim_names the ParallelGripper API requires
#                   (the Grav has a single actuation joint, so the second is a
#                   non-actuated placeholder driven with zero gains)
#   opened/closed : joint positions [deg] for the open / closed states
# Add a new gripper by adding an entry here + shipping its USD under
# data/flexiv/grippers/; no other code change is needed.
GRIPPER_PROFILES = {
    "Grav_gripper": {
        "ee": "right_finger_tip",
        "mount_body": "gripper_base",
        "joints": ["finger_joint", "right_outer_knuckle_joint"],
        "opened": [45.0, 0.0],
        "closed": [-8.88, 0.0],
    },
}


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

            # Determine from the serial number whether this model carries a wrist force-torque
            # sensor. The "s" variants report it, e.g. "Rizon 4s-ROn3YJ" / "Rizon10s-000001".
            # The model token before the dash may be written with or without a space
            # ("Rizon 4s" or "Rizon4s"), so normalize by stripping whitespace before matching.
            model = serial_num.split("-")[0].strip().lower().replace(" ", "")
            has_ft_sensor = model in ("rizon4s", "rizon10s")
            if has_ft_sensor:
                self._logger.info(
                    f"Robot [{serial_num}] is an 's' variant; wrist force-torque sensor enabled"
                )

            # Replace dash with underscore in serial number to avoid prim path error
            serial_num = serial_num.replace("-", "_")

            # Add this robot to stage
            prim_path = "/World/Flexiv/" + serial_num
            self._logger.info(
                f"Adding robot usd [{usd_path}] to stage at prim path [{prim_path}]"
            )
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

            # Attach a tool (gripper) if the robot config declares one. The tool
            # USD is referenced onto the arm and fixed to the flange at load time,
            # so no pre-combined "<robot>_with_Grav" asset is needed.
            gripper = None
            end_effector_prim_name = "flange"
            tool = r.get("tool")
            if tool:
                gripper, end_effector_prim_name = self._attach_tool(prim_path, tool)
            else:
                self._logger.info("No tool configured; gripper control is not enabled")

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
                    has_ft_sensor=has_ft_sensor,
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

    def _find_flange_path(self, robot_prim_path: str) -> str:
        """
        Resolve the flange prim path under a robot, tolerating both USD layouts.

        In the old flat layout the flange is a direct child (`<robot>/flange`); in
        the SimReady layout it is nested (`<robot>/Geometry/base_link/.../link7/
        flange`). Return the direct path if it exists, otherwise search the robot
        subtree for a prim named "flange".

        Params:
            robot_prim_path (str): Prim path of the robot articulation root.

        Return:
            str: Full prim path of the flange.
        """
        stage = get_current_stage()
        direct = robot_prim_path + "/flange"
        if stage.GetPrimAtPath(direct).IsValid():
            return direct
        root = stage.GetPrimAtPath(robot_prim_path)
        for prim in Usd.PrimRange(root):
            if prim.GetName() == "flange":
                return prim.GetPath().pathString
        raise RuntimeError(f"No 'flange' prim found under [{robot_prim_path}]")

    def _attach_tool(self, robot_prim_path: str, tool: Dict):
        """
        Reference a tool (gripper) USD onto the arm and fix it to the flange.

        The tool USD is added under `<robot_prim_path>/<prim_name>` and a fixed
        joint mounts its base rigid body (from the gripper profile) to the arm
        flange, so the tool joins the arm's articulation. A ParallelGripper is then
        constructed from the built-in profile keyed by `prim_name`.

        Params:
            robot_prim_path (str): Prim path of the robot articulation root.
            tool (Dict): Tool config block with keys:
                usd (str): Path to the tool USD (already resolved to absolute).
                prim_name (str): Mount prim name; also the GRIPPER_PROFILES key.

        Return:
            (ParallelGripper, str): The gripper instance and the end-effector prim
            name RELATIVE to the robot prim (e.g. "Grav_gripper/right_finger_tip").
        """
        usd_path = tool["usd"]
        prim_name = tool["prim_name"]
        profile = GRIPPER_PROFILES.get(prim_name)
        if profile is None:
            raise ValueError(
                f"No gripper profile for tool prim_name [{prim_name}]. "
                f"Known: {sorted(GRIPPER_PROFILES)}"
            )

        tool_prim_path = robot_prim_path + "/" + prim_name
        self._logger.info(
            f"Attaching tool usd [{usd_path}] at prim path [{tool_prim_path}]"
        )
        add_reference_to_stage(usd_path=usd_path, prim_path=tool_prim_path)

        # Fix the gripper base to the flange. Joint frames are coincident (the tool
        # USD is authored so its base sits at the flange), so both local anchors are
        # identity -- matching the old baked "flange_to_gripper" fixed joint.
        stage = get_current_stage()
        flange_path = self._find_flange_path(robot_prim_path)
        mount_body_path = tool_prim_path + "/" + profile["mount_body"]
        mount_joint_path = tool_prim_path + "/flange_to_" + profile["mount_body"]
        mount = UsdPhysics.FixedJoint.Define(stage, mount_joint_path)
        mount.CreateBody0Rel().SetTargets([Sdf.Path(flange_path)])
        mount.CreateBody1Rel().SetTargets([Sdf.Path(mount_body_path)])
        mount_prim = mount.GetPrim()
        mount_prim.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(
            Gf.Vec3f(0, 0, 0))
        mount_prim.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(
            Gf.Vec3f(0, 0, 0))
        mount_prim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1, 0, 0, 0))
        mount_prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(1, 0, 0, 0))

        end_effector_prim_name = prim_name + "/" + profile["ee"]
        # This gripper has only one actuation joint, but the ParallelGripper API
        # requires two, so the second is a non-actuation placeholder (gains = 0).
        gripper = ParallelGripper(
            end_effector_prim_path=robot_prim_path + "/" + end_effector_prim_name,
            joint_prim_names=list(profile["joints"]),
            joint_opened_positions=np.array(profile["opened"]),
            joint_closed_positions=np.array(profile["closed"]),
        )
        self._logger.info(
            f"Tool [{prim_name}] attached; gripper control enabled "
            f"(ee=[{end_effector_prim_name}])"
        )
        return gripper, end_effector_prim_name

    def on_physics_step(self, dt) -> None:
        """
        Physics call back to host the real-time joint control loop. The loop period of this callback is provided by [dt].

        Params:
            dt (float): Loop period, same as [physics_dt] [sec].
        """
        for robot in self._robots:
            # Publish fresh robot states to all Flexiv Nodes before doing anything else
            if robot.instance.has_ft_sensor:
                # "s" variants also report a simulated wrist 6-DoF force-torque sensor reading
                wrist_force, wrist_torque = robot.instance.wrist_wrench
                robot_states = flexivsimplugin.SimRobotStates(
                    self._servo_cycle,
                    robot.instance.q,
                    robot.instance.dq,
                    wrist_force,
                    wrist_torque,
                )
            else:
                robot_states = flexivsimplugin.SimRobotStates(
                    self._servo_cycle,
                    robot.instance.q,
                    robot.instance.dq,
                )
            robot.sim_plugin.SendRobotStates(robot_states)

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


def resolve_usd_paths(config):
    """Resolve relative ``usd`` / ``env_usd`` paths in the config.

    Relative paths are resolved against the Isaac Sim installation root (the
    ``ISAAC_PATH`` environment variable, set by ``python.sh``), so the default
    config works regardless of the current working directory. Absolute paths are
    left unchanged. This lets the shipped config point at the bundled example
    assets under ``extsDeprecated/`` without hardcoding a machine-specific path.
    """
    isaac_root = os.environ.get("ISAAC_PATH", "")

    def resolve(path):
        if path and not os.path.isabs(path):
            return os.path.join(isaac_root, path)
        return path

    if config.get("env_usd"):
        config["env_usd"] = resolve(config["env_usd"])
    for robot in config.get("robots", []):
        if robot.get("usd"):
            robot["usd"] = resolve(robot["usd"])
        tool = robot.get("tool")
        if tool and tool.get("usd"):
            tool["usd"] = resolve(tool["usd"])
    return config


def main():
    # Create runner to handle everything
    runner = BridgeRunner(
        physics_dt=1.0 / PHYSICS_FREQ,
        render_dt=1.0 / RENDER_FREQ,
        config=resolve_usd_paths(yaml.safe_load(open(args.config))),
        initial_q=[0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0],
    )
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
