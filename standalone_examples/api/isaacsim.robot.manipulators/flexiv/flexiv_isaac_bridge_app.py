# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# App version
APP_VERSION = "2.2.0"

# Compatible flexivsimplugin release line: any patch release in it works, e.g.
# 2.2.0 or 2.2.1, so plugin fixes need no new release of this app
COMPATIBLE_SIM_PLUGIN_VER = "2.2"

import os
import sys
import yaml
import logging
import numpy as np
from typing import List, Dict
from enum import Enum
from argparse import ArgumentParser
from dataclasses import dataclass
from isaacsim import SimulationApp

# Middleware plugin for connecting to Flexiv Elements Studio
import flexivsimplugin

# Check version. A plugin outside the COMPATIBLE_SIM_PLUGIN_VER line is warned
# about rather than fatal so the app can run against in-development plugin
# builds. Tighten to a hard failure once the plugin version is stable.
if flexivsimplugin.__version__.split(".")[:2] != COMPATIBLE_SIM_PLUGIN_VER.split("."):
    print(
        f"WARNING: this app targets flexivsimplugin {COMPATIBLE_SIM_PLUGIN_VER}.x, "
        f"but found {flexivsimplugin.__version__}. Continuing anyway; behavior may "
        f"differ if the plugin API has changed.",
        file=sys.stderr,
    )


# Send this app's log messages to the console. Done here, before Isaac Sim
# starts, so it also covers the loggers used by the Flexiv extension modules.
logging.basicConfig(level=logging.INFO, format="[%(name)s] [%(levelname)s] %(message)s")


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
from isaacsim.robot.manipulators.examples.flexiv.flexiv_serial import (
    controller_joint_order,
    usd_home_joint_positions,
)
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from pxr import Usd, UsdPhysics, Sdf, Gf

# Physics and render loop period [sec]
RENDER_FREQ = 60.0
PHYSICS_FREQ = 2000.0

# Number of digital output ports in a sim command (sim_plugin::kIOPorts, not exposed in Python)
SIM_PLUGIN_IO_PORTS = 16


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
        (Each robot's initial joint positions [rad] come from its `initial_q` config, else the
        home pose authored in its USD.)
    """

    # Data struct for a gripper driven by two digital outputs of the robot
    @dataclass
    class GripperData:
        instance: ParallelGripper
        dout_open: int
        dout_close: int
        status: GripperStatus = GripperStatus.INIT

    # Data struct for a single robot
    @dataclass
    class SingleRobotData:
        name: str
        instance: FlexivSerial
        sim_plugin: flexivsimplugin.UserNode
        last_connected: bool
        grippers: List["BridgeRunner.GripperData"]
        initial_q: List[float]
        size_error_logged: bool = False

    def __init__(
        self,
        physics_dt,
        render_dt,
        config: Dict,
    ) -> None:
        # Initialize logger
        self._logger = logging.getLogger("Flexiv-Isaac Bridge App")

        # fmt: off
        self._logger.info("——————————————————————————————————————————————————————————")
        self._logger.info(f"———            Flexiv-Isaac Bridge App v{APP_VERSION}            ———")
        self._logger.info("——————————————————————————————————————————————————————————")
        # fmt: on
        self._logger.info(f"Using flexivsimplugin v{flexivsimplugin.__version__}")

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

        # Stop the timeline while robots and tools are added. While it plays, PhysX parses every
        # stage change on its own, so a tool referenced before its mount joint exists is parsed
        # outside the robot articulation and never joins it (a second gripper then has no DoFs).
        # Stopped, PhysX parses each robot with its tools in one go at the next reset.
        self._world.stop()

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

            # Sanitize the serial number for use in a prim path, which allows
            # neither spaces nor dashes. Studio displays serial numbers with a
            # space in the model name ("Rizon 4-123456"), so stripping spaces is
            # required, not cosmetic. This is also exactly how the plugin derives
            # its own topic suffix, so both sides stay in agreement.
            serial_num = serial_num.replace(" ", "").replace("-", "_")

            # Add this robot to stage
            prim_path = "/World/Flexiv/" + serial_num
            self._logger.info(
                f"Adding robot usd [{usd_path}] to stage at prim path [{prim_path}]"
            )
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

            # Robot joints in controller order, before any tool adds its own joints. The sim
            # messages carry joint values without names, so this order must match the controller's.
            joint_names = r.get("joint_order") or controller_joint_order(prim_path)

            # Attach tools (grippers) if the robot config declares any: a single `tool`, or a
            # `tools` list, e.g. one per arm of a dual-arm robot. Each tool USD is referenced onto
            # the robot and fixed to a flange at load time, so no pre-combined "<robot>_with_Grav"
            # asset is needed.
            tools = r.get("tools") or ([r["tool"]] if r.get("tool") else [])
            grippers = []
            end_effector_prim_name = r.get("end_effector", "flange")
            for i, tool in enumerate(tools):
                gripper, ee = self._attach_tool(prim_path, tool)
                grippers.append(
                    self.GripperData(
                        instance=gripper,
                        dout_open=int(tool.get("dout_open", 2 * i)),
                        dout_close=int(tool.get("dout_close", 2 * i + 1)),
                    )
                )
                ports = (grippers[-1].dout_open, grippers[-1].dout_close)
                if not all(0 <= port < SIM_PLUGIN_IO_PORTS for port in ports):
                    raise ValueError(
                        f"Robot [{serial_num}] tool {i} digital outputs {ports} out of range "
                        f"[0, {SIM_PLUGIN_IO_PORTS})"
                    )
                if i == 0:
                    end_effector_prim_name = ee
            if not tools:
                self._logger.info("No tool configured; gripper control is not enabled")

            # Add robot to stage
            robot = self._world.scene.add(
                FlexivSerial(
                    prim_path=prim_path,
                    name=serial_num,
                    end_effector_prim_name=end_effector_prim_name,
                    pos_in_world=pos_in_world,
                    ori_in_world=ori_in_world,
                    grippers=[g.instance for g in grippers],
                    has_ft_sensor=has_ft_sensor,
                    joint_names=joint_names,
                )
            )

            # Initial joint positions [rad] in controller order. Without `initial_q`, start at the
            # home pose authored in the USD, the same SRDF home pose the controller starts from.
            initial_q = r.get("initial_q")
            if initial_q is None:
                home_q = usd_home_joint_positions(prim_path, robot.joint_names)
                missing = [n for n, q in zip(robot.joint_names, home_q) if q is None]
                if missing:
                    self._logger.warning(
                        f"Robot [{serial_num}] USD has no home position for joints {missing}; "
                        f"starting them at 0. Set `initial_q` to match the controller's home pose."
                    )
                initial_q = [0.0 if q is None else q for q in home_q]
            initial_q = [float(q) for q in initial_q]
            if len(initial_q) != robot.arm_dof:
                raise ValueError(
                    f"Robot [{serial_num}] initial_q has {len(initial_q)} values, but the robot "
                    f"has {robot.arm_dof} joints: {robot.joint_names}"
                )
            self._logger.info(
                f"Added robot [/World/Flexiv/{serial_num}] located at {pos_in_world} {ori_in_world} in world"
            )
            self._logger.info(
                f"Robot [{serial_num}] initial joint positions [rad]: "
                f"{np.round(initial_q, 4).tolist()}"
            )

            # Append single robot data struct
            self._robots.append(
                self.SingleRobotData(
                    name=serial_num,
                    instance=robot,
                    sim_plugin=flexivsimplugin.UserNode(serial_num),
                    last_connected=False,
                    grippers=grippers,
                    initial_q=initial_q,
                )
            )

        # Reset world once, which also initializes the robots
        self._world.reset()

        # Initialize cameras
        for cam in self._cameras:
            cam.initialize()

        # Initialize other members
        self._reset_needed = False
        self._servo_cycle = 0

        # Add physics callback, once the robots it drives are initialized
        self._world.add_physics_callback("robot_step", callback_fn=self.on_physics_step)

        # Put robot to initial pose
        for robot in self._robots:
            robot.instance.teleport_to(robot.initial_q)

    def _find_flange_path(self, robot_prim_path: str, flange: str = "flange") -> str:
        """
        Resolve a flange prim path under a robot, tolerating the USD layouts in use.

        In the old flat layout the flange is a direct child (`<robot>/flange`); in
        the SimReady layout it is nested (`<robot>/Geometry/base_link/.../link7/
        flange`); a dual-arm asset has one per arm, prefixed with the arm
        (`system1_left_arm_flange`, `system1_right_arm_flange`). Return the direct
        path if it exists, else the prim named [flange] in the robot subtree, else
        the only prim whose name ends with "_<flange>".

        Params:
            robot_prim_path (str): Prim path of the robot articulation root.
            flange (str): Flange prim name, e.g. "system1_right_arm_flange".

        Return:
            str: Full prim path of the flange.
        """
        stage = get_current_stage()
        direct = robot_prim_path + "/" + flange
        if stage.GetPrimAtPath(direct).IsValid():
            return direct
        root = stage.GetPrimAtPath(robot_prim_path)
        prims = list(Usd.PrimRange(root))
        for prim in prims:
            if prim.GetName() == flange:
                return prim.GetPath().pathString
        suffixed = [p.GetPath().pathString for p in prims if p.GetName().endswith("_" + flange)]
        if len(suffixed) == 1:
            return suffixed[0]
        if len(suffixed) > 1:
            names = [path.rsplit("/", 1)[-1] for path in suffixed]
            raise RuntimeError(
                f"Robot [{robot_prim_path}] has several flanges {names}; set `flange` in the "
                f"tool config to pick one"
            )
        raise RuntimeError(f"No [{flange}] prim found under [{robot_prim_path}]")

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
                prim_name (str): GRIPPER_PROFILES key; also the mount prim name
                    unless mount_name is given.
                mount_name (str, optional): Mount prim name, which must be unique
                    per robot, e.g. to attach one gripper per arm.
                flange (str, optional): Flange prim to mount on. Defaults to
                    "flange"; required on a dual-arm robot.

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

        mount_name = tool.get("mount_name", prim_name)
        tool_prim_path = robot_prim_path + "/" + mount_name
        self._logger.info(
            f"Attaching tool usd [{usd_path}] at prim path [{tool_prim_path}]"
        )
        add_reference_to_stage(usd_path=usd_path, prim_path=tool_prim_path)

        # Fix the gripper base to the flange. Joint frames are coincident (the tool
        # USD is authored so its base sits at the flange), so both local anchors are
        # identity -- matching the old baked "flange_to_gripper" fixed joint.
        # The flange may be a rigid body (older assets) or a site, a plain frame
        # with no physics (SimReady assets). A joint body may be any xformable: for
        # a site, USD physics attaches the joint to the site's rigid-body parent
        # (link7 or link7_distal), with the anchor at the site's pose on it.
        stage = get_current_stage()
        flange_path = self._find_flange_path(robot_prim_path, tool.get("flange", "flange"))
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

        end_effector_prim_name = mount_name + "/" + profile["ee"]
        # This gripper has only one actuation joint, but the ParallelGripper API
        # requires two, so the second is a non-actuation placeholder (gains = 0).
        gripper = ParallelGripper(
            end_effector_prim_path=robot_prim_path + "/" + end_effector_prim_name,
            joint_prim_names=list(profile["joints"]),
            joint_opened_positions=np.array(profile["opened"]),
            joint_closed_positions=np.array(profile["closed"]),
        )
        self._logger.info(
            f"Tool [{prim_name}] attached to [{flange_path}] as [{mount_name}]; gripper "
            f"control enabled (ee=[{end_effector_prim_name}])"
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
                    # Apply joint torques. The command carries one value per robot joint, in
                    # controller order; a size mismatch means the controller's robot model does
                    # not match this USD, so apply nothing rather than drive the wrong joints.
                    target_drives = robot.sim_plugin.robot_commands().target_drives
                    if len(target_drives) == robot.instance.arm_dof:
                        robot.instance.apply_torques(target_drives)
                    elif not robot.size_error_logged:
                        self._logger.error(
                            f"Robot [{robot.name}] received {len(target_drives)} joint commands, "
                            f"but its USD has {robot.instance.arm_dof} joints "
                            f"{robot.instance.joint_names}; check that the robot model in Elements "
                            f"Studio matches the configured USD. Commands are ignored."
                        )
                        robot.size_error_logged = True
                else:
                    self._logger.warning(f"Missed 1 message from [{robot.name}]")

                # Gripper control based on digital output signals. Each gripper has its own pair
                # of ports, DOUT[0] / DOUT[1] for the first gripper by default: open / close.
                dout_list = list(
                    robot.sim_plugin.robot_commands().digital_outputs
                )  # Convert map to list
                for i, gripper in enumerate(robot.grippers):
                    if not dout_list:
                        break
                    if dout_list[gripper.dout_open]:
                        # Ignore if already opened
                        if gripper.status != GripperStatus.OPENED:
                            self._logger.info(f"Opening gripper {i}")
                            gripper.instance.open()
                            gripper.status = GripperStatus.OPENED
                    if dout_list[gripper.dout_close]:
                        # Ignore if already closed
                        if gripper.status != GripperStatus.CLOSED:
                            self._logger.info(f"Closing gripper {i}")
                            gripper.instance.close()
                            gripper.status = GripperStatus.CLOSED

                # Set last connected status
                robot.last_connected = True

            else:
                if robot.last_connected:
                    # Upon disconnection, transit this robot from torque control to position control to hold its current pose
                    self._logger.error(f"Disconnected from robot [{robot.name}]")
                    robot.instance.switch_control_mode("position")
                    robot.instance.teleport_to(robot.instance.q)
                    for gripper in robot.grippers:
                        gripper.status = GripperStatus.INIT

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
                        robot.instance.teleport_to(robot.initial_q)


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
        for tool in robot.get("tools") or ([robot["tool"]] if robot.get("tool") else []):
            if tool.get("usd"):
                tool["usd"] = resolve(tool["usd"])
    return config


def main():
    # Create runner to handle everything
    runner = BridgeRunner(
        physics_dt=1.0 / PHYSICS_FREQ,
        render_dt=1.0 / RENDER_FREQ,
        config=resolve_usd_paths(yaml.safe_load(open(args.config))),
    )
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
