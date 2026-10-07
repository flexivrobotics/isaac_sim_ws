# Flexiv Isaac Sim Workspace

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)


## Overview

This workspace is an example integration of **NVIDIA Isaac Sim** with **Flexiv Elements Studio**, connected via [Flexiv Sim Plugin](https://github.com/flexivrobotics/flexiv_sim_plugin). It demonstrates how to use the Flexiv Sim Plugin API alongside the Nvidia Isaac Sim API to relay robot control commands and physics state between Elements Studio and Isaac Sim, so that simulated Flexiv robots are driven by the same force-torque controller used on real robots and can be programmed with the Flexiv RDK.

```
┌─────────────────────────────────────────┐
│       User Program with RDK Client      │
│          (send robot commands)          │
└───────────────────┬─────────────────────┘
                    │ RDK API
                    ▼
┌─────────────────────────────────────────┐
│         Flexiv Elements Studio          │
│   (simulated controller + RDK Server)   │
└───────────────────┬─────────────────────┘
                    │ Flexiv Sim Plugin API
                    ▼
┌─────────────────────────────────────────┐
│          Flexiv Isaac Sim Workspace     │
│ (Flexiv Sim Plugin API + Isaac Sim API) │
└───────────────────┬─────────────────────┘
                    │ Isaac Sim API
                    ▼
┌─────────────────────────────────────────┐
│           NVIDIA Isaac Sim              │
│           (physics engine)              │
└─────────────────────────────────────────┘
```


## Compatibility

| **Supported OS** | **Supported processor** | **Supported language** | **Isaac Sim version** |
| ---------------- | ----------------------- | ---------------------- | --------------------- |
| Ubuntu 22.04, 24.04 | x86_64               | Python                 | 6.x                   |

Each release of this workspace works with one release line of Flexiv Sim Plugin (the `flexivsimplugin` Python package) and of Flexiv Elements Studio:

| **Workspace release** | **Isaac Sim version** | **flexivsimplugin** | **Elements Studio** |
| --------------------- | --------------------- | ------------------- | ------------------- |
| v2.2.x                | 6.x                   | 2.2.x               | v3E.2               |
| v1.4.0                | 6.x                   | 1.3.0               | v3.11.2             |
| v1.3                  | 4.5, 5.0              | 1.2.x               | v3.10, v3.11        |

The `v2.x` branch holds the latest release line. For an earlier one, check out its release tag, e.g. `git checkout v1.4.0`.


## Demos

### 1. Tower of Hanoi

[![Rizon 4 Masters the Tower of Hanoi Game in Issac Sim with the Flexiv-Isaac Sim Bridge App](https://img.youtube.com/vi/jZT6Ei0L3gk/0.jpg)](https://www.youtube.com/watch?v=jZT6Ei0L3gk)

### 2. Peg-in-hole

https://github.com/user-attachments/assets/e575bf70-9ffb-47a5-8aec-4cda8d25c08e

### 3. Single robot polish

https://github.com/user-attachments/assets/a0c39e70-4469-4405-a07d-e0d8a0ad589b

### 4. Dual robot polish

https://github.com/user-attachments/assets/7462a9bd-3cfd-40cc-95f7-b4fda0a74f30


## Pre-requisites

Before using the Flexiv Isaac Sim Workspace, install Flexiv Elements Studio and
create a simulated robot by following
[Flexiv Elements Studio Setup](https://github.com/flexivrobotics/flexiv_sim_plugin/blob/v2.x/docs/elements_studio_setup.md).


## Workspace setup

The workspace setup installs the `flexivsimplugin` Python package into Isaac Sim's bundled Python. That plugin is the middleware connecting the Flexiv-Isaac Bridge App to Elements Studio. Everything else runs straight from this repo, so nothing is copied into Isaac Sim:

| **Folder**      | **Contents**                                                      |
| --------------- | ----------------------------------------------------------------- |
| `assets/`       | USDs of every Flexiv robot, the grippers and the example environments |
| `examples/`     | The Flexiv-Isaac Bridge App, its configuration files and the follow-target example |
| `flexiv_isaac/` | The Python package the examples import: robot, calibration, controllers and tasks. The `pick_place` and `stacking` tasks don't work yet. |
| `tools/`        | Connection check, RDK, and a calibrated USD export the bridge app does not need |

This workspace runs against either a natively installed Isaac Sim or the Isaac Sim container. Where later sections say `<isaac_sim_root_dir>`, they mean wherever Isaac Sim lives: `~/isaacsim` (or your install path) for Option A, and `/isaac-sim` inside the container for Option B. Run the commands in later sections from the root of this repo, which is `/workspace` inside the container.

### Option A: native Isaac Sim

1. Install NVIDIA [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html).
2. Note down Isaac Sim's installation directory, e.g. `~/isaacsim`.
3. From this repo, set up the workspace:

       bash setup_ws.sh ~/isaacsim

### Option B: Isaac Sim container

1. Follow NVIDIA's [container installation guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html) to set up the NGC prerequisites (an NGC account and `docker login`). The image itself is pulled automatically on first launch.

2. From this repo, launch a shell in the container:

       bash launch_isaac_sim.sh shell

   This mounts this repo at `/workspace` inside the container, along with the persistent cache directories, and sets the options Isaac Sim needs. The mount is read-only, so edit the configuration files in this repo on the host; the container sees each change straight away. When a local display is available, the shell forwards it, so the bridge app and the examples open their Isaac Sim window. The script's other modes, `native` and `webrtc`, start the plain Isaac Sim app straight away, without a shell to set up this workspace in; run `bash launch_isaac_sim.sh -h` for all options.

3. Inside the container, set up the workspace. The shell starts in `/workspace`:

       bash setup_ws.sh

   Isaac Sim is auto-detected at `/isaac-sim`.

   The container runs with `--rm`, so everything installed inside it is discarded on exit. Re-run this step after each launch; it takes seconds once the wheel is cached.

> The container runs on the host network, because the plugin discovers Elements Studio over Zenoh multicast, which does not cross Docker's default bridge network. On the default network the Bridge App starts normally but never connects.

If you used an earlier release of this workspace, its files installed into Isaac Sim are no longer used, and the Python package `isaacsim.robot.manipulators.examples.flexiv` is now `flexiv_isaac`.

The setup installs the newest 2.2.x release of `flexivsimplugin`, the release line this workspace supports, so re-running it picks up plugin bug fixes. To install a specific version instead, pass it with `--plugin-version`, e.g. `--plugin-version 2.2.0`.

## Verify setup

To verify that the workspace setup is successful, run the example Python application:

    <isaac_sim_root_dir>/python.sh examples/follow_target_with_rmpflow.py

WARNING: When running Isaac Sim for the first time, it takes a couple of minutes to warm up the shader cache. You will notice that the CPU is fully loaded and the Isaac Sim window seems frozen. Please wait patiently and do not force quit the program.

After the example program is up and running, select the `TargetCube` prim under `World` from the Stage view, then drag it around, the robot TCP should follow the cube. The flange keeps pointing down, so it follows wherever it can reach with that orientation, mostly in front of the robot.


### Run Flexiv-Isaac Bridge App

1. Edit the configuration file `examples/single_arm_app_config.yaml` according to the instructions in it.
   Optionally, with the simulated robot started in Elements Studio, check that Isaac Sim can reach it before starting the app:

       <isaac_sim_root_dir>/python.sh tools/check_sim_plugin_connection.py "Enlight L-123456"

   It reports for each serial number whether the plugin finds the robot, and what to check if not.
2. Start Flexiv-Isaac Bridge App using configurations in `single_arm_app_config.yaml`:

       <isaac_sim_root_dir>/python.sh examples/flexiv_isaac_bridge_app.py --config examples/single_arm_app_config.yaml

3. The app will launch an Isaac Sim window and start the physics loop (i.e. *Play*) automatically.
4. Go back to Elements Studio. Go to *Settings* → *Remote Mode*, then enable Remote Mode and select *Ethernet* from the drop-down list; the app needs it to pull the robot's calibration through RDK, see [Robot calibration](#robot-calibration). Then restart the exited simulator by toggle on the *Connect* button.
5. Wait for the connection to establish. If the connection is successful, you should see in Elements Studio a robot at home pose with no error. The app then calibrates the robot, see below, and logs `Calibrated robot [...]`.

### Robot calibration

Each robot's kinematics are calibrated per unit, e.g. where an Enlight LL's two arms are mounted. The bundled USDs carry a model's nominal kinematics, so the app calibrates each robot in Isaac Sim once it connects, in three steps:

1. It connects to the robot in Elements Studio with the nominal kinematics of the USD. The robot's RDK server is only reachable while it is connected.
2. It pulls the robot's calibrated kinematics through [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk), which `setup_ws.sh` installs, while the physics loop keeps running. RDK needs Remote Mode (*Ethernet*) on in Elements Studio.
3. It applies them to that robot in Isaac Sim, then resets the simulation for about a quarter of a second, keeping every robot's joint positions.

Nothing is written to disk, so the calibration is pulled again each time the app starts. The kinematics template RDK fills in is fetched from [flexiv_description](https://github.com/flexivrobotics/flexiv_description), so the computer needs internet access. If the calibration cannot be pulled, e.g. because Remote Mode is off, the app logs a warning and the robot keeps the nominal kinematics. Calibration is supported for the Rizon series, Enlight L and Enlight LL; other models log a warning and keep the nominal kinematics.

To use a calibrated USD outside this app, e.g. in your own Isaac Sim scenes, export one with [tools/calibration/export_calibrated_usd.py](tools/calibration/README.md).

### Verify everything is working

Jogging from Elements Studio is not possible in Remote Mode. Once the app has logged `Calibrated robot [...]`, turn Remote Mode off in Elements Studio and restart the simulated robot (*CHANGE CONNECTION*, then toggle the *Connect* button off and on). The robot keeps its calibration until the app is restarted.

1. In Elements Studio, use the simulated motion bar to enter free-drive mode.
2. Use Cartesian or joint jogging to move the robot around, check that the robot in Isaac Sim is also moving.
3. Jog the robot along Cartesian Z axis and let the robot make contact with the ground (or, with `multi_robot_app_config.yaml`, the desk in the example environment), check that a large TCP force is rendered as an orange line in Elements Studio visualization.
4. Create a test project and add some primitives, then execute the project and check that the robot in Isaac Sim is acting as desired.

## After the first run

To restart the whole setup after the first run:

1. Start the Flexiv-Isaac Bridge app first.
2. Then start the simulated robot in Elements Studio.

If you have made some changes and need to restart the Isaac Sim app:

1. Close Flexiv-Isaac Bridge app.
2. In Elements Studio, click *CHANGE CONNECTION*, then toggle off the *Connect* button to close the simulated robot. You do not need to exit the whole Elements Studio program.
3. Restart Flexiv-Isaac Bridge app.
4. Toggle on the *Connect* button to restart the simulated robot.

Alternatively, you can leave the simulated robot running and just restart the Isaac Sim app. The simulated robot in Elements Studio will sync with Isaac Sim after the app is restarted. However, soft error might occur and you can just clear them from Elements Studio and continue to normal operations.

## Dual-arm robots (MICO, Enlight LL)

A dual-arm robot, such as Enlight LL, MICO Core, MICO Plus or MICO Ultra, is **one** robot: one serial number and one controller drive both arms and any waist joints (external robot axes). So it is a single robot block in the configuration file, not two, and it needs only one Elements Studio.

1. Edit the configuration file `examples/mico_app_config.yaml`: set `serial_number` to the simulated robot you created in Elements Studio, and `usd` to the USD of the same model.
   For an Enlight LL, use `enlight_ll_app_config.yaml` instead. Where its two arms are mounted is part of the robot's calibration, so the bundled `enlight_ll` USD has both arms at the origin, overlapping, until the app [calibrates the robot](#robot-calibration) once it connects.
2. Start Flexiv-Isaac Bridge App using configurations in `mico_app_config.yaml` (or `enlight_ll_app_config.yaml`):

       <isaac_sim_root_dir>/python.sh examples/flexiv_isaac_bridge_app.py --config examples/mico_app_config.yaml

The robot states and commands carry joint values without joint names, so the app sends and applies them in the order the controller uses, which it derives from the USD the same way the controller derives it from the URDF; the order is printed at startup, and `initial_q` uses the same order. If the controller's robot model has a different number of joints than the USD, the app logs an error and ignores the commands. Each arm can carry its own gripper, mounted on `system1_left_arm_flange` or `system1_right_arm_flange` and driven by its own pair of digital outputs (`DOUT[0]` / `DOUT[1]` open / close the first gripper, `DOUT[2]` / `DOUT[3]` the second, by default).

## Multi-robot support

This framework supports simulating and controlling multiple robots, each with its own Elements Studio. (`multi_robot_app_config.yaml` sets up two single-arm robots; for one robot with two arms, see [Dual-arm robots](#dual-arm-robots-mico-enlight-ll).)

1. Add multiple robots in the configuration file `examples/multi_robot_app_config.yaml`.
2. Start Flexiv-Isaac Bridge App using the updated configurations in `multi_robot_app_config.yaml`:

       <isaac_sim_root_dir>/python.sh examples/flexiv_isaac_bridge_app.py --config examples/multi_robot_app_config.yaml

3. Find a second Ubuntu 22.04 computer, connect it to the first computer via Ethernet cable. Then on the first computer, check that this wired Ethernet connection is visible in the network settings, then change the IPv4 setting of this wired connection to "Shared to other computers". Alternatively, connect both computers to the same network router via **wired** connection.
4. Make sure both computers are able to ping each other.
5. Install Flexiv Elements Studio on the **second** computer, then create a new simulated robot. Now each computer has a robot controller with Elements Studio.
6. Start the first simulated robot on the first computer, then wait for connection with Isaac Sim. You should see one of the robots in Isaac Sim moves a little bit when the connection is established.
7. Start the second simulated robot on the second computer, then wait for connection with Isaac Sim. You should see the other robot in Isaac Sim moves a little bit when the connection is established.
8. Execute test projects from both Elements Studios and check that both robots are working in Isaac Sim.

## Collect data from the simulated robot(s)

You can collect data from the simulated robot(s) using [Flexiv DDK](https://github.com/flexivrobotics/flexiv_ddk) (Data Distribution Kit) at up to 1kHz frequency:

1. Set up Flexiv DDK according to the instructions found in the repo.
2. Start Isaac Sim and Elements Studio.
3. Run DDK programs to collect data from one or more simulated robots.

Note: the DDK program doesn't have to run on the same computer as the Elements Studio, it can be any computer that's under the same local network as the Elements Studio computer.

## Control the simulated robot(s) programmatically

Besides using the drag-and-drop graphical interface in Elements Studio to create projects to control the simulated robot(s), you can also control them programmatically using [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk) (Robotic Development Kit) in a real-time or non-real-time manner:

1. Set up Flexiv RDK according to the instructions found in the repo.
2. Start Isaac Sim and Elements Studio.
3. In Elements Studio, go to *Settings* → *Remote Mode*, then enable Remote Mode and select *Ethernet* from the drop-down list.
4. Restart the simulated robot by clicking *CHANGE CONNECTION*, then toggle off and on the *Connect* button.
5. Run RDK programs to control one or more simulated robots.

   For example, [tools/rdk/dual_arm_joint_swing.py](tools/rdk/dual_arm_joint_swing.py) swings every joint of every arm of a robot around its home pose, so you can see both arms of a dual-arm robot move in Isaac Sim:

       python3 tools/rdk/dual_arm_joint_swing.py "Enlight LL-123456" --amplitude 6 --cycles 3

Note: the RDK program doesn't have to run on the same computer as the Elements Studio, it can be any computer that's under the same local network as the Elements Studio computer.
