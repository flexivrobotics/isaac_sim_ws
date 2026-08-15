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
| Ubuntu 22.04     | x86_64                  | Python                 | 6.x                   |

> This branch targets **NVIDIA Isaac Sim 6.x**. For Isaac Sim 5.x, use the
> `isaac-sim-5` branch.


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

Before using the Flexiv Isaac Sim Workspace, follow the setup instructions in
[Flexiv Sim Plugin](https://github.com/flexivrobotics/flexiv_sim_plugin?tab=readme-ov-file#flexiv-elements-studio-setup).


## Workspace setup

1. Install NVIDIA [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html).
2. Note down Isaac Sim's installation directory, e.g. `~/isaacsim`.
3. Run the following script in this repo to install Flexiv's Isaac Sim extensions and apps to the Isaac Sim installation directory so that Isaac Sim can load them:

       bash install_ws.sh <isaac_sim_root_dir>

   For example:

       bash install_ws.sh ~/isaacsim

4. Install the Python dependencies into Isaac Sim's bundled Python:

       cd <isaac_sim_root_dir>
       ./python.sh -m pip install spdlog

5. Install the `flexivsimplugin` Python package into Isaac Sim's bundled Python.
   This is the middleware that connects the Flexiv-Isaac Bridge App to Elements
   Studio; follow the [Flexiv Sim Plugin](https://github.com/flexivrobotics/flexiv_sim_plugin)
   setup, but make sure it is installed under Isaac Sim's Python (`./python.sh
   -m pip ...`), not your system Python — the app runs under `./python.sh`.

   > The bridge app checks `flexivsimplugin.__version__` against the
   > `COMPATIBLE_SIM_PLUGIN_VER` constant at the top of
   > `flexiv_isaac_bridge_app.py` and refuses to start on a mismatch. Make sure
   > the installed plugin version matches that constant.

## Verify setup

To verify that the workspace setup is successful, run the example Python application:

    cd <isaac_sim_root_dir>
    ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/follow_target_with_rmpflow.py extsDeprecated/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd

WARNING: When running Isaac Sim for the first time, it takes a couple of minutes to warm up the shader cache. You will notice that the CPU is fully loaded and the Isaac Sim window seems frozen. Please wait patiently and do not force quit the program.

> Troubleshooting: if startup fails with `NVML_ERROR_LIB_RM_VERSION_MISMATCH` or
> `CUDA error 804`, your loaded NVIDIA kernel module and the installed userspace
> libraries are out of sync (common right after a driver upgrade). Reboot to
> load the matching module, then confirm with `nvidia-smi`.

After the example program is up and running, select the `TargetCube` prim under `World` from the Stage view, then drag it around, the robot TCP should follow the cube.


### Run Flexiv-Isaac Bridge App

1. Edit the configuration file `standalone_examples/api/isaacsim.robot.manipulators/flexiv/app_config.yaml` according to the instructions in it.
2. Start Flexiv-Isaac Bridge App using configurations in `app_config.yaml`:

       cd <isaac_sim_root_dir>
       ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py --config standalone_examples/api/isaacsim.robot.manipulators/flexiv/app_config.yaml

3. The app will launch an Isaac Sim window and start the physics loop (i.e. *Play*) automatically.
4. Go back to Elements Studio, then restart the exited simulator by toggle on the *Connect* button.
5. Wait for the connection to establish. If the connection is successful, you should see in Elements Studio a robot at home pose with no error.

### Verify everything is working

1. In Elements Studio, use the simulated motion bar to enter free-drive mode.
2. Use Cartesian or joint jogging to move the robot around, check that the robot in Isaac Sim is also moving.
3. Jog the robot along Cartesian Z axis and let the robot make contact with the desk in the example environment, check that a large TCP force is rendered as an orange line in Elements Studio visualization.
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

## Multi-robot support

This framework supports simulating and controlling multiple robots:

1. Add multiple robots in the configuration file `standalone_examples/api/isaacsim.robot.manipulators/flexiv/app_config.yaml`.
2. Start Flexiv-Isaac Bridge App using the updated configurations in `app_config.yaml`:

       cd <isaac_sim_root_dir>
       ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py --config standalone_examples/api/isaacsim.robot.manipulators/flexiv/app_config.yaml

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

Note: the RDK program doesn't have to run on the same computer as the Elements Studio, it can be any computer that's under the same local network as the Elements Studio computer.
