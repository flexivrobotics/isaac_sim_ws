# Isaac Sim Workspace

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Add Flexiv robots to NVIDIA Isaac Sim and control them using Flexiv Elements Studio or Flexiv RDK with the actual force/torque controller used on real robots.

## Compatibility

| **Supported OS** | **Supported processor** | **Supported language** |
| ---------------- | ----------------------- | ---------------------- |
| Ubuntu 22.04     | x86_64                  | Python                 |

## Why NVIDIA Isaac Sim + Flexiv Elements Studio?

The built-in simulator of Flexiv Elements Studio uses the same control software as the real Flexiv robots, providing high-fidelity simulation of robot kinematics, dynamics, and most importantly, force-control behaviors, which is not possible with thirdparty controllers like those shipped with Isaac Sim and ROS/ROS2.

However, the built-in simulator of Flexiv Elements Studio only supports simulating one robot and a simple scene. To solve this issue, the Flexiv-Isaac Bridge App is developed to utilize Isaac Sim as the vendor for world representation and physics engine, while keeping the control software and user interface provided by Flexiv Elements Studio. The roles can be briefly described as:

NVIDIA Isaac Sim:

- World representation.
- Scene creation.
- Physics engine.

Flexiv Elements Studio:

- High-performance controller used by real Flexiv robots.
- User interface

## Demos

### Peg-in-hole

https://github.com/user-attachments/assets/e575bf70-9ffb-47a5-8aec-4cda8d25c08e

### Single robot polish

https://github.com/user-attachments/assets/a0c39e70-4469-4405-a07d-e0d8a0ad589b

### Dual robot polish

https://github.com/user-attachments/assets/7462a9bd-3cfd-40cc-95f7-b4fda0a74f30

## Workspace setup

1. Install NVIDIA [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html).
2. Note down Isaac Sim's installation directory, e.g. `~/isaacsim`.
3. Run the following script in this repo to install Flexiv's Isaac Sim extensions and apps to the Isaac Sim installation directory so that Isaac Sim can load them:

       bash install_ws.sh <isaac_sim_root_dir>

   For example:

       bash install_ws.sh ~/isaacsim

## Verify setup

To verify that the workspace setup is successful, run the example Python application:

    cd <isaac_sim_root_dir>
    ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/follow_target_with_rmpflow.py exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd

WARNING: When running Isaac Sim for the first time, it takes a couple of minutes to warm up the shader cache. You will notice that the CPU is fully loaded and the Isaac Sim window seems frozen. Please wait patiently and do not force quit the program.

After the example program is up and running, select the `TargetCube` prim under `World` from the Stage view, then drag it around, the robot TCP should follow the cube.

## Use Isaac Sim with Flexiv Elements Studio for the first time

### Install Flexiv Elements Studio

1. Prepare a Ubuntu 22.04 computer, all operations below are done on this computer.
2. [Contact Flexiv](https://www.flexiv.com/contact) to obtain the installation package of Elements Studio.
3. Extract the package to a non-root directory.
4. Install Elements Studio:

       bash setup_FlexivElements.sh

5. Switch physics engine from built-in to Isaac Sim:

       bash switch_physics_engine.sh

   Select [Isaac Sim] when prompted.

### Create a simulated robot in Elements Studio

1. Start Flexiv Elements Studio from the application menu.
2. In the Robot Connection window, select *Simulator*, and click *CREATE*.
3. Choose "Create according to the selected robot type" and select one from the list, then click *CONFIRM*. A new simulated robot will be added to the simulator list.
4. Toggle on the *Connect* button for the newly added one, then wait for loading.
5. When loading is finished, you'll see a robot at its upright pose, with an "Exception" error at the bottom right corner. This is expected because we haven't started Isaac Sim yet. But if you see a normally operating robot, that again means you are running the wrong version of Elements Studio that only supports the built-in physics engine.
6. At the bottom of the window, click on the small robot icon with a "SIM" tag on it, then a small window will pop up, note down the displayed robot serial number.
7. In the same small pop-up window, click *CHANGE CONNECTION*, then toggle off the *Connect* button to close the simulated robot. We will restart it later. Note that you do NOT need to close the whole Elements Studio program.

### Install Python dependencies

Install the following dependencies using `pip`:

- spdlog

### Run Flexiv-Isaac Bridge App

1. Run Flexiv Isaac Bridge app with one robot added to the scene:

       cd <isaac_sim_root_dir>
       ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py --env exts/isaacsim.robot.manipulators.examples/data/flexiv/example_env.usd --robot Rizon4-GYdBow exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd -0.690 0.315 0.709

   To see details about the program arguments:

       ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py --help

   The robot serial number is the one noted down previously with any space removed.

2. The app will launch an Isaac Sim window and start the physics loop (i.e. *Play*) automatically.
3. Go back to Elements Studio, then restart the exited simulator by toggle on the *Connect* button.
4. Wait for the connection to establish. If the connection is successful, you should see in Elements Studio a robot at home pose with no error.

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

1. Start the Flexiv-Isaac Bridge app with multiple robots added, using two robots for example:

       cd <isaac_sim_root_dir>
       ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py --env exts/isaacsim.robot.manipulators.examples/data/flexiv/example_env.usd --robot Rizon4-GYdBow exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd -0.690 0.315 0.709 --robot Rizon4s-TPqXaI exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4s_with_Grav.usd -0.667 -0.464 0.706

   Note that additional robots are added by appending `--robot <args ...>` to the program arguments.

2. Find a second Ubuntu 22.04 computer, connect it to the first computer via Ethernet cable. Then on the first computer, check that this wired Ethernet connection is visible in the network settings, then change the IPv4 setting of this wired connection to "Shared to other computers". Alternatively, connect both computers to the same network router via **wired** connection.
3. Make sure both computers are able to ping each other.
4. Install Flexiv Elements Studio on the **second** computer, then create a new simulated robot. Now each computer has a robot controller with Elements Studio.
5. Start the first simulated robot on the first computer, then wait for connection with Isaac Sim. You should see one of the robots in Isaac Sim moves a little bit when the connection is established.
6. Start the second simulated robot on the second computer, then wait for connection with Isaac Sim. You should see the other robot in Isaac Sim moves a little bit when the connection is established.
7. Execute test projects from both Elements Studios and check that both robots are working in Isaac Sim.

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
