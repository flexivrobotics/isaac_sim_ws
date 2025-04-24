#!/bin/bash
# Uninstall Flexiv's Isaac Sim workspace from the NVIDIA Isaac Sim installation directory.

# Absolute path of this script
SCRIPT_PATH="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [isaac_sim_root]"
    echo "    isaac_sim_root: absolute path to Isaac Sim installation root directory"
    exit 1
fi
ISAAC_ROOT=$1

# Uninstalled files
rm -rf $ISAAC_ROOT/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/flexiv
rm -rf $ISAAC_ROOT/exts/isaacsim.robot.manipulators.examples/data/flexiv
rm -rf $ISAAC_ROOT/standalone_examples/api/isaacsim.robot.manipulators/flexiv
