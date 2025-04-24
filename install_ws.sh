#!/bin/bash
# Install Flexiv's Isaac Sim workspace to the NVIDIA Isaac Sim installation directory.

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

# Install files
cp -r $SCRIPT_PATH/exts/ $ISAAC_ROOT
cp -r $SCRIPT_PATH/standalone_examples/ $ISAAC_ROOT
