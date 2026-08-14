#!/bin/bash
# Install Flexiv's Isaac Sim workspace to the NVIDIA Isaac Sim installation directory.
#
# Isaac Sim 6.x note: NVIDIA moved the (non-experimental)
# isaacsim.robot.manipulators.examples extension from exts/ to extsDeprecated/,
# which is still on the default Python path. The Flexiv files must therefore be
# merged into the extsDeprecated/ copy so Isaac Sim can load them. The repo's
# exts/ directory mirrors the extension sub-tree; we copy it into extsDeprecated/.

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
# The extension sub-tree (isaacsim.robot.manipulators.examples) lives in
# extsDeprecated/ in Isaac Sim 6.x; standalone_examples/ remains at the root.
cp -r $SCRIPT_PATH/exts/. $ISAAC_ROOT/extsDeprecated/
cp -r $SCRIPT_PATH/standalone_examples/ $ISAAC_ROOT
