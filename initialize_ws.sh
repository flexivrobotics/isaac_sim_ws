#!/bin/bash
# This script does the following things:
# 1. Create symbolic links that maps contents in this repo to Isaac Sim directory.
# 2. Add "omni.isaac.flexiv" to Isaac Sim apps' dependency list so this extension module is enabled.

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

# Create symbolic links mapping to Isaac Sim root
ISAAC_ROOT=$1
ln -s $SCRIPT_PATH/exts/omni.isaac.flexiv/ $ISAAC_ROOT/exts/
ln -s $SCRIPT_PATH/standalone_examples/api/omni.isaac.flexiv/ $ISAAC_ROOT/standalone_examples/api/

# Add "omni.isaac.flexiv" to dependency list right after "omni.isaac.franka"
KEYWORD='"omni.isaac.franka" = {}'
NEW_LINE='"omni.isaac.flexiv" = {}'
sed -i "/$KEYWORD/a $NEW_LINE" "$ISAAC_ROOT/apps/omni.isaac.sim.python.kit"
sed -i "/$KEYWORD/a $NEW_LINE" "$ISAAC_ROOT/apps/omni.isaac.sim.base.kit"
