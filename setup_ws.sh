#!/bin/bash
# Set up Flexiv's Isaac Sim workspace: install the workspace files, then install
# flexivsimplugin into Isaac Sim's bundled Python.
#
# Works against both a natively installed Isaac Sim and the Isaac Sim container.
# The only thing that differs between them is where Isaac Sim lives, so this
# script resolves that one value and does the rest identically. In the container
# the workspace is typically bind-mounted (e.g. at /workspace), so run this from
# there rather than copying files in by hand.
#
# The container is disposable (docker run --rm), so re-run this after each
# launch; it takes seconds once the wheel is in pip's cache.

# Absolute path of this script
SCRIPT_PATH="$(dirname $(readlink -f $0))"
set -e

usage() {
    cat <<USAGE
Usage: $(basename $0) [isaac_sim_root] [OPTIONS]

Arguments:
  isaac_sim_root   Absolute path to the Isaac Sim installation root. Optional;
                   when omitted it is auto-detected (see below).

Options:
  -v, --plugin-version VER  Install exactly this flexivsimplugin version. By
                            default, the newest patch release in the
                            COMPATIBLE_SIM_PLUGIN_VER line the bridge app pins,
                            e.g. 2.2.x, is installed.
  -t, --test-pypi           Install flexivsimplugin from the PyPI test server
                            instead of the real one. Use for a release candidate
                            that has not been published yet.
  -s, --skip-deps           Only install the workspace files, no pip install.
  -h, --help                Show this help and exit.

Isaac Sim root auto-detection, in order:
  1. \$ISAAC_SIM_ROOT, when set
  2. /isaac-sim, the install path inside the Isaac Sim container
  3. ~/isaacsim, the default path for a native install
USAGE
}

# Parse arguments
ISAAC_ROOT=""
PLUGIN_VER=""
USE_TEST_PYPI=false
SKIP_DEPS=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        -v|--plugin-version) PLUGIN_VER="$2"; shift 2 ;;
        -t|--test-pypi)      USE_TEST_PYPI=true; shift ;;
        -s|--skip-deps)      SKIP_DEPS=true; shift ;;
        -h|--help)           usage; exit 0 ;;
        -*)                  echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *)
            if [ -n "$ISAAC_ROOT" ]; then
                echo "Error: unexpected extra argument: $1" >&2
                exit 1
            fi
            ISAAC_ROOT="$1"; shift
            ;;
    esac
done

# Resolve the Isaac Sim root. A directory only counts when it holds python.sh,
# so a wrong guess fails here with a clear message instead of surfacing later as
# a confusing "no such file" from one of the install steps.
if [ -z "$ISAAC_ROOT" ]; then
    for candidate in "$ISAAC_SIM_ROOT" /isaac-sim "$HOME/isaacsim"; do
        if [ -n "$candidate" ] && [ -x "$candidate/python.sh" ]; then
            ISAAC_ROOT="$candidate"
            echo "Auto-detected Isaac Sim at $ISAAC_ROOT"
            break
        fi
    done
fi
if [ -z "$ISAAC_ROOT" ]; then
    echo "Error: could not find an Isaac Sim installation." >&2
    echo "Pass the root directory explicitly, or set \$ISAAC_SIM_ROOT." >&2
    echo "Looked for python.sh in: \$ISAAC_SIM_ROOT, /isaac-sim, $HOME/isaacsim" >&2
    exit 1
fi
if [ ! -x "$ISAAC_ROOT/python.sh" ]; then
    echo "Error: $ISAAC_ROOT does not look like an Isaac Sim root (no python.sh)." >&2
    exit 1
fi

# Install the workspace files
echo "Installing workspace files into $ISAAC_ROOT ..."
bash $SCRIPT_PATH/install_ws.sh $ISAAC_ROOT

if $SKIP_DEPS; then
    echo ">>>>>>>>>> Workspace installed, plugin install skipped <<<<<<<<<<"
    exit 0
fi

# Take the plugin release line from the bridge app unless a version was given, so
# there is a single source of truth for which plugin this workspace expects.
# Installing the newest patch release in that line picks up plugin fixes.
bridge_app=$SCRIPT_PATH/standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py
if [ -n "$PLUGIN_VER" ]; then
    PLUGIN_SPEC="flexivsimplugin==$PLUGIN_VER"
else
    PLUGIN_LINE=$(sed -n 's/^COMPATIBLE_SIM_PLUGIN_VER *= *"\(.*\)"/\1/p' $bridge_app)
    if [ -z "$PLUGIN_LINE" ]; then
        echo "Error: could not read COMPATIBLE_SIM_PLUGIN_VER from $bridge_app." >&2
        echo "Pass the version explicitly with --plugin-version." >&2
        exit 1
    fi
    PLUGIN_SPEC="flexivsimplugin==$PLUGIN_LINE.*"
    echo "Using the newest flexivsimplugin $PLUGIN_LINE.x (the line the bridge app supports)"
fi

# Install into Isaac Sim's bundled interpreter, not a separate venv -- python.sh
# is what actually runs the examples.
echo "Installing $PLUGIN_SPEC ..."
if $USE_TEST_PYPI; then
    # Only the plugin is a test build, so real PyPI stays available for its deps.
    $ISAAC_ROOT/python.sh -m pip install --upgrade \
        --index-url https://test.pypi.org/simple/ \
        --extra-index-url https://pypi.org/simple/ \
        "$PLUGIN_SPEC"
else
    $ISAAC_ROOT/python.sh -m pip install --upgrade "$PLUGIN_SPEC"
fi

# Confirm the interpreter can actually import what was just installed, so a
# broken wheel is caught here rather than part-way into an Isaac Sim launch.
echo "Verifying ..."
$ISAAC_ROOT/python.sh -c "import flexivsimplugin as f; print('flexivsimplugin', f.__version__)"

echo ">>>>>>>>>> Setup finished <<<<<<<<<<"
echo "Run the bridge app with:"
echo "  cd $ISAAC_ROOT && ./python.sh standalone_examples/api/isaacsim.robot.manipulators/flexiv/flexiv_isaac_bridge_app.py \\"
echo "    --config standalone_examples/api/isaacsim.robot.manipulators/flexiv/single_arm_app_config.yaml"
