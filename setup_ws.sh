#!/bin/bash
# Set up Flexiv's Isaac Sim workspace: install flexivsimplugin and flexivrdk into
# Isaac Sim's bundled Python. The bridge app uses RDK to pull each robot's
# calibration. The examples run from this repo, so nothing is copied into
# Isaac Sim.
#
# Works against both a natively installed Isaac Sim and the Isaac Sim container.
# The only thing that differs between them is where Isaac Sim lives, so this
# script resolves that one value and does the rest identically.
#
# The container is disposable (docker run --rm), so re-run this after each
# launch; it takes seconds once the wheel is in pip's cache.

# Absolute path of this script
SCRIPT_PATH="$(dirname "$(readlink -f "$0")")"
set -e

usage() {
    cat <<USAGE
Usage: $(basename "$0") [isaac_sim_root] [OPTIONS]

Arguments:
  isaac_sim_root   Absolute path to the Isaac Sim installation root. Optional;
                   when omitted it is auto-detected (see below).

Options:
  -v, --plugin-version VER  Install exactly this flexivsimplugin version. By
                            default, the newest patch release in the
                            COMPATIBLE_SIM_PLUGIN_VER line the bridge app pins,
                            e.g. 2.2.x, is installed.
  --rdk-test-pypi           Install flexivrdk from the PyPI test server instead
                            of the real one. Use for a release candidate that
                            has not been published yet.
  --plugin-test-pypi        Same for flexivsimplugin. The test server also holds
                            its dev builds, so the newest one, not the release,
                            is installed unless --plugin-version pins it.
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
RDK_TEST_PYPI=false
PLUGIN_TEST_PYPI=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        -v|--plugin-version)
            if [ "$#" -lt 2 ]; then
                echo "Error: $1 needs a version, e.g. $1 2.2.0" >&2
                exit 1
            fi
            PLUGIN_VER="$2"; shift 2 ;;
        --rdk-test-pypi)     RDK_TEST_PYPI=true; shift ;;
        --plugin-test-pypi)  PLUGIN_TEST_PYPI=true; shift ;;
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

# Take the plugin and RDK release lines from the bridge app unless a plugin
# version was given, so there is a single source of truth for which versions this
# workspace expects. Installing the newest patch release in a line picks up fixes.
bridge_app=$SCRIPT_PATH/examples/flexiv_isaac_bridge_app.py
release_line() {
    local line
    line=$(sed -n "s/^$1 *= *\"\(.*\)\"/\1/p" "$bridge_app")
    if [ -z "$line" ]; then
        echo "Error: could not read $1 from $bridge_app." >&2
        exit 1
    fi
    echo "$line"
}
if [ -n "$PLUGIN_VER" ]; then
    PLUGIN_SPEC="flexivsimplugin==$PLUGIN_VER"
else
    PLUGIN_SPEC="flexivsimplugin==$(release_line COMPATIBLE_SIM_PLUGIN_VER).*"
fi
RDK_SPEC="flexivrdk==$(release_line COMPATIBLE_RDK_VER).*"

# Install into Isaac Sim's bundled interpreter, not a separate venv -- python.sh
# is what actually runs the examples. Each package is installed on its own, as each
# may come from a different server.
pip_install() {
    local from_test_pypi=$1 spec=$2
    if $from_test_pypi; then
        echo "Installing $spec from the PyPI test server ..."
        # Only the package itself is a test build, so real PyPI stays available
        # for its deps.
        "$ISAAC_ROOT/python.sh" -m pip install --upgrade \
            --index-url https://test.pypi.org/simple/ \
            --extra-index-url https://pypi.org/simple/ \
            "$spec"
    else
        echo "Installing $spec ..."
        "$ISAAC_ROOT/python.sh" -m pip install --upgrade "$spec"
    fi
}
pip_install "$PLUGIN_TEST_PYPI" "$PLUGIN_SPEC"
pip_install "$RDK_TEST_PYPI" "$RDK_SPEC"

# Confirm the interpreter can actually import what was just installed, so a
# broken wheel is caught here rather than part-way into an Isaac Sim launch.
echo "Verifying ..."
"$ISAAC_ROOT/python.sh" -c "import flexivsimplugin as f; print('flexivsimplugin', f.__version__)"
"$ISAAC_ROOT/python.sh" -c "import flexivrdk as r; print('flexivrdk', r.__version__)"

echo ">>>>>>>>>> Setup finished <<<<<<<<<<"
echo "Run the bridge app with:"
echo "  $ISAAC_ROOT/python.sh $SCRIPT_PATH/examples/flexiv_isaac_bridge_app.py \\"
echo "    --config $SCRIPT_PATH/examples/single_arm_app_config.yaml"
