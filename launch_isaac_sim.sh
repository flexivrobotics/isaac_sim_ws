#!/usr/bin/env bash
#
# Launches NVIDIA Isaac Sim in a docker container, either in a native X11
# window on this machine or streamed over WebRTC to the NVIDIA streaming client.
# Self-contained: safe to run fresh after a reboot, sets up everything it needs.
#
# Run with -h for usage.

set -euo pipefail

IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"
CONTAINER_NAME="${ISAACSIM_CONTAINER:-isaac-sim}"
CACHE_ROOT="$HOME/docker/isaac-sim"
# This workspace on the host, mounted at /workspace in the container. Isaac Sim
# itself only exists inside the image, so the workspace has to be visible in
# there for the examples to run from it.
WS_DIR="${ISAACSIM_WS_DIR:-$(dirname "$(readlink -f "$0")")}"

# Defaults come from the environment when set, and flags override them.
# The endpoint values map onto omni.kit.livestream.app primaryStream settings
# via runheadless.sh inside the container.
# Only forwarded into the container when explicitly set. The streaming app has
# its own defaults, and forcing publicIp/ports when the user did not ask breaks
# ICE candidate gathering -- signaling connects but no media ever flows.
HOST_SET="${ISAACSIM_HOST+1}"
SIGNAL_SET="${ISAACSIM_SIGNAL_PORT+1}"
STREAM_SET="${ISAACSIM_STREAM_PORT+1}"
ISAACSIM_HOST="${ISAACSIM_HOST:-127.0.0.1}"
ISAACSIM_SIGNAL_PORT="${ISAACSIM_SIGNAL_PORT:-49100}"   # TCP, signaling
ISAACSIM_STREAM_PORT="${ISAACSIM_STREAM_PORT:-47998}"   # UDP, media
LAUNCH_CLIENT="${LAUNCH_CLIENT:-1}"
CLIENT_TIMEOUT="${CLIENT_TIMEOUT:-600}"

usage() {
    cat <<USAGE
Usage: $(basename "$0") [OPTIONS] [MODE]

Launch NVIDIA Isaac Sim ($IMAGE) in a docker container.

Modes:
  native        GUI in a native X11 window on this machine (default).
                Renders straight to the host X server, so it needs a real
                local display -- not VNC/NoMachine/DCV. Alias: gui
  webrtc        Run the app windowless and stream it over WebRTC, so it works
                over a network or on a headless box. Opens the Isaac Sim
                WebRTC Streaming Client once the server is ready.
                Alias: headless
  shell         Drop into a bash shell in the container instead. Use this to
                run setup_ws.sh and the standalone examples by hand. X11 is
                forwarded when a local display is available, so examples that
                open an Isaac Sim window work from here too.

Options:
  -H, --host HOST   webrtc mode: host the client connects to, for streaming to
                    another machine (default: $ISAACSIM_HOST).
  -n, --no-client   webrtc mode: run the server only, do not open the
                    streaming client. Use with -H when the client is elsewhere.
  -h, --help        Show this help and exit.

Environment:
  Rarely needed overrides, all optional: ISAACSIM_SIGNAL_PORT (TCP, default
  $ISAACSIM_SIGNAL_PORT) and ISAACSIM_STREAM_PORT (UDP, default $ISAACSIM_STREAM_PORT) for port
  clashes, CLIENT_TIMEOUT (default $CLIENT_TIMEOUT) for how long to wait on the
  server before giving up on the client, plus ISAACSIM_IMAGE and
  ISAACSIM_CONTAINER. Host and ports are only passed to the app when set,
  so the app's own defaults apply otherwise.

Examples:
  $(basename "$0")                          # native window
  $(basename "$0") webrtc                   # stream, auto-open the client
  $(basename "$0") webrtc -H 192.168.1.5 -n # stream to another machine
  $(basename "$0") shell                    # poke around inside the container
USAGE
}

# Enhanced getopt signals its presence by exiting 4, so capture rather than test.
getopt_test=0
getopt --test >/dev/null 2>&1 || getopt_test=$?
if [ "$getopt_test" -ne 4 ]; then
    echo "This script needs enhanced getopt (util-linux)." >&2
    exit 1
fi

if ! parsed="$(getopt -o H:nh --long host:,no-client,help \
    -n "$(basename "$0")" -- "$@")"; then
    echo "Try '$(basename "$0") --help' for more information." >&2
    exit 1
fi
eval set -- "$parsed"

while true; do
    case "$1" in
        -H|--host)      ISAACSIM_HOST="$2"; HOST_SET=1; shift 2 ;;
        -n|--no-client) LAUNCH_CLIENT=0; shift ;;
        -h|--help)      usage; exit 0 ;;
        --)             shift; break ;;
        *)              echo "Unexpected option: $1" >&2; exit 1 ;;
    esac
done

if [ "$#" -gt 1 ]; then
    echo "Too many arguments: $*" >&2
    echo "Try '$(basename "$0") --help' for more information." >&2
    exit 1
fi
MODE="${1:-native}"

for v in ISAACSIM_SIGNAL_PORT ISAACSIM_STREAM_PORT CLIENT_TIMEOUT; do
    if ! [[ "${!v}" =~ ^[0-9]+$ ]]; then
        echo "$v must be a number, got: ${!v}" >&2
        exit 1
    fi
done

# --- sanity checks -----------------------------------------------------

if ! command -v docker &>/dev/null; then
    echo "docker not found. Install Docker Engine first." >&2
    exit 1
fi

if ! docker info &>/dev/null; then
    echo "Cannot reach the docker daemon (is it running? are you in the 'docker' group?)." >&2
    exit 1
fi

if ! nvidia-smi &>/dev/null; then
    echo "nvidia-smi failed. Check the NVIDIA driver is installed and loaded." >&2
    exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "A container named '$CONTAINER_NAME' already exists. Remove it first:" >&2
    echo "  docker rm -f $CONTAINER_NAME" >&2
    exit 1
fi

# --- pull image if missing ----------------------------------------------

if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "Pulling $IMAGE (this is a large image, first pull can take a while)..."
    docker pull "$IMAGE"
fi

# --- persistent cache/config dirs on host -------------------------------

# The container runs as uid 1234, so every bind-mounted dir has to exist and be
# writable by that uid. A dir this user creates is owned by this user, and one
# Docker autocreates lands as root:root, and uid 1234 can write neither. The tree
# under $CACHE_ROOT may also already be owned by 1234 (e.g. from NVIDIA's setup
# steps or an earlier run), in which case this user cannot mkdir inside it. So:
# create the dir as this user and hand it to 1234 from a throwaway root
# container, or, when this user cannot create it, create it there too. Dirs that
# already exist are left as they are. Needs the image, hence running after the
# pull.
ensure_host_dir() {
    local dir=$1
    [ -d "$dir" ] && return 0
    if mkdir -p "$dir" 2>/dev/null; then
        echo "Handing $dir to uid 1234..."
        docker run --rm -u 0:0 --entrypoint bash -v "$dir:/target" "$IMAGE" \
            -c "chown 1234:1234 /target"
        return 0
    fi
    echo "Creating $dir for uid 1234 (parent is not writable by $(id -un))..."
    docker run --rm -u 0:0 --entrypoint bash \
        -v "$(dirname "$dir"):/target" "$IMAGE" \
        -c "mkdir -p '/target/$(basename "$dir")' && chown 1234:1234 '/target/$(basename "$dir")'"
}

for dir in \
    "$CACHE_ROOT/cache/main" \
    "$CACHE_ROOT/cache/computecache" \
    "$CACHE_ROOT/cache/kit" \
    "$CACHE_ROOT/logs" \
    "$CACHE_ROOT/config" \
    "$CACHE_ROOT/data" \
    "$CACHE_ROOT/pkg" \
    "$HOME/.cache/ov/hub"
do
    ensure_host_dir "$dir"
done

# --- common docker run args ---------------------------------------------

COMMON_ARGS=(
    --name "$CONTAINER_NAME"
    --entrypoint bash
    -it
    --rm
    --gpus all
    --network=host
    -e "ACCEPT_EULA=Y"
    -e "PRIVACY_CONSENT=Y"
    -v "$CACHE_ROOT/cache/main:/isaac-sim/.cache:rw"
    -v "$CACHE_ROOT/cache/computecache:/isaac-sim/.nv/ComputeCache:rw"
    # Kit's shader/extension cache. Without it every launch pays a cold-start
    # cost that should only be paid once.
    -v "$CACHE_ROOT/cache/kit:/isaac-sim/kit/cache:rw"
    -v "$CACHE_ROOT/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
    -v "$CACHE_ROOT/config:/isaac-sim/.nvidia-omniverse/config:rw"
    -v "$CACHE_ROOT/data:/isaac-sim/.local/share/ov/data:rw"
    -v "$CACHE_ROOT/pkg:/isaac-sim/.local/share/ov/pkg:rw"
    -v "$HOME/.cache/ov/hub:/var/cache/hub:rw"
    # Flexiv's Isaac workspace, so setup_ws.sh and the examples can be run from
    # inside the container. Read-only: they only read from it, and this keeps a
    # stray write in the container from touching the git checkout.
    -v "$WS_DIR:/workspace:ro"
    -u 1234:1234
)

# --- mode-specific setup --------------------------------------------------

# Give the container access to the host's X server, for any mode that opens a
# window. Three things are needed and all three are easy to get wrong:
#   * the socket directory, since DISPLAY=:N means a unix socket under
#     /tmp/.X11-unix and the X server usually has no TCP listener at all, so
#     --network=host on its own gets you nothing;
#   * DISPLAY itself; and
#   * authorization, via "xhost +local:", which grants access for local
#     connections -- a bind-mounted socket counts as one.
#
# Deliberately no cookie file: under Wayland/Xwayland $HOME/.Xauthority is often
# missing or a root-owned empty directory, and the live cookie in $XAUTHORITY is
# mode 600 for the host user, so the container's uid 1234 cannot read it. Passing
# XAUTHORITY to a file the client cannot read only creates a confusing failure.
setup_x11_forwarding() {
    if [ ! -d /tmp/.X11-unix ]; then
        echo "No X11 socket directory at /tmp/.X11-unix; cannot forward a display." >&2
        echo "Use 'webrtc' mode to stream the GUI instead." >&2
        exit 1
    fi
    xhost +local: >/dev/null 2>&1 || true
    # --group-add is not optional: the socket is mode srwxrwxr-x owned by the
    # host user, connecting to a unix socket needs write permission, and "other"
    # does not have it. Putting the container's uid 1234 in the host user's
    # group is what makes the connection possible at all.
    COMMON_ARGS+=(
        -e DISPLAY
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        --group-add "$(id -g)"
    )
}

case "$MODE" in
    native|gui)
        # GUI mode requires a real local display (not VNC/NoMachine/DCV/headless).
        # See: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
        if [ -z "${DISPLAY:-}" ]; then
            echo "DISPLAY is not set. Native mode needs a local X display; use 'webrtc' mode instead." >&2
            exit 1
        fi
        setup_x11_forwarding
        LAUNCH_CMD="cd /isaac-sim && ./runapp.sh"
        ;;
    webrtc|headless)
        # No window is created, so no X display is needed. --network=host above
        # already makes the signal/stream ports reachable from the host.
        # runheadless.sh only adds each flag when its variable is non-empty, so
        # leaving these out keeps the app's own defaults.
        [ -n "$HOST_SET" ] && COMMON_ARGS+=(-e "ISAACSIM_HOST=$ISAACSIM_HOST")
        [ -n "$SIGNAL_SET" ] && COMMON_ARGS+=(-e "ISAACSIM_SIGNAL_PORT=$ISAACSIM_SIGNAL_PORT")
        [ -n "$STREAM_SET" ] && COMMON_ARGS+=(-e "ISAACSIM_STREAM_PORT=$ISAACSIM_STREAM_PORT")
        # The client is a separate NVIDIA app, not part of this image.
        client="$(command -v isaacsim-webrtc-streaming-client || true)"
        if [ -z "$client" ]; then
            echo "Note: the Isaac Sim WebRTC Streaming Client was not found on this host."
            echo "      Install isaacsim-webrtc-streaming-client, then enter host $ISAACSIM_HOST in it and click Connect."
        elif [ "$LAUNCH_CLIENT" != "1" ]; then
            echo "Streaming client found but auto-launch is off (--no-client)."
            echo "Start it yourself with: $client   (then enter host $ISAACSIM_HOST and click Connect)"
        elif [ -z "${DISPLAY:-}" ]; then
            # The client itself is a desktop app, so it needs a display even
            # though the streaming server does not.
            echo "Streaming client found but DISPLAY is unset, so it cannot be shown here."
            echo "Run '$client' on a machine with a display and connect it to $ISAACSIM_HOST."
        else
            # The app must be fully loaded before the client can connect, so poll
            # the signal port and launch only once it accepts connections. This
            # has to be backgrounded now because the exec below replaces us.
            (
                waited=0
                while [ "$waited" -lt "$CLIENT_TIMEOUT" ]; do
                    if ss -tln 2>/dev/null | grep -q ":$ISAACSIM_SIGNAL_PORT[[:space:]]"; then
                        echo "Signal port $ISAACSIM_SIGNAL_PORT is up; launching streaming client."
                        # Client takes the host in its connect field, not a port.
                        exec "$client" >/dev/null 2>&1
                    fi
                    sleep 2
                    waited=$((waited + 2))
                done
                echo "Streaming client not launched: port $ISAACSIM_SIGNAL_PORT never opened within ${CLIENT_TIMEOUT}s." >&2
            ) &
            echo "Streaming client will open automatically once the server is ready (up to ${CLIENT_TIMEOUT}s)."
        fi
        echo "Streaming server starting; it is ready once the log prints 'Isaac Sim Full Streaming App is loaded'."
        LAUNCH_CMD="cd /isaac-sim && ./runheadless.sh -v"
        ;;
    shell)
        # Forward X11 when a display is available, so GUI apps started from the
        # shell (e.g. the standalone examples, which open an Isaac Sim window)
        # can render. Optional rather than required: a shell is still useful for
        # setup and headless runs on a machine with no display.
        if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
            setup_x11_forwarding
        fi
        # Start in this workspace, where the examples are run from.
        LAUNCH_CMD="cd /workspace && bash"
        ;;
    *)
        echo "Unknown mode: $MODE (expected: native | webrtc | shell)" >&2
        echo "Try '$(basename "$0") --help' for more information." >&2
        exit 1
        ;;
esac

echo "Launching Isaac Sim ($MODE mode)..."
exec docker run "${COMMON_ARGS[@]}" "$IMAGE" -c "$LAUNCH_CMD"
