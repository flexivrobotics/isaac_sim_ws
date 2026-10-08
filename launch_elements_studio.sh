#!/usr/bin/env bash
#
# Runs Flexiv Elements Studio, from its release package, in an Ubuntu 22.04
# docker container. Elements Studio supports Ubuntu 22.04 only, so this is how to
# run it on a newer Ubuntu, and how to run more than one Elements Studio, one
# per simulated robot, on one computer.
# Self-contained: builds the image it needs on first use.
#
# Run with -h for usage.

set -euo pipefail

IMAGE="${STUDIO_IMAGE:-flexiv-elements-studio:ubuntu22.04}"
CONTAINER_NAME="${STUDIO_CONTAINER:-flexiv-elements-studio}"
# Persistent home directory of the container user, one per container name, so
# Elements Studio's settings survive the container's --rm.
HOME_ROOT="$HOME/docker/flexiv-elements-studio"

usage() {
    cat <<USAGE
Usage: $(basename "$0") [OPTIONS] STUDIO_DIR [MODE]

Run Flexiv Elements Studio in an Ubuntu 22.04 docker container ($IMAGE).

STUDIO_DIR is the FlexivElementsStudio folder of the extracted release package,
the one that contains run_FlexivElements.sh. It is mounted read-write at the
same path in the container, because Elements Studio keeps its robots and
settings in it.

Modes:
  studio        Start Elements Studio (default). A window opens on this
                machine's display.
  shell         Open a bash shell in the container instead, in STUDIO_DIR. Use
                it to run switch_physics_engine.sh, or to start Elements Studio
                by hand with: bash run_FlexivElements.sh

Options:
  -n, --name NAME       Container name (default: $CONTAINER_NAME). Each
                        container keeps its own home directory under
                        $HOME_ROOT/NAME.
  -r, --rebuild-image   Rebuild the image, e.g. after updating this script.
  -h, --help            Show this help and exit.

Examples:
  $(basename "$0") ~/FlexivElementsStudio
  $(basename "$0") ~/FlexivElementsStudio shell
USAGE
}

# Enhanced getopt signals its presence by exiting 4, so capture rather than test.
getopt_test=0
getopt --test >/dev/null 2>&1 || getopt_test=$?
if [ "$getopt_test" -ne 4 ]; then
    echo "This script needs enhanced getopt (util-linux)." >&2
    exit 1
fi

if ! parsed="$(getopt -o n:rh --long name:,rebuild-image,help \
    -n "$(basename "$0")" -- "$@")"; then
    echo "Try '$(basename "$0") --help' for more information." >&2
    exit 1
fi
eval set -- "$parsed"

REBUILD_IMAGE=0
while true; do
    case "$1" in
        -n|--name)          CONTAINER_NAME="$2"; shift 2 ;;
        -r|--rebuild-image) REBUILD_IMAGE=1; shift ;;
        -h|--help)          usage; exit 0 ;;
        --)                 shift; break ;;
        *)                  echo "Unexpected option: $1" >&2; exit 1 ;;
    esac
done

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Expected STUDIO_DIR and an optional MODE, got: $*" >&2
    echo "Try '$(basename "$0") --help' for more information." >&2
    exit 1
fi
STUDIO_DIR="$(readlink -f "$1")"
MODE="${2:-studio}"

# --- sanity checks -----------------------------------------------------

if [ ! -f "$STUDIO_DIR/run_FlexivElements.sh" ]; then
    echo "No run_FlexivElements.sh in $STUDIO_DIR." >&2
    echo "Pass the FlexivElementsStudio folder of the extracted release package." >&2
    exit 1
fi

if ! command -v docker &>/dev/null; then
    echo "docker not found. Install Docker Engine first." >&2
    exit 1
fi

if ! docker info &>/dev/null; then
    echo "Cannot reach the docker daemon (is it running? are you in the 'docker' group?)." >&2
    exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "A container named '$CONTAINER_NAME' already exists. Remove it first:" >&2
    echo "  docker rm -f $CONTAINER_NAME" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
    echo "Elements Studio needs a local X display, and DISPLAY is not set or" >&2
    echo "/tmp/.X11-unix is missing." >&2
    exit 1
fi

# Every Elements Studio here shares the host's network, so a second one would
# take the same ports as the first.
for name in $(docker ps --filter "ancestor=$IMAGE" --format '{{.Names}}'); do
    echo "Elements Studio is already running in container '$name'. Only one can run at a" >&2
    echo "time on this computer. Stop it first: docker rm -f $name" >&2
    exit 1
done

# Elements Studio drives the simulated robot with its own physics engine unless
# the external one is installed, and then no external simulator ever connects.
# switch_physics_engine.sh installs one by copying it over FlexivSimulation.
if [ -f "$STUDIO_DIR/bin/FlexivSimulation_external" ] \
    && ! cmp -s "$STUDIO_DIR/FlexivSimulation" "$STUDIO_DIR/bin/FlexivSimulation_external"; then
    echo "WARNING: this Elements Studio uses its built-in physics engine, so Isaac Sim can't"
    echo "         connect to it. To use Isaac Sim, select the external one, then restart"
    echo "         Elements Studio:"
    echo "           cd $STUDIO_DIR && bash switch_physics_engine.sh   # choose [2] External"
fi

# --- build image if missing ----------------------------------------------

# The system libraries Elements Studio needs on top of what its package bundles
# (Qt), as its setup_FlexivElements.sh installs them on a native Ubuntu 22.04,
# plus the X11/OpenGL runtime of a desktop install:
#   * gocryptfs and fuse3: RobotControlApp mounts the encrypted specs_enc with
#     gocryptfs;
#   * net-tools and wireless-tools: the hardware ID is read through ifconfig and
#     iwconfig;
#   * sudo: setup and maintenance scripts call it.
# The container user has the host user's uid and gid, so what Elements Studio
# writes into STUDIO_DIR stays owned by the host user.
build_image() {
    echo "Building image $IMAGE..."
    docker build -t "$IMAGE" \
        --build-arg "LOCAL_UID=$(id -u)" --build-arg "LOCAL_GID=$(id -g)" - <<'DOCKERFILE'
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        gocryptfs fuse3 \
        net-tools wireless-tools iproute2 iputils-ping \
        sudo procps ca-certificates \
        libssl3 zlib1g libgomp1 libatomic1 freeglut3 \
        libopengl0 libglx0 libgl1 libegl1 libglu1-mesa libgl1-mesa-dri \
        libdrm2 libgbm1 libxshmfence1 \
        libx11-6 libx11-xcb1 libxext6 libxrender1 libxi6 libxtst6 libxss1 \
        libxcb1 libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
        libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 \
        libxcb-shm0 libxcb-sync1 libxcb-util1 libxcb-xfixes0 libxcb-xinerama0 \
        libxcb-xkb1 libxkbcommon0 libxkbcommon-x11-0 \
        libxcomposite1 libxcursor1 libxdamage1 libxfixes3 libxrandr2 \
        libfontconfig1 libfreetype6 fonts-dejavu-core \
        libdbus-1-3 libglib2.0-0 libsm6 libice6 libcups2 libpci3 \
        libnss3 libnspr4 libasound2 libpulse0 \
        libatk1.0-0 libatk-bridge2.0-0 libatspi2.0-0 \
        libxml2 libxslt1.1 \
    && rm -rf /var/lib/apt/lists/*

ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
RUN if ! getent group "${LOCAL_GID}" >/dev/null; then \
        groupadd --gid "${LOCAL_GID}" flexiv; \
    fi && \
    if ! getent passwd "${LOCAL_UID}" >/dev/null; then \
        useradd --uid "${LOCAL_UID}" --gid "${LOCAL_GID}" --no-create-home --shell /bin/bash flexiv; \
    fi && \
    echo "$(getent passwd "${LOCAL_UID}" | cut -d: -f1) ALL=(ALL) NOPASSWD:ALL" \
        > /etc/sudoers.d/flexiv && chmod 0440 /etc/sudoers.d/flexiv
DOCKERFILE
}

if [ "$REBUILD_IMAGE" = "1" ] || ! docker image inspect "$IMAGE" &>/dev/null; then
    build_image
fi

# --- docker run args -------------------------------------------------------

CONTAINER_HOME="$HOME_ROOT/$CONTAINER_NAME"
mkdir -p "$CONTAINER_HOME"

RUN_ARGS=(
    --name "$CONTAINER_NAME"
    -it
    --rm
    -u "$(id -u):$(id -g)"
    -e "HOME=$CONTAINER_HOME"
    -v "$CONTAINER_HOME:$CONTAINER_HOME:rw"
    # Same path as on the host, so paths Elements Studio records in its own
    # files stay valid outside the container.
    -v "$STUDIO_DIR:$STUDIO_DIR:rw"
    -w "$STUDIO_DIR"
    # The host network and IPC namespace, so the external simulator, RDK and DDK
    # reach the simulated robot as they reach a natively installed Elements Studio.
    --network=host
    --ipc=host
    # The controller runs with real-time priority, and locks its memory.
    --cap-add SYS_NICE
    --ulimit rtprio=99
    --ulimit memlock=-1
    # gocryptfs mounts the encrypted specs with FUSE.
    --device /dev/fuse
    --cap-add SYS_ADMIN
    --security-opt apparmor=unconfined
    # The host's X server: its socket, DISPLAY, and access for local clients.
    # The container user is the host user, so it can also read the host's cookie.
    -e DISPLAY
    -e QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)
xhost +local: >/dev/null 2>&1 || true
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    RUN_ARGS+=(-e XAUTHORITY -v "$XAUTHORITY:$XAUTHORITY:ro")
fi
# Hardware OpenGL, when the host has a DRI device. Without it Elements Studio
# renders in software, which works but is slower.
if [ -d /dev/dri ]; then
    RUN_ARGS+=(--device /dev/dri)
fi

case "$MODE" in
    studio)
        LAUNCH_CMD="bash run_FlexivElements.sh"
        ;;
    shell)
        LAUNCH_CMD="bash"
        ;;
    *)
        echo "Unknown mode: $MODE (expected: studio | shell)" >&2
        echo "Try '$(basename "$0") --help' for more information." >&2
        exit 1
        ;;
esac

echo "Launching Elements Studio from $STUDIO_DIR ($MODE mode)..."
exec docker run "${RUN_ARGS[@]}" "$IMAGE" bash -c "$LAUNCH_CMD"
