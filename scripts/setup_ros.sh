#!/usr/bin/env bash
# Installs ROS 2 Jazzy and builds the robot's packages. Ubuntu 24.04 only.
# Run it as your normal user: it asks for sudo for the system packages.
#
#   bash scripts/setup_ros.sh           install ROS 2 if missing, then build
#   bash scripts/setup_ros.sh --build   only rebuild
#
# The workspace lives outside the repository, in ~/h3215_ws (or $H3215_WS), with the
# repository linked into its src/ folder: colcon finds the packages wherever they sit in it
# (in h3215/hardware and h3215/software), and its build
# output never lands in the repository.
#
# Afterwards, in every new shell (or add it to ~/.bashrc):
#   source ~/h3215_ws/install/setup.bash
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORKSPACE="${H3215_WS:-$HOME/h3215_ws}"
DISTRO=jazzy

if [ "$(id -u)" -eq 0 ]; then
    echo "Run it without sudo: the workspace is built as your user." >&2
    exit 1
fi

install_ros() {
    . /etc/os-release
    if [ "$VERSION_CODENAME" != noble ]; then
        echo "ROS 2 $DISTRO packages are for Ubuntu 24.04 (noble), this is $PRETTY_NAME." >&2
        echo "On another system, build in Docker instead: see h3215/software/control/Dockerfile." >&2
        exit 1
    fi
    # The ROS apt repository, set up by its own package as the ROS 2 documentation does it
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F tag_name | awk -F'"' '{print $4}')
    deb="$(mktemp --suffix=.deb)"
    curl -fsSL -o "$deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/ros2-apt-source_$version.${VERSION_CODENAME}_all.deb"
    sudo dpkg -i "$deb"
    rm "$deb"
    sudo apt-get update
    sudo apt-get install -y "ros-$DISTRO-ros-base" ros-dev-tools
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
}

if [ "${1:-}" != "--build" ]; then
    [ -f "/opt/ros/$DISTRO/setup.bash" ] || install_ros
fi

set +u  # the ROS setup scripts read unset variables
source "/opt/ros/$DISTRO/setup.bash"
set -u

# The repository, linked into the workspace
mkdir -p "$WORKSPACE/src"
link="$WORKSPACE/src/$(basename "$REPO")"
[ -e "$link" ] || ln -s "$REPO" "$link"

# Every package the package.xml files ask for: ros2_control, controllers, xacro...
rosdep update --rosdistro "$DISTRO"
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -y --rosdistro "$DISTRO"

cd "$WORKSPACE"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Done. In every new shell: source $WORKSPACE/install/setup.bash"
