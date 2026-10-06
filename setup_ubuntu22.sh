#!/usr/bin/env bash
# Native Ubuntu 22.04 dependency installation; the Dockerfile handles the container.
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /etc/os-release
if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
  echo "This hardware branch targets Ubuntu 22.04. Use ./scripts/build_image.sh on other hosts." >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html" >&2
  exit 1
fi
root_command=()
if [[ "$(id -u)" != "0" ]]; then root_command=(sudo); fi
"${root_command[@]}" apt-get update
"${root_command[@]}" apt-get install -y --no-install-recommends \
  build-essential cmake git curl ca-certificates pkg-config python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-pytest libboost-thread-dev ros-humble-asio-cmake-module
source /opt/ros/humble/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${root_command[@]}" rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "$hardware_root/src" --ignore-src --rosdistro humble -y
"${root_command[@]}" python3 "$hardware_root/scripts/fix_humble_cmake.py"
python3 -m pip install --user -r "$hardware_root/requirements-build.txt"
echo "Dependencies installed. Run ./scripts/build_hardware.sh and source ./source.sh."
