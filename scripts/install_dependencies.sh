#!/usr/bin/env bash
# Shared dependency installation for native Ubuntu and the local test image.
set -eo pipefail
dependency_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/os-release
if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
  echo "Dependencies require Ubuntu 22.04 and ROS 2 Humble." >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html" >&2
  exit 1
fi
if [[ ! -d "$dependency_root/src" || ! -r "$dependency_root/requirements-build.txt" ]]; then
  echo "Dependency root must contain src/ and requirements-build.txt: $dependency_root" >&2
  exit 1
fi

root_command=()
pip_options=(--no-cache-dir)
if [[ "$(id -u)" != "0" ]]; then
  root_command=(sudo)
  pip_options+=(--user)
fi
"${root_command[@]}" apt-get update -o APT::Update::Error-Mode=any
"${root_command[@]}" apt-get install -y --no-install-recommends \
  build-essential git curl ca-certificates pkg-config python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-pytest
source /opt/ros/humble/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${root_command[@]}" rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "$dependency_root/src" --ignore-src --rosdistro humble -y
"${root_command[@]}" /usr/bin/python3 "$dependency_root/scripts/fix_humble_cmake.py"
/usr/bin/python3 -m pip install "${pip_options[@]}" -r "$dependency_root/requirements-build.txt"
