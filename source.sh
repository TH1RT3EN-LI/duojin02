#!/usr/bin/env bash
_duojin02_hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required; use the hardware Docker image or install Humble on Ubuntu 22.04." >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
if [[ -r /opt/duojin02/slam_backend/setup.bash ]]; then
  source /opt/duojin02/slam_backend/setup.bash
fi
if [[ -r /opt/duojin02/install/setup.bash ]]; then
  source /opt/duojin02/install/setup.bash
fi
if [[ -r "$_duojin02_hardware_root/install/slam_backend/setup.bash" ]]; then
  if [[ "$(cat "$_duojin02_hardware_root/install/slam_backend/.target_architecture" 2>/dev/null)" != "$(uname -m)" ]]; then
    echo "SLAM overlay architecture is stale or incompatible; rebuild with ./scripts/build_hardware.sh." >&2
    return 1 2>/dev/null || exit 1
  fi
  source "$_duojin02_hardware_root/install/slam_backend/setup.bash"
fi
if [[ -r "$_duojin02_hardware_root/install/hardware/setup.bash" ]]; then
  if [[ "$(cat "$_duojin02_hardware_root/install/hardware/.target_architecture" 2>/dev/null)" != "$(uname -m)" ]]; then
    echo "Hardware overlay architecture is stale or incompatible; rebuild with ./scripts/build_hardware.sh." >&2
    return 1 2>/dev/null || exit 1
  fi
  source "$_duojin02_hardware_root/install/hardware/setup.bash"
elif [[ ! -r /opt/duojin02/install/setup.bash ]]; then
  echo "Hardware workspace has not been built. Run ./scripts/build_hardware.sh first." >&2
  unset _duojin02_hardware_root
  return 1 2>/dev/null || exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export DUOJIN02_WORKSPACE_ROOT="${DUOJIN02_WORKSPACE_ROOT:-$_duojin02_hardware_root}"
unset _duojin02_hardware_root
