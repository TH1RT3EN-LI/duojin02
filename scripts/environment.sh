#!/usr/bin/env bash

duojin02_setup_build_tools() {
  local cmake_directory
  if [[ ! "${BUILD_JOBS:-2}" =~ ^[1-9][0-9]*$ || ! "${BUILD_WORKERS:-1}" =~ ^[1-9][0-9]*$ ]]; then
    echo "BUILD_JOBS and BUILD_WORKERS must be positive integers." >&2
    return 1
  fi
  export DUOJIN02_PYTHON=/usr/bin/python3
  cmake_directory="$("$DUOJIN02_PYTHON" -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')" || return $?
  if [[ ! -x "$cmake_directory/cmake" ]]; then
    echo "The system Python's CMake installation is unavailable: $cmake_directory" >&2
    return 1
  fi
  case "$PATH" in
  "$cmake_directory" | "$cmake_directory":*) ;;
  *) export PATH="$cmake_directory:$PATH" ;;
  esac
  export BUILD_JOBS="${BUILD_JOBS:-2}"
  export BUILD_WORKERS="${BUILD_WORKERS:-1}"
  export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
  export MAKEFLAGS="-j$BUILD_JOBS"
}

duojin02_setup_environment() {
  local mode="${1:-runtime}"
  local workspace_root="${2:-}"
  local image_root="${3:-/opt/duojin02}"
  local ros_setup="${4:-/opt/ros/humble/setup.bash}"
  local domain_id="${ROS_DOMAIN_ID:-20}"
  local middleware="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  local data_root="${DUOJIN02_WORKSPACE_ROOT:-$workspace_root}"
  local overlay setup architecture overlay_architecture architecture_file installation_root
  local -a setups=()
  if [[ ( "$mode" != runtime && "$mode" != build ) || -z "$workspace_root" ]]; then
    echo "Environment mode must be runtime or build, with a workspace root." >&2
    return 1
  fi
  if [[ ! -r "$ros_setup" ]]; then
    echo "ROS 2 Humble is required; install Humble on Ubuntu 22.04 or use the hardware image." >&2
    return 1
  fi
  if [[ "$mode" == runtime ]]; then
    architecture="$(uname -m)" || return $?
    if [[ -e "$workspace_root/install/slam_backend" || -L "$workspace_root/install/slam_backend" ||
      -e "$workspace_root/install/hardware" || -L "$workspace_root/install/hardware" ]]; then
      installation_root="$workspace_root"
    else
      installation_root="$image_root"
    fi
    for overlay in slam_backend hardware; do
      setup="$installation_root/install/$overlay/setup.bash"
      if [[ ! -r "$setup" ]]; then
        echo "$overlay has not been built. Run ./scripts/build_hardware.sh first." >&2
        return 1
      fi
      architecture_file="${setup%/setup.bash}/.target_architecture"
      if [[ ! -r "$architecture_file" ]]; then
        echo "$overlay overlay architecture is missing; rebuild with ./scripts/build_hardware.sh." >&2
        return 1
      fi
      overlay_architecture="$(cat "$architecture_file")" || return $?
      if [[ "$overlay_architecture" != "$architecture" ]]; then
        echo "$overlay overlay architecture is stale or incompatible; rebuild with ./scripts/build_hardware.sh." >&2
        return 1
      fi
      setups+=("$setup")
    done
  fi
  source "$ros_setup" || return $?
  for setup in "${setups[@]}"; do
    source "$setup" || return $?
  done
  export ROS_DOMAIN_ID="$domain_id"
  export RMW_IMPLEMENTATION="$middleware"
  export DUOJIN02_WORKSPACE_ROOT="$data_root"
  if [[ "$mode" == build ]]; then
    duojin02_setup_build_tools || return $?
  fi
}
