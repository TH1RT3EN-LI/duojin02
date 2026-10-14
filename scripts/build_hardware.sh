#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
"$hardware_root/scripts/build_slam_backend.sh"
source "$hardware_root/install/slam_backend/setup.bash"
cd "$hardware_root"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
export MAKEFLAGS="-j${BUILD_JOBS:-2}"
colcon --log-base log/hardware build --base-paths src \
  --build-base build/hardware --install-base install/hardware \
  --symlink-install --parallel-workers "${BUILD_WORKERS:-1}" --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
uname -m >install/hardware/.target_architecture
