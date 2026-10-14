#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
backend_source="$(python3 "$hardware_root/scripts/prepare_slam_backend.py")"
cd "$hardware_root"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
export MAKEFLAGS="-j${BUILD_JOBS:-2}"
colcon --log-base log/slam_backend build --base-paths "$backend_source" \
  --build-base build/slam_backend --install-base install/slam_backend \
  --merge-install --parallel-workers 1 --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
cp "$backend_source/hardware_build.json" install/slam_backend/share/slam_toolbox/
uname -m >install/slam_backend/.target_architecture
