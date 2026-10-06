#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/source.sh"
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
lidar_test_root="${LIDAR_TEST_ROOT:-$hardware_root/build/lidar-tests}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base "$lidar_test_root/log" build \
  --base-paths "$hardware_root/src/lslidar_driver" \
  --build-base "$lidar_test_root/build" --install-base "$lidar_test_root/install" \
  --merge-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source "$lidar_test_root/install/setup.bash"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${LIDAR_TEST_DOMAIN_ID:-94}" \
  ctest --test-dir "$lidar_test_root/build/lslidar_driver" --output-on-failure
python3 "$hardware_root/tests/integration/lidar_output_probe.py"
