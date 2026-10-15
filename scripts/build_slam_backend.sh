#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/scripts/environment.sh"
duojin02_setup_environment build "$hardware_root"
backend_source="$("$DUOJIN02_PYTHON" "$hardware_root/scripts/prepare_slam_backend.py")"
cd "$hardware_root"
colcon --log-base log/slam_backend build --base-paths "$backend_source" \
  --build-base build/slam_backend --install-base install/slam_backend \
  --merge-install --parallel-workers "$BUILD_WORKERS" --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DPython3_EXECUTABLE="$DUOJIN02_PYTHON" -DPYTHON_EXECUTABLE="$DUOJIN02_PYTHON" "$@"
cp "$backend_source/hardware_build.json" install/slam_backend/share/slam_toolbox/
uname -m >install/slam_backend/.target_architecture
