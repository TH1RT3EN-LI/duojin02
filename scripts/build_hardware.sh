#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/scripts/environment.sh"
duojin02_setup_environment build "$hardware_root"
"$hardware_root/scripts/build_slam_backend.sh"
source "$hardware_root/install/slam_backend/setup.bash"
cd "$hardware_root"
colcon --log-base log/hardware build --base-paths src \
  --build-base build/hardware --install-base install/hardware \
  --parallel-workers "$BUILD_WORKERS" --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DDUOJIN_ENABLE_CUDA="${DUOJIN_ENABLE_CUDA:-ON}" \
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES:-native}" \
  -DPython3_EXECUTABLE="$DUOJIN02_PYTHON" -DPYTHON_EXECUTABLE="$DUOJIN02_PYTHON" "$@"
uname -m >install/hardware/.target_architecture
