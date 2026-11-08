#!/usr/bin/env bash
set -eo pipefail
source_root="${1:?source root required}"
output_root="${2:?empty output directory required}"
cuda_enabled="${3:-ON}"
case "$cuda_enabled" in ON|OFF) ;; *) exit 2 ;; esac
if [[ -e "$output_root/install" ]]; then
  echo "Refusing to replace an existing isolated install: $output_root" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if [[ -r /opt/duojin02/source.sh ]]; then source /opt/duojin02/source.sh; fi
if [[ -r /work/install/slam_backend/setup.bash ]]; then source /work/install/slam_backend/setup.bash; fi
mkdir -p "$output_root"
export PATH="/opt/cuda/bin:$PATH"
colcon --log-base "$output_root/log" build \
  --base-paths "$source_root/src/duojin02_scan_geometry" \
    "$source_root/src/duojin02_mppi_controller" \
    "$source_root/src/duojin02_amcl_cuda" "$source_root/src/duojin02_navigation" \
    "$source_root/src/duojin02_bringup" "$source_root/src/duojin02_slam_tools" \
  --build-base "$output_root/build" --install-base "$output_root/install" \
  --merge-install --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DDUOJIN_ENABLE_CUDA="$cuda_enabled" \
  -DDUOJIN_CUDA_REQUIRED="$cuda_enabled" -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
