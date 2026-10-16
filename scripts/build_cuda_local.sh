#!/usr/bin/env bash
# Build the source-pinned optimizer, localizer, and their actual navigation consumers.
set -eo pipefail
cuda_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$cuda_workspace/scripts/environment.sh"
duojin02_setup_environment build "$cuda_workspace"
cuda_enabled="${DUOJIN_ENABLE_CUDA:-ON}"
case "$cuda_enabled" in
  ON) cuda_variant=cuda ;;
  OFF) cuda_variant=cpu ;;
  *) echo 'DUOJIN_ENABLE_CUDA must be ON or OFF' >&2; exit 2 ;;
esac
# The image provides the remaining robot packages and pinned SLAM dependency.
if [[ -r /opt/duojin02/source.sh ]]; then
  source /opt/duojin02/source.sh
fi
cd "$cuda_workspace"
cuda_clean=()
if [[ "${DUOJIN_CLEAN_BUILD:-OFF}" == ON ]]; then
  cuda_clean=(--cmake-clean-first)
fi
colcon --log-base "log/$cuda_variant-local-v3" build \
  --base-paths src/duojin02_mppi_controller src/duojin02_amcl_cuda \
    src/duojin02_navigation src/duojin02_bringup \
  --build-base "build/$cuda_variant-local-v3" \
  --install-base "install/$cuda_variant-local-v3" --merge-install \
  --parallel-workers "$BUILD_WORKERS" --cmake-force-configure "${cuda_clean[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DDUOJIN_ENABLE_CUDA="$cuda_enabled" \
  -DDUOJIN_CUDA_REQUIRED="${DUOJIN_CUDA_REQUIRED:-$cuda_enabled}" \
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES:-native}" \
  -DPython3_EXECUTABLE="$DUOJIN02_PYTHON" -DPYTHON_EXECUTABLE="$DUOJIN02_PYTHON" "$@"
uname -m > "install/$cuda_variant-local-v3/.target_architecture"
