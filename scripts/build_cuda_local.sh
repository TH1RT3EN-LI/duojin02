#!/usr/bin/env bash
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
slam_prefix="${DUOJIN02_SLAM_PREFIX:-$cuda_workspace/install/slam_backend}"
if [[ -r "$slam_prefix/setup.bash" ]]; then
  "$DUOJIN02_PYTHON" "$cuda_workspace/scripts/cuda_install_provenance.py" verify-backend --prefix "$slam_prefix"
  source "$slam_prefix/setup.bash"
fi
cd "$cuda_workspace"
cuda_build_base="${DUOJIN02_CUDA_BUILD_BASE:-build/$cuda_variant-local-v3}"
cuda_install_base="${DUOJIN02_CUDA_INSTALL_BASE:-install/$cuda_variant-local-v3}"
cuda_log_base="${DUOJIN02_CUDA_LOG_BASE:-log/$cuda_variant-local-v3}"
mkdir -p "$cuda_install_base"
# Never leave a successful-build marker behind after an interrupted rebuild.
rm -f "$cuda_install_base/.duojin02-source-manifest.json"
cuda_before="$(mktemp)"
trap 'rm -f "$cuda_before"' EXIT
"$DUOJIN02_PYTHON" "$cuda_workspace/scripts/cuda_install_provenance.py" snapshot > "$cuda_before"
cuda_clean=(--cmake-clean-cache)
if [[ "${DUOJIN_CLEAN_BUILD:-OFF}" == ON ]]; then
  cuda_clean+=(--cmake-clean-first)
fi
colcon --log-base "$cuda_log_base" build \
  --base-paths src/duojin02_scan_geometry src/duojin02_mppi_controller src/duojin02_amcl_cuda \
    src/duojin02_navigation src/duojin02_bringup src/duojin02_slam_tools \
  --build-base "$cuda_build_base" \
  --install-base "$cuda_install_base" --merge-install \
  --parallel-workers "$BUILD_WORKERS" --cmake-force-configure "${cuda_clean[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DDUOJIN_ENABLE_CUDA="$cuda_enabled" \
  -DDUOJIN_CUDA_REQUIRED="${DUOJIN_CUDA_REQUIRED:-$cuda_enabled}" \
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES:-native}" \
  -DPython3_EXECUTABLE="$DUOJIN02_PYTHON" -DPYTHON_EXECUTABLE="$DUOJIN02_PYTHON" "$@"
uname -m > "$cuda_install_base/.target_architecture"
"$DUOJIN02_PYTHON" "$cuda_workspace/scripts/cuda_install_provenance.py" seal --before "$cuda_before" --prefix "$cuda_install_base"
