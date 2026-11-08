#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/scripts/environment.sh"
duojin02_setup_environment build "$hardware_root"
backend_cache="${DUOJIN02_BACKEND_SOURCE_DIR:-$hardware_root/.cache/slam_backend/src}"
backend_install="${DUOJIN02_SLAM_INSTALL_BASE:-install/slam_backend}"
backend_build="${DUOJIN02_SLAM_BUILD_BASE:-build/slam_backend}"
backend_log="${DUOJIN02_SLAM_LOG_BASE:-log/slam_backend}"
backend_before="$(mktemp)"
backend_after="$(mktemp)"
trap 'rm -f "$backend_before" "$backend_after"' EXIT
"$DUOJIN02_PYTHON" "$hardware_root/scripts/cuda_install_provenance.py" snapshot --scope scan > "$backend_before"
backend_source="$("$DUOJIN02_PYTHON" "$hardware_root/scripts/prepare_slam_backend.py" --destination "$backend_cache/slam_toolbox")"
map_server_source="$("$DUOJIN02_PYTHON" "$hardware_root/scripts/prepare_slam_backend.py" --dependency nav2_map_server --destination "$backend_cache/nav2_map_server")"
cd "$hardware_root"
mkdir -p "$backend_install/share/slam_toolbox" "$backend_install/share/nav2_map_server"
rm -f "$backend_install/share/slam_toolbox/hardware_build.json" "$backend_install/share/nav2_map_server/hardware_build.json" "$backend_install/.duojin02-scan-source.json"
colcon --log-base "$backend_log" build --base-paths "$backend_source" "$map_server_source" "$hardware_root/src/duojin02_scan_geometry" \
  --build-base "$backend_build" --install-base "$backend_install" \
  --merge-install --parallel-workers "$BUILD_WORKERS" --cmake-force-configure --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DPython3_EXECUTABLE="$DUOJIN02_PYTHON" -DPYTHON_EXECUTABLE="$DUOJIN02_PYTHON" "$@"
"$DUOJIN02_PYTHON" "$hardware_root/scripts/cuda_install_provenance.py" snapshot --scope scan > "$backend_after"
if ! cmp -s "$backend_before" "$backend_after"; then
  echo 'Project sources changed during the SLAM build; rebuild.' >&2
  exit 1
fi
PYTHONPATH="$hardware_root/scripts" "$DUOJIN02_PYTHON" -c 'from pathlib import Path; import sys; from prepare_slam_backend import verify_prepared_source; verify_prepared_source(Path(sys.argv[1]), Path(sys.argv[2])); verify_prepared_source(Path(sys.argv[1]), Path(sys.argv[3]), "nav2_map_server")' "$hardware_root" "$backend_source" "$map_server_source"
cp "$backend_source/hardware_build.json" "$backend_install/share/slam_toolbox/"
cp "$map_server_source/hardware_build.json" "$backend_install/share/nav2_map_server/"
cp "$backend_after" "$backend_install/.duojin02-scan-source.json"
chmod 644 "$backend_install/.duojin02-scan-source.json"
uname -m >"$backend_install/.target_architecture"
