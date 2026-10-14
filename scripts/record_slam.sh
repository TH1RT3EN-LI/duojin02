#!/usr/bin/env bash
# Record live sensor inputs; this script never starts hardware or motion nodes.
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 BAG_DIRECTORY (stop recording with Ctrl+C)" >&2
  exit 2
fi
source "$hardware_root/source.sh"
bag_directory="$(realpath -m "$1")"
if [[ -e "$bag_directory" ]]; then
  echo "Bag directory already exists: $bag_directory" >&2
  exit 1
fi
mkdir -p "$(dirname "$bag_directory")"
configuration_directory="${bag_directory}.configuration"
if ! mkdir "$configuration_directory"; then
  echo "Recording metadata directory must be new: $configuration_directory" >&2
  exit 1
fi
python3 - "$configuration_directory/slam-build.json" <<'PY'
import shutil
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
shutil.copyfile(Path(get_package_share_directory('slam_toolbox')) / 'hardware_build.json', sys.argv[1])
PY
ros2 bag record --storage sqlite3 --output "$bag_directory" \
  --qos-profile-overrides-path "$hardware_root/src/duojin02_bringup/config/rosbag_record_qos.yaml" \
  /scan /odom /imu /odometry/filtered /tf /tf_static /diagnostics &
recorder_pid=$!
metadata_pids=()
cleanup() {
  trap - INT TERM
  kill -INT "$recorder_pid" 2>/dev/null || true
  for pid in "${metadata_pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  # Preserve rosbag's normal metadata/database flush; this is not a force-kill
  # budget for recorded data. Disposable parameter queries can be killed.
  wait "$recorder_pid" 2>/dev/null || true
  for index in "${!metadata_pids[@]}"; do
    wait "${metadata_pids[$index]}" 2>/dev/null || true
    node="${metadata_nodes[$index]}"
    rm -f "$configuration_directory/$node.yaml.pending"
    echo "Parameter capture interrupted for /$node." >> "$configuration_directory/$node.error.log"
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Metadata must not postpone sensor recording. All queries share this window.
metadata_timeout=15
metadata_nodes=(base_driver lslidar_driver_node ekf_filter_node slam_toolbox)
for node in "${metadata_nodes[@]}"; do
  timeout --kill-after=1s "${metadata_timeout}s" ros2 param dump "/$node" \
    > "$configuration_directory/$node.yaml.pending" \
    2> "$configuration_directory/$node.error.log" &
  metadata_pids+=("$!")
done
for index in "${!metadata_nodes[@]}"; do
  node="${metadata_nodes[$index]}"
  if wait "${metadata_pids[$index]}"; then
    mv "$configuration_directory/$node.yaml.pending" "$configuration_directory/$node.yaml"
  else
    rm -f "$configuration_directory/$node.yaml.pending"
    echo "Parameter capture failed or exceeded the shared metadata window." >> "$configuration_directory/$node.error.log"
    echo "Parameters unavailable for /$node; see $configuration_directory/$node.error.log." >&2
  fi
  unset 'metadata_pids[index]'
done
metadata_pids=()
wait "$recorder_pid"
