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
mkdir -p "$configuration_directory"
for node in base_driver lslidar_driver_node ekf_filter_node slam_toolbox; do
  timeout 15s ros2 param dump "/$node" > "$configuration_directory/$node.yaml" || \
    echo "Parameters unavailable for /$node; inspect the live nodes before using the bag." >&2
done
python3 - "$configuration_directory/slam-build.json" <<'PY'
import shutil
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
shutil.copyfile(Path(get_package_share_directory('slam_toolbox')) / 'hardware_build.json', sys.argv[1])
PY
exec ros2 bag record --storage sqlite3 --output "$bag_directory" \
  --qos-profile-overrides-path "$(PYTHONPATH="$hardware_root/src/duojin01_bringup${PYTHONPATH:+:$PYTHONPATH}" python3 -c 'from duojin01_bringup.chassis_profile import component_file; print(component_file("recording_qos"))')" \
  /scan /odom /imu /odometry/filtered /tf /tf_static /diagnostics
