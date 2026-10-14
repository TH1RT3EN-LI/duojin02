#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/duojin02/slam_backend/setup.bash
source /opt/duojin02/install/setup.bash
if [[ -r /ws/install/slam_backend/setup.bash ]]; then
  [[ "$(cat /ws/install/slam_backend/.target_architecture 2>/dev/null)" == "$(uname -m)" ]] ||
    {
      echo "SLAM overlay architecture is stale or incompatible; rebuild it on this host." >&2
      exit 1
    }
  source /ws/install/slam_backend/setup.bash
fi
if [[ -r /ws/install/hardware/setup.bash ]]; then
  [[ "$(cat /ws/install/hardware/.target_architecture 2>/dev/null)" == "$(uname -m)" ]] ||
    {
      echo "Hardware overlay architecture is stale or incompatible; rebuild it on this host." >&2
      exit 1
    }
  source /ws/install/hardware/setup.bash
fi
exec "$@"
