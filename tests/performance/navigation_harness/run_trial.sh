#!/usr/bin/env bash
# One fresh graph of the selected current hardware build per invocation.
set -eo pipefail
case_name="${1:?case}"; seed="${2:?seed}"; output="${3:?output}"
source /opt/ros/humble/setup.bash
source /opt/duojin01/slam_backend/setup.bash
source /tmp/navbuild/install/local_setup.bash
source /ws/simulation/container_entry.sh
duojin_sim_verify_hardware_contract /hardware /ws /tmp/navbuild/install
export GZ_SIM_SYSTEM_PLUGIN_PATH="/tmp/duojin02_odometry_plugin:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="duojin_chassis_${case_name}_${seed}_$$"
export QT_QPA_PLATFORM=offscreen
if [[ ! -f /tmp/navbuild/navigation-build.json ]]; then
  printf '%s\n' 'Missing frozen build manifest; record it immediately after compilation.' >&2
  exit 3
fi
extra=()
if [[ "$case_name" == offplan_followpath ]]; then
  extra+=(--scenario-file /hardware/tests/performance/navigation_harness/offplan-followpath.json)
fi
exec python3 /ws/simulation/nav_benchmark.py \
  --case "$case_name" --seed "$seed" --output "$output" "${extra[@]}"
