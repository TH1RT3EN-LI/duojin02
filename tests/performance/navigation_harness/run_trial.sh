#!/usr/bin/env bash
# This reproducibility wrapper uses the isolated comparison-container mounts.
# It does not connect to or start hardware. One fresh graph per invocation.
set -eo pipefail
arm="${1:?original|cpu|gpu}";case_name="${2:?case}";seed="${3:?seed}";output="${4:?output}"
case "$arm" in
 original) hardware_root=/original;nav_config=/original/src/duojin01_bringup/config/nav2.yaml ;;
 cpu) hardware_root=/candidate;nav_config=/candidate/src/duojin01_bringup/config/nav2.yaml ;;
 gpu) hardware_root=/candidate;nav_config=/candidate/.cache/profiles/gpu.yaml ;;
 *) exit 2 ;;
esac
ln -sfn "$hardware_root" /hardware
ln -sfn "/candidate/.cache/full-$arm" /tmp/navbuild
source /opt/ros/humble/setup.bash
source /candidate/.cache/slam-install/setup.bash
source "/candidate/.cache/full-$arm/install/local_setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/tmp/duojin02_odometry_plugin:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="duojin_gpu_${arm}_${case_name}_${seed}_$$"
export QT_QPA_PLATFORM=offscreen
python3 /candidate/tests/performance/navigation_harness/record_build.py
extra=()
if [ "$case_name" = offplan_followpath ];then
 extra+=(--scenario-file /candidate/tests/performance/navigation_harness/offplan-followpath.json)
fi
exec python3 /candidate/tests/performance/navigation_harness/nav_benchmark.py --variant upgraded \
 --case "$case_name" --seed "$seed" --nav-config "$nav_config" --output "$output" "${extra[@]}"
