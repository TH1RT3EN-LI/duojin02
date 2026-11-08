#!/usr/bin/env bash
# Isolated new-process held-out global relocalization, no pose prior or GT read.
set -eo pipefail
prefix="${1:?isolated install prefix}"
trial="${2:?new trial output}"
arm="${3:?T0D0 or T0D1}"
params="${4:?parameter YAML}"
input_root="${5:?frozen heldout input directory}"
source /opt/ros/humble/setup.bash
source "$prefix/setup.bash"
export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${DUOJIN_HELDOUT_DOMAIN:-227}"
formal=/work/output/lilocbench-base-reference-formal-20261017
cell="B01_${arm#T0}_C000"
python3 /work/tests/realtime_goal/replay_heldout_global.py --arm "$arm" \
  --bag "$input_root/dynamics_0/dynamics_0_0.db3" \
  --adaptation "$input_root/dynamics_0.transport.json" \
  --map "$formal/query_maps/$cell/map.yaml" \
  --training-source-roster "$formal/support/source_scan_manifest.csv" \
  --parameters "$params" --output "$trial/replay" --rate 2 --max-scans 0 \
  --expected-prefix "$prefix" --executable "$prefix/lib/duojin02_amcl_cuda/amcl"
python3 /work/tests/realtime_goal/localization/audit_motion_selection.py \
  --bag "$input_root/dynamics_0/dynamics_0_0.db3" --replay-dir "$trial/replay"
