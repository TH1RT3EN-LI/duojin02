#!/usr/bin/env bash
set -eo pipefail
# This script runs INSIDE an isolated ROS container with /work and frozen inputs.
prefix="${1:?isolated install prefix}"; trial="${2:?new trial output}"; arm="${3:?T0D0 or T0D1}"
params="${4:?parameter YAML}"; max_scans="${5:-0}"
source /opt/ros/humble/setup.bash
source "$prefix/setup.bash"
export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=226
formal=/work/output/lilocbench-base-reference-formal-20261017
cell="B01_${arm#T0}_C000"
python3 /work/tests/realtime_goal/replay_recorded.py --arm "$arm" \
  --bag /oldresults/adapter-tf-lead/mapping_ros2_0.db3 \
  --adaptation /oldresults/adapter-tf-lead/adaptation.json \
  --map "$formal/query_maps/$cell/map.yaml" --prior "$formal/derived/$cell/first_graph_pose.json" \
  --parameters "$params" --output "$trial/replay" --rate 2 --max-scans "$max_scans" \
  --expected-prefix "$prefix" --executable "$prefix/lib/duojin02_amcl_cuda/amcl"

python3 /work/tests/realtime_goal/localization/audit_motion_selection.py \
  --bag /oldresults/adapter-tf-lead/mapping_ros2_0.db3 --replay-dir "$trial/replay"
