#!/usr/bin/env bash
set -euo pipefail
root=${ROOT:-/work}
out=${1:?fresh output directory}
source_tree=${2:?verified prepared source}
objects=${3:?native object directory compiled from same source}
test_source=${4:-regression_native.cpp}
mkdir -p "$out"
base=$source_tree/lib/karto_sdk
flags=(-I"$root/src/duojin02_scan_geometry/include" -std=c++17 -O1 -g0 -fPIC -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all -I"$base/include" -I/usr/include/eigen3)
for pkg in rclcpp ament_index_cpp libstatistics_collector builtin_interfaces rosidl_runtime_c rcutils rosidl_typesupport_interface fastcdr rosidl_runtime_cpp rosidl_typesupport_fastrtps_cpp rmw rosidl_typesupport_fastrtps_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp rcl rcl_interfaces rcl_logging_interface rcl_yaml_param_parser libyaml_vendor tracetools rcpputils statistics_msgs rosgraph_msgs rosidl_typesupport_cpp rosidl_typesupport_c; do flags+=(-isystem "/opt/ros/humble/include/$pkg"); done
g++ "${flags[@]}" "$root/tests/realtime_goal/mapping/$test_source" "$objects/Karto.o" "$objects/Mapper.o" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb -o "$out/probe"
if [[ "$test_source" == "capture_native.cpp" ]]; then
  "$out/probe" "$root/output/lilocbench-association-remediation-20261017/frozen_candidates/pool.json" "$root/output/lilocbench-model-remediation-20261016" "$out/results.jsonl"
else
  "$out/probe" "$out/results.jsonl"
fi
