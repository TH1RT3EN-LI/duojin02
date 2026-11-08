#!/usr/bin/env bash
set -euo pipefail
root=/work
out=$root/output/$(basename "$(cat /tmp/duojin02-implementation-output 2>/dev/null || echo output/lilocbench-realtime-goal-implementation-20261019-212448)")/mapping/E2
# Explicit output argument keeps this reusable across dated output directories.
out=${1:-$out}
source_tree=${2:-$out/verified-source}
mkdir -p "$out/native"
base=$source_tree/lib/karto_sdk
flags=(-I"$root/src/duojin02_scan_geometry/include" -std=c++17 -O1 -g0 -fPIC -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all -I"$base/include" -I/usr/include/eigen3)
for pkg in rclcpp ament_index_cpp libstatistics_collector builtin_interfaces rosidl_runtime_c rcutils rosidl_typesupport_interface fastcdr rosidl_runtime_cpp rosidl_typesupport_fastrtps_cpp rmw rosidl_typesupport_fastrtps_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp rcl rcl_interfaces rcl_logging_interface rcl_yaml_param_parser libyaml_vendor tracetools rcpputils statistics_msgs rosgraph_msgs rosidl_typesupport_cpp rosidl_typesupport_c; do flags+=(-isystem "/opt/ros/humble/include/$pkg"); done
for src in Karto Mapper; do g++ "${flags[@]}" -c "$base/src/$src.cpp" -o "$out/native/$src.o"; done
g++ "${flags[@]}" "$root/tests/realtime_goal/mapping/native_probe.cpp" "$out/native/Karto.o" "$out/native/Mapper.o" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb -o "$out/native/probe"
"$out/native/probe" "$root/output/lilocbench-association-remediation-20261017/frozen_candidates/pool.json" "$root/output/lilocbench-model-remediation-20261016" "$out"
g++ "${flags[@]}" "$root/tests/realtime_goal/mapping/extra_native.cpp" "$out/native/Karto.o" "$out/native/Mapper.o" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb -o "$out/native/extra_probe"
"$out/native/extra_probe" "$out/extra-results.jsonl"
g++ "${flags[@]}" "$root/tests/realtime_goal/mapping/pool_native.cpp" "$out/native/Karto.o" "$out/native/Mapper.o" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb -o "$out/native/pool_probe"
"$out/native/pool_probe" "$root/output/lilocbench-association-remediation-20261017/frozen_candidates/pool.json" "$root/output/lilocbench-model-remediation-20261016" "$out/full-pool-results.jsonl"
