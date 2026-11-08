#!/usr/bin/env bash
set -euo pipefail
# Run inside the project's pinned ROS/CUDA build container.
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
source_tree=${1:-$repo/.cache/slam_backend/src/slam_toolbox}
out=${2:-$repo/output/karto-measurement-contract}
base=$source_tree/lib/karto_sdk
mkdir -p "$out"
verify_snapshot() {
  python3 - "$repo" "$source_tree" "$out" "$1" <<'SNAPSHOT_PY'
import hashlib, importlib.util, json, sys
from pathlib import Path
repo, source, output = (Path(x).resolve() for x in sys.argv[1:4])
phase = sys.argv[4]
spec = importlib.util.spec_from_file_location("verified_slam_source", repo / "scripts/prepare_slam_backend.py")
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
module.verify_prepared_source(repo, source)
inputs = [repo / "src/duojin02_scan_geometry/include/duojin02_scan_geometry/geometry.hpp",
          repo / "tests/integration/scan_geometry_fixture.hpp",
          repo / "tests/integration/karto_deskew_probe.cpp",
          repo / "tests/integration/karto_measurement_contract_probe.cpp",
          repo / "tests/integration/karto_coverage_normalization_probe.cpp",
          repo / "tests/integration/fixtures/lilocbench_geometry_scan.txt",
          repo / "tests/integration/fixtures/karto_prepared_v1.scan"]
value = {"source": str(source), "source_provenance": json.loads((source / "hardware_build.json").read_text()),
         "inputs_sha256": {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
         "sanitizers": ["undefined", "float-cast-overflow"], "recover": False}
path = output / ("source-" + phase + ".json")
if phase == "after":
    assert value == json.loads((output / "source-before.json").read_text()), "source changed while native test was compiling/running"
path.write_text(json.dumps(value, indent=2) + "\n")
SNAPSHOT_PY
}
verify_snapshot before
flags=(-I"$repo/src/duojin02_scan_geometry/include" -std=c++17 -O1 -g0 -fPIC -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all -I"$base/include" -I/usr/include/eigen3)
for pkg in rclcpp ament_index_cpp libstatistics_collector builtin_interfaces rosidl_runtime_c rcutils rosidl_typesupport_interface fastcdr rosidl_runtime_cpp rosidl_typesupport_fastrtps_cpp rmw rosidl_typesupport_fastrtps_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp rcl rcl_interfaces rcl_logging_interface rcl_yaml_param_parser libyaml_vendor tracetools rcpputils statistics_msgs rosgraph_msgs rosidl_typesupport_cpp rosidl_typesupport_c; do flags+=(-isystem "/opt/ros/humble/include/$pkg"); done
for src in Karto Mapper; do
  ${CXX:-g++} "${flags[@]}" -c "$base/src/$src.cpp" -o "$out/$src.o"
done
${CXX:-g++} "${flags[@]}" "$repo/tests/integration/karto_deskew_probe.cpp" "$out/Karto.o" "$out/Mapper.o" -o "$out/deskew_probe" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb
"$out/deskew_probe"

${CXX:-g++} "${flags[@]}" "$repo/tests/integration/karto_measurement_contract_probe.cpp" "$out/Karto.o" "$out/Mapper.o" -o "$out/measurement_probe" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb
"$out/measurement_probe" "$repo/tests/integration/fixtures/lilocbench_geometry_scan.txt" "$repo/tests/integration/fixtures/karto_prepared_v1.scan"
${CXX:-g++} "${flags[@]}" "$repo/tests/integration/karto_coverage_normalization_probe.cpp" "$out/Karto.o" "$out/Mapper.o" -o "$out/coverage_probe" -lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb
"$out/coverage_probe"
verify_snapshot after
