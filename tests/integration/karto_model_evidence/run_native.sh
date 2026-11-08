#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source_tree="$repo/.cache/slam_backend/src/slam_toolbox"
out="$repo/output/karto-model-evidence"
while (($#)); do
  case "$1" in
    --source) source_tree=$2;shift 2;;
    --output) out=$2;shift 2;;
    *) echo "Usage: $0 [--source PREPARED_SOURCE] [--output OUTPUT_DIRECTORY]" >&2;exit 2;;
  esac
done
source_tree=$(realpath "$source_tree");mkdir -p "$out";out=$(realpath "$out")
base=$source_tree/lib/karto_sdk
test_sources=$repo/tests/integration/karto_model_evidence
verify_source() {
python3 -B - "$repo" "$source_tree" "$out" "$test_sources" "$1" <<'SOURCE_PY'
import hashlib,json,sys,importlib.util
from pathlib import Path
root,source,out,tests=map(Path,sys.argv[1:5]);phase=sys.argv[5]
spec=importlib.util.spec_from_file_location('verify_slam',root/'scripts/prepare_slam_backend.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.verify_prepared_source(root,source)
v={'source':str(source),'provenance':json.loads((source/'hardware_build.json').read_text()),'critical_sources':{str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source.rglob('*')) if p.is_file() and (p.suffix in ('.h','.hpp','.cpp') or p.name=='CMakeLists.txt')},'probe_sources':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(tests.glob('*')) if p.is_file()},'flags':['-std=c++17','-O1','-g0','-fPIC','-fsanitize=undefined,float-cast-overflow','-fno-sanitize-recover=all']}
if phase=='after':assert v==json.loads((out/'source-before.json').read_text()),'source changed while instrumented probes compiled/ran'
(out/('source-'+phase+'.json')).write_text(json.dumps(v,indent=2)+'\n')
SOURCE_PY
}
verify_source before
flags=(-I"$repo/src/duojin02_scan_geometry/include" -std=c++17 -O1 -g0 -fPIC -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all -I"$base/include" -I/usr/include/eigen3)
for pkg in rclcpp ament_index_cpp libstatistics_collector builtin_interfaces rosidl_runtime_c rcutils rosidl_typesupport_interface fastcdr rosidl_runtime_cpp rosidl_typesupport_fastrtps_cpp rmw rosidl_typesupport_fastrtps_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp rcl rcl_interfaces rcl_logging_interface rcl_yaml_param_parser libyaml_vendor tracetools rcpputils statistics_msgs rosgraph_msgs rosidl_typesupport_cpp rosidl_typesupport_c; do flags+=(-isystem "/opt/ros/humble/include/$pkg"); done
for src in Karto Mapper; do
  ${CXX:-g++} "${flags[@]}" -c "$base/src/$src.cpp" -o "$out/$src.o"
done
libs=(-lboost_serialization -lboost_system -lboost_filesystem -lboost_thread -ltbb)
${CXX:-g++} "${flags[@]}" "$test_sources/model_probe.cpp" "$out/Karto.o" "$out/Mapper.o" -o "$out/model_probe" "${libs[@]}"
"$out/model_probe" "$out" > "$out/model-probe.jsonl"
# Karto's public manager accessors are inline only in Mapper.cpp. Include that
# exact implementation in this test TU so the real accessor definitions are
# available, then link only Karto.o (no duplicate Mapper implementation).
${CXX:-g++} "${flags[@]}" -DKARTO_MAPPER_IMPLEMENTATION=\""$base/src/Mapper.cpp"\" "$test_sources/alternate_rollback.cpp" "$out/Karto.o" -o "$out/alternate_rollback" "${libs[@]}"
"$out/alternate_rollback" "$out" > "$out/alternate-rollback.jsonl"
verify_source after
