#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source_tree=${1:-"$repo/.cache/slam_backend/src/slam_toolbox"}
out=${2:-"$repo/output/karto-graph-update-coordinator"}
mkdir -p "$out"
out=$(realpath "$out")
source_tree=$(realpath "$source_tree")
printf 'pid=%s start_utc=%s\n' "$$" "$(date -u +%FT%TZ)" > "$out/runner-ledger.txt"
record() {
python3 -B - "$source_tree" "$repo" "$out" "$1" <<'PY'
from pathlib import Path
import hashlib,json,sys
source,repo,out=map(Path,sys.argv[1:4]);phase=sys.argv[4]
paths=[source/'solvers/ceres_solver.cpp',source/'solvers/ceres_solver.hpp',source/'solvers/ceres_utils.h',
       source/'lib/karto_sdk/src/Karto.cpp',source/'lib/karto_sdk/src/Mapper.cpp',
       source/'lib/karto_sdk/include/karto_sdk/Mapper.h',source/'lib/karto_sdk/include/karto_sdk/Karto.h',
       source/'lib/karto_sdk/include/karto_sdk/MeasurementModel.h',source/'lib/karto_sdk/include/karto_sdk/GraphUpdate.h',
       repo/'tests/integration/karto_graph_update/coordinator_probe.cpp',
       repo/'tests/integration/karto_graph_update/CMakeLists.txt',
       repo/'tests/integration/karto_graph_update/run_coordinator_native.sh']
v={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
if phase=='after':assert v==json.loads((out/'source-before.json').read_text()),'source changed during validation'
(out/('source-'+phase+'.json')).write_text(json.dumps(v,indent=2)+'\n')
PY
}
record before
timeout --signal=INT --kill-after=10 120 cmake -S "$repo/tests/integration/karto_graph_update" -B "$out/build" -DSLAM_SOURCE="$source_tree" -DCMAKE_BUILD_TYPE=Release > "$out/configure.log" 2>&1
timeout --signal=INT --kill-after=10 300 cmake --build "$out/build" -j3 > "$out/build.log" 2>&1
mkdir -p "$out/run"
timeout --signal=INT --kill-after=5 120 "$out/build/coordinator_probe" "$out/run" > "$out/probe.log" 2>&1
record after
python3 -B - "$out" <<'PY'
from pathlib import Path
import hashlib,json,sys
out=Path(sys.argv[1]);checks=[json.loads(l) for l in (out/'probe.log').read_text().splitlines() if l.startswith('{')]
assert checks and all(v['passed'] for v in checks)
v={'passed':True,'source_unchanged':True,'checks':checks,
   'binary_sha256':hashlib.sha256((out/'build/coordinator_probe').read_bytes()).hexdigest(),
   'source_manifest_sha256':hashlib.sha256((out/'source-before.json').read_bytes()).hexdigest(),
   'sanitizers':['undefined','float-cast-overflow'],
   'scope':'Native graph coordinator, both-container rollback and fail-stop, retained factors, dirty/covered and attempt budgets, two-state checkpoint; no physical accuracy claim'}
(out/'validation.json').write_text(json.dumps(v,indent=2)+'\n')
PY
printf 'complete_utc=%s\n' "$(date -u +%FT%TZ)" >> "$out/runner-ledger.txt"
