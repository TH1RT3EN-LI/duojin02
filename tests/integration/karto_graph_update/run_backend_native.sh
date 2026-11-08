#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source_tree=${1:-"$repo/.cache/slam_backend/graph-update-stage"}
out=${2:-"$repo/output/karto-graph-update-backend"}
mkdir -p "$out";out=$(realpath "$out");source_tree=$(realpath "$source_tree")
printf 'pid=%s start_utc=%s\n' "$$" "$(date -u +%FT%TZ)" > "$out/runner-ledger.txt"
record() {
python3 -B - "$source_tree" "$repo" "$out" "$1" <<'PY'
from pathlib import Path
import hashlib,json,sys
source,repo,out=map(Path,sys.argv[1:4]);phase=sys.argv[4]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=[source/'solvers/ceres_solver.cpp',source/'solvers/ceres_solver.hpp',source/'lib/karto_sdk/include/karto_sdk/Mapper.h',source/'lib/karto_sdk/src/Mapper.cpp',source/'lib/karto_sdk/include/karto_sdk/MeasurementModel.h',source/'lib/karto_sdk/include/karto_sdk/GraphUpdate.h',repo/'tests/integration/karto_graph_update/backend_probe.cpp',repo/'tests/integration/karto_graph_update/backend-CMakeLists.txt',repo/'tests/integration/karto_graph_update/run_backend_native.sh']
v={str(p):sha(p) for p in files};target=out/('source-'+phase+'.json')
if phase=='after':assert v==json.loads((out/'source-before.json').read_text()),'source changed during native validation'
target.write_text(json.dumps(v,indent=2)+'\n')
PY
}
record before
cp "$repo/tests/integration/karto_graph_update/backend-CMakeLists.txt" "$out/CMakeLists.txt"
cmake -S "$out" -B "$out/build" -DSLAM_SOURCE="$source_tree" -DCMAKE_BUILD_TYPE=Release > "$out/configure.log" 2>&1
timeout --signal=INT --kill-after=10 300 cmake --build "$out/build" -j3 > "$out/build.log" 2>&1
timeout --signal=INT --kill-after=5 120 "$out/build/backend_probe" > "$out/probe.log" 2>&1
record after
python3 -B - "$out" <<'PY'
from pathlib import Path
import hashlib,json,sys
out=Path(sys.argv[1]);checks=[json.loads(l) for l in (out/'probe.log').read_text().splitlines() if l.startswith('{')];assert checks and all(v['passed'] for v in checks)
v={'passed':True,'source_unchanged':True,'checks':checks,'binary_sha256':hashlib.sha256((out/'build/backend_probe').read_bytes()).hexdigest(),'source_manifest_sha256':hashlib.sha256((out/'source-before.json').read_bytes()).hexdigest(),'sanitizers':['undefined','float-cast-overflow'],'scope':'production Ceres prepared transaction, full candidate validation, active guard, abort after apply/finalize rejection, native fault recovery, profiles and absolute staging restore; not physical accuracy'}
(out/'validation.json').write_text(json.dumps(v,indent=2)+'\n')
PY
printf 'complete_utc=%s\n' "$(date -u +%FT%TZ)" >> "$out/runner-ledger.txt"
