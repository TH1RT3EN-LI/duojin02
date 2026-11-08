#!/usr/bin/env bash
set -eo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source_tree="$repo/.cache/slam_backend/src/slam_toolbox"
out="$repo/output/karto-association-lifecycle"
while (($#)); do
  case "$1" in
    --source) source_tree=$2;shift 2;;
    --output) out=$2;shift 2;;
    *) echo 'Usage: run_native.sh [--source PREPARED_SOURCE] [--output OUTPUT_DIRECTORY]' >&2;exit 2;;
  esac
done
source_tree=$(realpath "$source_tree");mkdir -p "$out";out=$(realpath "$out")
printf 'pid=%s start_utc=%s\n' "$$" "$(date -u +%FT%TZ)" > "$out/runner-ledger.txt"
record_sources() {
python3 -B - "$repo" "$source_tree" "$out" "$1" <<'PY'
import hashlib,json,sys
from pathlib import Path
repo,source,out=map(Path,sys.argv[1:4]);phase=sys.argv[4]
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
value={'source':str(source),'critical_sources':{str(p.relative_to(source)):digest(p) for p in sorted(source.rglob('*')) if p.is_file() and (p.suffix in ('.cpp','.hpp','.h') or p.name=='CMakeLists.txt')},'probe_sources':{str(p.relative_to(repo)):digest(p) for p in sorted((repo/'tests/integration/karto_association_lifecycle').glob('*')) if p.is_file()},'sanitizers':['undefined','float-cast-overflow'],'recover':False}
if phase=='after':assert value==json.loads((out/'source-before.json').read_text()),'sources changed during native build/run'
(out/('source-'+phase+'.json')).write_text(json.dumps(value,indent=2)+'\n')
PY
}
record_sources before
cmake -S "$repo/tests/integration/karto_association_lifecycle" -B "$out/build" -DSLAM_SOURCE="$source_tree" -DCMAKE_BUILD_TYPE=Release > "$out/configure.log" 2>&1
timeout --signal=INT --kill-after=10 300 cmake --build "$out/build" -j3 > "$out/build.log" 2>&1
timeout --signal=INT --kill-after=5 120 "$out/build/lifecycle_probe" "$out" > "$out/probe.log" 2>&1
record_sources after
python3 -B - "$out" <<'PY'
import hashlib,json,sys
from pathlib import Path
out=Path(sys.argv[1]);lines=(out/'probe.log').read_text().splitlines();checks=[json.loads(line) for line in lines if line.startswith('{')]
assert len(checks)==3 and all(c.get('passed') for c in checks)
value={'passed':True,'checks':checks,'binary_sha256':hashlib.sha256((out/'build/lifecycle_probe').read_bytes()).hexdigest(),'source_manifest_sha256':hashlib.sha256((out/'source-before.json').read_bytes()).hexdigest(),'source_unchanged':True,'scope':'native production Karto lifecycle and native Ceres numeric transaction; no physical calibration or arbitrary allocation-failure guarantee'}
(out/'validation.json').write_text(json.dumps(value,indent=2)+'\n')
PY
printf 'complete_utc=%s\n' "$(date -u +%FT%TZ)" >> "$out/runner-ledger.txt"
