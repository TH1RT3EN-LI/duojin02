from pathlib import Path
import ast
import hashlib
import json
import platform
import shutil
import subprocess
import tarfile

sim = Path('/home/th1rt3en/dev/forge/duojin02-sim')
main = sim.with_name('duojin02')
cache = sim / '.cache/navigation-narrow'
out = main / 'docs/evidence/navigation-narrow'
out.mkdir(parents=True, exist_ok=True)
expected = {'frozen-v7': 78, 'regression-v7': 6, 'startup-counterexamples-v7': 2,
    'online-counterexample-v7': 2, 'waypoints-v7': 2, 'checks-v7': 9,
    'paired-previous-final-v4': 10, 'paired-optimized-v7': 10, 'cpu-limited-v7': 5}
for name, count in expected.items():
    assert len(list((cache / name).glob('*/result.json'))) == count, name
primary = [json.loads(p.read_text()) for p in (cache / 'frozen-v7').glob('*/result.json')]
assert len({(r['case'], r['seed']) for r in primary}) == 78
assert {r['seed'] for r in primary} == set(range(44, 50))
binary_sets = []
for p in (cache / 'frozen-v7').glob('*/result.json'):
    r = json.loads(p.read_text())
    assert r.get('verified_build') and r['rmw_implementation'] == 'rmw_cyclonedds_cpp'
    assert r['configuration_sha256'] == hashlib.sha256((main / 'src/duojin01_bringup/config/nav2.yaml').read_bytes()).hexdigest()
    for name, expected_sha in r['algorithm_sha256'].items():
        assert hashlib.sha256((main / name).read_bytes()).hexdigest() == expected_sha, name
    manifest = json.loads(p.with_name('navigation-build.json').read_text())
    binary_sets.append(manifest['binaries'])
assert all(binary == binary_sets[0] for binary in binary_sets)

# Extra command observations changed the harness hash, not the metric logic.
with tarfile.open(cache / 'frozen-source.tar.gz') as archive:
    old_evaluator = archive.extractfile('sim/simulation/nav_benchmark.py').read().decode()
new_evaluator = (sim / 'simulation/nav_benchmark.py').read_text()
def functions(source):
    tree = ast.parse(source)
    probe = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Probe')
    return {n.name: ast.dump(n, include_attributes=False) for n in probe.body if isinstance(n, ast.FunctionDef) and n.name in {'receive_truth', 'resolve_estimates', 'run'}}
assert functions(old_evaluator) == functions(new_evaluator)

roles = {name: ('primary_frozen' if name == 'frozen-v7' else 'final_auxiliary') for name in expected}
roles.update({'frozen-v5': 'superseded_frozen_ten_graph_batch', 'validation-v5': 'interrupted_scheduling_completed_trials',
              'frozen-v6': 'interrupted_after_velocity_bound_review', 'regression-v6': 'superseded_regression'})
roots = [root for root in cache.iterdir() if root.is_dir() and list(root.glob('*/result.json'))]
index = []
for root in roots:
    for p in sorted(root.glob('*/result.json')):
        index.append(dict(artifact=str(p.relative_to(cache)), role=roles.get(root.name, 'exploratory_or_diagnostic'),
                          sha256=hashlib.sha256(p.read_bytes()).hexdigest(), result=json.loads(p.read_text())))
(out / 'experiment-index.json').write_text(json.dumps(index, indent=2))
for label, selection in [('frozen-trials', [cache / name for name in expected]),
                         ('exploratory-trials', [root for root in roots if root.name not in expected])]:
    with tarfile.open(out / (label + '.tar.gz'), 'w:gz') as archive:
        for root in selection:
            for p in sorted(root.rglob('*')):
                if p.is_file() and '__pycache__' not in p.parts:
                    archive.add(p, arcname=str(p.relative_to(cache)), recursive=False)
with tarfile.open(out / 'source-and-harness.tar.gz', 'w:gz') as archive:
    for root in [main / 'src/duojin01_navigation', main / 'src/duojin01_bringup', main / 'tests', sim / 'simulation/src', sim / 'tests',
                 cache / 'previous-hardware/src/duojin01_navigation', cache / 'previous-hardware/src/duojin01_bringup', cache / 'previous-simulation-src', cache / 'diagnostic-hardware/src/duojin01_navigation', cache / 'diagnostic-hardware/src/duojin01_bringup', cache / 'precision-hardware/src/duojin01_navigation', cache / 'precision-hardware/src/duojin01_bringup', cache / 'bounds-hardware/src/duojin01_navigation', cache / 'bounds-hardware/src/duojin01_bringup']:
        for p in sorted(root.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc'):
                prefix = 'hardware/' if main in p.parents else 'sim/' if sim / 'simulation' in p.parents or sim / 'tests' in p.parents else 'experiments/'
                archive.add(p, arcname=prefix + str(p.relative_to(main if main in p.parents else sim)), recursive=False)
    for p in (sim / 'simulation').iterdir():
        if p.is_file() and (p.suffix in {'.py', '.sh'} or p.name in {'Dockerfile', 'README.md'}):
            archive.add(p, arcname='sim/simulation/' + p.name, recursive=False)
    for p in cache.glob('*source.tar.gz'):
        archive.add(p, arcname='snapshots/' + p.name, recursive=False)
    for name in ['Dockerfile', 'source.sh', 'README_DEVELOPMENT.md', 'scripts/profile_navigation.py']:
        archive.add(main / name, arcname='hardware/' + name, recursive=False)
for root in [cache / 'report']:
    for p in root.iterdir():
        if p.is_file(): shutil.copyfile(p, out / p.name)
for p in cache.iterdir():
    if p.is_file() and (p.suffix in {'.log', '.py', '.json'} or p.name.endswith('build.tar.gz') or p.name == 'previous-build-final.tar.gz' or p.name.startswith('runtime-') and p.name.endswith('.tar.gz')):
        shutil.copyfile(p, out / p.name)
metadata = dict(date='2026-10-07', algorithm_commit=subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=main, capture_output=True, text=True, check=True).stdout.strip(),
    previous_commit='7bc9e10ef93d114c1c54a6a35ef3e6f8a9f70434', primary_count=78, formal_counts=expected,
    metric_logic_unchanged=True, primary_binaries=binary_sets[0], not_orin_native=True, architecture=platform.machine(),
    cpu=subprocess.run(['lscpu'], capture_output=True, text=True, check=True).stdout,
    gpu=subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader'], capture_output=True, text=True, check=True).stdout,
    concurrency='four primary workers plus one auxiliary worker; final paired and quota batches run after primary completion. Earlier v5 had ten simultaneous graphs.',
    limits='Kinematic chassis plus wheels; assumed 5mm lidar noise; 540 beams/10Hz. Seeds control adapter perturbations, not Gazebo laser RNG. No real Orin, slip, physical braking, reflective/transparent or out-of-plane obstacle evidence. Failed and interrupted batches retained.')
(out / 'metadata.json').write_text(json.dumps(metadata, indent=2))
sha = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name != 'artifact-sha256.json'}
(out / 'artifact-sha256.json').write_text(json.dumps(sha, indent=2))
print('Archived', len(index), 'records, formal', sum(expected.values()), 'primary', len(primary))
