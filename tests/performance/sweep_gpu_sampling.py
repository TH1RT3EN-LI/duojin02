#!/usr/bin/env python3
"""Preserve parameter sweeps separately from closed-loop navigation evidence."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

SETTINGS = {
    'baseline': [1600, 56, 8, .15, .12, .18, 12., 8., .3],
    'coverage': [6400, 56, 8, .15, .12, .18, 12., 8., .3],
    'wide_only': [3200, 56, 4, .30, .30, .12, 12., 8., .3],
    'balanced': [3200, 56, 4, .30, .30, .12, 3., 16., .3],
    'explore': [6400, 56, 4, .35, .35, .12, 1.5, 24., .3],
    'explore-lite': [3200, 56, 4, .35, .35, .12, 1.5, 24., .3],
    'long_horizon': [6400, 80, 6, .35, .35, .12, 1.5, 24., .3],
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--container', default='duojin02-gpu-dev-20261010')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--container-output', required=True)
    p.add_argument('--binary', default='/candidate/.cache/tuned-gpu/build/duojin01_navigation/gpu/gpu_sampling_benchmark')
    p.add_argument('--profiles', default=','.join(SETTINGS))
    p.add_argument('--seeds', default='20261010,101,202')
    p.add_argument('--iterations', type=int, default=20)
    p.add_argument('--backend-order', default='cuda', help='Use cpu,cuda,cuda,cpu for paired ABBA')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ledger_file = args.output/'ledger.json'
    if ledger_file.exists():
        p.error('Choose a fresh output directory')
    (args.output/'settings.json').write_text(json.dumps(SETTINGS, indent=2)+'\n')
    ledger = []
    for profile in args.profiles.split(','):
        settings = SETTINGS[profile]
        for seed in map(int, args.seeds.split(',')):
            for workload in ('open', 'corridor', 'detour', 'blocked'):
                for index, backend in enumerate(args.backend_order.split(',')):
                    stem = f'{profile}-{seed}-{workload}-{index}-{backend}'
                    cmd = ['docker', 'exec', '-w', '/candidate', args.container,
                           args.binary, backend, workload, str(args.iterations),
                           f'{args.container_output}/{stem}.bin', *map(str, settings), str(seed)]
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    (args.output/(stem+'.log')).write_text(result.stdout+result.stderr)
                    row = dict(profile=profile, seed=seed, workload=workload, backend=backend,
                               command=cmd, returncode=result.returncode, result=stem+'.json')
                    if not result.returncode:
                        data = json.loads(result.stdout.strip().splitlines()[-1])
                        (args.output/row['result']).write_text(json.dumps(data, indent=2)+'\n')
                        row['fixture_sha256'] = hashlib.sha256((args.output/(stem+'.bin')).read_bytes()).hexdigest()
                        print(dict(profile=profile, seed=seed, workload=workload, backend=backend,
                                   mean_ms=round(data['wall_mean_ms'], 3), p99_ms=round(data['wall_p99_ms'], 3),
                                   feasible=data['collision_free_min'], command=data['command']), flush=True)
                    ledger.append(row)
                    ledger_file.write_text(json.dumps(ledger, indent=2)+'\n')
                    if result.returncode:
                        raise RuntimeError(f'Benchmark failed: {stem}; evidence retained')


if __name__ == '__main__':
    main()
