#!/usr/bin/env python3
"""Fresh baseline/tuned navigation matrix, alternating arm order by seed."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--container', default='duojin02-gpu-dev-20261010')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--container-output', required=True)
    p.add_argument('--cases', default='open,corridor_0.50,s_bend,dynamic_crossing,offplan_followpath,offplan_obstacle')
    p.add_argument('--seeds', default='21,22')
    p.add_argument('--profiles', default='baseline,explore')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ledger_file = args.output/'ledger.json'
    if ledger_file.exists():
        p.error('Choose a fresh output directory')
    runner = Path(__file__).with_name('run_gpu_tuning.py')
    (args.output/'observer-driver.json').write_text(json.dumps({
        str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__), runner)
    }, indent=2)+'\n')
    monitor = subprocess.Popen(['nvidia-smi', '--query-gpu=timestamp,temperature.gpu,power.draw,utilization.gpu,memory.used',
                                '--format=csv', '-lms', '200'], stdout=(args.output/'gpu-telemetry.csv').open('w'),
                               stderr=subprocess.DEVNULL)
    ledger = []
    try:
        for i, seed in enumerate(map(int, args.seeds.split(','))):
            profiles = args.profiles.split(',')
            if i % 2:
                profiles.reverse()
            for case in args.cases.split(','):
                for profile in profiles:
                    stem = f'{case}-{seed}-{profile}'
                    directory = args.output/stem
                    cmd = [sys.executable, str(runner), '--container', args.container,
                           '--arm', 'baseline' if profile == 'baseline' else 'tuned',
                           '--profile', profile, '--config', f'/evidence/tuning-round2/profiles/{profile}.yaml',
                           '--output', str(directory), '--container-output', args.container_output+'/'+stem,
                           '--cases', case, '--seeds', str(seed)]
                    process = subprocess.run(cmd)
                    if (directory/'ledger.json').exists():
                        for row in json.loads((directory/'ledger.json').read_text()):
                            row['result'] = stem+'/'+row['result']
                            ledger.append(row)
                    ledger_file.write_text(json.dumps(ledger, indent=2)+'\n')
                    if process.returncode:
                        raise RuntimeError(f'Comparison failed; retained {directory}')
    finally:
        monitor.terminate()
        monitor.wait(timeout=5)


if __name__ == '__main__':
    main()
