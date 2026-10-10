#!/usr/bin/env python3
"""Run fresh, frozen simulator trials for an explicit optional-GPU profile."""
import argparse
import json
import subprocess
import time
from pathlib import Path

SCRIPT = r'''set -eo pipefail
root="$1";build="$2";manifest="$3";config="$4";case_name="$5";seed="$6";output="$7";fixture="$8"
ln -sfn "$root" /hardware
ln -sfn "$manifest" /tmp/navbuild
source /opt/ros/humble/setup.bash
source /candidate/.cache/slam-install/setup.bash
source "$build/install/local_setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/duojin02_odometry_plugin
export GZ_PARTITION="duojin_tuning_${case_name}_${seed}_$$"
export QT_QPA_PLATFORM=offscreen
test -f /tmp/navbuild/navigation-build.json
extra=()
if [ "$fixture" != - ];then extra+=(--scenario-file "$fixture");fi
exec timeout --signal=INT --kill-after=15s 180s python3 /candidate/tests/performance/navigation_harness/nav_benchmark.py --variant upgraded \
 --case "$case_name" --seed "$seed" --nav-config "$config" --output "$output" "${extra[@]}"
'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--container', default='duojin02-gpu-dev-20261010')
    p.add_argument('--arm', choices=['baseline', 'tuned', 'original'], required=True)
    p.add_argument('--profile', required=True)
    p.add_argument('--config', required=True, help='Frozen absolute YAML path inside container')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--container-output', required=True)
    p.add_argument('--cases', default='offplan_obstacle')
    p.add_argument('--seeds', default='21,22')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ledger_file = args.output/'ledger.json'
    if ledger_file.exists():
        p.error('Choose a fresh output directory; existing measurements are immutable')
    root, build, manifest = {
        'baseline': ('/candidate/.cache/frozen-round1', '/candidate/.cache/full-gpu', '/candidate/.cache/round1-manifest'),
        'tuned': ('/candidate', '/candidate/.cache/tuned-gpu', '/candidate/.cache/tuned-gpu'),
        'original': ('/original', '/candidate/.cache/full-original', '/candidate/.cache/full-original'),
    }[args.arm]
    (args.output/'invocation.sh').write_text(SCRIPT)
    fixtures = {
        'offplan_obstacle': dict(name='offplan_obstacle', start=[-1.7, 0, 0], goal=[1.7, 0, 0],
                                timeout=55, obstacles=[[-.5, 0, .12, .12]], controller_component=True),
        'offplan_followpath': dict(name='offplan_followpath', start=[-1.7, 0, 0], goal=[1.7, 0, 0],
                                  timeout=55, obstacles=[], controller_component=True),
    }
    ledger = []
    for seed in map(int, args.seeds.split(',')):
        for case in args.cases.split(','):
            stem = f'{case}-{seed}'
            directory = args.output/stem
            directory.mkdir()
            fixture = '-'
            if case in fixtures:
                (directory/'fixture.json').write_text(json.dumps(fixtures[case], indent=2)+'\n')
                fixture = f'{args.container_output}/{stem}/fixture.json'
            cmd = ['docker', 'exec', '-w', '/candidate', args.container, 'bash', '-lc', SCRIPT,
                   'tuning', root, build, manifest, args.config, case, str(seed),
                   f'{args.container_output}/{stem}', fixture]
            started = time.time()
            with (directory/'console.log').open('w') as log:
                process = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            ledger.append(dict(arm=args.arm, profile=args.profile, case=case, seed=seed,
                               started_unix=started, returncode=process.returncode,
                               result=stem+'/result.json', command=cmd))
            ledger_file.write_text(json.dumps(ledger, indent=2)+'\n')
            if process.returncode:
                raise RuntimeError(f'Observer failed; retained {directory}')
            d = json.loads((directory/'result.json').read_text())
            print(dict(profile=args.profile, seed=seed, case=case) | {
                k: d.get(k) for k in ('success', 'status', 'arrival_acceptance_pass', 'wall_elapsed_s',
                                     'collision_events', 'complete_flow', 'gpu_control_summaries')}, flush=True)


if __name__ == '__main__':
    main()
