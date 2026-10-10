#!/usr/bin/env python3
"""Audit all frozen tuning measurements, including failed goals and child exits."""
import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path


def aggregate(values):
    wall = [x for v in values for x in v['wall_samples_ms']]
    cpu = [x for v in values for x in v['cpu_samples_ms']]
    ordered = sorted(wall)
    return dict(processes=len(values), samples=len(wall), wall_mean_ms=statistics.mean(wall),
                wall_p95_ms=ordered[int(.95*(len(wall)-1))], wall_p99_ms=ordered[int(.99*(len(wall)-1))],
                wall_max_ms=max(wall), cpu_mean_ms=statistics.mean(cpu),
                cold_max_ms=max(v.get('cold_ms', v.get('cold_wall_ms')) for v in values))


def summarize(root):
    components = root/'final/components'
    groups = defaultdict(list)
    for row in json.loads((components/'ledger.json').read_text()):
        assert row['returncode'] == 0
        data = json.loads((components/row['result']).read_text())
        fixture = components/Path(row['result']).with_suffix('.bin')
        assert hashlib.sha256(fixture.read_bytes()).hexdigest() == row['fixture_sha256']
        groups[row['profile'], row['workload']].append(data | {'fixture_sha256': row['fixture_sha256']})
    result = dict(components=[], api=[], navigation=[], all_trials=[], input_checks_passed=True,
                  arrival_scope='Final truth and dwell include the unchanged 0.3 s post-result observer tail; no separately saved Action-result boundary snapshot.',
                  collision_scope='Filled 8-vertex navigation footprint from YAML, without costmap padding, planar yaw interpolation at <=4 mm corner/translation travel; no full-3D physical guarantee.')
    for (profile, workload), values in groups.items():
        assert len({v['fixture_sha256'] for v in values}) == 1
        assert len({(v['valid_calls'], v['collision_free_min']) for v in values}) == 1
        assert max(abs(a-b) for v in values for a,b in zip(v['command'], values[0]['command'])) < 1e-8
        for backend in ('cpu', 'cuda'):
            ds = [v for v in values if v['backend'] == backend]
            result['components'].append(dict(profile=profile, workload=workload, backend=backend,
                collision_free_min=min(v['collision_free_min'] for v in ds),
                valid_calls=sum(v['valid_calls'] for v in ds), **aggregate(ds)))
    api = root/'final/api'
    groups = defaultdict(list)
    for row in json.loads((api/'ledger.json').read_text()):
        assert row['returncode'] == 0
        d = json.loads((api/row['result']).read_text())
        assert d['native_calls'] == (d['iterations'] if row['arm'] == 'original' else 0)
        assert d['zero_calls'] == 0
        for x,y,w in d['commands']:
            assert -.20000001 <= x <= .50000001 and abs(y) <= .30000001 and abs(w) <= .65000001
        groups[row['arm']].append(d)
    for arm, ds in groups.items():
        result['api'].append(dict(arm=arm, native_calls=sum(d['native_calls'] for d in ds), **aggregate(ds)))
    nav = root/'final/navigation'
    groups = defaultdict(list)
    for row in json.loads((nav/'ledger.json').read_text()):
        assert row['returncode'] == 0
        directory = nav/Path(row['result']).parent
        d = json.loads((nav/row['result']).read_text())
        assert d['verified_build'] and d['navigation_goal_count'] == 1
        logs = (directory/'launch.log').read_text()
        d['child_failure_lines'] = [s for s in logs.splitlines() if 'process has died' in s]
        d['audited_complete_flow'] = bool(d['complete_flow']) and not d['child_failure_lines']
        d['sampling_stats'] = [dict(zip(('calls', 'valid', 'feasible_min', 'feasible_mean', 'wall_mean_ms', 'wall_max_ms'), map(float, m)))
            for m in re.findall(r'GPU_SAMPLING_SUMMARY calls=(\d+) valid=(\d+) feasible_min=(\d+) feasible_mean=([\d.]+) wall_mean_ms=([\d.]+) wall_max_ms=([\d.]+)', logs)]
        truth = list(csv.DictReader((directory/'trajectory.csv').open()))
        case = json.loads((directory/'case.json').read_text())
        x,y,yaw = case['goal']
        last = truth[-1]
        error = math.hypot(float(last['x'])-x, float(last['y'])-y)
        heading = abs(math.remainder(float(last['yaw'])-yaw, 2*math.pi))
        assert abs(error-d['goal_position_error_m']) < 1e-9
        dwell = 0.
        trailing = []
        for t in reversed(truth):
            if float(t['speed']) > .02 or math.hypot(float(t['x'])-x,float(t['y'])-y) > .02 or abs(math.remainder(float(t['yaw'])-yaw,2*math.pi)) > .035:
                break
            trailing.append(float(t['t']))
            dwell = float(last['t'])-float(t['t'])
        assert abs(dwell-d['stopped_dwell_s']) < 1e-9
        gaps = [a-b for a,b in zip(trailing,trailing[1:])]
        d['tail_truth_max_gap_s'] = max(gaps, default=0.)
        independent = d['status'] == 4 and not d['collision_events'] and error <= .02 and heading <= .035 and float(last['speed']) <= .02 and dwell >= .25
        assert independent == d['arrival_acceptance_pass']
        d['arrival_tail_audit_pass'] = independent and d['tail_truth_max_gap_s'] <= .10
        result['all_trials'].append(row | {'metrics':d})
        groups[row['case'],row['profile']].append(d)
    for (case,profile), ds in groups.items():
        counters = [s for d in ds for s in d['gpu_control_summaries']]
        resources = [d['node_resources']['controller_server'] for d in ds]
        sampling = [s for d in ds for s in d['sampling_stats']]
        result['navigation'].append(dict(case=case, profile=profile, trials=len(ds),
            success=sum(d['success'] for d in ds), arrival_tail_pass=sum(d['arrival_tail_audit_pass'] for d in ds),
            clean_shutdown=sum(d['audited_complete_flow'] for d in ds), collision_events=sum(d['collision_events'] for d in ds),
            wall_mean_s=statistics.mean(d['wall_elapsed_s'] for d in ds),
            controller_cpu_percent=statistics.mean(d['cpu_percent_one_core'] for d in resources),
            controller_peak_rss_mib=max(d['peak_rss_mib'] for d in resources),
            command_period_p95_ms=statistics.mean(d['command_period_p95_ms'] for d in ds),
            cuda_samples=sum(s['cuda_samples'] for s in counters), best_sequences=sum(s['best_sequence'] for s in counters),
            gpu_failures=sum(s['gpu_failures'] for s in counters), deadline_misses=sum(s['deadline_misses'] for s in counters),
            missed_loop_warnings=sum(d['controller_deadline_warnings'] for d in ds),
            sampler_wall_max_ms=max((s['wall_max_ms'] for s in sampling), default=None),
            min_clearance_m=min(d['min_clearance_m'] for d in ds), max_goal_error_m=max(d['goal_position_error_m'] for d in ds)))
    assert len(result['all_trials']) == 24 and len(result['api']) == 3 and len(result['components']) == 16
    (root/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    for key in ('components','api','navigation'):
        with (root/(key+'-summary.csv')).open('w') as f:
            writer = csv.DictWriter(f, fieldnames=list(result[key][0]))
            writer.writeheader();writer.writerows(result[key])
    return result


def plots(root, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    lookup = {(r['profile'],r['workload'],r['backend']):r for r in data['components']}
    fig,axes = plt.subplots(1,2,figsize=(12,4.4))
    workloads = ['open','corridor','detour','blocked']
    x = np.arange(len(workloads))
    for j,(profile,label,color) in enumerate([('baseline','Round-one GPU','#32698b'),('explore','Exploration GPU','#df7837')]):
        vals = [lookup[profile,w,'cuda']['wall_p99_ms'] for w in workloads]
        axes[0].bar(x+(j-.5)*.34,vals,.34,label=label,color=color)
    axes[0].axhline(40,color='#9c3b3b',linestyle='--',label='40 ms soft budget')
    axes[0].set_xticks(x,workloads);axes[0].set_ylabel('Complete sampler P99 (ms)');axes[0].legend(fontsize=8)
    ap = {r['arm']:r for r in data['api']}
    names = ['original','baseline','explore']
    axes[1].bar(names,[ap[n]['wall_mean_ms'] for n in names],color=['#89939c','#32698b','#df7837'])
    axes[1].set_ylabel('Actual controller API mean (ms)')
    for i,n in enumerate(names):axes[1].text(i,ap[n]['wall_mean_ms'],f"{ap[n]['wall_mean_ms']:.2f}",ha='center',va='bottom')
    for ax in axes:ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
    fig.suptitle('RTX 5070: spend additional compute on exploration; velocity limits unchanged')
    fig.tight_layout();fig.savefig(root/'compute-budget.png',dpi=170);fig.savefig(root/'compute-budget.svg');plt.close(fig)
    fig,axes = plt.subplots(1,2,figsize=(12,4.6),sharex=True,sharey=True)
    for ax,seed in zip(axes,(21,22)):
        for profile,label,color in [('baseline','Round-one GPU','#32698b'),('explore','Exploration GPU','#df7837')]:
            row = next(r for r in data['all_trials'] if r['case']=='offplan_obstacle' and r['seed']==seed and r['profile']==profile)
            path = root/'final/navigation'/Path(row['result']).parent/'trajectory.csv'
            points = list(csv.DictReader(path.open()))
            ax.plot([float(r['x']) for r in points],[float(r['y']) for r in points],label=label,color=color)
        ax.add_patch(plt.Rectangle((-.56,-.06),.12,.12,color='#444444'))
        ax.scatter([-1.7,1.7],[0,0],marker='x',color='#22502f');ax.axhline(0,color='#999',linestyle=':',linewidth=.8)
        ax.set_title(f'Fixed-path obstacle, seed {seed}');ax.set_aspect('equal');ax.set_xlim(-2.,2.);ax.set_ylim(-.8,.8)
        ax.set_xlabel('Truth x (m)');ax.set_ylabel('Truth y (m)');ax.legend(fontsize=8);ax.grid(alpha=.18)
    fig.suptitle('Same declared straight FollowPath; global replanning intentionally absent')
    fig.tight_layout();fig.savefig(root/'obstacle-trajectories.png',dpi=170);fig.savefig(root/'obstacle-trajectories.svg');plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--plots',action='store_true')
    args = parser.parse_args()
    data = summarize(args.root)
    if args.plots:
        plots(args.root,data)
    print(json.dumps({'navigation_trials':len(data['all_trials']), 'paired_inputs':data['input_checks_passed']}))
