#!/usr/bin/env python3
"""Read all retained ledgers; aggregate without selecting successful repeats."""
import argparse,json,statistics
from collections import defaultdict
from pathlib import Path


def mean(values):return statistics.mean(values) if values else None

def percentile(values,q):return sorted(values)[int(q*(len(values)-1))] if values else None


def summarize(root):
    groups=defaultdict(list);components=root/'components'
    for row in json.loads((components/'ledger.json').read_text()):
        if row['returncode']:raise ValueError(f'Failed component {row}')
        groups[(row['kind'],row['workload'],row['backend'])].append(json.loads((components/row['result']).read_text())|{'fixture_sha256':row.get('fixture_sha256')})
    aggregated=[]
    for (kind,workload,backend),values in groups.items():
        walls=[x for v in values for x in v['wall_samples_ms']];cpus=[x for v in values for x in v['cpu_samples_ms']]
        hashes={v['fixture_sha256'] for v in values};assert len(hashes)==1
        aggregated.append(dict(kind=kind,workload=workload,backend=backend,process_repeats=len(values),samples=len(walls),wall_mean_ms=mean(walls),wall_p95_ms=percentile(walls,.95),wall_p99_ms=percentile(walls,.99),wall_max_ms=max(walls),cpu_mean_ms=mean(cpus),cold_mean_ms=mean([v.get('cold_wall_ms',v.get('cold_ms')) for v in values]),process_means_ms=[v['wall_mean_ms'] for v in values],fixture_sha256=values[0]['fixture_sha256'],valid_calls=sum(v.get('valid_calls',0) for v in values),collision_free_min=min((v.get('collision_free_min',0) for v in values)),zero_calls=sum(v.get('zero_calls',0) for v in values),native_calls=sum(v.get('native_calls',0) for v in values),cuda_calls=sum(v.get('cuda_calls',v.get('cuda_batches',0)) for v in values)))
    # Paired inputs, hit decisions and deterministic replacement commands.
    for kind in ('geometry','sampling'):
        workloads={k[1] for k in groups if k[0]==kind}
        for workload in workloads:
            cpu=groups.get((kind,workload,'original' if kind=='geometry' else 'cpu'))
            gpu=groups.get((kind,workload,'cuda'))
            if not cpu or not gpu:continue
            assert {d['fixture_sha256'] for d in cpu+gpu}=={cpu[0]['fixture_sha256']}
            if kind=='geometry':assert len({d['checksum'] for d in cpu+gpu})==1
            else:
                assert len({(d['valid_calls'],d['collision_free_min']) for d in cpu+gpu})==1
                assert max(abs(a-b) for d in cpu+gpu for a,b in zip(d['command'],cpu[0]['command']))<1e-8
    rows=[]
    for directory in ('navigation','navigation-obstacle'):
        if not (root/directory/'ledger.json').exists():continue
        for row in json.loads((root/directory/'ledger.json').read_text()):
            data=json.loads((root/directory/row['result']).read_text())
            logs=(root/directory/Path(row['result']).parent/'launch.log').read_text()
            data['child_failure_lines']=[line for line in logs.splitlines() if 'process has died' in line]
            data['audited_complete_flow']=bool(data.get('complete_flow')) and not data['child_failure_lines']
            rows.append(row|{'metrics':data,'directory':directory})
    ng=defaultdict(list)
    for row in rows:ng[(row['case'],row['arm'])].append(row['metrics'])
    navigation=[]
    for (case,arm),values in ng.items():
        controllers=[v.get('node_resources',{}).get('controller_server',{}) for v in values]
        counters=[s for v in values for s in v.get('gpu_control_summaries',[])]
        navigation.append(dict(case=case,arm=arm,trials=len(values),success=sum(bool(v.get('success')) for v in values),strict_arrival=sum(bool(v.get('arrival_acceptance_pass')) for v in values),complete_flow=sum(bool(v.get('audited_complete_flow')) for v in values),complete_flow_raw=sum(bool(v.get('complete_flow')) for v in values),collision_events=sum(v.get('collision_events',0) for v in values),wall_mean_s=mean([v['wall_elapsed_s'] for v in values if 'wall_elapsed_s' in v]),controller_cpu_percent=mean([c['cpu_percent_one_core'] for c in controllers if c]),controller_peak_rss_mib=max((c['peak_rss_mib'] for c in controllers if c),default=None),command_period_p95_ms=mean([v['command_period_p95_ms'] for v in values if v.get('command_period_p95_ms')]),deadline_warnings=sum(v.get('controller_deadline_warnings',0) for v in values),cuda_samples=sum(s['cuda_samples'] for s in counters),native_samples=sum(s['native_samples'] for s in counters),gpu_failures=sum(s['gpu_failures'] for s in counters),deadline_misses=sum(s['deadline_misses'] for s in counters),max_goal_error_m=max((v['goal_position_error_m'] for v in values if v.get('goal_position_error_m') is not None),default=None),max_goal_yaw_error_rad=max((v['goal_yaw_error_rad'] for v in values if v.get('goal_yaw_error_rad') is not None),default=None)))
    summary=dict(component_input_checks_passed=True,components=aggregated,navigation=navigation,trial_count=len(rows),all_trial_rows=rows)
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary


def plots(root,s):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    lookup={(r['kind'],r['workload'],r['backend']):r for r in s['components']}
    fig,axes=plt.subplots(1,3,figsize=(13,4.4));color=['#255e91','#e17f32']
    specifications=[('geometry',['medium-rebuild','dense-rebuild'],'original','Original geometry CPU'),('sampling',['open','corridor','detour'],'cpu','New sampler CPU reference'),('controller',['off-plan-free'],'original','Original Nav2 controller')]
    for ax,(kind,workloads,baseline,label) in zip(axes,specifications):
        x=np.arange(len(workloads));a=[lookup[kind,w,baseline]['wall_mean_ms'] for w in workloads];b=[lookup[kind,w,'cuda']['wall_mean_ms'] for w in workloads]
        ax.bar(x-.18,a,.36,label=label,color=color[0]);ax.bar(x+.18,b,.36,label='CUDA candidate',color=color[1])
        for positions,values in ((x-.18,a),(x+.18,b)):
            for pos,value in zip(positions,values):ax.text(pos,value,f'{value:.2f}',ha='center',va='bottom',fontsize=9)
        ax.set_xticks(x,workloads);ax.set_ylabel('Whole call wall time (ms)');ax.set_ylim(0,max(a+b)*1.22);ax.legend(fontsize=8,loc='upper left');ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
    fig.suptitle('RTX 5070: three separate comparisons (ABBA, all repeats)',fontsize=13);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(root/'component-performance.png',dpi=180);fig.savefig(root/'component-performance.svg');plt.close(fig)
    cases=list(dict.fromkeys(r['case'] for r in s['navigation']));lookup={(r['case'],r['arm']):r for r in s['navigation']}
    fig,axes=plt.subplots(1,2,figsize=(13,4.7));x=np.arange(len(cases))
    for j,(arm,label,c) in enumerate([('original','Preserved current original','#255e91'),('cpu','Candidate CUDA OFF','#7c8791'),('gpu','Candidate auto/auto','#e17f32')]):
        for ax,key,ylabel in ((axes[0],'wall_mean_s','Task wall time (s)'),(axes[1],'controller_cpu_percent','Controller process CPU (% of one core)')):
            values=[lookup[case,arm][key] or 0 for case in cases];bars=ax.bar(x+(j-1)*.24,values,.24,label=label,color=c);ax.set_ylabel(ylabel)
            for bar,case in zip(bars,cases):
                if lookup[case,arm]['success']<lookup[case,arm]['trials']:bar.set_hatch('//')
    for ax in axes:ax.set_xticks(x,[c.replace('_','\n') for c in cases],fontsize=8);ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True);ax.legend(fontsize=8)
    fig.suptitle('Closed loop: hatched bars are failed/cancelled tasks, not faster completion',fontsize=13);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(root/'navigation-performance.png',dpi=180);fig.savefig(root/'navigation-performance.svg');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--plots',action='store_true');a=p.parse_args();s=summarize(a.root)
    if a.plots:plots(a.root,s)
    print(json.dumps({'component_groups':len(s['components']),'navigation_trials':s['trial_count'],'input_checks':s['component_input_checks_passed']}))
