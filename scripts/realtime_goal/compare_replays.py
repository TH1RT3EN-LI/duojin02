#!/usr/bin/env python3
"""Score completed current trials using the original frozen planar evaluator.

Aligned ATE is a trajectory diagnostic. It is not an arbitrary-goal task result,
and this script never exports deployment parameters or fits time offsets.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evaluator',type=Path,required=True)
    p.add_argument('--truth',type=Path,required=True)
    p.add_argument('--run',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists(): p.error('choose an unused output path')
    spec=importlib.util.spec_from_file_location('frozen_planar_evaluator',a.evaluator)
    native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
    truth,_=native.load_tum(a.truth)
    results={}; pairs={}
    for run in a.run:
        report=json.loads((run/'replay.json').read_text())
        audit=json.loads((run/'motion_selection_audit.json').read_text())
        if report['status']!='completed' or report['node_returncode']!=0 or audit['status']!='passed':
            raise RuntimeError(f'Unclosed node/input run: {run}')
        estimated,_=native.load_tum(run/'trajectory.tum')
        metrics,_,_=native.evaluate(truth,estimated,max_gt_gap=.2,max_est_gap=.5)
        poses=json.loads((run/'poses.json').read_text())
        results[run.parent.name]={'metrics':metrics,'input_scans':report['input_scans'],
            'received_poses':report['received_poses'],'observation_final':report['observation_final'],
            'publish_to_pose_ms':report['scan_publish_to_pose_ms'],
            'source_sha256':{name:sha(run/name) for name in ('replay.json','poses.json','trajectory.tum','motion_selection_audit.json')},
            'scope':report['scope']}
        events=[]
        for v in poses:
            if v['publish_to_receipt_ms'] is not None:
                received=v['receipt_monotonic_ns']
                sent=received-round(v['publish_to_receipt_ms']*1e6)
                events.extend([(sent,1),(received,-1)])
        depth=peak=0
        for _,change in sorted(events,key=lambda e:(e[0],-e[1])):
            depth+=change;peak=max(peak,depth)
        results[run.parent.name]['eligible_output_pending_proxy']={
            'peak':peak,'final':depth,
            'scope':'reconstructed publish-to-receipt intervals for motion-selected outputs only; not internal TF/scan queue or sensor-to-actuator latency'}
        results[run.parent.name]['publisher_lag_ms']=report['publisher_lag_ms']
        pairs[run.parent.name]={v['source_scan_id']:v for v in poses}
    comparisons={}
    for arm in ('D0','D1'):
        base=pairs.get('baseline-'+arm)
        candidate=pairs.get('E1-'+arm)
        if base is None or candidate is None: continue
        common=sorted(set(base)&set(candidate)); xy=[];yaw=[]
        for key in common:
            b,c=base[key]['pose'],candidate[key]['pose']
            xy.append(math.hypot(c[0]-b[0],c[1]-b[1]));yaw.append(abs(native.wrap(c[2]-b[2])))
        comparisons[arm]={'same_source_set':set(base)==set(candidate),'common_poses':len(common),
            'output_xy_difference_m':native.distribution(xy),'output_yaw_difference_rad':native.distribution(yaw),
            'interpretation':'output statistics only; no held-out independent map/physical task benefit established'}
    result={'scope':'current-binary same-sequence historical frozen-map trajectory diagnostics only',
            'parameters_fitted':False,'time_offset_fitted':False,'per_task_alignment':False,
            'alignment':'one SE2 fit per whole recorded trajectory; not task precision',
            'evaluator_sha256':sha(a.evaluator),'truth_sha256':sha(a.truth),'runs':results,'comparisons':comparisons}
    with a.output.open('x') as f: json.dump(result,f,indent=2,allow_nan=False);f.write('\n')


if __name__=='__main__':main()
