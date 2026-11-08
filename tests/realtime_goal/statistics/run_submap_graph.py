#!/usr/bin/env python3
"""Run the opt-in graph on frozen real scan geometry and explicit local candidates.

The old rejected/positive labels and local-mode-only status remain stress-input
provenance. This runner does not certify association or fit a noise model.
"""
import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/duojin02_slam_tools'))
from duojin02_slam_tools.local_submap import RawObservation, FrozenScan, RigidSubmap, ScanSubmapObservation
from duojin02_slam_tools.shadow_submap_graph import ShadowSubmapGraph


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-fixture',type=Path,required=True)
    parser.add_argument('--constraints',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    raw={f"{s['arm']}:{s['attempt_index']}:{s['label']}":s for s in
         map(json.loads,args.raw_fixture.read_text().splitlines())}
    constraints=json.loads(args.constraints.read_text())
    # Fixed before solving. A deterministic test shape defines the experimental
    # objective only; it is not inverse conditional_shape or physical covariance.
    protocol=dict(experiment_information_shape=np.diag([1.,1.,1.]).tolist(),
        parameter_scales=[.1,.1,.1], shape_semantics='explicit_identity_shape_in_declared_residual_units',
        physical_covariance_calibrated=False, production_pf_connected=False,
        mode_association_validated=False, input_scope=constraints['scope'],
        raw_sha256=hashlib.sha256(args.raw_fixture.read_bytes()).hexdigest(),
        constraints_sha256=hashlib.sha256(args.constraints.read_bytes()).hexdigest(),
        all_inputs_retained_including_old_negative_labels=True)
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2))
    anchor=constraints['components'][0]['submaps'][0]['submap_id']
    graph=ShadowSubmapGraph('real-four-frozen-component-stress',anchor,protocol['parameter_scales'])
    reconstruction=[]
    for component in constraints['components']:
        original=raw[component['sample_id']]
        references={item['source_id']:RawObservation.from_dict(item) for item in original['references']}
        for item in component['submaps']:
            frozen=tuple(FrozenScan(references[s['source_id']],s['local_pose']) for s in item['internal_scans'])
            submap=RigidSubmap(item['submap_id'],frozen,tuple(item['world_pose']),item['pose_version'])
            if submap.geometry_version!=item['geometry_version'] or tuple(item['source_ids'])!=submap.source_ids:
                raise RuntimeError('real frozen geometry/source hash reconstruction mismatch')
            graph.add_submap(submap)
            reconstruction.append(dict(submap_id=submap.submap_id,geometry_version=submap.geometry_version,
                full_source_ids=submap.source_ids,point_count=sum(len(s.observation.points) for s in frozen)))
        for index,entry in enumerate(component['constraints']):
            data=entry['observation'];observation=ScanSubmapObservation(**data)
            graph.add_observation(component['sample_id']+':'+str(index),component['query_id'],observation,
                component['query_initial_world_pose'],protocol['experiment_information_shape'],0)
    (args.output/'geometry-reconstruction.json').write_text(json.dumps(reconstruction,indent=2))
    before=graph.snapshot();(args.output/'snapshot-before.json').write_text(json.dumps(before,indent=2))
    runs=[]
    # Force actual bounded-yield/resume, then Finalize from the same memory.
    for _ in range(2):runs.append(graph.run_budget(max_iterations=1,budget_seconds=.01))
    runs.append(graph.finalize(max_iterations=30,budget_seconds=1.))
    after=graph.snapshot()
    (args.output/'transactions.json').write_text(json.dumps(runs,indent=2))
    (args.output/'snapshot-after.json').write_text(json.dumps(after,indent=2))
    if before['accepted_ledger']!=after['accepted_ledger']:
        raise RuntimeError('solver mutated accepted source/factor ledger')
    if any(before['submaps'][k]['geometry_version']!=v['geometry_version'] for k,v in after['submaps'].items()):
        raise RuntimeError('backend changed frozen internal geometry')
    if before['submaps'][anchor]!=after['submaps'][anchor]:
        raise RuntimeError('fixed anchor moved')
    summary=dict(passed=True,real_samples=len(constraints['components']),
        real_scan_submap_constraints=after['accepted_factor_count'],
        rigid_submaps=len(after['submaps']),explicit_query_states=len(after['queries']),
        objective_before=runs[0].get('objective_before'),objective_after=runs[-1].get('objective_after'),
        complete_accepted_ledger_retained=True,all_factors_included=not after['pending_factor_ids'],
        anchor_fixed=True,geometry_versions_unchanged=True,
        connected_components=len(after['connected_components']),
        components_without_fixed_anchor=after['disconnected_component_count'],
        reported_nullity=after['last_solver']['nullity'],rank=after['last_solver']['rank'],
        information_shape='declared_experiment_only_not_physical_covariance',
        diagonal_regularization=after['last_solver']['diagonal_regularization'],
        marginalization_implemented=False, production_pf_connected=False,
        association_or_trajectory_accuracy_claim=False)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
