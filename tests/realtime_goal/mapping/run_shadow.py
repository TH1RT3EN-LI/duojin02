#!/usr/bin/env python3
"""Execute frozen real endpoint E3 and E5 shadows; never writes historical input."""
import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from duojin02_slam_tools.local_submap import RawObservation, RigidSubmapBuilder, BudgetedUpdateQueue, AcceptedUpdate, ScanSubmapObservation, transform
from duojin02_slam_tools.continuous_geometry import refine_shadow


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    residual_file=(args.output/'real-shadow-residuals.jsonl').open('w')
    result_file=(args.output/'real-shadow-results.jsonl').open('w')
    summary=[]
    for sample in map(json.loads,args.input.read_text().splitlines()):
        sample_id=f"{sample['arm']}:{sample['attempt_index']}:{sample['label']}"
        references=sample['references'];anchor=tuple(references[0]['base_pose'])
        builder=RigidSubmapBuilder(sample_id,anchor,max_scans=len(references)+1)
        queue=BudgetedUpdateQueue()
        for item in references:queue.accept(AcceptedUpdate(item['source_id'],item))
        statuses=[]
        def consume(update):
            item=update.payload;observation=RawObservation.from_dict(item)
            builder.insert_once(observation,item['base_pose'],builder.view().query_receipt(observation))
        while queue.pending_ids:statuses.append(queue.run_budget(consume,max_updates=3,budget_seconds=.02))
        frozen=builder.view();query=RawObservation.from_dict(sample['query'])
        result=refine_shadow(query,frozen,sample['query']['base_pose'])
        receipt=result.pop('query_receipt')
        factor=ScanSubmapObservation.from_shadow(query,frozen,result)
        # The archived query is an accepted historical keyframe, not a new
        # acceptance decision inferred from this uncalibrated shadow score.
        queue.accept(AcceptedUpdate(query.source_id,sample['query']))
        def insert_query(update):
            builder.insert_once(query,update.payload['base_pose'],receipt)
        statuses.append(queue.run_budget(insert_query,max_updates=1,budget_seconds=.02))
        after_insert=builder.freeze()
        for row in result.pop('residuals'):residual_file.write(json.dumps({'sample_id':sample_id,**row})+'\n')
        row={'sample_id':sample_id,'query_source_id':query.source_id,'source_kind':query.geometry_kind,'input_sha256':hashlib.sha256(args.input.read_bytes()).hexdigest(),'no_GT':True,'shadow_only':True,**result}
        row['shadow_scan_to_submap_observation']=dataclasses.asdict(factor)
        row['query_inserted_after_scoring']={'accepted_input':'archived keyframe membership','before_geometry_version':frozen.geometry_version,'after_geometry_version':after_insert.geometry_version,'source_added':query.source_id in after_insert.source_ids,'retained_measurement_support':factor.validate_support(frozen)}
        row['source_overlap_with_query']=query.source_id in frozen.source_ids
        # A real common rigid pose revision leaves all internal geometry and
        # geometry identity intact; opposite reference deformation requires new
        # geometry and cannot be represented by moving this state alone.
        moved=frozen.moved((anchor[0]+1.,anchor[1]-.7,anchor[2]+.2))
        row['rigid_pose_revision']={'old_pose_version':frozen.pose_version,'new_pose_version':moved.pose_version,'geometry_unchanged':moved.geometry_version==frozen.geometry_version,'internal_poses_unchanged':moved.scans==frozen.scans}
        # Explicit overlapping second map, retaining the shared raw ancestry.
        overlap_builder=RigidSubmapBuilder(sample_id+':overlap',anchor)
        for item in references[-2:]:
            obs=RawObservation.from_dict(item);overlap_builder.insert_once(obs,item['base_pose'],overlap_builder.view().query_receipt(obs))
        row['overlap_sources']=frozen.overlap(overlap_builder.freeze())
        row['budgeted_updates']={'accepted':len(references)+1,'completed':len(queue.completed_ids),'pending':len(queue.pending_ids),'runs':statuses,'global_solver_changed':False,'marginalization_implemented':False}
        row['query_receipt']=dataclasses.asdict(receipt)
        result_file.write(json.dumps(row)+'\n');result_file.flush();summary.append(row)
    residual_file.close();result_file.close()
    (args.output/'summary.json').write_text(json.dumps({'samples':len(summary),'actual_raw_or_prepared_geometry':True,'self_match_count':sum(r['source_overlap_with_query'] for r in summary),'all_accepted_updates_completed':all(r['budgeted_updates']['accepted']==r['budgeted_updates']['completed'] for r in summary),'external_accuracy_evaluated':False,'covariance_calibrated':False,'latency_seconds':[r['elapsed_seconds'] for r in summary]},indent=2)+'\n')


if __name__=='__main__':main()
