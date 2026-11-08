#!/usr/bin/env python3
"""Two actual rigid reference subsets per held-out query, with explicit overlap."""
import argparse,dataclasses,json
from pathlib import Path
from duojin02_slam_tools.local_submap import RawObservation,RigidSubmapBuilder,ScanSubmapObservation
from duojin02_slam_tools.continuous_geometry import refine_shadow
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
result={'scope':'4 independent frozen actual-query components, two rigid submaps per query; no global trajectory/covariance/physical accuracy claim','components':[],'information_semantics':'conditional shape only; shared-source noise uncalibrated; must not claim independent calibrated factors'}
for sample in map(json.loads,a.input.read_text().splitlines()):
 sid=f"{sample['arm']}:{sample['attempt_index']}:{sample['label']}";query=RawObservation.from_dict(sample['query']);refs=sample['references'];mid=len(refs)//2
 subsets=(refs[:mid+1],refs[mid-1:]);component={'sample_id':sid,'query_id':query.source_id,'query_initial_world_pose':sample['query']['base_pose'],'submaps':[],'constraints':[]}
 maps=[]
 for index,subset in enumerate(subsets):
  builder=RigidSubmapBuilder(sid+f':S{index}',subset[0]['base_pose'])
  for item in subset:
   observation=RawObservation.from_dict(item);builder.insert_once(observation,item['base_pose'],builder.view().query_receipt(observation))
  view=builder.freeze();maps.append(view);estimate=refine_shadow(query,view,sample['query']['base_pose']);factor=ScanSubmapObservation.from_shadow(query,view,estimate)
  component['submaps'].append({'submap_id':view.submap_id,'world_pose':view.world_pose,'pose_version':view.pose_version,'geometry_version':view.geometry_version,'source_ids':view.source_ids,'internal_scans':[{'source_id':s.observation.source_id,'source_geometry_identity':s.observation.identity,'local_pose':s.local_pose} for s in view.scans]})
  component['constraints'].append({'observation':dataclasses.asdict(factor),'status':estimate['status'],'observable':estimate['observable'],'visibility':estimate['visibility'],'support_retention':estimate['support_retention'],'elapsed_seconds':estimate['elapsed_seconds'],'experimental_only':True})
 component['shared_reference_sources']=maps[0].overlap(maps[1]);assert len(component['shared_reference_sources'])==2
 result['components'].append(component)
 (a.output/'constraints.json').write_text(json.dumps(result,indent=2)+'\n')
