#!/usr/bin/env python3
"""Seal four actual warm-seed objectives, then exactly eight bounded native controls.

No truth data is read. Profiles vary only four declared Ceres stopping fields.
"""
from pathlib import Path
import argparse,datetime,hashlib,json,os,signal,subprocess,time
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--source',required=True);p.add_argument('--output',required=True);p.add_argument('inputs',nargs=4);a=p.parse_args()
 out=Path(a.output);out.mkdir(parents=True,exist_ok=False);source=Path(a.source);repo=Path(__file__).resolve().parents[3]
 files=[source/'solvers/ceres_solver.cpp',source/'solvers/ceres_solver.hpp',source/'solvers/ceres_utils.h',source/'lib/karto_sdk/src/Karto.cpp',source/'lib/karto_sdk/src/Mapper.cpp',source/'lib/karto_sdk/include/karto_sdk/Mapper.h',source/'lib/karto_sdk/include/karto_sdk/GraphUpdate.h',source/'lib/karto_sdk/include/karto_sdk/MeasurementModel.h',Path(__file__).resolve(),repo/'tests/integration/karto_graph_update/frozen_backend.cpp',repo/'tests/integration/karto_graph_update/frozen-backend-CMakeLists.txt',Path(a.binary)]
 manifest={str(path):sha(path) for path in files};inputs=[]
 for name in a.inputs:
  path=Path(name);j=json.loads(path.read_text());inputs.append({'path':str(path),'sha256':sha(path),'arm':j['arm'],'snapshot':j['snapshot'],'nodes':len(j['nodes']),'factors':len(j['factors']),'archive_base':j['archive_base']})
 require={(i['arm'],i['snapshot']) for i in inputs};assert len(require)==4,'inputs must be distinct declared objectives'
 protocol={'schema':'native_actual_warmseed_controls.v1','sealed_utc':now(),'design':'Exactly four input objectives, each independently initialized from the actual historical backend warm seed and solved once with normal and once with strict. No continuation, Mapper archive pose initialization, GT access or selection by accuracy. Original LinkInfo payload/mean/cov/owner binding is checked natively.','profiles':{'normal':{'function_tolerance':1e-3,'gradient_tolerance':1e-6,'parameter_tolerance':1e-3,'max_num_iterations':50},'strict':{'function_tolerance':1e-13,'gradient_tolerance':1e-10,'parameter_tolerance':1e-13,'max_num_iterations':500}},'other_options':'Resolved actual production ROS solver parameters are identical across paired controls; see native resolved_solve_options.','source_binary_sha256':manifest,'inputs':inputs,'per_run_timeout_s':180,'truth_use':'Only after all outputs are frozen by a separate evaluator; no GT is available to this runner.'}
 protocolpath=out/'protocol.json';protocolpath.write_text(json.dumps(protocol,indent=2)+'\n');records=[]
 for item in inputs:
  for profile in ['normal','strict']:
   assert sha(item['path'])==item['sha256'];target=out/f"{item['arm']}-{item['snapshot']}-{profile}.json";assert not target.exists();command=[str(a.binary),item['path'],profile,str(target)];start=time.monotonic();proc=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
   record={'arm':item['arm'],'snapshot':item['snapshot'],'profile':profile,'command':command,'pid':proc.pid,'started_utc':now(),'output':str(target)};records.append(record);(out/'runs.json').write_text(json.dumps({'protocol_sha256':sha(protocolpath),'complete':False,'runs':records},indent=2)+'\n')
   try:stdout,stderr=proc.communicate(timeout=180)
   except subprocess.TimeoutExpired:
    os.killpg(proc.pid,signal.SIGINT)
    try:stdout,stderr=proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);stdout,stderr=proc.communicate()
    record['timed_out']=True
   record.update(returncode=proc.returncode,stdout=stdout,stderr=stderr,elapsed_wall_s=time.monotonic()-start,finished_utc=now(),output_sha256=sha(target) if target.exists() else None)
   (out/'runs.json').write_text(json.dumps({'protocol_sha256':sha(protocolpath),'complete':False,'runs':records},indent=2)+'\n');print(json.dumps({k:record[k] for k in ['arm','snapshot','profile','returncode','elapsed_wall_s']}),flush=True)
   assert proc.returncode==0 and not record.get('timed_out'),record
   result=json.loads(target.read_text());assert result['passed'] and result['archive_pose_used_as_seed'] is False
 for name,digest in manifest.items():assert sha(name)==digest,'source or binary changed during controls'
 for item in inputs:assert sha(item['path'])==item['sha256']
 (out/'runs.json').write_text(json.dumps({'protocol_sha256':sha(protocolpath),'complete':True,'source_unchanged':True,'finished_utc':now(),'runs':records},indent=2)+'\n')
if __name__=='__main__':main()
