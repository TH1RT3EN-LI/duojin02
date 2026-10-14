#!/usr/bin/env python3
"""Host wrapper for sequential fresh-container-graph navigation trials."""
import argparse,json,subprocess,time
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--container',required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--container-output',default='/evidence/navigation')
p.add_argument('--cases',nargs='+',default=['open','corridor_0.60','s_bend','dynamic_crossing','offplan_followpath'],
               help='Regular cases default to a 0.60 m chassis aperture; select narrower corridor cases explicitly for stress testing.')
p.add_argument('--seeds',nargs='+',type=int,default=[21,22]);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=True);ledger=[]
for case in args.cases:
 for seed in args.seeds:
   stem=f'{case}-{seed}-chassis';directory=args.output/stem;directory.mkdir(exist_ok=True)
   container_directory=args.container_output+'/'+stem
   command=['docker','exec','-w','/hardware',args.container,'bash',
       '/hardware/tests/performance/navigation_harness/run_trial.sh',case,str(seed),container_directory]
   started=time.time()
   with (directory/'console.log').open('w') as log:process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
   record=dict(case=case,seed=seed,command=command,started_unix=started,returncode=process.returncode,result=str(Path(stem)/'result.json'))
   ledger.append(record);(args.output/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
   if process.returncode:raise RuntimeError(f'Observer failed, retained {directory}')
   d=json.loads((directory/'result.json').read_text())
   print(json.dumps({k:d.get(k) for k in ('success','arrival_acceptance_pass','complete_flow','wall_elapsed_s','error','gpu_control_summaries')}|{'case':case,'seed':seed}),flush=True)
