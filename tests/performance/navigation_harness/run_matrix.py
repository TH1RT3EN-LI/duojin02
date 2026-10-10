#!/usr/bin/env python3
"""Host wrapper for sequential fresh-container-graph navigation comparisons."""
import argparse,json,subprocess,time
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--container',required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--container-output',default='/evidence/navigation')
p.add_argument('--cases',nargs='+',default=['open','corridor_0.50','s_bend','dynamic_crossing','offplan_followpath'])
p.add_argument('--seeds',nargs='+',type=int,default=[21,22]);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=True);ledger=[]
for case in args.cases:
 for i,seed in enumerate(args.seeds):
  arms=['original','cpu','gpu'] if i%2==0 else ['gpu','cpu','original']
  for arm in arms:
   stem=f'{case}-{seed}-{arm}';directory=args.output/stem;directory.mkdir(exist_ok=True)
   container_directory=args.container_output+'/'+stem
   command=['docker','exec','-w','/candidate',args.container,'bash','/candidate/tests/performance/navigation_harness/run_trial.sh',arm,case,str(seed),container_directory]
   started=time.time()
   with (directory/'console.log').open('w') as log:process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
   record=dict(arm=arm,case=case,seed=seed,command=command,started_unix=started,returncode=process.returncode,result=str(Path(stem)/'result.json'))
   ledger.append(record);(args.output/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
   if process.returncode:raise RuntimeError(f'Observer failed, retained {directory}')
   d=json.loads((directory/'result.json').read_text())
   print(json.dumps({k:d.get(k) for k in ('success','arrival_acceptance_pass','complete_flow','wall_elapsed_s','error','gpu_control_summaries')}|{'arm':arm,'case':case,'seed':seed}),flush=True)
