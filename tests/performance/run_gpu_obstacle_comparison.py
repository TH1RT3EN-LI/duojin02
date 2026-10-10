#!/usr/bin/env python3
"""Extra frozen-observer FollowPath quality cases; intentionally no global planner."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
fixture=dict(name='offplan_obstacle',start=[-1.7,0,0],goal=[1.7,0,0],timeout=55,obstacles=[[-.5,0,.12,.12]],controller_component=True)
(args.output/'fixture.json').write_text(json.dumps(fixture,indent=2)+'\n')
script=r'''set -eo pipefail
arm="$1";seed="$2";output="$3"
root=/candidate;config=/candidate/src/duojin01_bringup/config/nav2.yaml
if [ "$arm" = original ];then root=/original;config=/original/src/duojin01_bringup/config/nav2.yaml;fi
if [ "$arm" = gpu ];then config=/candidate/.cache/profiles/gpu.yaml;fi
ln -sfn "$root" /hardware;ln -sfn "/candidate/.cache/full-$arm" /tmp/navbuild
source /opt/ros/humble/setup.bash;source /candidate/.cache/slam-install/setup.bash;source "/candidate/.cache/full-$arm/install/local_setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/duojin02_odometry_plugin
export GZ_PARTITION="duojin_gpu_obstacle_${arm}_${seed}_$$";export QT_QPA_PLATFORM=offscreen
# The manifest is frozen after compilation; never re-stamp edited source
# against old binaries at trial startup.
if [ ! -f /tmp/navbuild/navigation-build.json ];then
  printf '%s\n' 'Missing frozen build manifest; rebuild and run record_build.py immediately after compilation.' >&2
  exit 3
fi
exec python3 /candidate/tests/performance/navigation_harness/nav_benchmark.py --case offplan_obstacle --variant upgraded --seed "$seed" --nav-config "$config" --scenario-file /evidence/navigation-obstacle/fixture.json --output "$output"
'''
(args.output/'invocation.sh').write_text(script);ledger=[]
for seed,arms in [(21,['original','cpu','gpu']),(22,['gpu','cpu','original'])]:
 for arm in arms:
  stem=f'offplan_obstacle-{seed}-{arm}';directory=args.output/stem;directory.mkdir(exist_ok=True)
  cmd=['docker','exec','-w','/candidate',args.container,'bash','-lc',script,'obstacle-quality',arm,str(seed),'/evidence/navigation-obstacle/'+stem]
  started=time.time()
  with (directory/'console.log').open('w') as log:process=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
  ledger.append(dict(arm=arm,case='offplan_obstacle',seed=seed,started_unix=started,returncode=process.returncode,result=stem+'/result.json',command=cmd));(args.output/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
  if process.returncode:raise RuntimeError(f'Observer failed; retained {directory}')
  d=json.loads((directory/'result.json').read_text());print({'arm':arm,'seed':seed}|{k:d.get(k) for k in ('success','status','arrival_acceptance_pass','wall_elapsed_s','collision_events','complete_flow','gpu_control_summaries')},flush=True)
