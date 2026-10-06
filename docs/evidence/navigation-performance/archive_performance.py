from pathlib import Path
import hashlib,json,platform,shutil,subprocess,tarfile
sim=Path('/home/th1rt3en/dev/forge/duojin02-sim');main=sim.with_name('duojin02')
old=sim/'.cache/navigation-upgrade';new=sim/'.cache/navigation-performance';out=main/'docs/evidence/navigation-performance';out.mkdir(parents=True,exist_ok=True)
primary_roots=[old/'performance-frozen-v4',new/'frozen-v4']
primary=[json.loads(p.read_text()) for root in primary_roots for p in root.glob('*/result.json')]
assert len(primary)==72,len(primary)
assert len({(r['case'],r['seed']) for r in primary})==72
for r in primary:
 assert r.get('verified_build') and r.get('rmw_implementation')=='rmw_cyclonedds_cpp',r
 assert r['configuration_sha256']==hashlib.sha256((main/'src/duojin01_bringup/config/nav2.yaml').read_bytes()).hexdigest()
 for file,expected in r['algorithm_sha256'].items():assert hashlib.sha256((main/file).read_bytes()).hexdigest()==expected
formal=primary_roots+[new/name for name in ['paired-previous-v3','paired-optimized-v4','checks-v4','cpu-limited-v4']]
expected_counts={'paired-previous-v3':10,'paired-optimized-v4':10,'checks-v4':8,'cpu-limited-v4':5}
for name,count in expected_counts.items():assert len(list((new/name).glob('*/result.json')))==count,name
roles={str(primary_roots[0]):'primary_frozen',str(primary_roots[1]):'primary_frozen',str(new/'paired-previous-v3'):'previous_upgrade_common_startup',str(new/'paired-optimized-v4'):'optimized_common_startup',str(new/'checks-v4'):'safety_boundary',str(new/'cpu-limited-v4'):'resource_quota_stress'}
roles.update({str(new/'no-spin-shadow'):'recovery_shadow_not_adopted',str(new/'diagnostic-repeat-v4'):'repeat_diagnostic',str(new/'diagnostic-v4'):'repeat_diagnostic'})
roots=[p for p in old.glob('performance-*') if p.is_dir() and p.name!='report']
roots+=[p for p in new.iterdir() if p.is_dir() and p.name not in ['evaluators','previous-hardware','comparison-previous-hardware','report']]
index=[]
for root in roots:
 records=[]
 for p in sorted(root.glob('*/result.json')):
  r=json.loads(p.read_text());records.append(r);index.append(dict(artifact=str(p.relative_to(sim/'.cache')),role=roles.get(str(root),'exploratory_or_superseded'),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),result=r))
 if records:print(root.name,len(records),sum(r.get('arrival_acceptance_pass',False) for r in records))
(out/'experiment-index.json').write_text(json.dumps(index,indent=2))
for label,selection in [('frozen-trials',formal),('exploratory-trials',[p for p in roots if p not in formal])]:
 with tarfile.open(out/(label+'.tar.gz'),'w:gz') as t:
  for root in selection:
   for p in sorted(root.rglob('*')):
    if p.is_file() and '__pycache__' not in p.parts:t.add(p,arcname=str(p.relative_to(sim/'.cache')),recursive=False)
with tarfile.open(out/'source-and-harness.tar.gz','w:gz') as t:
 for root in [main/'src/duojin01_navigation',main/'src/duojin01_bringup',sim/'simulation/src',main/'tests',sim/'tests']:
  for p in sorted(root.rglob('*')):
   if p.is_file() and '__pycache__' not in p.parts:t.add(p,arcname=('hardware/' if main in p.parents else 'sim/')+str(p.relative_to(main if main in p.parents else sim)),recursive=False)
 for p in sorted((sim/'simulation').iterdir()):
  if p.is_file() and (p.suffix in {'.py','.sh'} or p.name in {'Dockerfile','README.md'}):t.add(p,arcname='sim/simulation/'+p.name,recursive=False)
 for name in ['Dockerfile','source.sh','README_DEVELOPMENT.md','scripts/profile_navigation.py','tests/integration/prior_index_probe.py','tests/test_hardware_boundary.py']:
  t.add(main/name,arcname='hardware/'+name,recursive=False)
 for root in [new/'evaluators',new/'comparison-previous-hardware']:
  for p in sorted(root.rglob('*')):
   if p.is_file() and '__pycache__' not in p.parts:t.add(p,arcname=str(p.relative_to(new)),recursive=False)
 for p in new.glob('candidate-*-source.tar.gz'):t.add(p,arcname=p.name,recursive=False)
for p in (new/'report').iterdir():
 if p.is_file():shutil.copyfile(p,out/p.name)
for p in new.glob('*.log'):shutil.copyfile(p,out/p.name)
for p in old.glob('performance-*.log'):shutil.copyfile(p,out/p.name)
for name in ['optimized-binaries.tar.gz','previous-binaries.tar.gz','runtime-containers.json','no-spin-recovery.xml','no-spin-nav.yaml','debug_command_chain.py','debug-chain16.json','limited-container-host-config.json','preserved-cad-model-hashes.json','main-sim-sync.json','gui-health.json','gui-launch.log','gui_health_probe.py']:
 if (new/name).exists():shutil.copyfile(new/name,out/name)
shutil.copyfile(Path(__file__),out/'archive_performance.py')
metadata=dict(date='2026-10-07',previous_commit='99b106606190d129610fdf7fc1e4593f655a9354',algorithm_commit=subprocess.run(['git','rev-parse','HEAD'],cwd=main,capture_output=True,text=True,check=True).stdout.strip(),not_orin_native=True,architecture=platform.machine(),cpu=subprocess.run(['lscpu'],capture_output=True,text=True,check=True).stdout,gpu=subprocess.run(['nvidia-smi','--query-gpu=name,memory.total,driver_version','--format=csv,noheader'],capture_output=True,text=True,check=True).stdout,limitations='Kinematic chassis plus wheels; lidar 540 beams/10Hz with assumed 5mm Gaussian noise. Velocity-only EKF with biased/noisy synthetic measurements; truth only evaluates full pose. Paired RNG perturbation seeds match, Gazebo laser RNG is not fixed. Small room trials are not universal reliability or Orin native evidence. Some early exploratory source versions lack full snapshots; final candidate sources and compiled provenance are archived.')
(out/'metadata.json').write_text(json.dumps(metadata,indent=2))
sha={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name!='artifact-sha256.json'}
(out/'artifact-sha256.json').write_text(json.dumps(sha,indent=2));print('Archived',len(index),'records; frozen primary',len(primary))
