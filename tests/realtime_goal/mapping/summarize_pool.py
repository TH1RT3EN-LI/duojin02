#!/usr/bin/env python3
"""Compare all archived attempts without changing their historical gate labels."""
import json, math, sys
from pathlib import Path
out=Path(sys.argv[1])
rows=[json.loads(s) for s in (out/'full-pool-results.jsonl').read_text().splitlines()]
assert len(rows)==433 and len({(r['arm'],r['attempt_index']) for r in rows})==433
policies=[p['policy'] for p in rows[0]['policies']]
assert policies[:2]==['legacy_peak','returned_pose_score']
def quantile(a,p):
 s=sorted(a); return s[min(int((len(s)-1)*p),len(s)-1)]
def posegap(a,b):
 return {'xy_m':math.hypot(a[0]-b[0],a[1]-b[1]),'yaw_rad':abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))}
comparisons={};changes=[]
for policy in policies[1:]:
 cg=fg=cp=fp=0;gaps=[];finegaps=[]
 for row in rows:
  a=row['policies'][0]; b=next(p for p in row['policies'] if p['policy']==policy)
  assert abs(b['coarse_score']-b['coarse_actual_score'])<1e-12
  gap=posegap(a['coarse_pose'],b['coarse_pose']);gaps.append(gap)
  coarse_changed=a['coarse_gate']!=b['coarse_gate'];fine_changed=a.get('fine_gate')!=b.get('fine_gate')
  cg+=coarse_changed;fg+=fine_changed;cp+=any(v>1e-12 for v in gap.values())
  finegap=None
  if a['fine_executed'] and b['fine_executed']:
   assert abs(b['fine_score']-b['fine_actual_score'])<1e-12
   finegap=posegap(a['fine_pose'],b['fine_pose']);finegaps.append(finegap);fp+=any(v>1e-12 for v in finegap.values())
  if coarse_changed or fine_changed or any(v>1e-12 for v in gap.values()) or finegap and any(v>1e-12 for v in finegap.values()):
   changes.append({'arm':row['arm'],'attempt_index':row['attempt_index'],'policy':policy,'coarse_gate_changed':coarse_changed,'fine_gate_changed':fine_changed,'coarse_pose_gap':gap,'fine_pose_gap':finegap})
 comparisons[policy]={'coarse_gate_changes':cg,'fine_gate_changes':fg,'coarse_pose_changes':cp,'fine_pose_changes':fp,'coarse_max_xy_gap_m':max(g['xy_m'] for g in gaps),'coarse_max_yaw_gap_rad':max(g['yaw_rad'] for g in gaps),'fine_compared':len(finegaps),'fine_max_xy_gap_m':max((g['xy_m'] for g in finegaps),default=None),'fine_max_yaw_gap_rad':max((g['yaw_rad'] for g in finegaps),default=None)}
timing={}
for policy in policies:
 a=[next(p for p in r['policies'] if p['policy']==policy)['elapsed_seconds'] for r in rows]
 timing[policy]={'median':quantile(a,.5),'p95':quantile(a,.95),'total':sum(a)}
summary={'samples':len(rows),'per_arm':{arm:sum(r['arm']==arm for r in rows) for arm in ['T0D0','T0D1']},'same_query_same_reference_support':True,'all_score_pose_consistency_passed':all(r['score_pose_consistency_passed'] for r in rows),'comparisons_against_legacy':comparisons,'timing_shared_host_seconds':timing,'changed_comparisons':len(changes),'physical_accuracy_evaluated':False,'scope':'all 433 archived explicit-loop attempts, true native coarse/fine rerun; no natural branch graph replay or physical truth labels; two-thread native work, sequential arms'}
(out/'full-pool-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/'full-pool-changes.json').write_text(json.dumps(changes,indent=2)+'\n')
print(json.dumps(summary,indent=2))
