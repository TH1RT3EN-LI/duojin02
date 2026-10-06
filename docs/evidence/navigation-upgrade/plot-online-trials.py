from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
root=Path('/evidence/release-final');out=Path('/evidence/report/online-trials.png');fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
for seed,ax in enumerate(axes,1):
 case=json.loads((root/f'online_thin-upgraded-{seed}/case.json').read_text())
 for o in case['obstacle_reference']:ax.add_patch(Rectangle((o['x']-o['sx']/2,o['y']-o['sy']/2),o['sx'],o['sy'],color='.55'))
 for v,color in [('baseline','#1671bd'),('upgraded','#d45b12')]:
  p=root/f'online_thin-{v}-{seed}';r=json.loads((p/'result.json').read_text());d=np.genfromtxt(p/'trajectory.csv',delimiter=',',names=True)
  ax.plot(d['x'],d['y'],color=color,label=v)
  if not r['success']:ax.plot(d['x'][-1],d['y'][-1],'rx',markersize=10,label='safety stall')
  if v=='upgraded':ax.set_title(f"Seed {seed}: {'arrived' if r['success'] else 'stopped, goal failed'}\nP95 {r['localization_p95_m']*1000:.1f} mm, clearance {r['min_clearance_m']*1000:.1f} mm")
 ax.plot(*case['goal'][:2],'k*',markersize=10);ax.set_aspect('equal');ax.set_xlim(-2,2);ax.set_ylim(-.9,.9);ax.grid(alpha=.2);ax.set_xlabel('x (m)')
axes[0].set_ylabel('y (m)');axes[-1].legend();fig.savefig(out,dpi=160)
