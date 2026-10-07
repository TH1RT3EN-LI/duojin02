from pathlib import Path
import csv
import json
import math
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = Path('/home/th1rt3en/dev/forge/duojin02-sim/.cache/navigation-narrow')
roots = sorted((base / 'frozen-v7').glob('online_corridor_0.50-upgraded-*/result.json'))
assert len(roots) == 6
fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, sharey=True)
for p, ax in zip(roots, axes.flat):
    r = json.loads(p.read_text())
    case = json.loads(p.with_name('case.json').read_text())
    with p.with_name('trajectory.csv').open() as f: rows = list(csv.DictReader(f))
    for obstacle in case['obstacle_reference']:
        ax.add_patch(plt.Rectangle((obstacle['x']-obstacle['sx']/2,obstacle['y']-obstacle['sy']/2),obstacle['sx'],obstacle['sy'],color='#777777'))
    ax.plot([float(q['x']) for q in rows],[float(q['y']) for q in rows],color='#007f84',lw=1.8,label='Truth')
    ax.plot([float(q['estimated_x']) for q in rows],[float(q['estimated_y']) for q in rows],color='#ce582a',ls='--',lw=.9,label='Estimated')
    middle = min(rows,key=lambda q:abs(float(q['x'])))
    parameters = yaml.safe_load(p.with_name('nav2-source.yaml').read_text())
    footprint = yaml.safe_load(parameters['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    yaw = float(middle['yaw']);c,s=math.cos(yaw),math.sin(yaw)
    corners=[(float(middle['x'])+c*x-s*y,float(middle['y'])+s*x+c*y) for x,y in footprint]
    ax.add_patch(plt.Polygon(corners,facecolor='#007f84',alpha=.25,edgecolor='#004a50'))
    ax.set_title(f"Seed {r['seed']}: clearance {r['min_clearance_m']*1000:.1f} mm; pass={r.get('arrival_acceptance_pass')}")
    ax.set_xlim(-.9,.9);ax.set_ylim(-.42,.42);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
axes[0,0].legend(loc='lower right',fontsize=8)
fig.suptitle('Unknown 0.50 m doorway: actual chassis/wheels footprint at the aperture')
fig.tight_layout();(base/'report').mkdir(exist_ok=True);fig.savefig(base/'report/online-door.png',dpi=170)
