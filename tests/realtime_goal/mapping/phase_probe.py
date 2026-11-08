#!/usr/bin/env python3
"""Predeclared signed millimetre phase/direction controls; no grid comparison."""
import json
from pathlib import Path
import sys
import numpy as np
from test_shadow import obs,room
from duojin02_slam_tools.local_submap import RigidSubmap,FrozenScan
from duojin02_slam_tools.continuous_geometry import refine_shadow
m=RigidSubmap('phase',(FrozenScan(obs('ref',room()),(0.,0.,0.)),),(0.,0.,0.))
out=Path(sys.argv[1]).open('w')
for phase in (-.004,-.002,-.001,.001,.002,.004):
 for direction in (0.,np.pi/2,np.pi,3*np.pi/2):
  initial=(phase*np.cos(direction),phase*np.sin(direction),np.sign(phase)*.001)
  r=refine_shadow(obs('query',room()),m,initial,query_stride=8)
  error=float(np.linalg.norm(r['local_pose']))
  assert r['observable'] and error<1e-5
  assert r['visibility']=='unvalidated' and not r['usable_for_precision']
  out.write(json.dumps({'phase_m':phase,'direction_rad':direction,'initial_pose':initial,'returned_pose':r['local_pose'],'error_norm_m_rad':error,'support_retention':r['support_retention'],'support_changed':r['support_changed'],'status':r['status'],'passed':True,'scope':'explicit synthetic local mode convergence only; no comparison against native grid or physical accuracy'})+'\n')
out.close()
