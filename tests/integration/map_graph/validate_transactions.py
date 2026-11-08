#!/usr/bin/env python3
"""Independent source-base robust objective/gradient and transaction audit."""
import pathlib,json,hashlib,math,datetime,argparse
import numpy as np
wrap=lambda a:(a+np.pi)%(2*np.pi)-np.pi;h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def eval_state(nodes,edges,anchor,loss,scale):
 if any(not np.isfinite(p).all() for p in nodes.values()):return None
 cost=0.;g={n:np.zeros(3) for n in nodes}
 for e in edges.values():
  i,j=e['source_id'],e['target_id'];p,q=nodes[i],nodes[j];c,s=np.cos(p[2]),np.sin(p[2]);R=np.array([[c,-s],[s,c]]);l=R.T@(q[:2]-p[:2]);r=np.r_[l,q[2]-p[2]]-e['measurement_base_i'];r[2]=wrap(r[2]);W=np.linalg.inv(np.array(e['covariance_base_i_row_major']).reshape(3,3));v=float(r@W@r);rho,weight=v,1.
  if loss=='HuberLoss' and v>scale*scale:rho=2*scale*np.sqrt(v)-scale*scale;weight=scale/np.sqrt(v)
  elif loss=='CauchyLoss':rho=scale*scale*np.log1p(v/(scale*scale));weight=1/(1+v/(scale*scale))
  cost+=rho/2;w=W@r*weight;t=R@w[:2];g[i][:2]-=t;g[j][:2]+=t;g[i][2]+=l[1]*w[0]-l[0]*w[1]-w[2];g[j][2]+=w[2]
 if anchor is not None:g[anchor]*=0
 return {'cost':cost,'gradient_inf':max([0.]+[float(np.max(np.abs(v))) for v in g.values()])}
def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('directory',type=pathlib.Path,help='Directory containing actual graph_diagnostics/transaction_probe JSONL files')
 parser.add_argument('--output',type=pathlib.Path)
 parser.add_argument('--binary',type=pathlib.Path,action='append',default=[])
 args=parser.parse_args();O=args.directory
 out={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'passed':True,'audited_files':[],'max_cost_abs_error':0.,'max_gradient_abs_error':0.,'accepted':0,'rolled_back':0,'nonstationary':0}
 for p in sorted(O.glob('*.jsonl')):
  nodes={};edges={};loss=None;pending=None;solves=0
  for line in p.read_text().splitlines():
   e=json.loads(line);k=e['event']
   if k=='session':loss=e['loss'];scale=e['loss_scale']
   elif k in ['node_added','node_modified']:nodes[e['node_id']]=np.array(e['pose'],dtype=float)
   elif k=='node_removed':nodes.pop(e['node_id'])
   elif k=='edge_added':edges[e['edge_id']]=e
   elif k=='edge_removed':edges.pop(e['edge_id'])
   elif k=='reset':nodes.clear();edges.clear()
   elif k in ['optimization_begin','optimization_end']:
    for r in e.get('nodes',e.get('changed_nodes',[])):nodes[r[0]]=np.array(r[1:],float)
    v=eval_state(nodes,edges,e['anchor_id'],loss,scale)
    if k=='optimization_begin':pending=(e,{n:x.copy() for n,x in nodes.items()},v)
    else:
     assert pending and pending[0]['optimization_id']==e['optimization_id'];solves+=1;be,bn,bv=pending;pending=None
     assert e['final_cost']==e['objective_committed']['cost']
     if v is not None:
      assert e['objective_committed']['valid'];err=abs(v['cost']-e['final_cost']);out['max_cost_abs_error']=max(out['max_cost_abs_error'],err);assert err<1e-8
      ge=abs(v['gradient_inf']-e['objective_committed']['free_gradient_infinity_norm']);out['max_gradient_abs_error']=max(out['max_gradient_abs_error'],ge);assert ge<1e-7
     else:assert not e['objective_committed']['valid'] and e['final_cost'] is None
     if bv is not None:assert abs(bv['cost']-e['initial_cost'])<1e-8
     if e['accepted']:
      out['accepted']+=1;assert v is not None and bv is not None and e['usable'] and e['anchor_max_component_change']==0;assert v['cost']<=bv['cost']+e['allowed_cost_increase']+1e-8
      if not e['free_gradient_tolerance_met']:out['nonstationary']+=1;assert e['acceptance_decision']=='accepted_not_stationary'
     else:
      out['rolled_back']+=1;assert all(np.array_equal(nodes[n],x,equal_nan=True) for n,x in bn.items()),p.name
   else:raise AssertionError(k)
  assert pending is None;out['audited_files'].append({'path':str(p.relative_to(O)),'sha256':h(p),'actual_solves':solves})
 out['test_binary_hashes']={str(p):h(p) for p in args.binary}
 require_files=len(out['audited_files'])
 assert require_files>0,'No actual event files found'
 if args.output:args.output.write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps(out,indent=2))
if __name__=='__main__':main()
