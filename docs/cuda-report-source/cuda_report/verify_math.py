"""Independent small numerical checks for the report; not a robot/CUDA benchmark."""
from pathlib import Path
import json
import math
import numpy as np

SEED = 20261015
rng = np.random.default_rng(SEED)
results = {}
J = np.array([[0., -1.], [1., 0.]])

def R(t):
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

def check(name, error, tolerance, count, detail=""):
    assert np.isfinite(error) and error <= tolerance, (name, error, tolerance)
    results[name] = {"passed": True, "max_error_or_violation": float(error),
                     "tolerance": tolerance, "cases": count, "detail": detail}

def sm(s, lam=.3):
    e = np.exp(-(s-s.min())/lam)
    return e/e.sum()

# Entropy first-order condition and perturbation bound.
stationarity, violation = 0., 0.
for _ in range(500):
    s = rng.normal(0., .5, 64)
    w = sm(s)
    grad = s+.3*(np.log(64*w)+1)
    stationarity = max(stationarity, np.ptp(grad))
    eps = .01
    wp = sm(s+rng.uniform(-eps, eps, len(s)))
    violation = max(violation, np.abs(wp-w).sum()-(np.exp(2*eps/.3)-1))
check("entropy_stationarity", stationarity, 1e-12, 500)
check("softmax_l1_bound", max(0.,violation), 1e-12, 500)

# Prefix-scan derivation vs literal Euler loop.
err = 0.
for _ in range(100):
    v = rng.normal(size=(56,3))*.2
    h=.05
    q0=rng.normal(size=3)
    q=[q0.copy()]
    for vel in v:
        old=q[-1]
        q.append(np.r_[old[:2]+h*R(old[2])@vel[:2],old[2]+h*vel[2]])
    q=np.array(q)
    theta=np.r_[q0[2],q0[2]+h*np.cumsum(v[:,2])]
    inc=np.array([h*R(theta[k])@v[k,:2] for k in range(56)])
    p=np.vstack([q0[:2],q0[:2]+np.cumsum(inc,axis=0)])
    err=max(err,np.max(np.abs(q-np.column_stack([p,theta]))))
check("omni_prefix_vs_sequential",err,1e-12,100)

# Exact SE(2) increment vs high resolution midpoint quadrature.
err, vio = 0.,0.
for _ in range(100):
    h=float(rng.uniform(.001,.1)); w=float(rng.uniform(-2,2))
    v=rng.normal(size=2); theta=float(rng.normal())
    a=w*h/2
    exact=h*np.sinc(a/np.pi)*R(theta+a)@v
    ts=(np.arange(10000)+.5)*h/10000
    rot=np.array([R(theta+w*t)@v for t in ts])
    numeric=h*rot.mean(axis=0)
    err=max(err,np.max(np.abs(exact-numeric)))
    euler=h*R(theta)@v
    vio=max(vio,np.linalg.norm(exact-euler)-.5*abs(w)*h*h*np.linalg.norm(v))
check("constant_twist_integral",err,1e-9,100)
check("euler_error_bound",max(0.,vio),1e-12,100)

# Smooth synthetic field for chain-rule Jacobian finite differences.
err=0.; step=1e-6; ell=.3; sigma=.025
for _ in range(200):
    z=rng.normal(size=3); y=rng.normal(size=2)
    def field(x): return .5*x@x+.2*x[0]*x[1]
    def residual(zz): return field(zz[:2]+R(zz[2]/ell)@y)/sigma
    e=z[:2]+R(z[2]/ell)@y
    grad=np.array([e[0]+.2*e[1],e[1]+.2*e[0]])
    jac=grad@np.column_stack([np.eye(2),R(z[2]/ell)@J@y/ell])/sigma
    fd=np.array([(residual(z+np.eye(3)[a]*step)-residual(z-np.eye(3)[a]*step))/(2*step) for a in range(3)])
    err=max(err,np.max(np.abs(jac-fd)))
check("scan_chain_rule_jacobian",err,2e-6,200,
      "Smooth synthetic field; does not assert differentiability at grid boundaries.")

# Pose graph residual Jacobians without wrap branch.
err=0.
for _ in range(200):
    qa=rng.normal(size=3); qb=rng.normal(size=3)
    def e(a,b): return np.r_[R(a[2]).T@(b[:2]-a[:2]),b[2]-a[2]]
    d=e(qa,qb)[:2]
    A=np.zeros((3,3)); B=np.zeros((3,3))
    A[:2,:2]=-R(qa[2]).T; A[:2,2]=-J@d; A[2,2]=-1
    B[:2,:2]=R(qa[2]).T; B[2,2]=1
    for j in range(3):
        v=np.eye(3)[j]*step
        err=max(err,np.max(np.abs((e(qa+v,qb)-e(qa-v,qb))/(2*step)-A[:,j])))
        err=max(err,np.max(np.abs((e(qa,qb+v)-e(qa,qb-v))/(2*step)-B[:,j])))
check("pose_graph_jacobians",err,2e-8,400)

# EDT lower bound to union of occupied squares, including query quantization.
rg=.01; rho=math.sqrt(2)*rg/2; vio=0.
centers=rng.integers(-50,51,size=(100,2))*rg
for _ in range(2000):
    x=rng.uniform(-.6,.6,2); xg=np.round(x/rg)*rg
    dc=np.linalg.norm(centers-xg,axis=1).min()
    lower=max(0.,dc-2*rho)
    actual=np.linalg.norm(np.maximum(np.abs(centers-x)-rg/2,0),axis=1).min()
    vio=max(vio,lower-actual)
check("grid_square_distance_lower_bound",max(0.,vio),1e-12,2000)

# Outer-circle clearance bound for a rectangular footprint and disc obstacles.
vio=0.; half=np.array([.21,.20]); rf=np.linalg.norm(half)
for _ in range(1000):
    p=rng.normal(size=2); theta=float(rng.normal())
    obs=rng.normal(size=(25,2)); radii=rng.uniform(0,.05,25)
    dc=np.min(np.maximum(np.linalg.norm(obs-p,axis=1)-radii,0))
    local=(obs-p)@R(theta)
    actual=np.min(np.maximum(np.linalg.norm(np.maximum(np.abs(local)-half,0),axis=1)-radii,0))
    vio=max(vio,dc-rf-actual)
check("footprint_outer_radius_lower_bound",max(0.,vio),1e-12,1000)

# Direction quantization maximum point displacement.
vio=0.; rf=.3; nt=64; bound=2*rf*np.sin(np.pi/(2*nt))
for _ in range(2000):
    theta=float(rng.uniform(-np.pi,np.pi))
    quant=round(theta/(2*np.pi/nt))*(2*np.pi/nt)
    a=rng.normal(size=2); a=a/np.linalg.norm(a)*rng.uniform(0,rf)
    vio=max(vio,np.linalg.norm(R(theta)@a-R(quant)@a)-bound)
check("orientation_quantization_bound",max(0.,vio),1e-12,2000)

# Direct full transition vs convolution then angular mixture on a finite box.
nx,ny,nt=5,4,3
bel=rng.random((nt,nx,ny)); bel/=bel.sum()
kern=rng.random((nt,3,3)); kern/=kern.sum(axis=(1,2))[:,None,None]
ang=rng.random((nt,nt)); ang/=ang.sum(axis=0)[None,:] # dest, source
direct=np.zeros_like(bel); conv=np.zeros_like(bel)
for ko in range(nt):
 for x in range(nx):
  for y in range(ny):
   for dx in range(-1,2):
    for dy in range(-1,2):
     xx,yy=x+dx,y+dy
     if 0<=xx<nx and 0<=yy<ny:
      mass=bel[ko,x,y]*kern[ko,dx+1,dy+1]
      conv[ko,xx,yy]+=mass
      for kn in range(nt):
       direct[kn,xx,yy]+=mass*ang[kn,ko]
factored=np.einsum("ab,bxy->axy",ang,conv)
check("finite_grid_prediction_factorization",np.max(np.abs(direct-factored)),1e-14,nx*ny*nt,
      "Zero outside finite domain; escaped mass is not silently renormalized.")

# Observation sums equal template correlation and zero-padded FFT correlation.
err=0.
for _ in range(100):
    g=rng.normal(size=(12,11))
    offsets=rng.integers(0,5,(15,2))
    stencil=np.zeros((5,5))
    for a,b in offsets: stencil[a,b]+=1
    shape=(g.shape[0]+4,g.shape[1]+4)
    fft=np.fft.irfftn(np.fft.rfftn(g,s=shape)*np.conj(np.fft.rfftn(stencil,s=shape)),s=shape).real
    for x in range(8):
     for y in range(7):
      direct=sum(g[x+a,y+b] for a,b in offsets)
      err=max(err,abs(direct-fft[x,y]))
check("fft_correlation_vs_beam_sum",err,1e-12,100*8*7,
      "Valid interior domain with zero padding; duplicates retained in template.")

# Discrete CVaR hinge minimization vs sorted fractional upper-tail mean.
err=0.
for _ in range(1000):
    costs=rng.normal(size=16); beta=float(rng.uniform(.1,.98))
    obj=np.array([eta+np.maximum(costs-eta,0).mean()/(1-beta) for eta in costs])
    hinge=obj.min()
    vals=np.sort(costs)[::-1]; mass=(1-beta)*len(vals)
    whole=int(np.floor(mass)); frac=mass-whole
    tail=(vals[:whole].sum()+(vals[whole]*frac if whole<len(vals) else 0))/mass
    err=max(err,abs(hinge-tail))
check("empirical_cvar_fractional_tail",err,1e-12,1000)

# Larger nested DAG cannot raise optimum if old edges are preserved.
vio=0.
for _ in range(200):
    edges=rng.uniform(0,2,size=(6,8,8))
    def solve(a):
        dp=np.zeros(a)
        for layer in edges:
            dp=np.min(dp[:,None]+layer[:a,:a],axis=0)
        return dp.min()
    vio=max(vio,solve(8)-solve(4))
check("nested_heading_dag_optimum",max(0.,vio),1e-12,200)

examples = {
 "old_gpu_cpu_time_ratio": .89514/.09295,
 "max_work_expansion_example":12*(1-2/(.75*40)),
 "expanded_latency_example_ms":(1-.75)*40+8*.75*40/12+2,
 "baseline_rollout_states":1600*56,
 "risk_rollout_two_iterations_states":800*8*56*2,
 "risk_states_memory_MB":800*8*56*3*4/1e6,
 "coverage_K1600_p001":1-(1-.001)**1600,
 "coverage_K6400_p001":1-(1-.001)**6400,
 "effective_beams_B120_rho01":120/(1+119*.1),
 "direction_error_N64_radius03_mm":2*.3*math.sin(math.pi/(2*64))*1e3,
 "query_quantization_1cm_mm":math.sqrt(2)*.01/2*1e3,
 "scenario_union_bound_M32":1-(.05/1600)**(1/32),
 "scenario_union_bound_M256":1-(.05/1600)**(1/256),
 "euler_step_bound_mm":.5*.65*.05**2*.5*1000,
 "euler_56_step_bound_mm":56*.5*.65*.05**2*.5*1000,
 "cspace_states":400*400*64,
 "cspace_bitpacked_MB":400*400*64/8/1e6,
 "cspace_float_MB":400*400*64*4/1e6,
 "full20m_float_grid_MB":2000*2000*64*4/1e6,
}
out={"date":"2026-10-15","seed":SEED,"numpy_version":np.__version__,
     "scope":"Independent mathematical sanity checks only; no CUDA benchmark or robot test.",
     "all_passed":True,"checks":results,"numeric_examples":examples}
Path(__file__).with_name("math_validation.json").write_text(json.dumps(out,ensure_ascii=False,indent=2)+"\n")
print(json.dumps(out,ensure_ascii=False,indent=2))
