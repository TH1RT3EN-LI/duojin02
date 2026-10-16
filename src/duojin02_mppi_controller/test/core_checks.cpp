#include "duojin02_mppi_controller/optimizer.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
using namespace duojin_mppi;
class Oracle : public Optimizer {
public:
  Oracle(bool omni, unsigned batch=513, unsigned steps=32) {
    optimizer_seed_=42;
    settings_.batch_size=batch;settings_.time_steps=steps;settings_.model_dt=.05f;
    settings_.temperature=.3f;settings_.gamma=.015f;settings_.sampling_std={.2f,.2f,.4f};
    settings_.base_constraints={.5f,-.25f,.3f,.9f};settings_.constraints=settings_.base_constraints;
    if(omni)motion_model_=std::make_shared<OmniMotionModel>();
    else motion_model_=std::make_shared<DiffDriveMotionModel>();
    reset();
    state_.pose.pose.position.x=1.4;state_.pose.pose.position.y=-.7;
    state_.pose.pose.orientation.z=std::sin(.7/2);state_.pose.pose.orientation.w=std::cos(.7/2);
    state_.speed.linear.x=.21;state_.speed.linear.y=-.13;state_.speed.angular.z=.17;
    for(unsigned t=0;t<steps;++t){control_sequence_.vx(t)=.2f*std::sin(t*.1f);control_sequence_.vy(t)=.1f;control_sequence_.wz(t)=.25f*std::cos(t*.2f);}
  }
  CoreSettings settings(){return coreSettings();}
  CoreBuffers buffers(){return coreBuffers();}
  void generate(){generateNoisedTrajectories();}
  void update(){updateControlSequence();}
  void cpuRolloutWithSupplied(const std::vector<float> &nx,const std::vector<float> &ny,const std::vector<float> &nw){
    const auto s=coreSettings();for(size_t k=0;k<s.batch;++k)for(size_t t=0;t<s.steps;++t){size_t i=k*s.steps+t;
      state_.cvx.data()[i]=control_sequence_.vx(t)+nx[i];state_.cwz.data()[i]=control_sequence_.wz(t)+nw[i];
      state_.cvy.data()[i]=s.holonomic?control_sequence_.vy(t)+ny[i]:0;}
    updateStateVelocities(state_);integrateStateVelocities(generated_trajectories_,state_);
  }
};
void close(const float*a,const float*b,size_t n,float tol,const char *label){
  double worst=0;for(size_t i=0;i<n;++i){assert(std::isfinite(a[i]));assert(std::isfinite(b[i]));worst=std::max(worst,double(std::abs(a[i]-b[i])));}
  if(worst>tol){std::cerr<<label<<" first="<<a[0]<<","<<b[0]<<" max_error="<<worst<<" tolerance="<<tol<<"\n";std::abort();}
}
void checkPathIndexBounds() {
  const std::vector<float> distances{0, 2, 4, 6};
  assert(utils::findClosestPathPt(distances, 9, 2) == 3);
  assert(utils::findClosestPathPt(distances, 4, 2) == 2);
  assert(utils::findClosestPathPt(distances, 5, 0) == 3);
  assert(utils::findClosestPathPt(distances, -1, 0) == 0);
  assert(utils::findClosestPathPt(distances, 7, 99) == 3);
  assert(utils::findClosestPathPt(std::vector<float>{0}, 9, 0) == 0);
  bool empty_rejected = false;
  try {utils::findClosestPathPt({}, 1, 0);} catch (const std::invalid_argument &) {empty_rejected = true;}
  assert(empty_rejected);
  size_t previous = 0;
  for (int i=0; i<100; ++i) {
    const size_t next = utils::findClosestPathPt(distances, i*.1f, previous);
    assert(next >= previous && next < distances.size()); previous = next;
  }
}
int main(int argc,char**argv){
  checkPathIndexBounds();
  const std::string mode=argc>1?argv[1]:"--cuda";
  if(mode=="--cpu"){
    Oracle a(true),b(true);a.generate();b.generate();auto s=a.settings();auto x=a.buffers(),y=b.buffers();
    close(x.x,y.x,s.batch*s.steps,0,"CPU deterministic");a.setSpeedLimit(20,true);a.reset();
    assert(std::abs(a.settings().vx_max-.1f)<1e-6);a.setSpeedLimit(0,false);assert(std::abs(a.settings().vx_max-.5f)<1e-6);
    std::cout<<"MPPI CPU deterministic reset and speed limits passed\n";return 0;
  }
  CudaOptimizerCore gpu;
  if(mode=="--no-device"){assert(!gpu.available());std::cout<<"MPPI no-device detected\n";return 0;}
  if(!gpu.available()){std::cout<<gpu.deviceName()<<"\n";return 77;}
  unsigned trials=0;
  for(bool omni:{false,true})for(unsigned batch:{1u,31u,257u,2000u}){
    Oracle cpu(omni,batch),dev(omni,batch);auto s=cpu.settings();auto a=cpu.buffers(),b=dev.buffers();size_t n=s.batch*s.steps;
    std::vector<float> nx(n),ny(n),nw(n);fillDeterministicNoises(nx.data(),ny.data(),nw.data(),n,s.std_vx,s.std_vy,s.std_wz,42,0);
    cpu.cpuRolloutWithSupplied(nx,ny,nw);
    gpu.rollout(s,b,{1.4f,-.7f,.7f,.21f,-.13f,.17f},42,0,nx.data(),ny.data(),nw.data());
    close(a.cvx,b.cvx,n,1e-6,"fixed-noise controls");close(a.vx,b.vx,n,1e-6,"velocities");
    close(a.x,b.x,n,2e-5,"trajectory x");close(a.y,b.y,n,2e-5,"trajectory y");close(a.yaw,b.yaw,n,2e-5,"trajectory yaw");
    for(size_t k=0;k<s.batch;++k)a.cost[k]=b.cost[k]=float(k%13)*.17f;
    cpu.update();a=cpu.buffers();gpu.update(s,b);
    close(a.ux,b.ux,s.steps,2e-5,"weighted vx");close(a.uw,b.uw,s.steps,2e-5,"weighted wz");
    if(omni)close(a.uy,b.uy,s.steps,2e-5,"weighted vy");
    // GPU RNG must use the same counter stream as CPU deterministic fixtures.
    Oracle generated(omni,batch);auto c=generated.buffers();
    gpu.rollout(s,c,{1.4f,-.7f,.7f,.21f,-.13f,.17f},42,0);
    close(a.cvx,c.cvx,n,2e-6,"device Gaussian samples");
    ++trials;
  }
  Oracle fixture(true);fixture.generate();auto s=fixture.settings();auto b=fixture.buffers();
  gpu.rollout(s,b,{1.4f,-.7f,.7f,.21f,-.13f,.17f},42,0);
  for(unsigned kind=0;kind<5;++kind){
    std::vector<float> expected(s.batch,0);std::fill_n(b.cost,s.batch,0);
    const float gx=.8f,gy=-.2f,weight=3.f,minv=-.4f,maxv=.6f,dt=.05f;
    for(size_t k=0;k<s.batch;++k){double sum=0;for(size_t t=0;t<s.steps;++t){size_t i=k*s.steps+t;
      if(kind==0)sum+=std::hypot(b.x[i]-gx,b.y[i]-gy);
      else if(kind==1)sum+=std::abs(std::remainder(double(gx)-b.yaw[i],2*M_PI));
      else if(kind==2)sum+=std::abs(b.wz[i]);
      else if(kind==3){float v=(b.vx[i]>0?1.f:-1.f)*std::hypot(b.vx[i],b.vy[i]);sum+=(std::max(v-maxv,0.f)+std::max(minv-v,0.f))*dt;}
      else if(t+1==s.steps)sum=std::hypot(b.x[i]-gx,b.y[i]-gy);
    }if(kind<=2)sum/=s.steps;expected[k]=std::pow(sum*weight,2);}
    gpu.scoreCritic(kind,weight,2,gx,gy,minv,maxv,0,dt,b.cost);
    close(expected.data(),b.cost,s.batch,2e-4,"CUDA critic");
  }
  // Invalid limits encode a stop; upstream 0 means NO_SPEED_LIMIT and must reset.
  fixture.setSpeedLimit(-1,true);fixture.reset();s=fixture.settings();b=fixture.buffers();
  fixture.generate();b=fixture.buffers();gpu.update(s,b);
  for(size_t t=0;t<s.steps;++t)assert(b.ux[t]==0 && b.uy[t]==0 && b.uw[t]==0);
  fixture.setSpeedLimit(.1,false);fixture.reset();assert(std::abs(fixture.settings().vx_max-.1f)<1e-6);
  fixture.setSpeedLimit(std::numeric_limits<double>::quiet_NaN(),false);fixture.reset();
  assert(fixture.settings().vx_max==0 && fixture.settings().wz_max==0);
  fixture.setSpeedLimit(0,false);fixture.reset();fixture.generate();s=fixture.settings();b=fixture.buffers();
  std::fill_n(b.cost,s.batch,0);b.cost[5]=std::numeric_limits<float>::quiet_NaN();
  std::vector<float> before(b.ux,b.ux+s.steps);bool failed=false;
  try{gpu.update(s,b);}catch(const OptimizerComputeError&){failed=true;}assert(failed);
  close(before.data(),b.ux,s.steps,0,"atomic invalid update");
  const auto stats=gpu.statistics();assert(stats.cuda_rollout_batches==2*trials+1);assert(stats.cuda_update_batches==trials+1);assert(stats.cuda_critic_batches==5);assert(stats.failures==1);
  std::cout<<"MPPI CUDA core passed device="<<gpu.deviceName()<<" trials="<<trials<<" rollouts="<<stats.cuda_rollout_batches<<" updates="<<stats.cuda_update_batches<<" critics="<<stats.cuda_critic_batches<<"\n";
}
