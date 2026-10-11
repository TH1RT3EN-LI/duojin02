#include "compute_internal.hpp"
#include <chrono>
#include <numeric>
#include <random>
#include <stdexcept>
namespace duojin01_navigation {
namespace {
// Start at the nearest segment projection, retain bends, and merge only
// collinear forward edges. No obstacle information or evaluator truth is used.
std::vector<Pose> guidancePath(const std::vector<Pose> &path,Pose start) {
  if(path.size()<2)return path;
  size_t edge=1;double minimum=1e100;Pose projection=path.front();
  for(size_t i=1;i<path.size();++i) {
    const auto a=path[i-1],b=path[i];const double dx=b.x-a.x,dy=b.y-a.y,length=dx*dx+dy*dy;
    if(length<1e-20)continue;
    const double f=std::clamp(((start.x-a.x)*dx+(start.y-a.y)*dy)/length,0.,1.);
    Pose p{a.x+f*dx,a.y+f*dy,start.yaw};const double d=std::hypot(p.x-start.x,p.y-start.y);
    if(d<minimum){minimum=d;projection=p;edge=i;}
  }
  std::vector<Pose> result{projection};
  for(size_t i=edge;i<path.size();++i) {
    const auto p=path[i];
    if(std::hypot(p.x-result.back().x,p.y-result.back().y)<1e-9)continue;
    if(result.size()>1) {
      const auto a=result[result.size()-2],b=result.back();
      const double dx=b.x-a.x,dy=b.y-a.y,ex=p.x-b.x,ey=p.y-b.y;
      if(dx*ex+dy*ey>0 && std::abs(dx*ey-dy*ex)<=1e-7*std::hypot(dx,dy)*std::hypot(ex,ey)) {
        result.back()=p;continue;
      }
    }
    result.push_back(p);
  }
  return result;
}
}
struct SamplingCompute::Impl {
  SamplingSettings settings;ComputeOptions options;
  std::unique_ptr<CudaExecutor> cuda;std::string reason;
  std::mt19937_64 random;std::normal_distribution<double> normal{0,1};
  std::vector<Velocity> nominal;
  Impl(SamplingSettings s,ComputeOptions o):settings(s),options(o),random(s.seed) {
    if(s.batch_size<2 || s.batch_size>32768 || !s.time_steps || s.time_steps>512 ||
       s.batch_size*s.time_steps>1048576 || !s.support_points || s.support_points>s.time_steps ||
       !std::isfinite(s.dt+s.temperature+s.gamma+s.noise.x+s.noise.y+s.noise.yaw+s.acceleration+s.angular_acceleration+s.margin+s.path_weight+s.goal_weight+s.yaw_weight+s.cost_weight) ||
       s.dt<=0 || s.dt>.2 || s.temperature<=0 || s.gamma<0 || s.noise.x<=0 || s.noise.y<=0 || s.noise.yaw<=0 ||
       s.acceleration<=0 || s.angular_acceleration<=0 || s.margin<0 || s.path_weight<0 || s.goal_weight<0 || s.yaw_weight<0 || s.cost_weight<0 ||
       !std::isfinite(s.primitive_fraction) || s.primitive_fraction<=0 || s.primitive_fraction>1 ||
       !std::isfinite(o.deadline_ms) || o.deadline_ms<=0)
      throw std::invalid_argument("Invalid optional MPPI settings");
    if(o.backend!=ComputeBackend::Cpu)cuda=makeCudaExecutor(reason);
    if(o.backend==ComputeBackend::Cuda && !cuda)throw std::runtime_error("CUDA sampler unavailable: "+reason);
  }
};
SamplingCompute::SamplingCompute(SamplingSettings s,ComputeOptions o):impl_(new Impl(s,o)){}
SamplingCompute::~SamplingCompute()=default;
bool SamplingCompute::available() const{return impl_->cuda!=nullptr;}
std::string SamplingCompute::deviceName() const{return impl_->cuda ? impl_->cuda->name():impl_->reason;}
void SamplingCompute::reset(){impl_->nominal.clear();impl_->random.seed(impl_->settings.seed);impl_->normal.reset();}
SamplingResult SamplingCompute::optimize(const LocalGridSnapshot &map,const Polygon &polygon,
    const std::vector<Pose> &path,const Pose &start,const Velocity &measured,const OmniVelocityBounds &bounds) {
  const auto begin=std::chrono::steady_clock::now();SamplingResult result;
  if(!map.width || !map.height || map.data.size()!=static_cast<size_t>(map.width)*map.height ||
     !std::isfinite(map.resolution+map.origin_x+map.origin_y) || map.resolution<=0 || path.empty() ||
     !std::isfinite(start.x+start.y+start.yaw+measured.x+measured.y+measured.yaw))throw std::invalid_argument("Invalid sampling snapshot");
  OmniVelocityLimits checked;checked.configure(bounds);
  GeometryCompute validator;const std::vector<Point> empty;validator.setScene(polygon,empty);
  for(const auto &p:path)if(!std::isfinite(p.x+p.y+p.yaw))throw std::invalid_argument("Invalid sampling path");
  const auto &s=impl_->settings;
  size_t nearest=0;double nearest_distance=1e100;
  for(size_t i=0;i<path.size();++i){const double d=std::hypot(path[i].x-start.x,path[i].y-start.y);if(d<nearest_distance){nearest_distance=d;nearest=i;}}
  size_t goal_index=nearest;double distance=0;
  const double horizon=std::max(.35,.75*s.time_steps*s.dt*std::hypot(bounds.vx_max,bounds.vy_max));
  while(goal_index+1<path.size() && distance<horizon){distance+=std::hypot(path[goal_index+1].x-path[goal_index].x,path[goal_index+1].y-path[goal_index].y);++goal_index;}
  Pose goal=path[goal_index];
  // Hold the chassis heading for ordinary Omni detours. Terminal rotation is
  // left to the existing precision controller when its chord is feasible.
  if(goal_index+1<path.size() || std::hypot(goal.x-start.x,goal.y-start.y)>.45)goal.yaw=start.yaw;
  if(impl_->nominal.empty()) {
    const double c=std::cos(start.yaw),ss=std::sin(start.yaw),dx=goal.x-start.x,dy=goal.y-start.y;
    const Velocity desired{std::clamp(c*dx+ss*dy,bounds.vx_min,bounds.vx_max),
      std::clamp(-ss*dx+c*dy,-bounds.vy_max,bounds.vy_max),std::clamp(1.8*std::remainder(goal.yaw-start.yaw,2*M_PI),-bounds.wz_max,bounds.wz_max)};
    impl_->nominal.assign(s.time_steps,desired);
  }
  for(auto &u:impl_->nominal){u.x=std::clamp(u.x,bounds.vx_min,bounds.vx_max);u.y=std::clamp(u.y,-bounds.vy_max,bounds.vy_max);u.yaw=std::clamp(u.yaw,-bounds.wz_max,bounds.wz_max);}
  std::vector<Velocity> noise(s.batch_size*s.support_points);
  // Deterministic host noise, antithetic pairs and a zero-noise nominal sample.
  for(size_t k=1;k<s.batch_size;k+=2)for(size_t j=0;j<s.support_points;++j) {
    const Velocity n{impl_->normal(impl_->random)*s.noise.x,impl_->normal(impl_->random)*s.noise.y,impl_->normal(impl_->random)*s.noise.yaw};
    noise[k*s.support_points+j]=n;
    if(k+1<s.batch_size)noise[(k+1)*s.support_points+j]={-n.x,-n.y,-n.yaw};
  }
  std::vector<unsigned int> rows(map.height+1),columns;
  for(unsigned int y=0;y<map.height;++y){rows[y]=columns.size();for(unsigned int x=0;x<map.width;++x)if(map.data[y*map.width+x]>=254)columns.push_back(x);}rows[map.height]=columns.size();
  double radius=0;for(const auto &p:polygon)radius=std::max(radius,std::hypot(p.x,p.y));
  SamplingView view{map.width,map.height,map.crop_x,map.crop_y,map.resolution,map.origin_x,map.origin_y,radius,
    map.data.data(),rows.data(),columns.data(),polygon.data(),polygon.size(),path.data(),path.size(),start,goal,measured,bounds,s};
  const auto guidance=s.path_primitives ? guidancePath(path,start):std::vector<Pose>{};
  view.guidance=guidance.data();view.guidance_size=guidance.size();
  result.primitive_samples=primitiveCount(view);
  if(impl_->cuda) {
    try {
      auto output=impl_->cuda->sampling(view,map.data,rows,columns,polygon,path,impl_->nominal,noise,guidance,impl_->options.deadline_ms);
      result.costs=std::move(output.first);result.sequence=std::move(output.second);result.used_cuda=true;
    }catch(const std::exception &e){throw ComputeFailure(e.what());}
  } else {
    result.costs.resize(s.batch_size);std::vector<Velocity> controls(s.batch_size*s.time_steps);
    for(size_t k=0;k<s.batch_size;++k) {
      Pose pose=start;Velocity u=measured;double cost=0;size_t waypoint=0;
      for(size_t t=0;t<s.time_steps;++t) {
        u=candidateControl(view,impl_->nominal.data(),noise.data(),k,t,pose,u,waypoint);controls[k*s.time_steps+t]=u;
        const auto next=sampleAdvance(pose,u,s.dt);cost+=sampleCost(view,pose,next,u,impl_->nominal[t],t);pose=next;
      }
      result.costs[k]=cost;
    }
    const double minimum=*std::min_element(result.costs.begin(),result.costs.end());
    std::vector<double> weights(s.batch_size);double sum=0;
    for(size_t k=0;k<s.batch_size;++k){weights[k]=result.costs[k]<1e9 ? std::exp(-(result.costs[k]-minimum)/s.temperature):0;sum+=weights[k];}
    result.sequence.resize(s.time_steps);
    if(sum>0)for(size_t t=0;t<s.time_steps;++t)for(size_t k=0;k<s.batch_size;++k) {
      const auto &u=controls[k*s.time_steps+t];const double w=weights[k]/sum;
      result.sequence[t].x+=w*u.x;result.sequence[t].y+=w*u.y;result.sequence[t].yaw+=w*u.yaw;
    }
  }
  result.collision_free=std::count_if(result.costs.begin(),result.costs.end(),[](double c){return std::isfinite(c) && c<1e9;});
  for(size_t k=1;k<=result.primitive_samples;++k)result.primitive_collision_free+=std::isfinite(result.costs[k]) && result.costs[k]<1e9;
  result.valid=result.collision_free>0;
  if(std::any_of(result.costs.begin(),result.costs.end(),[](double c){return !std::isfinite(c);}))
    throw ComputeFailure("Non-finite sampling cost");
  if(result.sequence.size()!=s.time_steps)throw ComputeFailure("Invalid sampling sequence size");
  auto validSequence=[&](const std::vector<Velocity> &sequence) {
    Pose pose=start;
    for(const auto &u:sequence) {
      if(!std::isfinite(u.x+u.y+u.yaw))return false;
      const auto next=sampleAdvance(pose,u,s.dt);
      if(rasterSweep(view,pose,next))return false;
      pose=next;
    }
    return true;
  };
  // Averaging two collision-free modes can create a colliding trajectory.
  // Validate the complete weighted sequence on CPU; if needed, retain the
  // best validated sampled sequence rather than sending a mode average.
  if(result.valid && !validSequence(result.sequence)) {
    const size_t best=std::min_element(result.costs.begin(),result.costs.end())-result.costs.begin();
    Velocity u=measured;Pose pose=start;size_t waypoint=0;
    for(size_t t=0;t<s.time_steps;++t){u=candidateControl(view,impl_->nominal.data(),noise.data(),best,t,pose,u,waypoint);result.sequence[t]=u;pose=sampleAdvance(pose,u,s.dt);}
    result.used_best_trajectory=true;result.valid=validSequence(result.sequence);
  }
  if(result.valid) {
    result.command=result.sequence.front();impl_->nominal=result.sequence;
    std::rotate(impl_->nominal.begin(),impl_->nominal.begin()+1,impl_->nominal.end());impl_->nominal.back()=impl_->nominal[impl_->nominal.size()>1 ? impl_->nominal.size()-2:0];
  } else impl_->nominal.clear();
  result.wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  if(result.used_cuda && result.wall_ms>impl_->options.deadline_ms)throw ComputeFailure("CUDA sampler exceeded whole-call deadline");
  return result;
}
}
