#include "duojin01_navigation/gpu_sampling.hpp"
#include "sampling_math.hpp"
#include <cassert>
#include <iostream>
using namespace duojin01_navigation;
int main(int argc,char **argv) {
  const bool required=argc>1 && std::string(argv[1])=="--require-cuda";
  SamplingSettings settings;settings.batch_size=64;settings.time_steps=16;settings.support_points=4;
  size_t compared=0;double max_delta=0;
  for(int configuration=0;configuration<3;++configuration) {
  if(configuration) {settings.noise={.30,.30,.12};settings.path_weight=3.;settings.goal_weight=16.;}
  settings.path_primitives=configuration==2;
  SamplingCompute cpu(settings,{ComputeBackend::Cpu,0,2000});
  SamplingCompute gpu(settings,{ComputeBackend::Auto,0,2000});
  if(required && !gpu.available()){std::cout<<"CUDA device unavailable\n";return 77;}
  const Polygon polygon{{-.213,-.203},{.213,-.203},{.213,.203},{-.213,.203}};
  LocalGridSnapshot map;map.width=map.height=400;map.resolution=.01;map.origin_x=map.origin_y=-2;map.data.resize(160000);
  std::vector<Pose> path;for(int i=0;i<=50;++i)path.push_back({-.6+.02*i,.02,.013});
  const Pose start{-.6,.02,.013};const Velocity measured{.12,.02,.04};const OmniVelocityBounds bounds{.5,-.2,.3,.65};
  for(int scenario=0;scenario<6;++scenario) {
    std::fill(map.data.begin(),map.data.end(),0);
    if(scenario==1)for(unsigned int x=0;x<400;++x){map.data[245*400+x]=254;map.data[155*400+x]=254;}
    if(scenario==2)map.data[202*400+140]=254; // isolated interior obstacle at start
    if(scenario==3)map.data[202*400+140]=255; // unknown is not free
    if(scenario==4)for(unsigned int y=180;y<225;++y)map.data[y*400+225]=254;
    if(scenario==5) {
      path.clear();for(int i=0;i<=60;++i)path.push_back({-.6+.01*i,.02,0});
      for(int i=1;i<=80;++i)path.push_back({0,.02+.01*i,0});
      for(unsigned int y=0;y<400;++y)for(unsigned int x=0;x<400;++x) {
        const double wx=-2+(x+.5)*.01,wy=-2+(y+.5)*.01;
        if(!((wx>=-1.5 && wx<=.25 && std::abs(wy)<=.25) || (std::abs(wx)<=.25 && wy>=-.25 && wy<=1.5)))map.data[y*400+x]=254;
      }
    }
    cpu.reset();gpu.reset();
    for(int cycle=0;cycle<3;++cycle) {
      const auto a=cpu.optimize(map,polygon,path,start,measured,bounds),b=gpu.optimize(map,polygon,path,start,measured,bounds);
      assert(a.valid==b.valid && a.collision_free==b.collision_free);
      assert(a.primitive_samples==b.primitive_samples && a.primitive_collision_free==b.primitive_collision_free);
      if(settings.path_primitives)assert(b.primitive_samples==16);
      if(settings.path_primitives && scenario==5)assert(b.valid && b.primitive_collision_free>0);
      assert(a.used_best_trajectory==b.used_best_trajectory);
      if(required)assert(b.used_cuda);
      assert(a.costs.size()==settings.batch_size && b.costs.size()==settings.batch_size);
      for(size_t i=0;i<a.costs.size();++i){const double delta=std::abs(a.costs[i]-b.costs[i]);max_delta=std::max(max_delta,delta);assert(delta<1e-7 || (a.costs[i]>=1e9 && b.costs[i]>=1e9));++compared;}
      assert(std::abs(a.command.x-b.command.x)<1e-8 && std::abs(a.command.y-b.command.y)<1e-8 && std::abs(a.command.yaw-b.command.yaw)<1e-8);
      assert(b.command.x>=bounds.vx_min-1e-9 && b.command.x<=bounds.vx_max+1e-9);
      assert(std::abs(b.command.y)<=bounds.vy_max+1e-9 && std::abs(b.command.yaw)<=bounds.wz_max+1e-9);
      // Independent oracle: the unchanged original filled-polygon predicate,
      // occupied cell discs, and 1 mm / corner-travel integration. This does
      // not call the shared CPU/CUDA raster implementation under test.
      if(b.valid) {
        std::vector<Point> obstacles;
        for(size_t y=0;y<map.height;++y)for(size_t x=0;x<map.width;++x)
          if(map.data[y*map.width+x]>=254)obstacles.push_back({
            map.origin_x+(map.crop_x+x+.5)*map.resolution,
            map.origin_y+(map.crop_y+y+.5)*map.resolution,
            map.resolution*std::sqrt(2.)/2});
        Pose p=start;Velocity previous=measured;
        for(const auto &u:b.sequence) {
          assert(std::hypot(u.x-previous.x,u.y-previous.y)<=settings.acceleration*settings.dt+1e-8);
          assert(std::abs(u.yaw-previous.yaw)<=settings.angular_acceleration*settings.dt+1e-8);
          const int n=std::max(1,static_cast<int>(std::ceil((std::hypot(u.x,u.y)+.3*std::abs(u.yaw))*settings.dt/.001)));
          for(int j=0;j<n;++j){p=advance(p,u,settings.dt/n);assert(!collision(p,polygon,obstacles,settings.margin));}
          previous=u;
        }
      }
      if(scenario==2 || scenario==3)assert(!b.valid && b.command.x==0 && b.command.y==0 && b.command.yaw==0);
    }
  }
  // Raster checks include swept thin obstacles and unknown/outside behaviour.
  std::fill(map.data.begin(),map.data.end(),0);map.data[200*400+200]=254;
  std::vector<unsigned int> rows(401),columns{200};for(size_t r=201;r<rows.size();++r)rows[r]=1;
  SamplingView view{400,400,0,0,.01,-2,-2,.3,map.data.data(),rows.data(),columns.data(),polygon.data(),polygon.size(),path.data(),path.size(),start,path.back(),measured,bounds,settings};
  assert(rasterSweep(view,{-.5,0,0},{.5,0,0}));
  assert(rasterCollision(view,{0,0,0},.01));assert(rasterCollision(view,{1.95,0,0},.01));
  map.data.clear();bool rejected=false;try{gpu.optimize(map,polygon,path,start,measured,bounds);}catch(const std::invalid_argument &){rejected=true;}assert(rejected);
  std::cout<<"SAMPLING_CHECKS compared_costs="<<compared<<" max_cost_delta="<<max_delta<<" device="<<gpu.deviceName()<<" passed\n";
  }
}
