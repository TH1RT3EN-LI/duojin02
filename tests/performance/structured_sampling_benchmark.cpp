// Matched-input component and ideal kinematic feedback trials. No localization,
// ROS command gate, sensor simulation or physical timing is implied.
#include "duojin01_navigation/gpu_sampling.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <time.h>
using namespace duojin01_navigation;
struct Fixture {std::string name;LocalGridSnapshot map;std::vector<Pose> path;Pose start;Velocity measured;};
static Polygon footprint(){Polygon p{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},{-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};for(auto &q:p){q.x+=std::copysign(.008,q.x);q.y+=std::copysign(.008,q.y);}return p;}
static double cpuMs(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1000.+t.tv_nsec/1e6;}
static double percentile(std::vector<double> v,double q){std::sort(v.begin(),v.end());return v[static_cast<size_t>(q*(v.size()-1))];}
static std::vector<Point> obstacles(const LocalGridSnapshot &m){std::vector<Point> out;for(unsigned y=0;y<m.height;++y)for(unsigned x=0;x<m.width;++x)if(m.data[y*m.width+x]>=254)out.push_back({m.origin_x+(x+.5)*m.resolution,m.origin_y+(y+.5)*m.resolution,m.resolution*std::sqrt(2.)/2});return out;}
// Original filled-polygon oracle with 1 mm substeps, independent of the shared
// sampling_math.hpp raster implementation. Excludes no nearby occupied cells.
static bool safeStep(Pose p,Velocity u,const Polygon &polygon,const std::vector<Point> &all,double dt,Pose &next){
 std::vector<Point> near;for(const auto &q:all)if(std::hypot(q.x-p.x,q.y-p.y)<.5)near.push_back(q);
 const int n=std::max(1,int(std::ceil((std::hypot(u.x,u.y)+.32*std::abs(u.yaw))*dt/.001)));
 for(int i=0;i<n;++i){p=advance(p,u,dt/n);if(collision(p,polygon,near,.010))return false;}next=p;return true;
}
static Fixture makeFixture(const std::string &name){
 Fixture f;f.name=name;f.map.width=f.map.height=400;f.map.resolution=.01;f.map.origin_x=f.map.origin_y=-2;f.map.data.resize(160000);
 const bool reverse=name.rfind("reverse",0)==0,corner=name.rfind("corner",0)==0 || reverse;
 if(corner){const double width=name.find("50")!=std::string::npos ? .5:.6;
  for(unsigned y=0;y<400;++y)for(unsigned x=0;x<400;++x){const double wx=-2+(x+.5)*.01,wy=-2+(y+.5)*.01;const bool free=(wx>=-1.5 && wx<=width/2 && std::abs(wy)<=width/2) || (std::abs(wx)<=width/2 && wy>=-width/2 && wy<=1.5);if(!free)f.map.data[y*400+x]=254;}
  for(int i=0;i<=80;++i)f.path.push_back({-.8+.01*i,0,0});for(int i=1;i<=80;++i)f.path.push_back({0,.01*i,0});
  if(reverse)std::reverse(f.path.begin(),f.path.end());f.start=f.path.front();
 }else{f.start={-.6,.017,0};for(int i=0;i<=120;++i)f.path.push_back({-.6+.01*i,.017,0});
  if(name=="blocked")f.map.data[201*400+140]=255;
  else if(name=="detour")for(unsigned y=197;y<=203;++y)for(unsigned x=197;x<=203;++x)f.map.data[y*400+x]=254;
  else if(name!="open")throw std::invalid_argument("Unknown fixture");
 }return f;
}
static std::vector<Fixture> recorded(const std::string &input,const std::string &map_path){
 std::ifstream in(input);size_t count=0;in>>count;if(!in || count>100)throw std::runtime_error("Invalid recorded fixture");std::vector<Fixture> result;
 for(size_t k=0;k<count;++k){Fixture f;double tx,ty,ta;size_t n;
  in>>f.name>>f.start.x>>f.start.y>>f.start.yaw>>f.measured.x>>f.measured.y>>f.measured.yaw>>tx>>ty>>ta>>n;
  if(!in || n>10000)throw std::runtime_error("Invalid recorded path");f.path.resize(n);for(auto &p:f.path)in>>p.x>>p.y>>p.yaw;
  f.map.width=920;f.map.height=720;f.map.resolution=.01;f.map.origin_x=-4.6;f.map.origin_y=-3.6;f.map.data.resize(920*720);
  std::ifstream bytes(map_path,std::ios::binary);bytes.read(reinterpret_cast<char*>(f.map.data.data()),f.map.data.size());if(bytes.gcount()!=std::streamsize(f.map.data.size()))throw std::runtime_error("Invalid recorded map");result.push_back(std::move(f));
 }return result;
}
int main(int argc,char **argv){try{
 if(argc!=9 && argc!=11){std::cerr<<"backend mode case seed batch steps repetitions output.json [recorded-input.txt map.bin]\n";return 2;}
 const std::string backend=argv[1],mode=argv[2],scenario=argv[3];const int repetitions=std::stoi(argv[7]);if(repetitions<1)return 2;
 SamplingSettings settings;settings.seed=std::stoull(argv[4]);settings.batch_size=std::stoul(argv[5]);settings.time_steps=std::stoul(argv[6]);settings.support_points=4;
 settings.noise={.35,.35,.12};settings.path_weight=1.5;settings.goal_weight=24.;
#ifdef DUOJIN_STRUCTURED_API
 settings.path_primitives=mode=="structured";
#else
 if(mode!="legacy")return 2;
#endif
 if(mode!="legacy" && mode!="structured")return 2;
 auto fixtures=scenario=="recorded" && argc==11 ? recorded(argv[9],argv[10]):std::vector<Fixture>{makeFixture(scenario)};
 std::ofstream out(argv[8]);out<<std::setprecision(12)<<"{\"backend\":\""<<backend<<"\",\"mode\":\""<<mode<<"\",\"seed\":"<<settings.seed<<",\"batch\":"<<settings.batch_size<<",\"steps\":"<<settings.time_steps<<",\"scope\":\"sampler only; ideal kinematic feedback or static recorded diagnostic\",\"fixtures\":[";
 const Polygon polygon=footprint();const OmniVelocityBounds bounds{.5,-.2,.3,.65};
 for(size_t index=0;index<fixtures.size();++index){const auto &f=fixtures[index];const auto occupied=obstacles(f.map);
  SamplingCompute sampler(settings,{parseComputeBackend(backend),0,10000});Pose pose=f.start;Velocity measured=f.measured;
  std::vector<double> wall,cpu;size_t valid=0,feasible=0,primitive_feasible=0,best=0,cuda=0,violations=0,hits=0;bool arrived=false,crossed=false;double dwell=0,cold=0,elapsed=0;std::vector<Pose> trajectory{pose};
  const int cycles=repetitions==1 && scenario!="recorded" ? 640:repetitions;
  out<<(index ? ",":"")<<"{\"name\":\""<<f.name<<"\",\"calls\":[";
  for(int cycle=0;cycle<cycles;++cycle){if(repetitions>1 || scenario=="recorded")sampler.reset();
   const auto begin=std::chrono::steady_clock::now();const double c=cpuMs();const auto r=sampler.optimize(f.map,polygon,f.path,pose,measured,bounds);cpu.push_back(cpuMs()-c);wall.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());if(!cycle)cold=wall.back();
   valid+=r.valid;feasible+=r.collision_free;best+=r.used_best_trajectory;cuda+=r.used_cuda;
#ifdef DUOJIN_STRUCTURED_API
   primitive_feasible+=r.primitive_collision_free;
#endif
   const auto u=r.command;const bool limit=u.x>=bounds.vx_min-1e-9 && u.x<=bounds.vx_max+1e-9 && std::abs(u.y)<=bounds.vy_max+1e-9 && std::abs(u.yaw)<=bounds.wz_max+1e-9;
   const bool acceleration=!r.valid || (std::hypot(u.x-measured.x,u.y-measured.y)<=settings.acceleration*settings.dt+1e-8 && std::abs(u.yaw-measured.yaw)<=settings.angular_acceleration*settings.dt+1e-8);
   violations+=!limit || !acceleration;
   out<<(cycle ? ",":"")<<"["<<wall.back()<<","<<cpu.back()<<","<<r.valid<<","<<r.collision_free<<","<<u.x<<","<<u.y<<","<<u.yaw<<"]";
   if(repetitions>1 || scenario=="recorded")continue;
   // A failed proposal requests an ideal acceleration-limited stop. Record its
   // inadmissibility separately; a real downstream smoother is not simulated.
   Velocity executed=u;if(!r.valid){const double speed=std::hypot(measured.x,measured.y),scale=speed>0 ? std::max(0.,1.-settings.acceleration*settings.dt/speed):0;executed={measured.x*scale,measured.y*scale,std::clamp(0.,measured.yaw-settings.angular_acceleration*settings.dt,measured.yaw+settings.angular_acceleration*settings.dt)};}
   Pose next;if(!safeStep(pose,executed,polygon,occupied,settings.dt,next)){++hits;break;}pose=next;measured=executed;trajectory.push_back(pose);elapsed+=settings.dt;
   if(scenario.rfind("corner",0)==0)crossed|=pose.y>.35;else if(scenario.rfind("reverse",0)==0)crossed|=pose.x<-.35;
   const auto goal=f.path.back();const bool qualifying=std::hypot(pose.x-goal.x,pose.y-goal.y)<=.02 && std::abs(std::remainder(pose.yaw-goal.yaw,2*M_PI))<=.035 && std::hypot(measured.x,measured.y)<=.02 && std::abs(measured.yaw)<=.02;
   if(qualifying)dwell+=settings.dt;else dwell=0;if(dwell>=.30){arrived=true;break;}
  }
  out<<"],\"valid_calls\":"<<valid<<",\"feasible_sum\":"<<feasible<<",\"primitive_feasible_sum\":"<<primitive_feasible<<",\"best_calls\":"<<best<<",\"cuda_calls\":"<<cuda<<",\"violations\":"<<violations<<",\"collision_events\":"<<hits<<",\"arrived\":"<<arrived<<",\"crossed_corner\":"<<crossed<<",\"simulated_elapsed_s\":"<<elapsed<<",\"cold_ms\":"<<cold<<",\"wall_mean_ms\":"<<std::accumulate(wall.begin(),wall.end(),0.)/wall.size()<<",\"wall_p95_ms\":"<<percentile(wall,.95)<<",\"wall_p99_ms\":"<<percentile(wall,.99)<<",\"wall_max_ms\":"<<*std::max_element(wall.begin(),wall.end())<<",\"final\":["<<pose.x<<","<<pose.y<<","<<pose.yaw<<"],\"trajectory\":[";
  for(size_t i=0;i<trajectory.size();++i)out<<(i ? ",":"")<<"["<<trajectory[i].x<<","<<trajectory[i].y<<","<<trajectory[i].yaw<<"]";out<<"]}";
 }out<<"]}\n";if(!out)throw std::runtime_error("Result write failed");return 0;
 }catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}}
