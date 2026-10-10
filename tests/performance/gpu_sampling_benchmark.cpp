#include "duojin01_navigation/gpu_sampling.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <time.h>
using namespace duojin01_navigation;
static double cpuNs(){timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t))throw std::runtime_error("clock failed");return t.tv_sec*1e9+t.tv_nsec;}
static double quantile(std::vector<double> v,double p){std::sort(v.begin(),v.end());return v[static_cast<size_t>(p*(v.size()-1))];}
int main(int argc,char **argv) {
  if(argc!=5){std::cerr<<"Usage: gpu_sampling_benchmark cpu|cuda open|corridor|detour|blocked iterations fixture.bin\n";return 2;}
  const std::string backend=argv[1],workload=argv[2];const int iterations=std::stoi(argv[3]);if(iterations<1)return 2;
  SamplingSettings settings;SamplingCompute compute(settings,{parseComputeBackend(backend),0,10000});
  LocalGridSnapshot map;map.width=map.height=400;map.resolution=.01;map.origin_x=map.origin_y=-2;map.data.resize(160000);
  if(workload=="corridor")for(size_t x=0;x<400;++x){map.data[175*400+x]=254;map.data[225*400+x]=254;}
  else if(workload=="detour")for(size_t y=197;y<=203;++y)for(size_t x=197;x<=203;++x)map.data[y*400+x]=254;
  else if(workload=="blocked")map.data[201*400+140]=255;
  else if(workload!="open")return 2;
  const Polygon polygon{{-.213,-.203},{.213,-.203},{.213,.203},{-.213,.203}};
  std::vector<Pose> path;for(int i=0;i<=64;++i)path.push_back({-.6+1.5*i/64,.017,0});
  std::ofstream fixture(argv[4],std::ios::binary);fixture.write(reinterpret_cast<const char *>(map.data.data()),map.data.size());
  fixture.write(reinterpret_cast<const char *>(path.data()),path.size()*sizeof(Pose));fixture.close();if(!fixture)throw std::runtime_error("fixture write failed");
  std::vector<double> wall,cpu;double cold=0;size_t valid=0,best=0,cuda_calls=0,clear_min=1600;
  std::vector<Velocity> commands;
  for(int i=-5;i<iterations;++i) {
    compute.reset();const auto begin=std::chrono::steady_clock::now();const double c=cpuNs();
    const auto result=compute.optimize(map,polygon,path,{-.6,.017,0},{},{.5,-.2,.3,.65});
    const double cpu_ms=(cpuNs()-c)/1e6;
    const double wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    if(i==-5)cold=wall_ms;
    if(i>=0){wall.push_back(wall_ms);cpu.push_back(cpu_ms);valid+=result.valid;best+=result.used_best_trajectory;cuda_calls+=result.used_cuda;clear_min=std::min(clear_min,result.collision_free);commands.push_back(result.command);}
  }
  const auto mean=[](const std::vector<double> &v){return std::accumulate(v.begin(),v.end(),0.)/v.size();};
  const auto command=commands.front();
  std::cout<<std::setprecision(12)<<"{\"backend\":\""<<backend<<"\",\"workload\":\""<<workload<<"\",\"iterations\":"<<iterations
    <<",\"batch_size\":1600,\"time_steps\":56,\"support_points\":8,\"reset_each_call\":true,\"cold_wall_ms\":"<<cold
    <<",\"wall_mean_ms\":"<<mean(wall)<<",\"wall_p95_ms\":"<<quantile(wall,.95)<<",\"wall_p99_ms\":"<<quantile(wall,.99)
    <<",\"wall_max_ms\":"<<*std::max_element(wall.begin(),wall.end())<<",\"cpu_mean_ms\":"<<mean(cpu)
    <<",\"valid_calls\":"<<valid<<",\"best_sequence_calls\":"<<best<<",\"cuda_calls\":"<<cuda_calls<<",\"collision_free_min\":"<<clear_min
    <<",\"command\":["<<command.x<<","<<command.y<<","<<command.yaw<<"],\"wall_samples_ms\":[";
  for(size_t i=0;i<wall.size();++i)std::cout<<(i ? ",":"")<<wall[i];
  std::cout<<"],\"cpu_samples_ms\":[";
  for(size_t i=0;i<cpu.size();++i)std::cout<<(i ? ",":"")<<cpu[i];
  std::cout<<"]}\n";
}
