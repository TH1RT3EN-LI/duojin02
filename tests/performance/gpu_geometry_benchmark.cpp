#include "duojin01_navigation/gpu_backend.hpp"
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
  if(argc!=6){std::cerr<<"Usage: gpu_geometry_benchmark original|cuda|auto tiny|medium|dense|blocked reuse|rebuild iterations fixture.bin\n";return 2;}
  const std::string backend=argv[1],workload=argv[2],mode=argv[3];const int iterations=std::stoi(argv[4]);
  if(iterations<1 || (mode!="reuse" && mode!="rebuild"))return 2;
  size_t n=workload=="tiny" ? 721 : workload=="medium" ? 20000 : workload=="dense" || workload=="blocked" ? 104637 : 0;
  if(!n)return 2;
  const Polygon polygon{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},{-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  std::vector<Point> points;points.reserve(n);
  for(size_t i=0;i<n;++i) {
    // Occupied wall interiors surround a collision-free longitudinal chord.
    points.push_back({-1.+(i%503)*.01,(i%2 ? 1.:-1.)*(.32+(i/503%100)*.01),.007071067812});
  }
  if(workload=="blocked")points[0]={0,0,.007071067812};
  std::vector<Pose> poses;const int steps=workload=="tiny" ? 8 : 120;
  for(int i=0;i<=steps;++i)poses.push_back({.004*i,0,0});
  std::ofstream fixture(argv[5],std::ios::binary);
  for(const auto &v:{polygon,points}){uint64_t count=v.size();fixture.write(reinterpret_cast<const char *>(&count),8);fixture.write(reinterpret_cast<const char *>(v.data()),v.size()*sizeof(Point));}
  uint64_t count=poses.size();fixture.write(reinterpret_cast<const char *>(&count),8);fixture.write(reinterpret_cast<const char *>(poses.data()),poses.size()*sizeof(Pose));fixture.close();if(!fixture)throw std::runtime_error("fixture write failed");
  std::unique_ptr<GeometryCompute> compute;
  if(backend!="original") {
    compute=std::make_unique<GeometryCompute>(ComputeOptions{parseComputeBackend(backend),262144,2000});compute->setScene(polygon,points);
  }
  auto original=[&](){for(const auto &pose:poses)if(collision(pose,polygon,points,.010))return true;return false;};
  std::vector<double> wall,cpu;double cold=0;uint64_t checksum=1469598103934665603ULL;
  for(int i=-6;i<iterations;++i) {
    if(mode=="rebuild")points.back().x+=(i%2 ? 1.:-1.)*1e-7;
    const auto begin=std::chrono::steady_clock::now();const double c=cpuNs();bool hit=false;
    if(compute) {
      if(mode=="rebuild")compute->setScene(polygon,points);
      if(backend=="auto" && collision(poses.front(),polygon,points,.010))hit=true;
      else if(compute->willUseCuda(poses.size())) {
        const auto r=compute->evaluate(poses,.010);hit=std::any_of(r.collisions.begin(),r.collisions.end(),[](uint8_t x){return x;});
      } else hit=original();
    } else hit=original();
    const double cpu_ms=(cpuNs()-c)/1e6;
    const double wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    if(i==-6)cold=wall_ms;
    if(hit!=original())throw std::runtime_error("Original chord predicate mismatch");
    if(i>=0){wall.push_back(wall_ms);cpu.push_back(cpu_ms);checksum^=hit;checksum*=1099511628211ULL;}
  }
  const auto mean=[](const std::vector<double> &v){return std::accumulate(v.begin(),v.end(),0.)/v.size();};
  const ComputeStatistics stats=compute ? compute->statistics() : ComputeStatistics{};
  std::cout<<std::setprecision(12)<<"{\"backend\":\""<<backend<<"\",\"workload\":\""<<workload<<"\",\"mode\":\""<<mode
    <<"\",\"iterations\":"<<iterations<<",\"points\":"<<n<<",\"poses\":"<<poses.size()
    <<",\"cold_wall_ms\":"<<cold<<",\"wall_mean_ms\":"<<mean(wall)<<",\"wall_p95_ms\":"<<quantile(wall,.95)
    <<",\"wall_p99_ms\":"<<quantile(wall,.99)<<",\"wall_max_ms\":"<<*std::max_element(wall.begin(),wall.end())
    <<",\"cpu_mean_ms\":"<<mean(cpu)<<",\"cuda_batches\":"<<stats.cuda_batches<<",\"boundary_rechecks\":"<<stats.boundary_rechecks
    <<",\"checksum\":\""<<std::hex<<checksum<<std::dec<<"\",\"wall_samples_ms\":[";
  for(size_t i=0;i<wall.size();++i)std::cout<<(i ? ",":"")<<wall[i];
  std::cout<<"],\"cpu_samples_ms\":[";
  for(size_t i=0;i<cpu.size();++i)std::cout<<(i ? ",":"")<<cpu[i];
  std::cout<<"]}\n";
}
