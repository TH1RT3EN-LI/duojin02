// Compile with the frozen pre-change include directory for the reference,
// or define DUOJIN_ASSOCIATION_CACHE_BENCH with the candidate include directory.
// Synthetic component timing only; does not estimate robot/device performance.
#include "duojin02_navigation/guard_query.hpp"
#ifdef DUOJIN_ASSOCIATION_CACHE_BENCH
#include "duojin02_navigation/prior_association_cache.hpp"
#endif
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <time.h>
using namespace duojin02_navigation;
static double cpuNs() {
  timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)) throw std::runtime_error("thread clock failed");
  return static_cast<double>(t.tv_sec)*1e9+t.tv_nsec;
}
static uint64_t bits(double value) {uint64_t result;static_assert(sizeof(result)==sizeof(value));std::memcpy(&result,&value,sizeof(value));return result;}
int main(int argc,char **argv) {
  if(argc!=3) {std::cerr<<"Usage: benchmark scan10_guard20|transform_each_cycle|scan20_guard20 fixture-output\n";return 2;}
  const std::string mode=argv[1];
  if(mode!="scan10_guard20" && mode!="transform_each_cycle" && mode!="scan20_guard20") return 2;
  std::vector<Point> prior,base_scan,scan;
  for(int y=-400;y<=400;++y) for(int x=25;x<=65;++x) prior.push_back({x*.01,y*.01,.007071067812});
  for(int y=250;y<=280;++y) for(int x=-400;x<=400;++x) prior.push_back({x*.01,y*.01,.007071067812});
  for(int i=0;i<721;++i) base_scan.push_back({.25+(i%13)*.001,-2.5+5.*i/720,.012});
  // Save actual base point doubles separately from timing. The deterministic
  // trace below supplies the per-frame offsets/epochs/poses for this corpus.
  std::ofstream fixture(argv[2],std::ios::binary);
  for(const auto &collection:{prior,base_scan}) {
    const uint64_t size=collection.size();fixture.write(reinterpret_cast<const char *>(&size),sizeof(size));
    for(const auto &p:collection) for(double value:{p.x,p.y,p.radius}) fixture.write(reinterpret_cast<const char *>(&value),sizeof(value));
  }
  fixture.close();if(!fixture) throw std::runtime_error("fixture write failed");
  SpatialPointIndex prior_index(.10),observed_index(.05);prior_index.rebuild(prior);
  Polygon polygon{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},
    {-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  constexpr int cycles=10000,warmup=200;
  uint64_t observation_epoch=0,checksum=1469598103934665603ULL;
  size_t hits=0,misses=0,queries=0,matched_count=0;
  std::vector<double> cpu,wall;
#ifdef DUOJIN_ASSOCIATION_CACHE_BENCH
  PriorAssociationCache cache;
#endif
  for(int cycle=0;cycle<cycles+warmup;++cycle) {
    if(cycle%(mode=="scan20_guard20" ? 1:2)==0) {
      ++observation_epoch;scan=base_scan;
      for(auto &p:scan) p.x+=(static_cast<int>(observation_epoch%7)-3)*.00001;
      observed_index.rebuild(scan);
    }
    const Pose map_pose{0,0,mode=="transform_each_cycle" ? cycle*.00001:0};
    const Pose robot{(cycle%9-4)*.0001,0,0};
    const double c=std::cos(map_pose.yaw),s=std::sin(map_pose.yaw);
    const double mx=c*(robot.x-map_pose.x)+s*(robot.y-map_pose.y);
    const double my=-s*(robot.x-map_pose.x)+c*(robot.y-map_pose.y);
    const auto wall_start=std::chrono::steady_clock::now();const double cpu_start=cpuNs();
    std::vector<Point> output=scan;
    const double radius=guardQueryRadius(polygon,{.25,.10,.1},{.3,0,.2},.008,.2,.4,prior_index.maxRadius(),.025,2.5);
#ifdef DUOJIN_ASSOCIATION_CACHE_BENCH
    cache.begin(prior.size(),{1,observation_epoch,map_pose,.025,.008});
    prior_index.forEachNearIndexed(prior,mx,my,radius,[&](const Point &q,size_t index) {
#else
    prior_index.forEachNear(prior,mx,my,radius,[&](const Point &q) {
#endif
      Point p{map_pose.x+c*q.x-s*q.y,map_pose.y+s*q.x+c*q.y,q.radius};
      if(std::hypot(p.x-robot.x,p.y-robot.y)>2.5) return;
      const double association=p.radius+std::min(.10,.025)+.008+.015;
#ifdef DUOJIN_ASSOCIATION_CACHE_BENCH
      const bool matched=cache.match(index,[&](){return observed_index.hasWithin(scan,p,association);});
#else
      const bool matched=observed_index.hasWithin(scan,p,association);
#endif
      if(!matched) {p.radius+=.025;output.push_back(p);}
      if(cycle>=warmup) {++queries;matched_count+=matched;}
    });
    const double cpu_elapsed=cpuNs()-cpu_start;
    const double wall_elapsed=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-wall_start).count();
    if(cycle>=warmup) {
      cpu.push_back(cpu_elapsed);wall.push_back(wall_elapsed);
#ifdef DUOJIN_ASSOCIATION_CACHE_BENCH
      hits+=cache.hits();misses+=cache.misses();
#else
      misses+=queries-(hits+misses);
#endif
      // Full ordered output identity, outside the timed region.
      checksum^=output.size();checksum*=1099511628211ULL;
      for(const auto &p:output) for(double value:{p.x,p.y,p.radius}) {checksum^=bits(value);checksum*=1099511628211ULL;}
    }
  }
  const auto mean=[](const std::vector<double> &v){double total=0;for(double x:v) total+=x;return total/v.size()/1e6;};
  const auto percentile=[](std::vector<double> v){std::sort(v.begin(),v.end());return v[static_cast<size_t>(.95*(v.size()-1))]/1e6;};
  std::cout<<std::setprecision(9)<<"{\"mode\":\""<<mode<<"\",\"cycles\":"<<cycles<<",\"prior_points\":"<<prior.size()
    <<",\"scan_points\":"<<scan.size()<<",\"association_queries\":"<<queries<<",\"matched\":"<<matched_count
    <<",\"cache_hits\":"<<hits<<",\"association_calculations\":"<<misses<<",\"output_checksum\":\""<<std::hex<<checksum<<std::dec
    <<"\",\"cpu_mean_ms\":"<<mean(cpu)<<",\"cpu_p95_ms\":"<<percentile(cpu)
    <<",\"wall_mean_ms\":"<<mean(wall)<<",\"wall_p95_ms\":"<<percentile(wall)<<"}\n";
}
