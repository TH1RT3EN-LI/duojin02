// Frozen hash index versus production bounded dense/sparse index. Measures
// query batches and, separately, rebuilding every batch. No safety/ROS timing.
#include "duojin01_navigation/guard_query.hpp"
#include "spatial_index_reference.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <time.h>
using namespace duojin01_navigation;
static double cpuNs() {
  timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t))throw std::runtime_error("thread clock failed");
  return static_cast<double>(t.tv_sec)*1e9+t.tv_nsec;
}
static void hashWord(uint64_t &hash,uint64_t word) {
  for(int i=0;i<8;++i){hash^=(word>>(i*8))&255;hash*=1099511628211ULL;}
}
template<class Index> static int run(const std::string &arm,const std::string &mode,const char *corpus_path) {
  std::vector<Point> original,points;
  const bool sparse=mode=="sparse_rebuild_query",rebuild=mode!="compact_queries";
  for(int i=0;i<721;++i) {
    const double angle=i*2*M_PI/721,r=.55+.10*std::sin(angle*5);
    original.push_back(sparse ? Point{-40.+(i%41)*2.,-40.+(i/41)*4.,.012} : Point{r*std::cos(angle),r*std::sin(angle),.012});
  }
  std::ofstream corpus(corpus_path,std::ios::binary);const uint64_t count=original.size();
  corpus.write(reinterpret_cast<const char *>(&count),sizeof(count));
  for(const auto &p:original)for(double v:{p.x,p.y,p.radius})corpus.write(reinterpret_cast<const char *>(&v),sizeof(v));
  corpus.close();if(!corpus)throw std::runtime_error("corpus write failed");
  points=original;Index index(.05);index.rebuild(points);
  const int cycles=rebuild ? 5000 : 20000,warmup=100;
  std::vector<double> cpu,wall;cpu.reserve(cycles);wall.reserve(cycles);
  std::vector<uint64_t> outputs;outputs.reserve(200000);uint64_t checksum=1469598103934665603ULL;
  size_t examined=0,visited=0,matches=0;
  for(int iteration=-warmup;iteration<cycles;++iteration) {
    const int n=iteration+warmup;const double shift=rebuild ? .001*(n%7) : 0;
    if(rebuild)for(size_t i=0;i<points.size();++i)points[i]={original[i].x+shift,original[i].y-shift,original[i].radius};
    Point queries[256];
    for(int i=0;i<256;++i) {
      const auto &p=points[(i*19+n*13)%points.size()];
      queries[i]={p.x+(i%2 ? .13 : .015),p.y+(i%3 ? .006 : -.013),0};
    }
    outputs.clear();size_t examined_batch=0,visited_batch=0,matches_batch=0;
    const auto begin=std::chrono::steady_clock::now();const double cpu_begin=cpuNs();
    if(rebuild)index.rebuild(points);
    for(const auto &q:queries) {
      const bool matched=index.hasWithin(points,q,.07);outputs.push_back(matched);matches_batch+=matched;
      const auto slot=outputs.size();outputs.push_back(0);
      const auto count=index.forEachNearIndexed(points,q.x,q.y,.12,[&](const Point &,size_t id){outputs.push_back(id);++visited_batch;});
      outputs[slot]=count;examined_batch+=count;
    }
    const double cpu_elapsed=cpuNs()-cpu_begin;
    const double wall_elapsed=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-begin).count();
    if(iteration>=0) {
      cpu.push_back(cpu_elapsed);wall.push_back(wall_elapsed);
      for(auto word:outputs)hashWord(checksum,word);
      examined+=examined_batch;visited+=visited_batch;matches+=matches_batch;
    }
  }
  auto stats=[](std::vector<double> values) {
    double sum=0;for(double v:values)sum+=v;std::sort(values.begin(),values.end());
    return std::vector<double>{sum/values.size()/1e6,values[static_cast<size_t>(std::ceil(values.size()*.95))-1]/1e6,values.back()/1e6};
  };
  const auto c=stats(cpu),w=stats(wall);size_t cells=0;
  if constexpr(std::is_same_v<Index,SpatialPointIndex>)cells=index.denseStorageCells();
  std::cout<<std::setprecision(12)<<"{\"arm\":\""<<arm<<"\",\"mode\":\""<<mode<<"\",\"points\":"<<count
    <<",\"cycles\":"<<cycles<<",\"warmup\":"<<warmup<<",\"queries_per_batch\":256,\"dense_cells\":"<<cells
    <<",\"cpu_mean_ms\":"<<c[0]<<",\"cpu_p95_ms\":"<<c[1]<<",\"cpu_max_ms\":"<<c[2]
    <<",\"wall_mean_ms\":"<<w[0]<<",\"wall_p95_ms\":"<<w[1]<<",\"wall_max_ms\":"<<w[2]
    <<",\"examined\":"<<examined<<",\"visited\":"<<visited<<",\"matches\":"<<matches<<",\"output_fingerprint\":"<<checksum<<"}\n";
  return 0;
}
int main(int argc,char **argv) {
  if(argc!=4){std::cerr<<"Usage: baseline|candidate compact_queries|compact_rebuild_query|sparse_rebuild_query corpus-out\n";return 2;}
  const std::string arm=argv[1],mode=argv[2];
  if((arm!="baseline"&&arm!="candidate")||(mode!="compact_queries"&&mode!="compact_rebuild_query"&&mode!="sparse_rebuild_query"))return 2;
  return arm=="candidate" ? run<SpatialPointIndex>(arm,mode,argv[3]) : run<ReferenceSpatialPointIndex>(arm,mode,argv[3]);
}
