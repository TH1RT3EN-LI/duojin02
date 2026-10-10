#include "duojin01_navigation/gpu_backend.hpp"
#include <cassert>
#include <iostream>
#include <random>
#include <limits>
using namespace duojin01_navigation;
template<class F> void rejects(F &&f) {bool failed=false;try{f();}catch(const std::exception &){failed=true;}assert(failed);}
int main(int argc,char **argv) {
  const bool required=argc>1 && std::string(argv[1])=="--require-cuda";
  GeometryCompute cpu;
  GeometryCompute automatic({ComputeBackend::Auto,0,2000});
  if(required && !automatic.available()){std::cout<<"CUDA device unavailable\n";return 77;}
  Polygon polygon{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},
    {-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  std::mt19937_64 random(20261010);std::uniform_real_distribution<double> xy(-3.,3.),angle(-3.14159,3.14159);
  size_t comparisons=0;
  for(int scene=0;scene<18;++scene) {
    std::vector<Point> points;std::vector<Pose> poses;
    for(int i=0;i<1000;++i)points.push_back({xy(random),xy(random),.007071067812+(i%7)*.0005});
    points.push_back({0,0,.001});points.push_back({.223,0,0});
    for(int i=0;i<192;++i)poses.push_back({xy(random),xy(random),angle(random)});
    poses.push_back({0,0,0});poses.push_back({.5,0,0});poses.push_back({0,0,1.5707963267948966});
    cpu.setScene(polygon,points);automatic.setScene(polygon,points);
    const auto a=cpu.evaluate(poses,.010,true),b=automatic.evaluate(poses,.010,true);
    assert(a.collisions==b.collisions);
    if(required)assert(b.used_cuda);
    for(size_t i=0;i<poses.size();++i){assert(std::abs(a.clearances[i]-b.clearances[i])<1e-10);++comparisons;}
    // New scene epochs must replace the previous device data completely.
    points.clear();cpu.setScene(polygon,points);automatic.setScene(polygon,points);
    const auto empty=automatic.evaluate(poses,.010,true);
    for(size_t i=0;i<poses.size();++i){assert(!empty.collisions[i]);assert(empty.clearances[i]==.080);}
  }
  // Test the exact segment-distance contact and adjacent ULPs independently
  // of random data. Boundary rows must agree with the original CPU helper.
  const Polygon square{{-.2,-.2},{.2,-.2},{.2,.2},{-.2,.2}};
  for(double x:{.210,std::nextafter(.210,0.),std::nextafter(.210,1.),.210-1e-9,.210+1e-9}) {
    std::vector<Point> points{{x,0,0}};std::vector<Pose> poses{{0,0,0}};
    cpu.setScene(square,points);automatic.setScene(square,points);
    const auto a=cpu.evaluate(poses,.010,true),b=automatic.evaluate(poses,.010,true);
    assert(a.collisions==b.collisions);assert(std::abs(a.clearances[0]-b.clearances[0])<1e-10);++comparisons;
  }
  std::vector<Point> points{{0,0,0}};automatic.setScene(square,points);
  rejects([&]{automatic.evaluate({{0,0,std::numeric_limits<double>::quiet_NaN()}},.010);});
  rejects([&]{automatic.evaluate({{}},-.1);});
  rejects([&]{automatic.evaluate({{}},.01,true,0);});
  rejects([&]{automatic.setScene({},points);});
  rejects([&]{automatic.setScene({{-.2,-.2},{-.2,.2},{.2,.2},{.2,-.2}},points);});
  rejects([&]{automatic.setScene({{0,0},{1,0},{2,0}},points);});
  rejects([&]{automatic.setScene(square,{{0,0,-.1}});});
  GeometryCompute sparse({ComputeBackend::Auto,std::numeric_limits<uint64_t>::max(),2000});
  sparse.setScene(square,points);assert(!sparse.willUseCuda(1));assert(!sparse.evaluate({{}},.010).used_cuda);
  assert(sparse.statistics().cuda_batches==0);
  rejects([]{parseComputeBackend("typo");});
  rejects([]{GeometryCompute invalid({ComputeBackend::Cpu,0,-1});});
  if(!automatic.available()) {
    bool rejected=false;
    try{GeometryCompute forced({ComputeBackend::Cuda,0,2000});}catch(const std::runtime_error &){rejected=true;}
    assert(rejected);
  }
  const auto &stats=automatic.statistics();
  std::cout<<"GEOMETRY_CHECKS comparisons="<<comparisons<<" device="<<automatic.deviceName()
    <<" cuda_batches="<<stats.cuda_batches<<" boundary_rechecks="<<stats.boundary_rechecks<<" passed\n";
}
