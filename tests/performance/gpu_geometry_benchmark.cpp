#include "duojin02_navigation/gpu_geometry.hpp"
#include <chrono>
#include <iostream>
#include <random>

using namespace duojin02_navigation;
int main() {
  const Polygon polygon{{-.214,-.203},{.213,-.203},{.213,.203},{-.214,.203}};
  std::mt19937_64 random(20261010);
  std::uniform_real_distribution<double> xy(-2.,2.);
  std::vector<Point> points;
  std::vector<Pose> poses;
  for (int i=0;i<8000;++i) points.push_back({xy(random),xy(random),.007071067812});
  for (int i=0;i<256;++i) poses.push_back({xy(random),xy(random),xy(random)});
  GeometryCompute cpu({false,0}), gpu({true,0});
  cpu.setScene(polygon,points);gpu.setScene(polygon,points);
  const auto reference=cpu.evaluate(poses,.010);
  const auto warm=gpu.evaluate(poses,.010);
  if(reference.collisions!=warm.collisions) return 1;
  std::cout<<"device="<<gpu.deviceName()<<" cuda="<<warm.used_cuda<<'\n';
  for (auto *backend : {&cpu,&gpu}) {
    const auto start=std::chrono::steady_clock::now();
    for(int i=0;i<10;++i) {
      const auto result=backend->evaluate(poses,.010);
      if(result.collisions!=reference.collisions) return 1;
    }
    std::cout<<(backend==&cpu ? "cpu" : "gpu_requested")<<" mean_ms="
      <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/10<<'\n';
  }
}
