#include <karto_sdk/Mapper.h>
#include <nav2_map_server/map_io.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <cmath>
#include <stdexcept>
namespace fs = std::filesystem;
void require(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
std::string bytes(const fs::path & path) {std::ifstream f(path, std::ios::binary); return {std::istreambuf_iterator<char>(f), {}};}
nav2_map_server::SaveParameters parameters(const fs::path & prefix) {
  nav2_map_server::SaveParameters p;
  p.map_file_name=prefix.string(); p.image_format="pgm"; p.mode=nav2_map_server::MapMode::Trinary;
  p.free_thresh=0.19600000000000004; p.occupied_thresh=0.65000000000000013; return p;
}
void check(const nav_msgs::msg::OccupancyGrid & map, const fs::path & prefix) {
  const auto p=parameters(prefix);
  require(nav2_map_server::saveMapToFile(map,p),"native save failed");
  const auto metadata=YAML::LoadFile(prefix.string()+".yaml");
  require(metadata["origin"][0].as<double>()==map.info.origin.position.x,"origin.x double roundtrip failed");
  require(metadata["origin"][1].as<double>()==map.info.origin.position.y,"origin.y double roundtrip failed");
  require(metadata["resolution"].as<float>()==map.info.resolution,"resolution float roundtrip failed");
  require(metadata["free_thresh"].as<double>()==p.free_thresh,"free threshold double roundtrip failed");
  require(metadata["occupied_thresh"].as<double>()==p.occupied_thresh,"occupied threshold double roundtrip failed");
  nav_msgs::msg::OccupancyGrid loaded;
  require(nav2_map_server::loadMapFromYaml(prefix.string()+".yaml",loaded)==nav2_map_server::LOAD_MAP_SUCCESS,"native load failed");
  require(loaded.info.resolution==map.info.resolution,"loader float resolution changed");
  require(loaded.info.width==map.info.width && loaded.info.height==map.info.height,"dimensions changed");
  require(loaded.info.origin.position.x==map.info.origin.position.x && loaded.info.origin.position.y==map.info.origin.position.y,"loader double origin changed");
  require(loaded.data==map.data,"trinary occupancy or vertical flip changed");
  auto yaw=[](const geometry_msgs::msg::Quaternion & q) {double y,p,r;tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z,q.w)).getEulerYPR(y,p,r);return y;};
  const double angle_error=std::remainder(yaw(loaded.info.origin.orientation)-yaw(map.info.origin.orientation),2*M_PI);
  require(std::abs(angle_error)<8*std::numeric_limits<double>::epsilon(),"physical yaw changed");
  // Identical typed metadata implies identical map coordinates, including corners.
  for (const auto & xy : {std::pair<double,double>{0,0},{map.info.width-0.5,map.info.height-0.5}}) {
    const auto project=[&](const nav_msgs::msg::OccupancyGrid & m) {const double a=yaw(m.info.origin.orientation),x=xy.first*m.info.resolution,y=xy.second*m.info.resolution;return std::pair<double,double>{m.info.origin.position.x+std::cos(a)*x-std::sin(a)*y,m.info.origin.position.y+std::sin(a)*x+std::cos(a)*y};};
    auto a=project(map),b=project(loaded); const double bound=32*std::numeric_limits<double>::epsilon()*(1+std::abs(a.first)+std::abs(a.second));
    require(std::hypot(a.first-b.first,a.second-b.second)<=bound,"cell coordinates changed beyond propagated roundoff");
  }
}
int main(int argc,char ** argv) {
  try {
    const bool synthetic_only = argc == 3 && std::string(argv[1]) == "--synthetic-only";
    require(synthetic_only || argc >= 4, "usage: metadata_roundtrip --synthetic-only output-directory OR graph-prefix output-directory legacy-pgm-or-write-only");
    fs::create_directories(argv[2]);
    nav_msgs::msg::OccupancyGrid map;
    if (!synthetic_only) {
    karto::Mapper m; m.LoadFromFile(std::string(argv[1])+".posegraph");
    karto::Dataset d; d.LoadFromFile(std::string(argv[1])+".data");
    for (auto * p : d.GetLasers()) {karto::SensorManager::GetInstance()->RegisterSensor(dynamic_cast<karto::Sensor *>(p),true);}
    std::unique_ptr<karto::OccupancyGrid> g(karto::OccupancyGrid::CreateFromScans(m.GetAllProcessedScans(),.01,(kt_int32u)m.getParamMinPassThrough(),(kt_double)m.getParamOccupancyThreshold()));
    auto o=g->GetCoordinateConverter()->GetOffset();
    map.info.width=g->GetWidth();map.info.height=g->GetHeight();map.info.resolution=.01;map.info.origin.position.x=o.GetX();map.info.origin.position.y=o.GetY();map.info.origin.orientation.w=1;
    for(int y=0;y<g->GetHeight();++y)for(int x=0;x<g->GetWidth();++x){auto v=g->GetValue(karto::Vector2<kt_int32s>(x,y));map.data.push_back(v==karto::GridStates_Occupied?100:v==karto::GridStates_Free?0:-1);}
    const fs::path prefix=fs::path(argv[2])/"frozen";
    if(std::string(argv[3])=="--write-only") {require(nav2_map_server::saveMapToFile(map,parameters(prefix)),"legacy save failed");return 0;}
    check(map,prefix);require(bytes(prefix.string()+".pgm")==bytes(argv[3]),"PGM differs from native legacy saver for same frozen graph");
    }
    const double origins[][3]={{-26.125937054897271,-12.140692030483075,0},{1e8+0.12345679,-1e8-0.9876543,.371234567890123},{-1e-8,1e-9,-2.1345678901234},{123.456789012345,-234.567890123456,3.14}};
    for (int n=0;n<4;++n) {
      map.info.width=n%2?4:3;map.info.height=n%2?3:2;map.info.resolution=std::nextafter(0.01f,n%2?1.f:0.f);map.info.origin.position.x=origins[n][0];map.info.origin.position.y=origins[n][1];tf2::Quaternion q;q.setRPY(0,0,origins[n][2]);map.info.origin.orientation.x=q.x();map.info.origin.orientation.y=q.y();map.info.origin.orientation.z=q.z();map.info.origin.orientation.w=q.w();map.data.clear();for(unsigned i=0;i<map.info.width*map.info.height;++i)map.data.push_back(i%3==0?-1:i%3==1?0:100);
      check(map,fs::path(argv[2])/("synthetic_"+std::to_string(n)));
    }
    auto p=parameters(fs::path(argv[2])/"invalid");
    map.info.resolution=0;require(!nav2_map_server::saveMapToFile(map,p),"zero resolution accepted");
    map.info.resolution=.01f;map.info.origin.position.x=std::numeric_limits<double>::quiet_NaN();require(!nav2_map_server::saveMapToFile(map,p),"NaN origin accepted");
    map.info.origin.position.x=0;p.free_thresh=std::numeric_limits<double>::infinity();require(!nav2_map_server::saveMapToFile(map,p),"infinite threshold accepted");
    p=parameters(fs::path(argv[2])/"missing"/"bad");require(!nav2_map_server::saveMapToFile(map,p),"unwritable path accepted");
    std::cout << "native_saver_loader_roundtrip=passed; synthetic_cases=4; invalid_cases=4; "
      << (synthetic_only ? "frozen_pgm_byte_identical=not_run; legacy_linkinfo_load=not_run" :
        "frozen_pgm_byte_identical=passed; legacy_linkinfo_load=passed") << '\n';
  }catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
