#include "slam_toolbox/laser_utils.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
static void require(bool value){if(!value)throw std::runtime_error("Laser inversion sequence mismatch");}
int main(int argc,char ** argv){
  if(argc!=2)return 2;laser_utils::LaserMetadata metadata(nullptr,true);sensor_msgs::msg::LaserScan scan;
  scan.ranges={1,2,3,4};scan.intensities={10,20,30,40};metadata.invertScan(scan);require(scan.ranges==std::vector<float>({4,3,2,1})&&scan.intensities==std::vector<float>({40,30,20,10}));
  scan.ranges={5,6,7};scan.intensities.clear();metadata.invertScan(scan);require(scan.ranges==std::vector<float>({7,6,5})&&scan.intensities.empty());
  scan.ranges={8,9,10};scan.intensities={11,12};metadata.invertScan(scan);require(scan.ranges==std::vector<float>({10,9,8})&&scan.intensities==std::vector<float>({12,11}));
  scan.ranges.clear();scan.intensities.clear();metadata.invertScan(scan);require(scan.ranges.empty()&&scan.intensities.empty());
  scan.ranges={42};scan.intensities={43};metadata.invertScan(scan);require(scan.ranges==std::vector<float>({42})&&scan.intensities==std::vector<float>({43}));
  const auto directory=std::filesystem::path(argv[1]);std::filesystem::create_directories(directory);std::ofstream(directory/"result.json")<<"{\"passed\":true,\"endpoints\":true,\"empty_intensities\":true,\"short_intensities_independent\":true,\"empty_and_singleton\":true,\"instrumentation\":\"ASan+UBSan; real laser_utils.cpp compilation\"}\n";
}
