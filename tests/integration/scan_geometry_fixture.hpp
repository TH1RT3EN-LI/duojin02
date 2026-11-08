// Small original-data component fixture. No ground truth is read or required.
#ifndef DUOJIN02_TEST_SCAN_GEOMETRY_FIXTURE_HPP_
#define DUOJIN02_TEST_SCAN_GEOMETRY_FIXTURE_HPP_
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "duojin02_scan_geometry/geometry.hpp"

struct RecordedScanFixture
{
  int64_t stamp;
  std::string frame;
  double angle_min, angle_increment, time_increment, range_min, range_max;
  std::vector<float> ranges;
  duojin02_scan_geometry::History history;
  explicit RecordedScanFixture(const std::string & path)
  {
    std::ifstream input(path);
    if (!input) {throw std::runtime_error("cannot read real scan fixture: " + path);}
    std::string line, kind;
    size_t count;
    std::getline(input, line);
    std::istringstream header(line);
    header >> kind >> stamp >> frame >> angle_min >> angle_increment >> time_increment >>
      range_min >> range_max >> count;
    if (!header || kind != "SCAN") {throw std::runtime_error("invalid scan fixture header");}
    while (std::getline(input, line)) {
      std::istringstream fields(line); fields >> kind;
      if (kind == "R") {
        std::string text; fields >> text;
        ranges.push_back(static_cast<float>(std::stod(text)));
      } else if (kind == "TF") {
        std::string parent, child;
        int64_t time; duojin02_scan_geometry::Pose pose; int mirror, fixed;
        fields >> parent >> child >> time >> pose.x >> pose.y >> pose.yaw >> mirror >> fixed;
        if (!fields) {throw std::runtime_error("invalid TF fixture row");}
        pose.mirrored = mirror;
        history.insert(parent, child, time, pose, fixed, 120000000000LL);
      } else {throw std::runtime_error("unexpected fixture record");}
    }
    if (ranges.size() != count) {throw std::runtime_error("fixture beam count changed");}
  }
  duojin02_scan_geometry::TimedScan prepare(double fraction) const
  {
    duojin02_scan_geometry::Config config;
    config.enabled = true; config.reference_fraction = fraction;
    return duojin02_scan_geometry::prepare(stamp, frame, angle_min, angle_increment,
      time_increment, range_min, range_max, ranges, config, history, "odom", "base_footprint");
  }
};
#endif
