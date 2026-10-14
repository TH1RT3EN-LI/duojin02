#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/slam_toolbox_common.hpp>

class ScanGateProbe : public slam_toolbox::SlamToolbox {
public:
  using slam_toolbox::SlamToolbox::SlamToolbox;

  void initialize() {
    setParams();
  }

  void minimumDistance(double value) {
    smapper_->getMapper()->setParamMinimumTravelDistance(value);
  }

  bool accepts(double x, double y, double yaw, double timestamp) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    const bool result = shouldProcessScan(scan, karto::Pose2(x, y, yaw));
    return result;
  }

protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr) override {}
};

void require(bool condition, const char *description) {
  if (!condition)
    throw std::runtime_error(description);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("minimum_time_interval", 0.05),
                                 rclcpp::Parameter("minimum_travel_distance", 0.1),
                                 rclcpp::Parameter("minimum_travel_heading", 0.1)});
    auto probe = std::make_shared<ScanGateProbe>(options);
    probe->initialize();
    require(probe->accepts(0, 0, 0, 1), "first scan");
    int rotations = 0;
    for (int i = 1; i <= 8; ++i) {
      rotations += probe->accepts(0, 0, i * M_PI / 4, 1.0 + i * 0.2);
    }
    require(rotations == 8, "pure rotation scans were dropped");
    require(!probe->accepts(1, 0, 0, 2.5), "out-of-order stamp was accepted");
    auto second = std::make_shared<ScanGateProbe>(options);
    second->initialize();
    second->minimumDistance(0.05);
    require(second->accepts(0, 0, 3.10, 1), "second instance initial scan");
    require(!second->accepts(0, 0, -3.10, 1.2), "wrapped small yaw falsely accepted");
    require(second->accepts(0, 0, -2.8, 1.4), "wrapped large yaw rejected");
    require(second->accepts(0.06, 0, -2.8, 1.6), "per-instance distance threshold ignored");
    require(!second->accepts(1, 0, -2.8, 1.61), "minimum time interval ignored");
    std::cout << "pure_rotation=8/8; wrapped_heading=passed; per_instance_state=passed; "
                 "per_instance_threshold=passed; stale_time=passed; minimum_interval=passed\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
