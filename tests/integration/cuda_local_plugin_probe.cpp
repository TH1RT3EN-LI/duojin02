// Installed-plugin parity and complete public-call timing on frozen synthetic maps.
// This fixture neither launches a base driver nor certifies closed-loop navigation.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

geometry_msgs::msg::PoseStamped pose(double x = 0, double y = 0) {
  geometry_msgs::msg::PoseStamped value;
  value.header.frame_id = "odom";
  value.pose.position.x = x;
  value.pose.position.y = y;
  value.pose.orientation.w = 1;
  return value;
}

double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

Json quantiles(std::vector<double> values) {
  require(!values.empty(), "No public controller calls were measured");
  std::sort(values.begin(), values.end());
  auto q = [&](double fraction) {
    return values[static_cast<size_t>(std::ceil(fraction * values.size())) - 1];
  };
  return {{"calls", values.size()}, {"p50_ms", q(.5)}, {"p95_ms", q(.95)},
          {"p99_ms", q(.99)}, {"max_ms", values.back()}};
}

std::set<std::string> mappedLibraries(const std::string &expected_prefix) {
  std::ifstream maps("/proc/self/maps");
  std::set<std::string> result;
  for (std::string line; std::getline(maps, line);) {
    if (line.find("libprecise_mppi.so") == std::string::npos &&
        line.find("libduojin02_gpu_geometry.so") == std::string::npos)
      continue;
    const auto begin = line.find('/');
    require(begin != std::string::npos, "Mapped plugin has no absolute path");
    const auto path = line.substr(begin);
    require(path.rfind(expected_prefix + "/", 0) == 0,
            "Loaded a library outside the selected installation: " + path);
    result.insert(path);
  }
  require(result.size() == 2, "Expected both installed controller and geometry libraries");
  return result;
}

struct Case {
  std::string name;
  double goal_x, goal_y, measured_x;
  bool dense, narrow, expect_stop;
  double blocker_x;
};

const std::vector<Case> cases = {
    {"empty_direct", .35, 0, 0, false, false, false, 0},
    {"dense_direct", .35, 0, 0, true, false, false, 0},
    {"narrow_centered", .8, 0, 0, true, true, false, 0},
    {"centered_braking_batch", 1., 0, 0, false, true, false, .45},
    {"measured_braking_early_stop", 0, .30, .25, false, false, true, .30},
    {"requested_braking_scale", .195, 0, 0, false, false, false, .44}};

void fillMap(nav2_costmap_2d::Costmap2D &map, const Case &fixture) {
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  map.resetMap(0, 0, map.getSizeInCellsX(), map.getSizeInCellsY());
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y)
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      const double inner = fixture.narrow ? .25 : .45;
      const bool band = fixture.dense && std::abs(wx) < 1.15 &&
                        std::abs(wy) >= inner && std::abs(wy) < .72;
      const bool blocker = fixture.blocker_x > 0 &&
                           std::abs(wx - fixture.blocker_x) <= .006 && std::abs(wy) < .025;
      map.setCost(x, y, band || blocker ? 254 : 0);
    }
  if (fixture.narrow) {
    unsigned int x, y;
    require(map.worldToMap(0, 0, x, y), "Fixture origin lies outside map");
    map.setCost(x, y, 120);
  }
}

struct Arm {
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  std::shared_ptr<nav2_core::Controller> controller;
  nav2_controller::SimpleGoalChecker checker;
};

Arm createArm(pluginlib::ClassLoader<nav2_core::Controller> &loader,
              const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &map,
              const std::string &parameters, bool gpu, bool force_cuda = false) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", parameters});
  options.append_parameter_override("FollowPath.use_gpu", gpu);
  if (force_cuda)
    options.append_parameter_override("FollowPath.gpu_minimum_work", 0);
  Arm arm;
  arm.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  arm.tf = std::make_shared<tf2_ros::Buffer>(arm.node->get_clock());
  arm.listener = std::make_shared<tf2_ros::TransformListener>(*arm.tf);
  arm.controller = loader.createSharedInstance("duojin02_navigation::PreciseMPPI");
  arm.controller->configure(arm.node, "FollowPath", arm.tf, map);
  arm.controller->activate();
  arm.checker.initialize(arm.node, "local_probe_goal", map);
  return arm;
}

std::vector<double> command(Arm &arm, const Case &fixture, bool set_plan, double &elapsed) {
  auto start = pose();
  start.header.stamp = arm.node->now();
  if (set_plan) {
    nav_msgs::msg::Path path;
    path.header = start.header;
    for (int i = 0; i <= 100; ++i) {
      auto p = pose(fixture.goal_x * i / 100, fixture.goal_y * i / 100);
      p.header.stamp = start.header.stamp;
      path.poses.push_back(p);
    }
    arm.controller->setPlan(path);
  }
  geometry_msgs::msg::Twist measured;
  measured.linear.x = fixture.measured_x;
  const auto begin = Clock::now();
  const auto result = arm.controller->computeVelocityCommands(start, measured, &arm.checker).twist;
  elapsed = milliseconds(begin);
  const std::vector<double> values{result.linear.x, result.linear.y, result.angular.z};
  for (double value : values)
    require(std::isfinite(value), "Controller emitted a non-finite command");
  const double norm = std::hypot(values[0], values[1]) + std::abs(values[2]);
  require(fixture.expect_stop ? norm == 0 : norm > 0,
          "Unexpected stop/motion in fixture " + fixture.name);
  return values;
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: cuda_local_plugin_probe NAV2_YAML EXPECTED_INSTALL_PREFIX OUTPUT_JSON\n";
    return 2;
  }
  const std::string parameters = argv[1], prefix = argv[2], output = argv[3];
  rclcpp::init(argc, argv);
  try {
    auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cuda_local_fixture_costmap");
    map->set_parameter(rclcpp::Parameter("global_frame", std::string("odom")));
    map->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_footprint")));
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
    map->declare_parameter("inflation_layer.plugin", std::string("nav2_costmap_2d::InflationLayer"));
    map->set_parameter(rclcpp::Parameter("footprint", std::string(
        "[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],"
        "[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
    map->set_parameter(rclcpp::Parameter("footprint_padding", .008));
    require(map->on_configure(rclcpp_lifecycle::State()) ==
                nav2_util::CallbackReturn::SUCCESS, "Costmap configure failed");
    map->getCostmap()->resizeMap(600, 600, .01, -3, -3);
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    const auto configure_begin = Clock::now();
    auto cpu = createArm(loader, map, parameters, false);
    auto gpu = createArm(loader, map, parameters, true, true);
    auto default_gpu = createArm(loader, map, parameters, true);
    Json report{{"schema_version", 1}, {"scope", "installed_plugin_synthetic_static_inputs"},
                {"configure_three_arms_ms", milliseconds(configure_begin)},
                {"default_gpu_minimum_work", default_gpu.node->get_parameter("FollowPath.gpu_minimum_work").as_int()},
                {"maps", mappedLibraries(prefix)}, {"force_gpu_minimum_work", 0},
                {"timing_scope", "whole_computeVelocityCommands_including_snapshot_and_validation"},
                {"cases", Json::array()}};
    double maximum_error = 0;
    for (const auto &fixture : cases) {
      fillMap(*map->getCostmap(), fixture);
      double cpu_cold, gpu_cold, default_cold;
      const auto a = command(cpu, fixture, true, cpu_cold);
      const auto b = command(gpu, fixture, true, gpu_cold);
      const auto c = command(default_gpu, fixture, true, default_cold);
      std::vector<double> cpu_times, gpu_times, default_times;
      double case_error = 0;
      for (size_t axis = 0; axis < 3; ++axis) {
        case_error = std::max(case_error, std::abs(a[axis] - b[axis]));
        case_error = std::max(case_error, std::abs(a[axis] - c[axis]));
      }
      for (int repeat = 0; repeat < 24; ++repeat) {
        // Alternate CPU/forced/default and its reverse in an ABBA schedule.
        std::vector<double> ca, cb, cc;
        double ta, tb, tc;
        if ((repeat % 4) == 0 || (repeat % 4) == 3) {
          ca = command(cpu, fixture, false, ta);
          cb = command(gpu, fixture, false, tb);
          cc = command(default_gpu, fixture, false, tc);
        } else {
          cc = command(default_gpu, fixture, false, tc);
          cb = command(gpu, fixture, false, tb);
          ca = command(cpu, fixture, false, ta);
        }
        for (size_t axis = 0; axis < 3; ++axis) {
          case_error = std::max(case_error, std::abs(ca[axis] - cb[axis]));
          case_error = std::max(case_error, std::abs(ca[axis] - cc[axis]));
        }
        cpu_times.push_back(ta);
        gpu_times.push_back(tb);
        default_times.push_back(tc);
      }
      require(case_error <= 1e-12, "CPU/GPU command mismatch: " + fixture.name);
      maximum_error = std::max(maximum_error, case_error);
      report["cases"].push_back({{"name", fixture.name}, {"command_cpu", a},
                                  {"command_gpu", b}, {"cpu_first_ms", cpu_cold},
                                  {"command_default_gpu", c}, {"default_gpu_first_ms", default_cold},
                                  {"gpu_first_ms", gpu_cold}, {"cpu", quantiles(cpu_times)},
                                  {"gpu", quantiles(gpu_times)},
                                  {"default_gpu", quantiles(default_times)},
                                  {"maximum_command_error", case_error}});
    }
    report["maximum_command_error"] = maximum_error;
    for (auto *arm : {&cpu, &gpu, &default_gpu}) {
      arm->controller->deactivate();
      arm->controller->cleanup();
      arm->controller.reset();
    }
    map->on_cleanup(rclcpp_lifecycle::State());
    std::ofstream file(output);
    require(file.good(), "Cannot write probe JSON");
    file << report.dump(2) << '\n';
    std::cout << "INSTALLED_PLUGIN_PARITY cases=" << cases.size()
              << " max_command_error=" << maximum_error << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "INSTALLED_PLUGIN_PROBE_FAILED " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
