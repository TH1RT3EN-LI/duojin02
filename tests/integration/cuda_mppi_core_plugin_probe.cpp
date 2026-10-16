// Actual installed MPPI public-call replay. No chassis driver is launched.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <exception>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <cuda_runtime_api.h>
#include <nlohmann/json.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <duojin02_mppi_controller/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Controller = duojin02_mppi_controller::MPPIController;

namespace {
void require(bool ok, const std::string &reason) { if (!ok) throw std::runtime_error(reason); }
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
Json quantiles(std::vector<double> values) {
  require(!values.empty(), "Empty timing sequence");
  std::sort(values.begin(), values.end());
  auto q = [&](double f) {return values[std::max<size_t>(1, std::ceil(f * values.size())) - 1];};
  return {{"calls", values.size()}, {"p50_ms", q(.5)}, {"p95_ms", q(.95)}, {"p99_ms", q(.99)}, {"max_ms", q(1)}};
}
geometry_msgs::msg::PoseStamped pose(double x = 0, double y = 0, double yaw = 0) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2); p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
Json statistics(const duojin_mppi::OptimizerStatistics &s) {
  return {{"cuda_noise_batches", s.cuda_noise_batches}, {"cuda_rollout_batches", s.cuda_rollout_batches},
    {"cuda_update_batches", s.cuda_update_batches}, {"cuda_critic_batches", s.cuda_critic_batches},
    {"cpu_rollout_batches", s.cpu_rollout_batches}, {"cpu_update_batches", s.cpu_update_batches}, {"failures", s.failures}};
}
struct Arm {
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  std::shared_ptr<Controller> controller;
  nav2_controller::SimpleGoalChecker checker;
  bool configured = false, activated = false;

  Arm() = default;
  Arm(const Arm &) = delete;
  Arm &operator=(const Arm &) = delete;
  Arm(Arm &&) = default;
  Arm &operator=(Arm &&) = delete;
  void close() noexcept {
    // Lifecycle cleanup releases callbacks/resources before the plugin loader
    // is destroyed, including when a numerical assertion aborts the replay.
    if (controller) {
      try { if (activated) controller->deactivate(); } catch (...) {}
      try { if (configured) controller->cleanup(); } catch (...) {}
      controller.reset();
    }
    activated = configured = false;
  }
  ~Arm() {
    // TransformListener owns a spinning thread. Wake it before member teardown
    // if an assertion/plugin exception aborts the probe.
    if (std::uncaught_exceptions()) rclcpp::shutdown();
    close();
  }
};
struct CostmapCleanup {
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  ~CostmapCleanup() {
    try { if (map) map->on_cleanup(rclcpp_lifecycle::State()); } catch (...) {}
  }
};
Arm createArm(pluginlib::ClassLoader<nav2_core::Controller> &loader,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &map,
  const std::string &yaml, const std::string &backend, bool precise = false,
  bool geometry_gpu = true)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", yaml});
  options.append_parameter_override("FollowPath.optimizer_backend", backend);
  options.append_parameter_override("FollowPath.optimizer_seed", 20261016);
  options.append_parameter_override("FollowPath.reset_period", 1000.0);
  options.append_parameter_override("FollowPath.regenerate_noises", false);
  options.append_parameter_override("FollowPath.visualize", false);
  options.append_parameter_override("FollowPath.use_gpu", backend == "cuda" && geometry_gpu);
  Arm arm;
  arm.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  arm.tf = std::make_shared<tf2_ros::Buffer>(arm.node->get_clock());
  arm.listener = std::make_shared<tf2_ros::TransformListener>(*arm.tf);
  auto plugin = loader.createSharedInstance(precise ? "duojin02_navigation::PreciseMPPI" :
    "duojin02_mppi_controller::MPPIController");
  arm.controller = std::dynamic_pointer_cast<Controller>(plugin);
  require(static_cast<bool>(arm.controller), "Loaded plugin is not the CUDA-capable MPPI implementation");
  arm.controller->configure(arm.node, "FollowPath", arm.tf, map);
  arm.configured = true;
  arm.controller->activate();
  arm.activated = true;
  arm.checker.initialize(arm.node, "replay_goal", map);
  return arm;
}
nav_msgs::msg::Path path(double x, double y, double curve = 0) {
  nav_msgs::msg::Path result;
  result.header.frame_id = "odom";
  for (int i = 0; i <= 150; ++i) {
    double f = i / 150.;
    result.poses.push_back(pose(x * f, y * f + curve * std::sin(M_PI * f)));
  }
  return result;
}
std::vector<double> command(Arm &arm, geometry_msgs::msg::PoseStamped p,
  const geometry_msgs::msg::Twist &velocity, double &milliseconds)
{
  p.header.stamp = arm.node->now();
  const auto start = Clock::now();
  const auto twist = arm.controller->computeVelocityCommands(p, velocity, &arm.checker).twist;
  milliseconds = elapsed(start);
  std::vector<double> result{twist.linear.x, twist.linear.y, twist.angular.z};
  for (auto v : result) require(std::isfinite(v), "Nonfinite public controller command");
  require(result[0] >= -.20001 && result[0] <= .50001 && std::abs(result[1]) <= .30001 &&
    std::abs(result[2]) <= .65001, "MPPI command violates configured velocity bounds");
  return result;
}
Json libraryMaps(const std::string &prefix) {
  std::ifstream input("/proc/self/maps");
  std::set<std::string> paths;
  for (std::string line; std::getline(input, line);) {
    if (line.find("mppi") == std::string::npos || line.find(".so") == std::string::npos) continue;
    auto start = line.find('/');
    require(start != std::string::npos, "MPPI shared object has no path");
    const auto p = line.substr(start);
    require(p.rfind(prefix + "/", 0) == 0, "MPPI library loaded outside selected prefix: " + p);
    paths.insert(p);
  }
  require(paths.size() >= 2, "Expected installed MPPI optimizer/controller and critic libraries");
  return paths;
}
}  // namespace

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: cuda_mppi_core_plugin_probe NAV2_YAML INSTALL_PREFIX OUTPUT_JSON\n";
    return 2;
  }
  const std::string yaml = argv[1], prefix = argv[2], output = argv[3];
  rclcpp::init(argc, argv);
  Json report{{"schema_version", 1}, {"status", "running"},
    {"scope", "actual_installed_mppi_plugin_common_input_replay_and_kinematic_smoke"},
    {"timing_scope", "whole_computeVelocityCommands_including_path_transform_native_critics_cuda_copies_sync_and_control_update"},
    {"seed", 20261016}, {"maximum_command_delta_tolerance", .005},
    {"fixture_costmap", {{"resolution_m", .01}, {"size_cells", {800, 800}},
      {"origin_m", {-4., -4.}}, {"footprint_padding_m", .008},
      {"inflation_layer_resolution_matches_grid", true},
      {"cell_values", "synthetic free interior and lethal outer bands; no costmap update/publish cycle"}}},
    {"kinematic_acceptance_scope", "entry into position/yaw envelope only; stopped speed, dwell and Nav2 Action success are not evaluated"},
    {"limitations", {"Synthetic static costmap; no sensor transport, costmap updater, base driver or physical robot",
      "Kinematic smoke uses ideal command integration; it is not a full Nav2 graph or physical collision certificate"}}};
  try {
    auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cuda_mppi_core_fixture_costmap");
    map->set_parameter(rclcpp::Parameter("global_frame", std::string("odom")));
    map->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_footprint")));
    // Configure the layer at the final resolution before allocating the larger
    // fixture. Resizing only Costmap2D later leaves InflationLayer's cached
    // resolution inconsistent and disables the native critic's fast check.
    map->set_parameter(rclcpp::Parameter("resolution", .01));
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
    map->declare_parameter("inflation_layer.plugin", std::string("nav2_costmap_2d::InflationLayer"));
    map->set_parameter(rclcpp::Parameter("footprint", std::string(
      "[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],"
      "[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
    map->set_parameter(rclcpp::Parameter("footprint_padding", .008));
    require(map->on_configure(rclcpp_lifecycle::State()) == nav2_util::CallbackReturn::SUCCESS, "Costmap configure failed");
    CostmapCleanup map_cleanup{map};
    map->getCostmap()->resizeMap(800, 800, .01, -4., -4.);
    auto *grid = map->getCostmap();
    for (unsigned int y = 0; y < grid->getSizeInCellsY(); ++y)
      for (unsigned int x = 0; x < grid->getSizeInCellsX(); ++x) {
        double wx, wy; grid->mapToWorld(x, y, wx, wy);
        unsigned char cost = std::abs(wy) > 3.0 || std::abs(wx) > 3.5 ? 254 : 0;
        grid->setCost(x, y, cost);
      }
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    const auto configure_start = Clock::now();
    auto cpu = createArm(loader, map, yaml, "cpu"), gpu = createArm(loader, map, yaml, "cuda");
    report["configure_two_arms_ms"] = elapsed(configure_start);
    report["mapped_libraries"] = libraryMaps(prefix);
    report["cases"] = Json::array();
    double maximum_delta = 0.;
    const std::vector<std::vector<double>> fixtures{{2.5, 0, 0}, {1.5, 1.3, 0}, {2.5, 0, .35}};
    for (size_t fixture = 0; fixture < fixtures.size(); ++fixture) {
      const auto &f = fixtures[fixture];
      cpu.controller->reset(); gpu.controller->reset();
      const auto plan = path(f[0], f[1], f[2]);
      cpu.controller->setPlan(plan); gpu.controller->setPlan(plan);
      Json rows = Json::array();
      std::vector<double> cpu_times, gpu_times;
      for (int call = 0; call < 32; ++call) {
        // The supplied pose and measured velocity remain common inputs. Warm
        // control sequences evolve separately, as in actual repeated calls.
        const double fraction = .2 * call / 31.;
        auto p = pose(f[0] * fraction, f[1] * fraction + f[2] * std::sin(M_PI * fraction));
        geometry_msgs::msg::Twist measured;
        measured.linear.x = call ? .15 : 0.;
        double a_ms, b_ms;
        std::vector<double> a, b;
        if (call % 4 == 0 || call % 4 == 3) {
          a = command(cpu, p, measured, a_ms); b = command(gpu, p, measured, b_ms);
        } else {
          b = command(gpu, p, measured, b_ms); a = command(cpu, p, measured, a_ms);
        }
        double delta = 0.;
        for (size_t axis = 0; axis < 3; ++axis) delta = std::max(delta, std::abs(a[axis] - b[axis]));
        maximum_delta = std::max(maximum_delta, delta);
        rows.push_back({{"call", call}, {"pose", {p.pose.position.x, p.pose.position.y, 0.}},
          {"command_cpu", a}, {"command_cuda", b}, {"cpu_ms", a_ms}, {"cuda_ms", b_ms}, {"maximum_delta", delta}});
        if (call > 0) {cpu_times.push_back(a_ms); gpu_times.push_back(b_ms);}
      }
      report["cases"].push_back({{"fixture", fixture}, {"path_endpoint", f}, {"rows", rows},
        {"cpu_warm", quantiles(cpu_times)}, {"cuda_warm", quantiles(gpu_times)}});
    }
    report["maximum_command_delta"] = maximum_delta;
    report["cpu_statistics"] = statistics(cpu.controller->optimizerStatistics());
    report["cuda_statistics"] = statistics(gpu.controller->optimizerStatistics());
    require(maximum_delta <= .005, "CPU/CUDA actual-plugin commands exceeded declared common-input tolerance");
    const auto stats = gpu.controller->optimizerStatistics();
    require(stats.cuda_rollout_batches > 0 && stats.cuda_update_batches > 0 &&
      stats.cuda_noise_batches > 0 && stats.failures == 0, "Actual public plugin did not complete CUDA core stages");

    report["kinematic_smoke"] = Json::array();
    for (auto *arm : {&cpu, &gpu}) {
      arm->controller->reset(); arm->controller->setPlan(path(2., 0));
      double x = 0, y = 0, yaw = 0;
      geometry_msgs::msg::Twist velocity;
      Json trajectory = Json::array();
      bool reached = false;
      for (int i = 0; i < 240; ++i) {
        double ms;
        const auto u = command(*arm, pose(x, y, yaw), velocity, ms);
        const double c = std::cos(yaw), s = std::sin(yaw);
        x += .05 * (c * u[0] - s * u[1]); y += .05 * (s * u[0] + c * u[1]); yaw += .05 * u[2];
        velocity.linear.x = u[0]; velocity.linear.y = u[1]; velocity.angular.z = u[2];
        trajectory.push_back({{"step", i}, {"pose", {x, y, yaw}}, {"command", u}, {"call_ms", ms}});
        if (std::hypot(2. - x, y) < .08 && std::abs(yaw) < .08) {reached = true; break;}
      }
      report["kinematic_smoke"].push_back({{"backend", arm == &cpu ? "cpu" : "cuda"},
        {"reached_position_yaw_envelope", reached}, {"position_envelope_m", .08}, {"yaw_envelope_rad", .08},
        {"trajectory", trajectory}, {"final_statistics", statistics(arm->controller->optimizerStatistics())}});
      require(reached, "MPPI kinematic smoke failed to enter declared terminal envelope");
    }
    // A path initially 35 cm away exceeds PreciseMPPI's 20 cm tracking capture
    // condition and therefore invokes the actual MPPI optimizer through the
    // production wrapper. The map/plan remain admissible to native MPPI.
    auto wrapper_cpu = createArm(loader, map, yaml, "cpu", true);
    // The geometry backend is disabled in this pair to isolate the optimizer
    // route; the separate geometry plugin probe validates its CUDA branches.
    auto wrapper_gpu = createArm(loader, map, yaml, "cuda", true, false);
    auto offset_plan = path(2., 0);
    for (auto &p : offset_plan.poses) p.pose.position.y = .35;
    wrapper_cpu.controller->setPlan(offset_plan); wrapper_gpu.controller->setPlan(offset_plan);
    Json wrapper_rows = Json::array();
    double wrapper_delta = 0.;
    for (int call = 0; call < 12; ++call) {
      double a_ms, b_ms;
      geometry_msgs::msg::Twist measured;
      auto a = command(wrapper_cpu, pose(), measured, a_ms);
      auto b = command(wrapper_gpu, pose(), measured, b_ms);
      for (size_t axis = 0; axis < 3; ++axis) wrapper_delta = std::max(wrapper_delta, std::abs(a[axis] - b[axis]));
      wrapper_rows.push_back({{"command_cpu", a}, {"command_cuda", b}, {"cpu_ms", a_ms}, {"cuda_ms", b_ms}});
    }
    const auto wrapper_stats = wrapper_gpu.controller->optimizerStatistics();
    report["production_wrapper_sampling"] = {{"fixture", "robot_35cm_from_path_exceeds_direct_tracking_capture"},
      {"rows", wrapper_rows}, {"maximum_command_delta", wrapper_delta},
      {"cpu_statistics", statistics(wrapper_cpu.controller->optimizerStatistics())},
      {"cuda_statistics", statistics(wrapper_stats)}};
    require(wrapper_stats.cuda_rollout_batches >= 12 && wrapper_stats.cuda_update_batches >= 12 &&
      wrapper_stats.failures == 0, "Production wrapper bypassed CUDA optimizer or had a computation failure");
    require(wrapper_delta <= .005, "Production wrapper CPU/CUDA command difference exceeds tolerance");
    report["mapped_libraries"] = libraryMaps(prefix);

    // Destroy only this process's device context, after all timing/trajectory
    // work. Persistent optimizer pointers must fail on the next device call.
    // This probes the production wrapper's actual exception/latching path.
    require(cudaDeviceReset() == cudaSuccess, "Could not inject process-local CUDA context loss");
    double fault_ms, latched_ms;
    geometry_msgs::msg::Twist stationary;
    const auto fault_command = command(wrapper_gpu, pose(), stationary, fault_ms);
    const auto after_fault = wrapper_gpu.controller->optimizerStatistics();
    require(std::all_of(fault_command.begin(), fault_command.end(), [](double x) {return x == 0.;}),
      "Production wrapper emitted motion after optimizer device failure");
    require(after_fault.failures == wrapper_stats.failures + 1 &&
      after_fault.cuda_rollout_batches == wrapper_stats.cuda_rollout_batches &&
      after_fault.cuda_update_batches == wrapper_stats.cuda_update_batches,
      "Device failure was not recorded atomically before a completed optimizer batch");
    // A new clear short plan would normally take the CPU direct-approach route;
    // the latched compute fault must stop that route as well.
    wrapper_gpu.controller->setPlan(path(.1, 0));
    const auto latched_command = command(wrapper_gpu, pose(), stationary, latched_ms);
    const auto after_latched = wrapper_gpu.controller->optimizerStatistics();
    require(std::all_of(latched_command.begin(), latched_command.end(), [](double x) {return x == 0.;}) &&
      statistics(after_fault) == statistics(after_latched),
      "Latched optimizer fault allowed another device attempt or direct-approach motion");
    report["production_wrapper_device_fault"] = {{"injection", "cudaDeviceReset_in_this_process_after_all_normal_trials"},
      {"geometry_gpu", false}, {"first_command", fault_command}, {"subsequent_clear_direct_plan_command", latched_command},
      {"first_call_ms", fault_ms}, {"latched_call_ms", latched_ms},
      {"before", statistics(wrapper_stats)}, {"after_fault", statistics(after_fault)},
      {"after_latched", statistics(after_latched)}, {"passed", true}};
    for (auto *arm : {&cpu, &gpu, &wrapper_cpu, &wrapper_gpu}) {
      arm->close();
    }
    map->on_cleanup(rclcpp_lifecycle::State());
    map_cleanup.map.reset();
    report["status"] = "passed";
    rclcpp::shutdown();
  } catch (const std::exception &error) {
    report["status"] = "failed"; report["error"] = error.what();
    std::cerr << error.what() << '\n';
  }
  std::ofstream stream(output); stream << report.dump(2) << '\n';
  rclcpp::shutdown();
  return report["status"] == "passed" ? 0 : 1;
}
