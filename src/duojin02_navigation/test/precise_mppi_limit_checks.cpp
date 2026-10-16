#include <cassert>
#include <iostream>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../src/precise_mppi.cpp"

using namespace duojin02_navigation;

class NativeOracle : public duojin_mppi::Optimizer {
public:
  NativeOracle() {
    settings_.base_constraints = {.5f, -.2f, .3f, .65f};
  }

  OmniVelocityBounds bounds() const {
    const auto &b = settings_.constraints;
    return {b.vx_max, b.vx_min, b.vy, b.wz};
  }
};

class TestController : public PreciseMPPI {
public:
  void forcePeriodicReset() {
    last_time_called_ = clock_->now() - rclcpp::Duration::from_seconds(2.0);
  }
};

geometry_msgs::msg::PoseStamped pose(double x = 0, double y = 0, double a = 0) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(a / 2);
  p.pose.orientation.w = std::cos(a / 2);
  return p;
}

void bounded(const geometry_msgs::msg::Twist &v, const OmniVelocityBounds &b, bool stop = false) {
  assert(std::isfinite(v.linear.x + v.linear.y + v.angular.z));
  assert(v.linear.x >= b.vx_min - 1e-9 && v.linear.x <= b.vx_max + 1e-9);
  assert(std::abs(v.linear.y) <= b.vy_max + 1e-9 && std::abs(v.angular.z) <= b.wz_max + 1e-9);
  if (stop)
    assert(v.linear.x == 0 && v.linear.y == 0 && v.angular.z == 0);
}

int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  const bool use_gpu = mode == "--gpu" || mode == "--fallback";
  if (mode == "--fallback")
    assert(!GeometryCompute().available());
  if (mode == "--gpu" && !GeometryCompute().available()) {
    std::cout << "CUDA device unavailable\n";
    return 77;
  }
  NativeOracle oracle;
  OmniVelocityLimits reference;
  reference.configure({.5, -.2, .3, .65});
  for (const auto request : std::vector<std::pair<double, bool>>{
           {20, true}, {60, true}, {.1, false}, {0, false}, {0, true}}) {
    oracle.setSpeedLimit(request.first, request.second);
    reference.update(request.first, request.second);
    const auto a = oracle.bounds(), b = reference.effective();
    assert(std::abs(a.vx_max - b.vx_max) < 1e-6 && std::abs(a.vx_min - b.vx_min) < 1e-6 &&
           std::abs(a.vy_max - b.vy_max) < 1e-6 && std::abs(a.wz_max - b.wz_max) < 1e-6);
  }
  reference.update(-1, true);
  oracle.setSpeedLimit(reference.nativePercentage(), true);
  const auto stopped_native = oracle.bounds();
  assert(stopped_native.vx_max == 0 && stopped_native.vx_min == 0 && stopped_native.vy_max == 0 &&
         stopped_native.wz_max == 0);
  std::cout
      << "INSTALLED_HUMBLE_ORACLE percentage/absolute/zero-reset and fail-closed encoding passed\n";
  rclcpp::init(argc, argv);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("controller_repair_costmap");
  map->set_parameter(rclcpp::Parameter("global_frame", std::string("odom")));
  map->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_footprint")));
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
  map->declare_parameter("inflation_layer.plugin", std::string("nav2_costmap_2d::InflationLayer"));
  map->declare_parameter("inflation_layer.inflation_radius", .4);
  map->declare_parameter("inflation_layer.cost_scaling_factor", 6.);
  map->set_parameter(rclcpp::Parameter(
      "footprint",
      std::string(
          "[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
  map->set_parameter(rclcpp::Parameter("footprint_padding", .008));
  map->on_configure(rclcpp_lifecycle::State());
  map->getCostmap()->resizeMap(600, 600, .01, -3., -3.);

  auto make_options = [use_gpu]() {
    rclcpp::NodeOptions o;
    o.parameter_overrides(
        {rclcpp::Parameter("controller_frequency", 20.),
         rclcpp::Parameter("FollowPath.use_gpu", use_gpu),
         rclcpp::Parameter("FollowPath.optimizer_backend", std::string(use_gpu ? "auto" : "cpu")),
         rclcpp::Parameter("FollowPath.optimizer_seed", 42),
         rclcpp::Parameter("FollowPath.gpu_minimum_work", 0),
         rclcpp::Parameter("FollowPath.vx_max", .5), rclcpp::Parameter("FollowPath.vx_min", -.2),
         rclcpp::Parameter("FollowPath.vy_max", .3), rclcpp::Parameter("FollowPath.wz_max", .65),
         rclcpp::Parameter("FollowPath.narrow_speed_max", .5),
         rclcpp::Parameter("FollowPath.motion_model", std::string("Omni")),
         rclcpp::Parameter("FollowPath.time_steps", 56),
         rclcpp::Parameter("FollowPath.model_dt", .05),
         rclcpp::Parameter("FollowPath.batch_size", 1600),
         rclcpp::Parameter("FollowPath.vx_std", .15), rclcpp::Parameter("FollowPath.vy_std", .12),
         rclcpp::Parameter("FollowPath.wz_std", .18),
         rclcpp::Parameter("FollowPath.reset_period", 1.),
         rclcpp::Parameter("FollowPath.critics",
                           std::vector<std::string>{"ConstraintCritic", "GoalCritic",
                                                    "GoalAngleCritic", "PathFollowCritic"})});
    return o;
  };
  auto options = make_options();
  options.append_parameter_override("FollowPath.wz_max", .1);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener listener(*tf);
  TestController controller;
  controller.configure(node, "FollowPath", tf, map);
  controller.activate();
  assert(node->describe_parameter("FollowPath.use_gpu").read_only);
  assert(node->describe_parameter("FollowPath.gpu_minimum_work").read_only);
  assert(!node->set_parameter(rclcpp::Parameter("FollowPath.use_gpu", !use_gpu)).successful);
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "probe_goal", map);
  OmniVelocityLimits expected;
  expected.configure({.5, -.2, .3, .1});
  const std::vector<geometry_msgs::msg::PoseStamped> goals = {
      pose(.4, 0), pose(-.4, 0), pose(0, .4), pose(.3, .3, 1), pose(0, 0, 1),
      pose(.8, 0), pose(-.8, 0), pose(0, .8), pose(.8, .2, 1)};
  int count = 0;
  for (const auto request :
       std::vector<std::pair<double, bool>>{{20, true},
                                            {60, true},
                                            {20, true},
                                            {.1, false},
                                            {0, false},
                                            {200, true},
                                            {2, false},
                                            {-1, true},
                                            {std::numeric_limits<double>::quiet_NaN(), false},
                                            {std::numeric_limits<double>::infinity(), true},
                                            {0, true}}) {
    controller.setSpeedLimit(request.first, request.second);
    expected.update(request.first, request.second);
    for (int narrow : {0, 1}) {
      map->getCostmap()->setCost(300, 300, narrow ? 120 : 0);
      for (auto goal : goals) {
        auto start = pose();
        start.header.stamp = node->now();
        goal.header.stamp = start.header.stamp;
        nav_msgs::msg::Path path;
        path.header = start.header;
        const double a = tf2::getYaw(goal.pose.orientation);
        for (int i = 0; i <= 100; ++i)
          path.poses.push_back(
              pose(goal.pose.position.x * i / 100, goal.pose.position.y * i / 100, a * i / 100));
        controller.setPlan(path);
        geometry_msgs::msg::Twist speed;
        const auto out = controller.computeVelocityCommands(start, speed, &checker).twist;
        bounded(out, expected.effective(), expected.stopped());
        ++count;
      }
    }
  }

  map->getCostmap()->setCost(300, 300, 0);
  map->getCostmap()->setCost(305, 334, 254);
  controller.setSpeedLimit(20, true);
  {
    auto s = pose();
    s.header.stamp = node->now();
    auto g = pose(.4, 0);
    g.header = s.header;
    nav_msgs::msg::Path clear_path;
    clear_path.header = s.header;
    clear_path.poses = {s, g};
    controller.setPlan(clear_path);
    geometry_msgs::msg::Twist v;
    const auto result = controller.computeVelocityCommands(s, v, &checker).twist;
    bounded(result, {.1, -.04, .06, .02});
    assert(result.linear.x > 0);
  }
  map->getCostmap()->setCost(305, 334, 0);
  map->getCostmap()->setCost(300, 300, 0);

  // The lookahead chord is clear, but the requested velocity needs a longer
  // braking distance. Force both centered-clearance and later-scale braking
  // batches and compare the admitted command with an independent CPU instance.
  controller.setSpeedLimit(0, false);
  map->getCostmap()->setCost(300, 300, 120);
  map->getCostmap()->setCost(345, 300, 254);
  auto cpu_options = make_options();
  cpu_options.append_parameter_override("FollowPath.use_gpu", false);
  cpu_options.append_parameter_override("FollowPath.wz_max", .1);
  auto cpu_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("cpu_controller_oracle", cpu_options);
  TestController cpu_controller;
  cpu_controller.configure(cpu_node, "FollowPath", tf, map);
  cpu_controller.activate();
  auto batch_start = pose();
  batch_start.header.stamp = node->now();
  nav_msgs::msg::Path batch_path;
  batch_path.header = batch_start.header;
  for (int i = 0; i <= 100; ++i)
    batch_path.poses.push_back(pose(i * .01, 0));
  controller.setPlan(batch_path);
  cpu_controller.setPlan(batch_path);
  const auto before_batch = controller.controllerStatistics();
  const auto cpu_command = cpu_controller.computeVelocityCommands(batch_start, {}, &checker).twist;
  const auto batch_command = controller.computeVelocityCommands(batch_start, {}, &checker).twist;
  assert(std::abs(batch_command.linear.x - cpu_command.linear.x) < 1e-12);
  assert(std::abs(batch_command.linear.y - cpu_command.linear.y) < 1e-12);
  assert(std::abs(batch_command.angular.z - cpu_command.angular.z) < 1e-12);
  assert(batch_command.linear.x > .2 && batch_command.linear.x < .34);
  const auto after_batch = controller.controllerStatistics();
  assert(after_batch.cycles == before_batch.cycles + 1);
  assert(after_batch.last_cycle_ms > 0 && after_batch.max_cycle_ms >= after_batch.last_cycle_ms);
  if (mode == "--gpu") {
    assert(after_batch.centering_cuda_batches > before_batch.centering_cuda_batches);
    assert(after_batch.braking_cuda_batches > before_batch.braking_cuda_batches);
    assert(after_batch.braking_final_cpu_candidates > before_batch.braking_final_cpu_candidates);
  } else {
    assert(after_batch.centering_cpu_batches > before_batch.centering_cpu_batches);
    assert(after_batch.braking_cuda_batches == 0);
  }
  std::cout << "CONTROLLER_BATCH_PARITY vx=" << batch_command.linear.x
            << " centering_cuda=" << after_batch.centering_cuda_batches - before_batch.centering_cuda_batches
            << " braking_cuda=" << after_batch.braking_cuda_batches - before_batch.braking_cuda_batches
            << " final_cpu_candidates=" << after_batch.braking_final_cpu_candidates - before_batch.braking_final_cpu_candidates
            << " cycle_wall_ms=" << after_batch.last_cycle_ms << "\n";
  cpu_controller.deactivate();
  cpu_controller.cleanup();
  map->getCostmap()->setCost(300, 300, 0);
  map->getCostmap()->setCost(345, 300, 0);
  controller.setSpeedLimit(20, true);
  const auto changed = node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.vx_max", .4), rclcpp::Parameter("FollowPath.vx_min", -.1),
       rclcpp::Parameter("FollowPath.vy_max", .2), rclcpp::Parameter("FollowPath.wz_max", .04)});
  assert(changed.successful);
  expected.configure({.4, -.1, .2, .04});
  expected.update(20, true);
  auto start = pose();
  start.header.stamp = node->now();
  auto goal = pose(.3, .2, 1);
  goal.header = start.header;
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.poses = {start, goal};
  controller.setPlan(path);
  geometry_msgs::msg::Twist speed;
  auto out = controller.computeVelocityCommands(start, speed, &checker).twist;
  bounded(out, expected.effective());
  assert(out.angular.z > 0);
  assert(node->set_parameter(
                 rclcpp::Parameter("FollowPath.wz_max", std::numeric_limits<double>::quiet_NaN()))
             .successful);
  out = controller.computeVelocityCommands(start, speed, &checker).twist;
  bounded(out, {}, true);
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.wz_max", .04)).successful);
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.vy_max", 0.)).successful);
  controller.setSpeedLimit(0, false);
  goal = pose(.3, 0);
  goal.header = start.header;
  path.poses = {start, goal};
  controller.setPlan(path);
  out = controller.computeVelocityCommands(start, speed, &checker).twist;
  assert(out.linear.x > 0 && out.linear.y == 0);
  bounded(out, {.4, -.1, 0, .04});
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.vy_max", .2)).successful);
  controller.setSpeedLimit(20, true);

  path.poses = {pose(.35, 0), pose(1, 0)};
  controller.setPlan(path);
  for (int i = 0; i < 3; ++i) {
    controller.forcePeriodicReset();
    start.header.stamp = node->now();
    out = controller.computeVelocityCommands(start, speed, &checker).twist;
    bounded(out, expected.effective());
    std::cout << "NATIVE_PUBLIC_LIMIT vx=" << out.linear.x << " vy=" << out.linear.y
              << " wz=" << out.angular.z << "\n";
  }
  if (mode == "--gpu") {
    controller.setSpeedLimit(0, false);
    auto valid_footprint = map->getUnpaddedRobotFootprint();
    auto clockwise = valid_footprint;
    std::reverse(clockwise.begin(), clockwise.end());
    map->setRobotFootprint(clockwise);
    start.header.stamp = node->now();
    path.header = start.header;
    path.poses = {start, pose(.1, 0)};
    controller.setPlan(path);
    const auto before_fault = controller.controllerStatistics();
    out = controller.computeVelocityCommands(start, {}, &checker).twist;
    bounded(out, {}, true);
    const auto after_fault = controller.controllerStatistics();
    assert(after_fault.compute_failures == before_fault.compute_failures + 1);
    assert(after_fault.cycles == before_fault.cycles + 1);
    // Restore valid geometry and make a small clear query. This path would
    // normally take CPU early exits and must still honor the controller latch.
    map->setRobotFootprint(valid_footprint);
    out = controller.computeVelocityCommands(start, {}, &checker).twist;
    bounded(out, {}, true);
    const auto after_latched = controller.controllerStatistics();
    assert(after_latched.compute_failures == after_fault.compute_failures);
    assert(after_latched.fault_latched_stops == after_fault.fault_latched_stops + 1);
    assert(after_latched.cycles == after_fault.cycles + 1);
    std::cout << "CONTROLLER_FAULT_LATCH invalid_scene_failure_then_small_clear_cpu_path_stopped "
              << "failure_cycle_wall_ms=" << after_fault.last_cycle_ms
              << " latched_cycle_wall_ms=" << after_latched.last_cycle_ms << "\n";
  }
  controller.deactivate();
  controller.cleanup();
  for (const auto &bad : std::vector<std::pair<std::string, double>>{
           {"vx_max", 0},
           {"vx_min", .2},
           {"vy_max", -.1},
           {"wz_max", -.1},
           {"wz_max", std::numeric_limits<double>::infinity()}}) {
    auto o = make_options();
    o.append_parameter_override("FollowPath." + bad.first, bad.second);
    auto invalid_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", o);
    TestController invalid;
    bool rejected = false;
    try {
      invalid.configure(invalid_node, "FollowPath", tf, map);
    } catch (const std::invalid_argument &e) {
      rejected = std::string(e.what()).find(bad.first) != std::string::npos;
      std::cout << "INVALID_CONFIG_REJECTED " << e.what() << "\n";
    }
    assert(rejected);
    invalid.cleanup();
  }
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  std::cout
      << "PUBLIC_API_CHECKS " << count
      << " deterministic/docking/narrow percentage/absolute/reset/invalid commands; dynamic config; native periodic reset; invalid setup passed\n";
}
