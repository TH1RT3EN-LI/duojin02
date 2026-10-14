#include <cassert>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <thread>
#include <nlohmann/json.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../src/precise_lattice.cpp"

struct LatticeFixture {
  std::filesystem::path directory, path;

  LatticeFixture() {
#ifdef PRECISE_LATTICE_FIXTURE_SOURCE
    const std::filesystem::path source = PRECISE_LATTICE_FIXTURE_SOURCE;
#else
    const auto source =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
        "duojin02_bringup/config/lattice_primitives/duojin02_omni_2cm_rich_lattice.json";
#endif
    std::ifstream input(source);
    assert(input.good());
    nlohmann::json data;
    input >> data;
    const double factor = .01 / data["lattice_metadata"]["grid_resolution"].get<double>();
    data["lattice_metadata"]["grid_resolution"] = .01;
    data["lattice_metadata"]["turning_radius"] =
        data["lattice_metadata"]["turning_radius"].get<double>() * factor;
    for (auto &primitive : data["primitives"]) {
      for (const auto *field :
           {"trajectory_radius", "trajectory_length", "arc_length", "straight_length"})
        primitive[field] = primitive[field].get<double>() * factor;
      for (auto &point : primitive["poses"]) {
        point[0] = point[0].get<double>() * factor;
        point[1] = point[1].get<double>() * factor;
      }
    }
    auto name =
        (std::filesystem::temp_directory_path() / "duojin02-planner-checks-XXXXXX").string();
    std::vector<char> buffer(name.begin(), name.end());
    buffer.push_back('\0');
    const auto *created = mkdtemp(buffer.data());
    assert(created);
    directory = created;
    path = directory / "lattice-1cm.json";
    std::ofstream output(path);
    output << data.dump();
    output.close();
    assert(output.good());
  }

  ~LatticeFixture() {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
  }
};

struct Stats {
  double length = 0, weighted = 0, max_y = 0, ms = 0;
  int soft = 0;
};

Stats report(const std::string &name, const nav_msgs::msg::Path &path,
             nav2_costmap_2d::Costmap2D *map, double ms) {
  assert(!path.poses.empty());
  Stats s;
  s.ms = ms;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto &p = path.poses[i].pose.position;
    s.max_y = std::max(s.max_y, std::abs(p.y));
    unsigned int x = 0, y = 0;
    unsigned char cost = 0;
    if (map->worldToMap(p.x, p.y, x, y))
      cost = map->getCost(x, y);
    if (cost > 0 && cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      ++s.soft;
    if (i) {
      const auto &q = path.poses[i - 1].pose.position;
      const double ds = std::hypot(p.x - q.x, p.y - q.y);
      s.length += ds;
      s.weighted += ds * (1 + 3. * cost / 252.);
    }
  }
  std::cout << "AUDIT_PLAN " << name << " poses=" << path.poses.size() << " length=" << s.length
            << " max_abs_y=" << s.max_y << " soft_samples=" << s.soft
            << " length_cost_proxy=" << s.weighted << " elapsed_ms=" << ms << std::endl;
  return s;
}

auto pose(double x, double y = 0, double yaw = 0) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}

int main(int argc, char **argv) {
  LatticeFixture fixture;
  rclcpp::init(argc, argv);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("audit_global_costmap");
  map->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
  map->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_footprint")));
  map->set_parameter(rclcpp::Parameter("resolution", .01));
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
  auto *costmap = map->getCostmap();
  costmap->resizeMap(500, 400, .01, -1., -2.);
  rclcpp::NodeOptions options;

  options.append_parameter_override("GridBased.lattice_filepath", fixture.path.string());
  options.append_parameter_override("GridBased.cache_obstacle_heuristic", false);
  options.append_parameter_override("GridBased.tolerance", .02);
  options.append_parameter_override("GridBased.allow_unknown", false);
  options.append_parameter_override("GridBased.max_iterations", 1000000);
  options.append_parameter_override("GridBased.max_on_approach_iterations", 1000);
  options.append_parameter_override("GridBased.max_planning_time", 15.);
  options.append_parameter_override("GridBased.allow_reverse_expansion", false);
  options.append_parameter_override("GridBased.reverse_penalty", 1.5);
  options.append_parameter_override("GridBased.change_penalty", 0.);
  options.append_parameter_override("GridBased.non_straight_penalty", 1.1);
  options.append_parameter_override("GridBased.cost_penalty", 3.);
  options.append_parameter_override("GridBased.rotation_penalty", 5.);
  options.append_parameter_override("GridBased.retrospective_penalty", .015);
  options.append_parameter_override("GridBased.lookup_table_size", 5.);
  options.append_parameter_override("GridBased.analytic_expansion_ratio", 3.5);
  options.append_parameter_override("GridBased.analytic_expansion_max_length", 1.);
  options.append_parameter_override("GridBased.smooth_path", false);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server", options);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener listener(*tf);
  duojin02_navigation::PreciseLattice precise;
  precise.configure(node, "GridBased", tf, map);
  precise.activate();
  size_t native_publications = 0;
  auto raw_subscription = node->create_subscription<nav_msgs::msg::Path>(
      "unsmoothed_plan", rclcpp::QoS(10),
      [&](const nav_msgs::msg::Path::SharedPtr) { ++native_publications; });
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (raw_subscription->get_publisher_count() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(raw_subscription->get_publisher_count() > 0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  const auto rejected =
      node->set_parameter(rclcpp::Parameter("GridBased.cache_obstacle_heuristic", true));
  assert(!rejected.successful &&
         !node->get_parameter("GridBased.cache_obstacle_heuristic").as_bool());
  std::cout << "AUDIT_PARAMETER dynamic_true_rejected=1 reason=" << rejected.reason << std::endl;
  auto reject_options = options;
  reject_options.append_parameter_override("GridBased.cache_obstacle_heuristic", true);
  auto reject_node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_reject", reject_options);
  duojin02_navigation::PreciseLattice forbidden;
  bool config_rejected = false;
  try {
    forbidden.configure(reject_node, "GridBased", tf, map);
  } catch (const nav2_core::PlannerException &) {
    config_rejected = true;
  }
  assert(config_rejected);
  std::cout << "AUDIT_PARAMETER configure_true_rejected=1" << std::endl;
  auto start = pose(0), goal = pose(3);
  auto run = [&](const std::string &name, bool expect_native = true) {
    executor.spin_some();
    const auto before_publications = native_publications;
    const auto begin = std::chrono::steady_clock::now();
    auto path = precise.createPlan(start, goal);
    assert(path.poses.front().pose == start.pose && path.poses.back().pose == goal.pose);
    auto stats =
        report(name, path, costmap,
               std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
                   .count());
    const auto publication_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    do {
      executor.spin_some();
      if (native_publications > before_publications || !expect_native)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < publication_deadline);
    assert((native_publications > before_publications) == expect_native);
    std::cout << "AUDIT_NATIVE " << name
              << " published=" << (native_publications - before_publications) << std::endl;
    return std::make_pair(path, stats);
  };
  auto [free_short, free_stats] = run("free_shortcut", false);
  assert(free_short.poses.size() == 301 && free_stats.max_y == 0);
  for (unsigned int y = 170; y <= 230; ++y)
    for (unsigned int x = 200; x <= 300; ++x)
      costmap->setCost(x, y, 250);
  auto [detour, detour_stats] = run("precise_soft_strip");
  assert(detour_stats.soft == 0 && detour_stats.max_y > .3);
  auto [same, same_stats] = run("same_map_cached_soft_strip", false);
  assert(same.poses.size() == detour.poses.size());

  start = detour.poses[detour.poses.size() / 2];
  start.pose.position.y -= .05;
  run("unchanged_map_shifted_start_soft_connector");
  start = pose(0);

  assert(node->set_parameter(rclcpp::Parameter("GridBased.cost_penalty", 0.)).successful);
  auto [zero_penalty, zero_stats] = run("dynamic_zero_cost_penalty");
  assert(node->get_parameter("GridBased.cost_penalty").as_double() == 0.);
  assert(node->set_parameter(rclcpp::Parameter("GridBased.cost_penalty", 3.)).successful);
  auto [restored_penalty, restored_stats] = run("dynamic_restored_cost_penalty");
  assert(restored_stats.soft == 0);
  detour = restored_penalty;

  start = detour.poses[detour.poses.size() / 2];
  auto [suffix, suffix_stats] = run("same_map_on_segment_suffix", false);
  assert(suffix.poses.size() < detour.poses.size());
  start = pose(0);
  goal = pose(3.0005, 0, .0005);
  auto [exact, exact_stats] = run("submillimeter_goal_change");
  assert(exact.poses.back().pose == goal.pose);
  costmap->resetMap(0, 0, 500, 400);
  goal = pose(3);

  costmap->setCost(250, 215, 1);
  auto [filled, filled_stats] = run("inside_footprint_soft_cell");
  assert(filled.poses.size() > 2);
  costmap->resetMap(0, 0, 500, 400);
  start = pose(-.75);
  goal = pose(-.75, .1);
  run("map_border_actual_polygon_inside", false);
  start = pose(-.75, 0, M_PI / 4);
  goal = pose(-.75, .1, M_PI / 4);
  bool edge_rejected = false;
  try {
    run("map_border_rotated_outside");
  } catch (const nav2_core::PlannerException &) {
    edge_rejected = true;
  }
  assert(edge_rejected);
  std::cout << "AUDIT_BORDER rotated_outside_rejected=1" << std::endl;
  for (const auto &pair :
       std::vector<std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>>{
           {pose(-1.01), pose(0)},
           {pose(0), pose(4.01)},
           {pose(NAN), pose(0)},
           {pose(0), pose(NAN)},
           {pose(1e100), pose(1e100)}}) {
    start = pair.first;
    goal = pair.second;
    bool rejected_endpoint = false;
    try {
      run("invalid_endpoint");
    } catch (const nav2_core::PlannerException &) {
      rejected_endpoint = true;
    }
    assert(rejected_endpoint);
  }
  std::cout << "AUDIT_BORDER invalid_start_goal_center_and_nan_rejected=1" << std::endl;
  costmap->resizeMap(1000, 400, .01, -1., -2.);
  start = pose(0);
  goal = pose(7);
  auto [large, large_stats] = run("large_initial_free");
  auto [large_cached, large_cached_stats] = run("large_unchanged_cache", false);
  assert(large_cached.poses.size() == large.poses.size());
  const auto original_footprint = map->getUnpaddedRobotFootprint();
  auto changed_footprint = original_footprint;
  changed_footprint.front().y += .0001;
  map->setRobotFootprint(changed_footprint);
  run("unchanged_bytes_footprint_change");
  map->setRobotFootprint(original_footprint);
  run("unchanged_bytes_footprint_restore");
  for (unsigned int y = 170; y <= 230; ++y)
    for (unsigned int x = 400; x <= 600; ++x)
      costmap->setCost(x, y, 250);
  auto [updated, updated_stats] = run("cache_updated_soft_strip");
  assert(updated_stats.soft == 0 && updated_stats.max_y > .3);
  costmap->resetMap(0, 0, 1000, 400);
  auto [drop, drop_stats] = run("cache_cost_decrease");
  assert(drop_stats.length < updated_stats.length && drop_stats.max_y < .1);

  costmap->resizeMap(500, 400, .01, -1., -2.);
  start = pose(0);
  goal = pose(3);
  costmap->setCost(250, 215, 1);
  auto [shrunk, shrunk_stats] = run("large_to_small_native");
  assert(nav2_smac_planner::NodeHybrid::obstacle_heuristic_lookup_table.size() == 250 * 200);
  costmap->resizeMap(400, 500, .01, -1., -2.);
  goal = pose(2.5);
  costmap->setCost(220, 215, 1);
  auto [reshaped, reshape_stats] = run("same_bytes_different_shape_native");
  assert(nav2_smac_planner::NodeHybrid::obstacle_heuristic_lookup_table.size() == 200 * 250);
  executor.remove_node(node->get_node_base_interface());
  precise.deactivate();
  precise.cleanup();
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  std::cout << "AUDIT_RESULT PASS (all assertions, normal shutdown)" << std::endl;
}
