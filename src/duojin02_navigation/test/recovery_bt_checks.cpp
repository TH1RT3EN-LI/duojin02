#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

nav_msgs::msg::Path path() {
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  result.poses.resize(2);
  for (size_t i = 0; i < result.poses.size(); ++i) {
    result.poses[i].header.frame_id = "map";
    result.poses[i].pose.position.x = static_cast<double>(i);
    result.poses[i].pose.orientation.w = 1.;
  }
  return result;
}

class FakePlanner : public BT::StatefulActionNode {
public:
  FakePlanner(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::InputPort<std::string>("planner_id"), BT::OutputPort<nav_msgs::msg::Path>("path")};
  }
  BT::NodeStatus onStart() override {
    geometry_msgs::msg::PoseStamped target;
    std::vector<geometry_msgs::msg::PoseStamped> targets;
    if (!getInput("goal", target)) {assert(getInput("goals", targets) && !targets.empty()); target = targets.back();}
    replacement_ = config().output_ports.at("path") == "{replacement_path}";
    planned_ = path();
    planned_.poses.back() = target;
    auto board = config().blackboard;
    board->set("plans", board->get<int>("plans") + 1);
    if (replacement_) {
      board->set("replans", board->get<int>("replans") + 1);
      auto midpoint = planned_.poses.front();
      midpoint.pose.position.x = target.pose.position.x * .5;
      midpoint.pose.position.y = .2;
      planned_.poses.insert(planned_.poses.begin() + 1, midpoint);
      board->set("replan_pending", true);
    }
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    if (replacement_ && config().blackboard->get<bool>("hold_replan")) return BT::NodeStatus::RUNNING;
    if (replacement_) config().blackboard->set("replan_pending", false);
    setOutput("path", planned_);
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {
    auto board = config().blackboard;
    board->set("planner_halts", board->get<int>("planner_halts") + 1);
    board->set("replan_pending", false);
  }
private:
  nav_msgs::msg::Path planned_;
  bool replacement_{false};
};

class FakeFollower : public BT::StatefulActionNode {
public:
  FakeFollower(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("controller_id")};
  }
  BT::NodeStatus onStart() override {
    auto board = config().blackboard;
    const auto failures = board->get<int>("forced_failures");
    if (failures) {board->set("forced_failures", failures - 1); return BT::NodeStatus::FAILURE;}
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    auto board = config().blackboard;
    nav_msgs::msg::Path current;
    assert(getInput("path", current) && !current.poses.empty());
    board->set("following_target", current.poses.back().pose.position.x);
    if (board->get<bool>("force_failure")) {board->set("force_failure", false); return BT::NodeStatus::FAILURE;}
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {
    auto board = config().blackboard;
    board->set("follower_halts", board->get<int>("follower_halts") + 1);
  }
};

class FakeRemovePassedGoals : public BT::SyncActionNode {
public:
  FakeRemovePassedGoals(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    using Goals = std::vector<geometry_msgs::msg::PoseStamped>;
    return {BT::InputPort<Goals>("input_goals"), BT::OutputPort<Goals>("output_goals"),
      BT::InputPort<double>("radius"), BT::InputPort<std::string>("robot_base_frame")};
  }
  BT::NodeStatus tick() override {
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    assert(getInput("input_goals", goals));
    if (config().blackboard->get<bool>("trim_goals")) {
      assert(goals.size() > 1);
      goals.erase(goals.begin());
      config().blackboard->set("trim_goals", false);
    }
    setOutput("output_goals", goals);
    return BT::NodeStatus::SUCCESS;
  }
};

struct Fixture {
  BT::BehaviorTreeFactory factory;
  rclcpp::Node::SharedPtr server, client;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr service;
  rclcpp::CallbackGroup::SharedPtr server_group;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 2};
  std::thread worker;
  std::atomic<bool> valid{false}, delayed{false};
  std::atomic<int> requests{0};

  Fixture() {
    const auto scope = "/duojin02_recovery_checks_" + std::to_string(getpid());
    server = std::make_shared<rclcpp::Node>("server", scope);
    client = std::make_shared<rclcpp::Node>("client", scope);
    server_group = server->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service = server->create_service<nav2_msgs::srv::IsPathValid>("is_path_valid",
      [this](const nav2_msgs::srv::IsPathValid::Request::SharedPtr request,
             nav2_msgs::srv::IsPathValid::Response::SharedPtr response) {
        assert(!request->path.poses.empty());
        const bool reply_valid = valid.load();
        const bool reply_delayed = delayed.load();
        ++requests;
        if (reply_delayed) std::this_thread::sleep_for(500ms);
        response->is_valid = reply_valid;
      }, rmw_qos_profile_services_default, server_group);
    executor.add_node(server);
    worker = std::thread([this] {executor.spin();});
    factory.registerFromPlugin(RECOVERY_BT_LIBRARY);
    for (const auto & library : {"nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node", "nav2_rate_controller_bt_node"}) {
      factory.registerFromPlugin(std::string(NAV2_BT_LIBRARY_DIRECTORY) + "/lib" + library + ".so");
    }
    factory.registerNodeType<FakePlanner>("ComputePathToPose");
    factory.registerNodeType<FakePlanner>("ComputePathThroughPoses");
    factory.registerNodeType<FakeFollower>("FollowPath");
    factory.registerNodeType<FakeRemovePassedGoals>("RemovePassedGoals");
  }
  ~Fixture() {
    executor.cancel();
    worker.join();
  }
  BT::Blackboard::Ptr blackboard() {
    auto result = BT::Blackboard::create();
    result->set("node", client);
    result->set("server_timeout", 200ms);
    return result;
  }
  BT::Tree query(const nav_msgs::msg::Path & input, BT::Blackboard::Ptr board = {}) {
    if (!board) board = blackboard();
    board->set("path", input);
    return factory.createTreeFromText(
      "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
      "<PathNeedsReplan path=\"{path}\"/></BehaviorTree></root>", board);
  }
  BT::Tree replacement(const nav_msgs::msg::Path & old_path,
    const nav_msgs::msg::Path & new_path) {
    auto board = blackboard();
    board->set("path", old_path);
    board->set("replacement_path", new_path);
    return factory.createTreeFromText(
      "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
      "<AcceptChangedPath old_path=\"{path}\" new_path=\"{replacement_path}\" path=\"{path}\"/>"
      "</BehaviorTree></root>", board);
  }
  BT::Tree navigation(bool multiple = false) {
    auto board = blackboard();
    auto target = path().poses.back(); target.pose.position.x = 3.;
    if (multiple) board->set("goals", std::vector<geometry_msgs::msg::PoseStamped>{path().poses.back(), target});
    else board->set("goal", target);
    for (const auto & name : {"plans", "replans", "planner_halts", "follower_halts", "forced_failures"}) board->set(name, 0);
    for (const auto & name : {"hold_replan", "replan_pending", "force_failure", "trim_goals"}) board->set(name, false);
    board->set("following_target", -1.);
    return factory.createTreeFromFile(std::string(RECOVERY_BT_TREE_DIRECTORY) +
      (multiple ? "/navigate_through_poses.xml" : "/navigate_to_pose.xml"), board);
  }
};

template<typename Predicate>
void advance(BT::Tree & tree, Predicate predicate, const char * phase) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!predicate()) {
    const auto status = tree.tickRoot();
    if (status != BT::NodeStatus::RUNNING) std::cerr << phase << ": " << BT::toStr(status) << std::endl;
    assert(status == BT::NodeStatus::RUNNING);
    assert(std::chrono::steady_clock::now() < deadline);
    std::this_thread::sleep_for(1ms);
  }
}

void newGoal(BT::Tree & tree, double x, bool multiple = false) {
  auto target = path().poses.back(); target.pose.position.x = x;
  if (multiple) tree.rootBlackboard()->set("goals", std::vector<geometry_msgs::msg::PoseStamped>{target});
  else tree.rootBlackboard()->set("goal", target);
}

BT::NodeStatus finish(BT::Tree & tree) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  BT::NodeStatus status;
  do {
    status = tree.tickRoot();
    if (status != BT::NodeStatus::RUNNING) return status;
    std::this_thread::sleep_for(1ms);
  } while (std::chrono::steady_clock::now() < deadline);
  assert(false && "Service BT failed to reach a terminal status");
  return BT::NodeStatus::FAILURE;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int checks = 0;
  {
    Fixture fixture;
    auto rejected = fixture.query(path());
    assert(finish(rejected) == BT::NodeStatus::SUCCESS);
    ++checks;
    fixture.valid = true;
    auto accepted = fixture.query(path());
    assert(finish(accepted) == BT::NodeStatus::FAILURE);
    ++checks;

    const auto original = path();
    auto stamp_only = original;
    stamp_only.header.stamp.sec = 42;
    for (auto & pose : stamp_only.poses) pose.header.stamp.sec = 43;
    auto same = fixture.replacement(original, stamp_only);
    assert(finish(same) == BT::NodeStatus::FAILURE);
    assert(same.rootBlackboard()->get<nav_msgs::msg::Path>("path") == original);
    ++checks;
    auto opposite_quaternion = original;
    for (auto & pose : opposite_quaternion.poses) pose.pose.orientation.w = -1.;
    auto same_rotation = fixture.replacement(original, opposite_quaternion);
    assert(finish(same_rotation) == BT::NodeStatus::FAILURE);
    ++checks;
    auto changed = original;
    changed.poses.back().pose.position.y = .2;
    auto different = fixture.replacement(original, changed);
    assert(finish(different) == BT::NodeStatus::SUCCESS);
    assert(different.rootBlackboard()->get<nav_msgs::msg::Path>("path") == changed);
    ++checks;
    auto turned = original;
    turned.poses.back().pose.orientation.z = std::sin(.2);
    turned.poses.back().pose.orientation.w = std::cos(.2);
    auto rotated = fixture.replacement(original, turned);
    assert(finish(rotated) == BT::NodeStatus::SUCCESS);
    ++checks;

    std::vector<nav_msgs::msg::Path> malformed;
    auto invalid = original; invalid.poses.clear(); malformed.push_back(invalid);
    invalid = original; invalid.header.frame_id.clear(); malformed.push_back(invalid);
    invalid = original; invalid.poses[0].header.frame_id = "odom"; malformed.push_back(invalid);
    invalid = original; invalid.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    malformed.push_back(invalid);
    invalid = original; invalid.poses[0].pose.orientation.w = 0.; malformed.push_back(invalid);
    invalid = original; invalid.poses[0].pose.orientation.w = std::numeric_limits<double>::infinity();
    malformed.push_back(invalid);
    for (const auto & bad : malformed) {
      const auto before = fixture.requests.load();
      auto no_query = fixture.query(bad);
      assert(finish(no_query) == BT::NodeStatus::FAILURE);
      assert(fixture.requests.load() == before);
      auto no_replace = fixture.replacement(original, bad);
      assert(finish(no_replace) == BT::NodeStatus::FAILURE);
      assert(no_replace.rootBlackboard()->get<nav_msgs::msg::Path>("path") == original);
      checks += 2;
    }
    // Repeated poses are legitimate geometry, not grounds for a new heuristic.
    auto repeated = original;
    repeated.poses.push_back(repeated.poses.back());
    fixture.valid = false;
    auto duplicates = fixture.query(repeated);
    assert(finish(duplicates) == BT::NodeStatus::SUCCESS);
    ++checks;

    fixture.delayed = true;
    auto timeout = fixture.query(path());
    assert(timeout.tickRoot() == BT::NodeStatus::RUNNING);
    assert(finish(timeout) == BT::NodeStatus::FAILURE);
    ++checks;
    std::this_thread::sleep_for(550ms);
    auto halted = fixture.query(path());
    assert(halted.tickRoot() == BT::NodeStatus::RUNNING);
    halted.haltTree();
    fixture.valid = true;
    fixture.delayed = false;
    std::this_thread::sleep_for(550ms);
    assert(finish(halted) == BT::NodeStatus::FAILURE);
    ++checks;

    fixture.valid = false;
    auto contextual = fixture.query(path());
    assert(contextual.tickRoot() == BT::NodeStatus::RUNNING);
    contextual.rootBlackboard()->set("path", changed);
    assert(finish(contextual) == BT::NodeStatus::FAILURE);
    ++checks;
    auto stamp_context = fixture.query(path());
    assert(stamp_context.tickRoot() == BT::NodeStatus::RUNNING);
    stamp_context.rootBlackboard()->set("path", stamp_only);
    assert(finish(stamp_context) == BT::NodeStatus::SUCCESS);
    ++checks;

    // The existing server_timeout must still bound I/O when ROS time is paused.
    auto paused_node = std::make_shared<rclcpp::Node>("paused", fixture.client->get_namespace(),
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    assert(paused_node->get_clock()->now().nanoseconds() == 0);
    auto paused_board = fixture.blackboard();
    paused_board->set("node", paused_node);
    fixture.delayed = true;
    auto paused = fixture.query(path(), paused_board);
    assert(paused.tickRoot() == BT::NodeStatus::RUNNING);
    assert(finish(paused) == BT::NodeStatus::FAILURE);
    fixture.delayed = false;
    std::this_thread::sleep_for(550ms);
    ++checks;

    // Load the shipped XML and stock PipelineSequence / RecoveryNode / rate
    // plugins; only the hardware-facing planner and follower are fake actions.
    auto pending_query = fixture.navigation();
    auto board = pending_query.rootBlackboard();
    board->set("forced_failures", 1);
    fixture.delayed = true;
    const auto before_query = fixture.requests.load();
    advance(pending_query, [&] {return fixture.requests.load() > before_query;}, "old query pending");
    fixture.delayed = false;
    newGoal(pending_query, 9.);
    board->set("forced_failures", 1);
    assert(pending_query.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 9.);
    advance(pending_query, [&] {return board->get<double>("following_target") == 9. && board->get<int>("replans") == 1;}, "new goal after query preemption");
    std::this_thread::sleep_for(550ms);
    assert(pending_query.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 9.);
    ++checks;

    auto pending_plan = fixture.navigation();
    board = pending_plan.rootBlackboard();
    board->set("forced_failures", 1);
    board->set("hold_replan", true);
    advance(pending_plan, [&] {return board->get<bool>("replan_pending");}, "old replan pending");
    newGoal(pending_plan, 12.);
    board->set("forced_failures", 1);
    board->set("hold_replan", false);
    assert(pending_plan.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<int>("planner_halts") == 1);
    assert(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 12.);
    advance(pending_plan, [&] {return board->get<double>("following_target") == 12. && board->get<int>("replans") == 2;}, "new goal after replan preemption");
    assert(board->get<nav_msgs::msg::Path>("replacement_path").poses.back().pose.position.x == 12.);
    ++checks;

    auto recovered = fixture.navigation();
    board = recovered.rootBlackboard();
    board->set("forced_failures", 1);
    advance(recovered, [&] {return board->get<double>("following_target") == 3. && board->get<int>("replans") == 1;}, "first goal recovery");
    newGoal(recovered, 15.);
    board->set("forced_failures", 1);
    assert(recovered.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 15.);
    advance(recovered, [&] {return board->get<double>("following_target") == 15. && board->get<int>("replans") == 2;}, "new goal fresh recovery budget");
    board->set("force_failure", true);
    assert(recovered.tickRoot() == BT::NodeStatus::FAILURE);
    ++checks;

    auto waypoints = fixture.navigation(true);
    board = waypoints.rootBlackboard();
    board->set("trim_goals", true);
    assert(waypoints.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals").size() == 1);
    assert(waypoints.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<int>("follower_halts") == 0);
    assert(board->get<int>("plans") == 1);
    newGoal(waypoints, 18., true);
    board->set("forced_failures", 1);
    assert(waypoints.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 18.);
    advance(waypoints, [&] {return board->get<double>("following_target") == 18. && board->get<int>("replans") == 1;}, "external waypoint preemption");
    assert(board->get<int>("follower_halts") == 1);
    ++checks;

    auto metadata = fixture.navigation();
    board = metadata.rootBlackboard();
    assert(metadata.tickRoot() == BT::NodeStatus::RUNNING);
    auto same_pose_new_message = board->get<geometry_msgs::msg::PoseStamped>("goal");
    same_pose_new_message.header.stamp.sec = 42;
    board->set("goal", same_pose_new_message);
    assert(metadata.tickRoot() == BT::NodeStatus::RUNNING);
    assert(board->get<int>("plans") == 2);
    assert(board->get<int>("follower_halts") == 1);
    ++checks;

    auto disappeared = fixture.query(path());
    fixture.service.reset();
    assert(finish(disappeared) == BT::NodeStatus::FAILURE);
    ++checks;
    auto unavailable = fixture.query(path());
    assert(finish(unavailable) == BT::NodeStatus::FAILURE);
    ++checks;
  }
  rclcpp::shutdown();
  std::cout << "{\"checks\":" << checks << ",\"failed\":0}" << std::endl;
}
