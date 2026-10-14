#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace duojin02_navigation {
namespace {
bool usablePath(const nav_msgs::msg::Path &path) {
  if (path.header.frame_id.empty() || path.poses.empty())
    return false;
  for (const auto &stamped : path.poses) {
    if (stamped.header.frame_id != path.header.frame_id)
      return false;
    const auto &p = stamped.pose.position;
    const auto &q = stamped.pose.orientation;
    for (const auto value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
      if (!std::isfinite(value))
        return false;
    }
    const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm2) || norm2 <= 0.)
      return false;
  }
  return true;
}

bool sameGeometry(const nav_msgs::msg::Path &a, const nav_msgs::msg::Path &b) {
  if (a.header.frame_id != b.header.frame_id || a.poses.size() != b.poses.size())
    return false;
  for (size_t i = 0; i < a.poses.size(); ++i) {
    const auto &p = a.poses[i];
    const auto &q = b.poses[i];
    if (p.header.frame_id != q.header.frame_id || p.pose.position != q.pose.position)
      return false;
    const auto &u = p.pose.orientation;
    const auto &v = q.pose.orientation;

    if (u != v && !(u.x == -v.x && u.y == -v.y && u.z == -v.z && u.w == -v.w))
      return false;
  }
  return true;
}
}

class PathNeedsReplan : public BT::StatefulActionNode {
  using Service = nav2_msgs::srv::IsPathValid;
  using Client = rclcpp::Client<Service>;
  using Clock = std::chrono::steady_clock;

public:
  PathNeedsReplan(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    timeout_ = config.blackboard->get<std::chrono::milliseconds>("server_timeout");
    getInput("server_timeout", timeout_);
    std::string service_name;
    getInput("service_name", service_name);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    client_ = node_->create_client<Service>(service_name,
                                            rclcpp::ServicesQoS().get_rmw_qos_profile(), group_);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
  }

  ~PathNeedsReplan() override {
    clear();
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
            BT::OutputPort<nav_msgs::msg::Path>("checked_path"),
            BT::InputPort<std::string>("service_name", std::string("is_path_valid"),
                                       "Path-check service"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
  }

  BT::NodeStatus onStart() override {
    clear();
    try {
      if (timeout_.count() <= 0 || !getInput("path", checked_path_) || !usablePath(checked_path_) ||
          !client_->service_is_ready())
        return BT::NodeStatus::FAILURE;
      auto request = std::make_shared<Service::Request>();
      request->path = checked_path_;
      deadline_ = Clock::now() + timeout_;
      pending_.emplace(client_->async_send_request(request));
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception &error) {
      return failed(error);
    }
  }

  BT::NodeStatus onRunning() override {
    try {
      if (!pending_) {
        clear();
        return BT::NodeStatus::FAILURE;
      }
      executor_.spin_some();
      if (pending_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        if (Clock::now() >= deadline_) {
          clear();
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
      auto response = pending_->get();

      const bool rejected = response && !response->is_valid;
      if (rejected)
        setOutput("checked_path", checked_path_);
      clear();
      return rejected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const std::exception &error) {
      return failed(error);
    }
  }

  void onHalted() override {
    clear();
  }

private:
  void clear() {
    if (pending_)
      client_->remove_pending_request(*pending_);
    pending_.reset();
  }

  BT::NodeStatus failed(const std::exception &error) {
    RCLCPP_WARN(node_->get_logger(), "Path validation unavailable: %s", error.what());
    clear();
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  Client::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::optional<Client::FutureAndRequestId> pending_;
  nav_msgs::msg::Path checked_path_;
  Clock::time_point deadline_;
  std::chrono::milliseconds timeout_;
};

class AcceptChangedPath : public BT::SyncActionNode {
public:
  AcceptChangedPath(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("old_path"),
            BT::InputPort<nav_msgs::msg::Path>("new_path"),
            BT::OutputPort<nav_msgs::msg::Path>("path")};
  }

  BT::NodeStatus tick() override {
    nav_msgs::msg::Path old_path, new_path;
    if (!getInput("old_path", old_path) || !getInput("new_path", new_path) ||
        !usablePath(old_path) || !usablePath(new_path) || sameGeometry(old_path, new_path)) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("path", new_path);
    return BT::NodeStatus::SUCCESS;
  }
};

class RestartOnGoalUpdate : public BT::DecoratorNode {
  using Goal = geometry_msgs::msg::PoseStamped;
  using Goals = std::vector<Goal>;

public:
  RestartOnGoalUpdate(const std::string &name, const BT::NodeConfiguration &config)
      : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<Goal>("goal"), BT::InputPort<Goals>("goals")};
  }

  BT::NodeStatus tick() override {
    std::optional<Goal> next_goal;
    std::optional<Goals> next_goals;
    snapshot(next_goal, next_goals);
    if (!next_goal && !next_goals) {
      resetChild();
      return BT::NodeStatus::FAILURE;
    }
    if (status() != BT::NodeStatus::IDLE && (next_goal != goal_ || next_goals != goals_))
      resetChild();
    setStatus(BT::NodeStatus::RUNNING);
    const auto result = child_node_->executeTick();

    snapshot(goal_, goals_);
    return result;
  }

  void halt() override {
    goal_.reset();
    goals_.reset();
    BT::DecoratorNode::halt();
  }

private:
  void snapshot(std::optional<Goal> &single, std::optional<Goals> &multiple) {
    Goal goal;
    Goals goals;
    single.reset();
    multiple.reset();
    if (getInput("goal", goal))
      single = std::move(goal);
    if (getInput("goals", goals))
      multiple = std::move(goals);
  }

  std::optional<Goal> goal_;
  std::optional<Goals> goals_;
};
}

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<duojin02_navigation::PathNeedsReplan>("PathNeedsReplan");
  factory.registerNodeType<duojin02_navigation::AcceptChangedPath>("AcceptChangedPath");
  factory.registerNodeType<duojin02_navigation::RestartOnGoalUpdate>("RestartOnGoalUpdate");
}
