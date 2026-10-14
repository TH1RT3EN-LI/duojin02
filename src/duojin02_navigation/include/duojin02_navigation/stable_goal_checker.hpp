#pragma once

#include <nav2_controller/plugins/stopped_goal_checker.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cmath>
#include <stdexcept>

namespace duojin02_navigation {
class StableStoppedGoalChecker : public nav2_controller::StoppedGoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    nav2_controller::StoppedGoalChecker::initialize(parent, name, costmap);
    auto node = parent.lock();
    clock_ = node->get_clock();
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Required continuous stopped dwell; configured at startup.";
    if (!node->has_parameter(name + ".settle_time")) {
      node->declare_parameter(name + ".settle_time", .25, descriptor);
    } else if (!node->describe_parameter(name + ".settle_time").read_only) {
      const double configured = node->get_parameter(name + ".settle_time").as_double();
      node->undeclare_parameter(name + ".settle_time");
      node->declare_parameter(name + ".settle_time", configured, descriptor, true);
    }
    settle_time_ = node->get_parameter(name + ".settle_time").as_double();
    if (!std::isfinite(settle_time_) || settle_time_ < .10)
      throw std::invalid_argument("Goal settle_time must be at least 0.10 seconds");
    reset();
  }

  void reset() override {
    nav2_controller::StoppedGoalChecker::reset();
    settling_ = false;
  }

  bool isGoalReached(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Pose &goal,
                     const geometry_msgs::msg::Twist &velocity) override {
    stateful_ = false;
    check_xy_ = true;
    if (!nav2_controller::StoppedGoalChecker::isGoalReached(pose, goal, velocity)) {
      settling_ = false;
      return false;
    }
    const auto now = clock_->now();
    if (!settling_ || now < last_check_ || (now - last_check_).seconds() > .15) {
      began_ = now;
      settling_ = true;
    }
    last_check_ = now;
    return (now - began_).seconds() >= settle_time_;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time began_{0, 0, RCL_ROS_TIME}, last_check_{0, 0, RCL_ROS_TIME};
  double settle_time_{.25};
  bool settling_{false};
};
}
