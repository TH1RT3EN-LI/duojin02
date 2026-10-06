#include "duojin01_navigation/stable_goal_checker.hpp"
#include <cassert>
#include <chrono>
#include <thread>
#include <iostream>
using namespace std::chrono_literals;
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_settling_checks");
  node->declare_parameter("goal.xy_goal_tolerance",.01);
  node->declare_parameter("goal.yaw_goal_tolerance",.02);
  node->declare_parameter("goal.trans_stopped_velocity",.01);
  node->declare_parameter("goal.rot_stopped_velocity",.02);
  duojin01_navigation::StableStoppedGoalChecker checker;checker.initialize(node,"goal",nullptr);
  geometry_msgs::msg::Pose pose,goal;pose.orientation.w=goal.orientation.w=1;
  geometry_msgs::msg::Twist velocity;
  assert(!checker.isGoalReached(pose,goal,velocity));
  for(int i=0;i<3;++i) {std::this_thread::sleep_for(100ms);const bool arrived=checker.isGoalReached(pose,goal,velocity);assert(arrived==(i==2));}
  pose.position.x=.02;assert(!checker.isGoalReached(pose,goal,velocity));
  pose.position.x=0;assert(!checker.isGoalReached(pose,goal,velocity));
  std::this_thread::sleep_for(180ms);assert(!checker.isGoalReached(pose,goal,velocity)); // interrupted dwell
  velocity.linear.x=.022;assert(!checker.isGoalReached(pose,goal,velocity));
  velocity.linear.x=0;assert(!checker.isGoalReached(pose,goal,velocity));
  for(int i=0;i<3;++i) {std::this_thread::sleep_for(100ms);const bool arrived=checker.isGoalReached(pose,goal,velocity);assert(arrived==(i==2));}
  checker.reset();assert(!checker.isGoalReached(pose,goal,velocity));
  std::cout<<"Continuous stopped dwell, position recheck, residual velocity, interrupted dwell and reset passed\n";
  rclcpp::shutdown();
}
