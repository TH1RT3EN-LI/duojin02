#pragma once
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>

inline bool referencePathTarget(const nav_msgs::msg::Path &path,
                                const geometry_msgs::msg::PoseStamped &pose,
                                const geometry_msgs::msg::TransformStamped &tf,
                                const geometry_msgs::msg::Twist &speed,
                                geometry_msgs::msg::PoseStamped &target) {
  if (path.poses.empty())
    return false;
  nav_msgs::msg::Path local;
  for (auto p : path.poses) {
    p.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped q;
    tf2::doTransform(p, q, tf);
    local.poses.push_back(q);
  }
  size_t nearest = 0;
  double minimum = 1e100;
  for (size_t i = 0; i < local.poses.size(); ++i) {
    const auto &p = local.poses[i].pose;
    const double d =
        std::hypot(p.position.x - pose.pose.position.x, p.position.y - pose.pose.position.y);
    const double dyaw =
        std::remainder(tf2::getYaw(p.orientation) - tf2::getYaw(pose.pose.orientation), 2 * M_PI);
    const double score = d * d + .02 * dyaw * dyaw;
    if (score < minimum) {
      minimum = score;
      nearest = i;
    }
  }
  if (std::hypot(local.poses[nearest].pose.position.x - pose.pose.position.x,
                 local.poses[nearest].pose.position.y - pose.pose.position.y) > .20)
    return false;
  size_t index = nearest;
  double arc = 0;
  while (index + 1 < local.poses.size() &&
         arc < std::clamp(.18 + .4 * std::hypot(speed.linear.x, speed.linear.y), .18, .35)) {
    const auto &a = local.poses[index].pose.position, &b = local.poses[index + 1].pose.position;
    arc += std::hypot(b.x - a.x, b.y - a.y);
    ++index;
  }
  target = local.poses[index];
  return true;
}
