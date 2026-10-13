#pragma once
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace duojin01_navigation {
// Cache only the immutable input conversion performed by Humble's
// PoseStamped doTransform. Every query still transforms and scores all poses
// under its fresh TF, with the original strict tie/offset/lookahead rules.
class PathTarget {
 public:
  void setPath(const nav_msgs::msg::Path &path) {
    frame_id_=path.header.frame_id;
    frames_.clear();frames_.reserve(path.poses.size());
    for(const auto &p:path.poses) frames_.emplace_back(
      KDL::Rotation::Quaternion(p.pose.orientation.x,p.pose.orientation.y,
        p.pose.orientation.z,p.pose.orientation.w),
      KDL::Vector(p.pose.position.x,p.pose.position.y,p.pose.position.z));
  }
  bool empty() const {return frames_.empty();}
  const std::string &frameId() const {return frame_id_;}
  bool select(const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::TransformStamped &tf,
      const geometry_msgs::msg::Twist &speed,
      geometry_msgs::msg::PoseStamped &target) const {
    if(empty()) return false;
    const auto frame=tf2::gmTransformToKDL(tf);
    const double yaw=tf2::getYaw(pose.pose.orientation);
    size_t nearest=0;double minimum=1e100;
    geometry_msgs::msg::Pose closest;
    for(size_t i=0;i<frames_.size();++i) {
      const auto p=transformed(frame,i);
      if(i==0) closest=p;
      const double d=std::hypot(p.position.x-pose.pose.position.x,p.position.y-pose.pose.position.y);
      const double dyaw=std::remainder(tf2::getYaw(p.orientation)-yaw,2*M_PI);
      const double score=d*d+.02*dyaw*dyaw;
      if(score<minimum) {minimum=score;nearest=i;closest=p;}
    }
    if(std::hypot(closest.position.x-pose.pose.position.x,closest.position.y-pose.pose.position.y)>.20) return false;
    size_t index=nearest;double arc=0;
    const double lookahead=std::clamp(.18+.4*std::hypot(speed.linear.x,speed.linear.y),.18,.35);
    auto a=closest;
    while(index+1<frames_.size() && arc<lookahead) {
      auto b=transformed(frame,index+1);
      arc+=std::hypot(b.position.x-a.position.x,b.position.y-a.position.y);
      ++index;a=b;
    }
    target.pose=a;target.header=tf.header;
    return true;
  }
 private:
  geometry_msgs::msg::Pose transformed(const KDL::Frame &frame,size_t index) const {
    const auto out=frame*frames_[index];
    geometry_msgs::msg::Pose result;
    result.position.x=out.p[0];result.position.y=out.p[1];result.position.z=out.p[2];
    out.M.GetQuaternion(result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w);
    return result;
  }
  std::string frame_id_;std::vector<KDL::Frame> frames_;
};
}
