// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: GPL-3.0-only
#ifndef DUOJIN02_SCAN_GEOMETRY__ROS_SCAN_GEOMETRY_HPP_
#define DUOJIN02_SCAN_GEOMETRY__ROS_SCAN_GEOMETRY_HPP_
#include <memory>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "duojin02_scan_geometry/geometry.hpp"

namespace duojin02_scan_geometry
{
inline int64_t stampNs(const builtin_interfaces::msg::Time & time)
{
  if (time.nanosec >= 1000000000u) {throw Error(Failure::InvalidInput, "invalid ROS nanosecond field");}
  return static_cast<int64_t>(time.sec) * 1000000000LL + time.nanosec;
}
inline int64_t referenceStampNs(const sensor_msgs::msg::LaserScan & scan, const Config & c)
{return referenceStampNs(stampNs(scan.header.stamp), scan.time_increment, scan.ranges.size(), c);}
inline builtin_interfaces::msg::Time stampMsg(int64_t ns)
{
  if (ns < 0 || ns / 1000000000LL > std::numeric_limits<int32_t>::max()) {
    throw Error(Failure::InvalidInput, "physical stamp outside ROS Time support");
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(ns / 1000000000LL);
  result.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return result;
}
template<class NodeT>
Config declareConfig(NodeT & node)
{
  Config c;
  rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
  c.enabled = node.template declare_parameter<bool>("scan_motion_compensation", false, descriptor);
  c.capture_time_offset_sec = node.template declare_parameter<double>("capture_time_offset_sec", 0.0, descriptor);
  c.mapping_id = node.template declare_parameter<std::string>(
    "capture_time_mapping_id", "header-first-ray-assumed-v1", descriptor);
  c.mapping_source = node.template declare_parameter<std::string>("capture_time_mapping_source", "", descriptor);
  c.reference_fraction = node.template declare_parameter<double>("scan_reference_fraction", 0.0, descriptor);
  c.max_interpolation_gap_sec = node.template declare_parameter<double>("scan_max_tf_gap_sec", 0.1, descriptor);
  c.tf_history_sec = node.template declare_parameter<double>("scan_tf_history_sec", 120.0, descriptor);
  validate(c); return c;
}
// Own raw TF history is necessary: tf2 lookup alone does not expose internal interpolation gaps.
// Caller retries MissingFuture/MissingFrame in a bounded pending queue, preserving source order.
class RosScanGeometry
{
  std::shared_ptr<History> history_ = std::make_shared<History>();
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_, static_;
  int64_t retention_ns_;
public:
  template<class NodeT>
  explicit RosScanGeometry(NodeT & node, double history_sec = 120.0)
  : retention_ns_(secondsToNs(history_sec))
  {
    auto logger = node.get_logger();
    auto receive = [history = history_, retention = retention_ns_, logger](
        const tf2_msgs::msg::TFMessage::ConstSharedPtr message, bool fixed) {
        for (const auto & tf : message->transforms) {
          try {
            const auto & q = tf.transform.rotation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-5) {
              throw Error(Failure::InvalidInput, "non-unit TF quaternion");
            }
            const double r00 = 1 - 2*(q.y*q.y + q.z*q.z);
            const double r10 = 2*(q.x*q.y + q.z*q.w);
            const double r20 = 2*(q.x*q.z - q.y*q.w);
            const double r21 = 2*(q.y*q.z + q.x*q.w);
            const double r22 = 1 - 2*(q.x*q.x + q.y*q.y);
            if (std::abs(r20) > 1e-6 || std::abs(r21) > 1e-6 ||
              std::abs(std::abs(r22) - 1.0) > 1e-6)
            {throw Error(Failure::InvalidInput, "TF is not planar (tilted scans unsupported)");}
            history->insert(tf.header.frame_id, tf.child_frame_id, stampNs(tf.header.stamp),
              {tf.transform.translation.x, tf.transform.translation.y,
                std::atan2(r10, r00), r22 < 0.0}, fixed, retention);
          } catch (const Error & error) {
            RCLCPP_WARN(logger, "Rejected scan geometry TF %s <- %s: %s",
              tf.header.frame_id.c_str(), tf.child_frame_id.c_str(), error.what());
          }
        }
      };
    dynamic_ = node.template create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).best_effort(),
      [receive](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {receive(m, false);});
    static_ = node.template create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
      [receive](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {receive(m, true);});
  }
  const History & history() const {return *history_;}
  TimedScan prepareReference(const sensor_msgs::msg::LaserScan & scan, const Config & c,
    const std::string & odom_frame, const std::string & base_frame) const
  {
    return duojin02_scan_geometry::prepareReference(stampNs(scan.header.stamp),
      scan.header.frame_id, scan.time_increment, scan.ranges.size(), c,
      *history_, odom_frame, base_frame);
  }
  TimedScan prepare(const sensor_msgs::msg::LaserScan & scan, const Config & c,
    const std::string & odom_frame, const std::string & base_frame) const
  {
    return duojin02_scan_geometry::prepare(stampNs(scan.header.stamp), scan.header.frame_id,
      scan.angle_min, scan.angle_increment, scan.time_increment, scan.range_min, scan.range_max,
      scan.ranges, c, *history_, odom_frame, base_frame);
  }
};
}  // namespace duojin02_scan_geometry
#endif  // DUOJIN02_SCAN_GEOMETRY__ROS_SCAN_GEOMETRY_HPP_
