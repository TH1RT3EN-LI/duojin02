// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#include "lslidar_driver/lslidar_x10_driver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace lslidar_driver {

LslidarX10Driver::LslidarX10Driver(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), publisher_(std::make_unique<PublicationWorker>()) {
  rcl_interfaces::msg::ParameterDescriptor startup_parameter;
  startup_parameter.read_only = true;
  serial_port_ =
      node_->declare_parameter<std::string>("serial_port", "/dev/lslidar", startup_parameter);
  frame_id = node_->declare_parameter<std::string>("frame_id", "laser", startup_parameter);
  san_topic_ = node_->declare_parameter<std::string>("laserscan_topic", "/scan", startup_parameter);
  pointcloud_topic = node_->declare_parameter<std::string>(
      "pointcloud_topic", "/lslidar_point_cloud", startup_parameter);
  min_range = node_->declare_parameter<double>("min_range", 0.3, startup_parameter);
  max_range = node_->declare_parameter<double>("max_range", 100.0, startup_parameter);
  points.reserve(packet_points_max * 2);
}

LslidarX10Driver::~LslidarX10Driver() {
  publisher_.reset();
}

void LslidarX10Driver::initialize() {
  createRosIO();
  for (size_t angle = 0; angle < cos_azimuth_table.size(); ++angle) {
    const float radians = static_cast<float>(angle * M_PI / 18000.0);
    cos_azimuth_table[angle] = std::cos(radians);
    sin_azimuth_table[angle] = -std::sin(radians);
  }
  cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_->header.frame_id = frame_id;
  cloud_->height = 1;
}

void LslidarX10Driver::createRosIO() {
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
  laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(san_topic_, 10);
  serial_ = std::make_shared<LSIOSR>(serial_port_);
  data_acquisition_strategy_ = std::make_shared<SerialStrategy>(serial_);
  motor_control_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "motor_control", 1, [this](std_msgs::msg::Int8::SharedPtr msg) { motorControl(msg); });
}

bool LslidarX10Driver::checkPacketValidity(const Packet &packet, int packet_size) const {
  if (packet_size != packet_length || packet[0] != 0xA5 || packet[1] != 0x5A) {
    return false;
  }
  uint8_t checksum = 0;
  for (int index = 0; index < packet_length - 1; ++index)
    checksum += packet[index];
  return checksum == packet[packet_length - 1];
}

void LslidarX10Driver::decodePacket(const Packet &packet) {
  const int start_angle = ((packet[5] << 8) + packet[6]) % 36000;
  const int end_angle = ((packet[105] << 8) + packet[106]) % 36000;
  const int interval = (end_angle - start_angle + 36000) % 36000;
  const float increment = static_cast<float>(interval) / (packet_points_max - 1);
  points.clear();
  for (int group = 0; group < packet_points_max; ++group) {
    const int angle = (start_angle + static_cast<int>(increment * group)) % 36000;
    for (int echo = 0; echo < 2; ++echo) {
      const size_t offset = data_bits_start + group * 6 + echo * 3;
      points.push_back({angle,
                        static_cast<float>((packet[offset] << 8) + packet[offset + 1]) * 0.001f,
                        static_cast<float>(packet[offset + 2])});
    }
  }
}

void LslidarX10Driver::appendPoints(size_t begin, size_t end) {
  for (size_t index = begin; index < end; ++index) {
    const auto &firing = points[index];
    if (firing.distance < min_range || firing.distance > max_range)
      continue;
    pcl::PointXYZI point;
    point.x = firing.distance * cos_azimuth_table[firing.azimuth];
    point.y = firing.distance * sin_azimuth_table[firing.azimuth];
    point.z = 0.0f;
    point.intensity = firing.intensity;
    cloud_->points.push_back(point);
    ++cloud_->width;
  }
}

bool LslidarX10Driver::poll() {
  if (!motor_running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return true;
  }
  Packet packet{};
  int packet_size;
  do {
    packet_size = data_acquisition_strategy_->getPacket(packet);
    if (packet_size < 0)
      return false;
  } while (packet_size == 0 && rclcpp::ok());
  if (!rclcpp::ok())
    return false;

  if (!checkPacketValidity(packet, packet_size))
    return true;
  decodePacket(packet);
  const double packet_time = node_->now().seconds();
  size_t boundary = 0;
  while (boundary < points.size()) {
    if (std::abs(points[boundary].azimuth - last_azimuth_) > 35000)
      break;
    last_azimuth_ = points[boundary++].azimuth;
  }
  size_t begin = 0, end = boundary;
  if (first_sweep_) {
    if (boundary == points.size())
      return true;
    first_sweep_ = false;
    begin = boundary;
    end = points.size();
  }
  appendPoints(begin, end);
  if (end != points.size()) {
    const double frame_time = last_packet_time_ > 0.0
                                  ? packet_time - (packet_time - last_packet_time_) *
                                                      (points.size() - end) / points.size()
                                  : packet_time;
    const double duration = last_frame_time_ > 0.0 ? frame_time - last_frame_time_ : 0.0;
    last_frame_time_ = frame_time;
    auto frame = std::move(cloud_);
    publisher_->enqueue(
        [this, frame, frame_time, duration]() { publishFrame(frame, frame_time, duration); });
    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud_->header.frame_id = frame_id;
    cloud_->height = 1;
    appendPoints(end, points.size());
  }
  last_azimuth_ = points.back().azimuth;
  last_packet_time_ = packet_time;
  return true;
}

void LslidarX10Driver::pointcloudToLaserscan(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                             sensor_msgs::msg::LaserScan &scan) const {
  pcl_conversions::fromPCL(cloud.header, scan.header);
  scan.angle_min = -M_PI;
  const double bin_width = 2.0 * M_PI / points_size;
  scan.angle_increment = static_cast<float>(bin_width);
  scan.angle_max = scan.angle_min + scan.angle_increment * (points_size - 1);
  scan.range_min = min_range;
  scan.range_max = max_range;
  scan.ranges.assign(points_size, std::numeric_limits<float>::infinity());
  scan.intensities.assign(points_size, std::numeric_limits<float>::quiet_NaN());

  for (const auto &point : cloud.points) {
    const double x = point.x, y = point.y;
    const double range = std::hypot(x, y);
    double angle = std::atan2(point.y, point.x);
    if (std::abs(angle) >= M_PI)
      angle = -M_PI;
    const size_t index = std::min(static_cast<size_t>(points_size - 1),
                                  static_cast<size_t>(std::floor((angle + M_PI) / bin_width)));
    if (range < scan.ranges[index]) {
      scan.ranges[index] = range;
      scan.intensities[index] = point.intensity;
    }
  }

  scan.time_increment = 0.0f;
}

void LslidarX10Driver::publishFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double frame_time,
                                    double frame_duration) {
  const rclcpp::Time stamp(static_cast<int64_t>(frame_time * 1e9));
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  pointcloudToLaserscan(*cloud, *scan);
  scan->header.stamp = stamp;
  scan->scan_time = static_cast<float>(std::max(0.0, frame_duration));
  laserscan_pub_->publish(std::move(scan));
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud, *message);
  message->header.stamp = stamp;
  pointcloud_pub_->publish(std::move(message));
}

void LslidarX10Driver::motorControl(const std_msgs::msg::Int8::SharedPtr msg) {
  if (msg->data != 0 && msg->data != 1)
    return;
  std::array<char, 188> command{};
  command[0] = static_cast<char>(0xA5);
  command[1] = 0x5A;
  command[2] = 0x55;
  command[184] = msg->data == 0 ? 0x03 : 0x01;
  command[185] = msg->data;
  command[186] = static_cast<char>(0xFA);
  command[187] = static_cast<char>(0xFB);
  if (serial_->send(command.data(), command.size()) == static_cast<ssize_t>(command.size())) {
    motor_running_.store(msg->data == 1);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send LiDAR motor command");
  }
}

}
