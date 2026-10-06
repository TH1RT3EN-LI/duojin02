// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "lslidar_driver/publication_worker.hpp"
#include "lslidar_driver/data_acquisition_strategy.hpp"

namespace lslidar_driver {

struct FiringX10 {
    int azimuth;
    float distance;
    float intensity;
};

// One hardware protocol: N10Plus, 460800 baud, 108-byte dual-echo packets at 10 Hz.
class LslidarX10Driver {
public:
    explicit LslidarX10Driver(rclcpp::Node::SharedPtr node);
    virtual ~LslidarX10Driver();

    void initialize();
    virtual void createRosIO();
    bool poll();
    bool checkPacketValidity(const Packet &packet,
                             int packet_size) const;
    void decodePacket(const Packet &packet);
    void motorControl(const std_msgs::msg::Int8::SharedPtr msg);

    static constexpr int packet_length = 108;
    static constexpr int packet_points_max = 16;
    static constexpr int angle_bits_start = 5;
    static constexpr int data_bits_start = 7;
    static constexpr int end_angle_bits_start = 105;
    static constexpr int points_size = 540;

    std::shared_ptr<DataAcquisitionStrategy> data_acquisition_strategy_;
    std::vector<FiringX10> points;

protected:
    void pointcloudToLaserscan(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                              sensor_msgs::msg::LaserScan &scan) const;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    std::string serial_port_, frame_id, san_topic_, pointcloud_topic;

private:
    void appendPoints(size_t begin, size_t end);
    void publishFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                      double frame_time, double frame_duration);

    double min_range, max_range;
    std::array<float, 36000> cos_azimuth_table{}, sin_azimuth_table{};
    int last_azimuth_{0};
    bool first_sweep_{true};
    double last_packet_time_{0.0}, last_frame_time_{0.0};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    std::shared_ptr<LSIOSR> serial_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr motor_control_sub_;
    std::atomic<bool> motor_running_{true};
    std::unique_ptr<PublicationWorker> publisher_;
};

} // namespace lslidar_driver
