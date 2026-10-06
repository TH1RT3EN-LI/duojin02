// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <thread>
#include "lslidar_driver/lslidar_x10_driver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<rclcpp::Node>("lslidar_driver_node", options);
    lslidar_driver::LslidarX10Driver driver(node);
    try {
        driver.initialize();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(node->get_logger(), "LiDAR initialization failed: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    std::atomic<bool> failed{false};
    std::thread acquisition([&]() {
        while (rclcpp::ok()) {
            if (!driver.poll()) {
                if (rclcpp::ok()) {
                    RCLCPP_ERROR(node->get_logger(), "LiDAR acquisition failed");
                    failed.store(true);
                    rclcpp::shutdown();
                }
                break;
            }
        }
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    acquisition.join();
    return failed.load() ? 1 : 0;
}
