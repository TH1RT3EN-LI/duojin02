// SPDX-License-Identifier: Apache-2.0
// Offline protocol and ROS-output tests; no physical device is opened.
#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <poll.h>
#include <thread>
#include "lslidar_driver/lslidar_x10_driver.hpp"

using lslidar_driver::Packet;
using lslidar_driver::LslidarX10Driver;

namespace {
Packet makePacket(int start_angle, int end_angle, int background_distance = 1000) {
  Packet packet{};
  packet.fill(0);
  packet[0] = 0xA5;
  packet[1] = 0x5A;
  packet[2] = 108;
  const auto write16 = [&packet](int index, int value) {
    packet[index] = value >> 8;
    packet[index + 1] = value & 0xFF;
  };
  write16(5, start_angle % 36000);
  write16(105, end_angle % 36000);
  const int interval = (end_angle - start_angle + 36000) % 36000;
  for (int group = 0; group < 16; ++group) {
    const int angle = (start_angle + interval * group / 15) % 36000;
    const int distance = background_distance && angle >= 8900 && angle <= 9100 ? 2300 : background_distance;
    write16(7 + group * 6, distance);
    packet[9 + group * 6] = 41;
    write16(10 + group * 6, background_distance ? 3200 : 0);
    packet[12 + group * 6] = 72;
  }
  uint8_t checksum = 0;
  for (size_t i = 0; i < 107; ++i) checksum += packet[i];
  packet[107] = checksum;
  return packet;
}

class PacketSource : public lslidar_driver::DataAcquisitionStrategy {
 public:
  std::vector<Packet> packets;
  size_t next = 0;
  int getPacket(Packet &packet) override {
    if (next >= packets.size()) return -1;
    packet = std::move(packets[next++]);
    return 108;
  }
};

class OfflineX10Driver : public LslidarX10Driver {
 public:
  using LslidarX10Driver::LslidarX10Driver;
  using LslidarX10Driver::pointcloudToLaserscan;
  int io_created = 0;
  void createRosIO() override {
    ++io_created;
    pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
    laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(san_topic_, 10);
    if (publish_raw_) {
      raw_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(raw_pointcloud_topic_, 10);
      raw_laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(raw_scan_topic_, 10);
    }
  }
};

class X10Test : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void createDriver(const std::vector<rclcpp::Parameter> &overrides = {},
                    const std::vector<std::string> &extra_ros_arguments = {},
                    bool load_hardware_config = true) {
    driver.reset();
    node.reset();
    rclcpp::NodeOptions options;
    std::vector<std::string> arguments{"--ros-args"};
    if (load_hardware_config)
      arguments.insert(arguments.end(), {"--params-file", HARDWARE_LIDAR_CONFIG});
    arguments.insert(arguments.end(), extra_ros_arguments.begin(), extra_ros_arguments.end());
    options.arguments(arguments);
    options.parameter_overrides(overrides);
    node = std::make_shared<rclcpp::Node>("lslidar_driver_node", options);
    driver.reset(new OfflineX10Driver(node));
  }
  void SetUp() override {
    createDriver();
    driver->initialize();
  }
  void TearDown() override {
    driver.reset();
    node.reset();
  }
  pcl::PointCloud<pcl::PointXYZI> cloud(const std::vector<pcl::PointXYZI> &points) {
    pcl::PointCloud<pcl::PointXYZI> source;
    source.points.assign(points.begin(), points.end());
    source.width = source.points.size();
    source.height = 1;
    source.header.frame_id = "laser";
    source.header.stamp = 123400000;
    return source;
  }
  pcl::PointXYZI point(float x, float y, float intensity) {
    pcl::PointXYZI p;
    p.x = x; p.y = y; p.z = 0.0f; p.intensity = intensity;
    return p;
  }
  std::vector<sensor_msgs::msg::LaserScan> collectFrames(int angle_offset = 0, bool empty = false) {
    std::vector<sensor_msgs::msg::LaserScan> received;
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { received.push_back(*scan); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    for (int attempt = 0; attempt < 50 && node->count_subscribers("/scan") == 0; ++attempt) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto source = std::make_shared<PacketSource>();
    for (int turn = 0; turn < 4; ++turn) {
      for (int group = 0; group < 24; ++group) {
        source->packets.push_back(makePacket(angle_offset + group * 1500, angle_offset + (group + 1) * 1500, empty ? 0 : 1000 + turn * 100));
      }
    }
    driver->data_acquisition_strategy_ = source;
    for (int packet = 0; packet < 96; ++packet) EXPECT_TRUE(driver->poll());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.size() < 3 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.spin_some();
    return received;
  }
  struct PairedFrames {
    std::vector<sensor_msgs::msg::LaserScan> scans, raw_scans;
    std::vector<sensor_msgs::msg::PointCloud2> clouds, raw_clouds;
  };
  PairedFrames collectPairedPackets(const std::vector<Packet> &packets, size_t expected) {
    PairedFrames received;
    auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.scans.push_back(*msg); });
    auto raw_scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_raw", 10, [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.raw_scans.push_back(*msg); });
    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lslidar_point_cloud", 10, [&received](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { received.clouds.push_back(*msg); });
    auto raw_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lslidar_point_cloud_raw", 10, [&received](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { received.raw_clouds.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    for (int attempt = 0; attempt < 100; ++attempt) {
      executor.spin_some();
      if (node->count_subscribers("/scan") && node->count_subscribers("/scan_raw") &&
          node->count_subscribers("/lslidar_point_cloud") && node->count_subscribers("/lslidar_point_cloud_raw")) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto source = std::make_shared<PacketSource>();
    source->packets = packets;
    driver->data_acquisition_strategy_ = source;
    for (size_t i = 0; i < packets.size(); ++i) {
      EXPECT_TRUE(driver->poll());
      executor.spin_some();
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((received.scans.size() < expected || received.raw_scans.size() < expected ||
            received.clouds.size() < expected || received.raw_clouds.size() < expected) &&
           std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    executor.spin_some();
    return received;
  }
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<OfflineX10Driver> driver;
};

std::vector<rclcpp::Parameter> syntheticSelfFilterParameters() {
  return {
    rclcpp::Parameter("self_filter.enabled", true),
    rclcpp::Parameter("self_filter.laser_to_body_xyz_m", std::vector<double>{0.09, 0.0, 0.14}),
    rclcpp::Parameter("self_filter.laser_to_body_rpy_rad", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("self_filter.body_polygon_xy_m", std::vector<double>{-0.21, -0.2, 0.21, -0.2, 0.21, 0.2, -0.21, 0.2}),
    rclcpp::Parameter("self_filter.body_z_bounds_m", std::vector<double>{0.12, 0.16}),
    rclcpp::Parameter("self_filter.boundary_ambiguity_m", 0.0),
  };
}

TEST_F(X10Test, HardwareConfigurationMatchesFixedPacketLayout) {
  EXPECT_EQ(driver->packet_length, 108);
  EXPECT_EQ(driver->packet_points_max, 16);
  EXPECT_EQ(driver->angle_bits_start, 5);
  EXPECT_EQ(driver->data_bits_start, 7);
  EXPECT_EQ(driver->end_angle_bits_start, 105);
  EXPECT_EQ(driver->points_size, 540);
  EXPECT_EQ(node->get_parameter("serial_port").as_string(), "/dev/lslidar");
  EXPECT_EQ(node->get_parameter("laserscan_topic").as_string(), "/scan");
  EXPECT_EQ(node->get_parameter("frame_id").as_string(), "laser");
  EXPECT_FALSE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_TRUE(node->get_parameter("self_filter.enabled").as_bool());
  EXPECT_TRUE(node->get_parameter("self_filter.publish_raw").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("min_range").as_double(), 0.02);
  EXPECT_DOUBLE_EQ(node->get_parameter("self_filter.boundary_ambiguity_m").as_double(), 0.0);
}

TEST_F(X10Test, DecodesBothEchoesAcrossZeroAngle) {
  auto packet = makePacket(35000, 500);
  ASSERT_TRUE(driver->checkPacketValidity(packet, 108));
  driver->decodePacket(packet);
  ASSERT_EQ(driver->points.size(), 32u);
  EXPECT_EQ(driver->points.front().azimuth, 35000);
  EXPECT_EQ(driver->points.back().azimuth, 500);
  EXPECT_NEAR(driver->points[0].distance, 1.0f, 1e-6);
  EXPECT_NEAR(driver->points[1].distance, 3.2f, 1e-6);
  EXPECT_FLOAT_EQ(driver->points[0].intensity, 41.0f);
  EXPECT_FLOAT_EQ(driver->points[1].intensity, 72.0f);
}

TEST_F(X10Test, RejectsCorruptedAndTruncatedPackets) {
  auto packet = makePacket(0, 1500);
  EXPECT_TRUE(driver->checkPacketValidity(packet, 108));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 1206));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 0));
  EXPECT_FALSE(driver->checkPacketValidity(packet, -1));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 107));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 1207));
  ++packet[8];
  EXPECT_FALSE(driver->checkPacketValidity(packet, 108));
}

TEST_F(X10Test, KeepsNearestEchoAndConsistentScanGeometry) {
  auto input = cloud({point(2, 0, 72), point(1, 0, 41), point(-1, 0, 11),
                      point(-1, -0.0f, 12)});
  sensor_msgs::msg::LaserScan scan;
  driver->pointcloudToLaserscan(input, scan);
  ASSERT_EQ(scan.ranges.size(), 540u);
  EXPECT_FLOAT_EQ(scan.ranges[270], 1.0f);
  EXPECT_FLOAT_EQ(scan.intensities[270], 41.0f);
  EXPECT_FLOAT_EQ(scan.ranges[0], 1.0f);
  EXPECT_EQ(scan.header.frame_id, "laser");
  EXPECT_EQ(rclcpp::Time(scan.header.stamp).nanoseconds(), input.header.stamp * 1000);
  EXPECT_NEAR(scan.angle_max, scan.angle_min + scan.angle_increment * 539, 1e-6);
  EXPECT_FLOAT_EQ(scan.time_increment, 0.0f);
}

TEST_F(X10Test, FiltersBothEchoesBeforeNearestReductionAndPreservesRawCloud) {
  auto parameters = syntheticSelfFilterParameters();
  parameters.emplace_back("min_range", 0.02);  // Test-only sensor contract.
  createDriver(parameters);
  driver->initialize();
  auto input = cloud({point(0.10, 0.0, 41), point(0.15, 0.0, 72), point(0.0, 0.25, 83)});
  pcl::PointCloud<pcl::PointXYZI> retained;
  sensor_msgs::msg::LaserScan scan, raw_scan;
  driver->pointcloudToLaserscan(input, scan, &retained);
  driver->pointcloudToLaserscan(input, raw_scan, nullptr, false);
  EXPECT_FLOAT_EQ(scan.ranges[270], 0.15f);
  EXPECT_FLOAT_EQ(scan.intensities[270], 72.0f);
  EXPECT_FLOAT_EQ(raw_scan.ranges[270], 0.10f);
  ASSERT_EQ(retained.points.size(), 2u);
  EXPECT_FLOAT_EQ(retained.points[0].x, 0.15f);
  EXPECT_FLOAT_EQ(retained.points[1].y, 0.25f);
  ASSERT_EQ(input.points.size(), 3u);
  EXPECT_FLOAT_EQ(input.points[0].x, 0.10f);
  EXPECT_EQ(scan.header, raw_scan.header);
  EXPECT_EQ(retained.header.frame_id, input.header.frame_id);
}

TEST_F(X10Test, MaskedOnlyBinsBecomeUnknownWhileUntouchedBinsKeepLegacyInfinity) {
  createDriver(syntheticSelfFilterParameters());
  driver->initialize();
  const auto input = cloud({point(0.10, 0.0, 41)});
  sensor_msgs::msg::LaserScan scan;
  pcl::PointCloud<pcl::PointXYZI> retained;
  driver->pointcloudToLaserscan(input, scan, &retained);
  EXPECT_TRUE(std::isnan(scan.ranges[270]));
  EXPECT_TRUE(std::isnan(scan.intensities[270]));
  EXPECT_TRUE(std::isinf(scan.ranges[271]));
  EXPECT_TRUE(retained.points.empty());
}

TEST_F(X10Test, DisabledGeometryKeepsLegacyNearestReturnAndAllEchoes) {
  auto parameters = syntheticSelfFilterParameters();
  parameters[0] = rclcpp::Parameter("self_filter.enabled", false);
  createDriver(parameters);
  driver->initialize();
  const auto input = cloud({point(0.10, 0.0, 41), point(0.15, 0.0, 72)});
  sensor_msgs::msg::LaserScan scan;
  pcl::PointCloud<pcl::PointXYZI> retained;
  driver->pointcloudToLaserscan(input, scan, &retained);
  EXPECT_FLOAT_EQ(scan.ranges[270], 0.10f);
  EXPECT_FLOAT_EQ(scan.intensities[270], 41.0f);
  EXPECT_EQ(retained.points.size(), input.points.size());
}

TEST_F(X10Test, EnablingWithoutTransformOrHeightFailsBeforeOpeningDevice) {
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.enabled", true)}, {}, false), std::invalid_argument);
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.publish_raw", true),
                            rclcpp::Parameter("self_filter.raw_scan_topic", "/scan")}), std::invalid_argument);
}

TEST_F(X10Test, HardwareCropPreservesExternalNearBoundaryAndSecondEchoThroughPublication) {
  std::vector<Packet> packets;
  for (int turn = 0; turn < 4; ++turn) {
    for (int block = 0; block < 24; ++block) {
      auto packet = makePacket(block * 1500, (block + 1) * 1500, 0);
      for (int group = 0; group < 16; ++group) {
        const int angle = (block * 1500 + group * 100) % 36000;
        int first = 0, second = 0, intensity = 0;
        if (angle == 0) { first = 50; second = 700; intensity = 50; }
        if (angle == 9000) { first = 250; intensity = 25; }   // Outside right edge, within old .3 gate.
        if (angle == 18000) { first = 230; intensity = 23; } // Strictly inside the rear region.
        if (angle == 27000) { first = 180; intensity = 18; } // Strict interior, formerly retained by the 30 mm band.
        packet[7 + group * 6] = first >> 8;
        packet[8 + group * 6] = first & 0xff;
        packet[9 + group * 6] = intensity;
        packet[10 + group * 6] = second >> 8;
        packet[11 + group * 6] = second & 0xff;
        packet[12 + group * 6] = second ? 70 : 0;
      }
      uint8_t checksum = 0;
      for (size_t i = 0; i < 107; ++i) checksum += packet[i];
      packet[107] = checksum;
      packets.push_back(packet);
    }
  }
  const auto received = collectPairedPackets(packets, 3);
  ASSERT_EQ(received.scans.size(), 3u);
  ASSERT_EQ(received.raw_scans.size(), 3u);
  ASSERT_EQ(received.clouds.size(), 3u);
  ASSERT_EQ(received.raw_clouds.size(), 3u);
  for (size_t i = 0; i < 3; ++i) {
    pcl::PointCloud<pcl::PointXYZI> kept, raw;
    pcl::fromROSMsg(received.clouds[i], kept);
    pcl::fromROSMsg(received.raw_clouds[i], raw);
    const auto has = [](const auto &points, float intensity) {
      return std::any_of(points.begin(), points.end(), [intensity](const auto &p) { return p.intensity == intensity; });
    };
    EXPECT_TRUE(has(raw.points, 50));
    EXPECT_TRUE(has(raw.points, 23));
    EXPECT_TRUE(has(raw.points, 18));
    EXPECT_TRUE(has(kept.points, 25));
    EXPECT_FALSE(has(kept.points, 18));
    EXPECT_TRUE(has(kept.points, 70));
    EXPECT_FALSE(has(kept.points, 50));
    EXPECT_FALSE(has(kept.points, 23));
    EXPECT_NEAR(received.raw_scans[i].ranges[270], .05f, 1e-6);
    EXPECT_NEAR(received.scans[i].ranges[270], .70f, 1e-6);
    // The float32 sine lookup can place the 180 degree beam on either side
    // of the -pi/pi seam; both bins denote the same rear direction.
    EXPECT_TRUE(std::isnan(received.scans[i].ranges[0]) ||
                std::isnan(received.scans[i].ranges[539]));
    EXPECT_EQ(received.scans[i].header.stamp, received.raw_scans[i].header.stamp);
    EXPECT_EQ(received.scans[i].header.stamp, received.clouds[i].header.stamp);
    EXPECT_EQ(received.scans[i].header.stamp, received.raw_clouds[i].header.stamp);
  }
}

TEST_F(X10Test, SensorFloorKeepsExactlyTwentyMillimetresAndRejectsNineteenThroughPublication) {
  createDriver({rclcpp::Parameter("self_filter.enabled", false)});
  driver->initialize();
  std::vector<Packet> packets;
  for (int turn = 0; turn < 4; ++turn) {
    for (int block = 0; block < 24; ++block) {
      auto packet = makePacket(block * 1500, (block + 1) * 1500, 0);
      for (int group = 0; group < 16; ++group) {
        packet[8 + group * 6] = group == 1 ? 21 : 20;
        packet[9 + group * 6] = group == 1 ? 21 : 20;
        packet[11 + group * 6] = 19;
        packet[12 + group * 6] = 19;
      }
      uint8_t checksum = 0;
      for (size_t i = 0; i < 107; ++i) checksum += packet[i];
      packet[107] = checksum;
      packets.push_back(packet);
    }
  }
  const auto received = collectPairedPackets(packets, 3);
  ASSERT_EQ(received.raw_clouds.size(), 3u);
  ASSERT_EQ(received.clouds.size(), 3u);
  for (size_t i = 0; i < 3; ++i) {
    pcl::PointCloud<pcl::PointXYZI> raw, kept;
    pcl::fromROSMsg(received.raw_clouds[i], raw);
    pcl::fromROSMsg(received.clouds[i], kept);
    EXPECT_EQ(raw.points.size(), kept.points.size());
    EXPECT_TRUE(std::any_of(raw.points.begin(), raw.points.end(), [](const auto &p) { return p.intensity == 20; }));
    EXPECT_TRUE(std::any_of(raw.points.begin(), raw.points.end(), [](const auto &p) { return p.intensity == 21; }));
    EXPECT_FALSE(std::any_of(raw.points.begin(), raw.points.end(), [](const auto &p) { return p.intensity == 19; }));
  }
}

TEST_F(X10Test, RejectsRawTopicNamespaceAliasesAndRemapsToFilteredDestinations) {
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.publish_raw", true),
                            rclcpp::Parameter("self_filter.raw_scan_topic", "scan")}), std::invalid_argument);
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.publish_raw", true)},
                           {"-r", "/scan_raw:=/scan"}), std::invalid_argument);
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.publish_raw", true)},
                           {"-r", "/lslidar_point_cloud_raw:=/lslidar_point_cloud"}), std::invalid_argument);
  EXPECT_THROW(createDriver({rclcpp::Parameter("self_filter.publish_raw", true),
                            rclcpp::Parameter("self_filter.raw_pointcloud_topic", "/scan_raw")}), std::invalid_argument);
}

TEST_F(X10Test, DropsDamagedPacketAndReportsTransportFailure) {
  auto source = std::make_shared<PacketSource>();
  auto damaged = makePacket(0, 1500);
  ++damaged[8];
  source->packets.push_back(std::move(damaged));
  source->packets.push_back(makePacket(0, 1500));
  driver->data_acquisition_strategy_ = source;
  EXPECT_TRUE(driver->poll());
  EXPECT_TRUE(driver->points.empty());
  EXPECT_TRUE(driver->poll());
  EXPECT_EQ(driver->points.size(), 32u);
  EXPECT_FALSE(driver->poll());
}

TEST_F(X10Test, PublishesCompleteFramesWithRobotAngleConvention) {
  const auto received = collectFrames();
  ASSERT_EQ(received.size(), 3u);
  for (size_t i = 0; i < received.size(); ++i) {
    const auto &scan = received[i];
    ASSERT_EQ(scan.ranges.size(), 540u);
    EXPECT_EQ(scan.header.frame_id, "laser");
    bool has_clockwise_target = false;
    for (int index = 134; index <= 136; ++index) {
      has_clockwise_target |= std::abs(scan.ranges[index] - 2.3f) < 1e-5;
    }
    EXPECT_TRUE(has_clockwise_target);
    // The initial partial revolution is discarded before the first zero crossing.
    EXPECT_NEAR(scan.ranges[405], 1.1f + i * 0.1f, 1e-5);
    if (i > 0) {
      EXPECT_GT(rclcpp::Time(scan.header.stamp).nanoseconds(),
                rclcpp::Time(received[i - 1].header.stamp).nanoseconds());
      EXPECT_GT(scan.scan_time, 0.0f);
    }
  }
}

TEST_F(X10Test, FirstZeroCrossingInsidePacketDoesNotPublishAPartialFrame) {
  const auto received = collectFrames(35900);
  ASSERT_EQ(received.size(), 3u);
  EXPECT_NEAR(received.front().ranges[270], 1.0f, 1e-5);
  EXPECT_NEAR(received.front().ranges[405], 1.0f, 1e-5);
}

TEST_F(X10Test, PublishesEmptySweepsInsteadOfLeavingStaleScanData) {
  const auto received = collectFrames(0, true);
  ASSERT_EQ(received.size(), 3u);
  for (const auto &scan : received) {
    ASSERT_EQ(scan.ranges.size(), 540u);
    EXPECT_TRUE(std::all_of(scan.ranges.begin(), scan.ranges.end(),
                           [](float range) { return std::isinf(range); }));
  }
}

TEST_F(X10Test, FiltersRawDistancesAndStillPublishesSparseSweeps) {
  createDriver({rclcpp::Parameter("min_range", 1.5), rclcpp::Parameter("max_range", 2.5)});
  driver->initialize();
  const auto received = collectFrames();
  ASSERT_EQ(received.size(), 3u);
  for (const auto &scan : received) {
    EXPECT_TRUE(std::isinf(scan.ranges[270]));
    EXPECT_TRUE(std::isinf(scan.ranges[405]));
    const auto valid = std::count_if(scan.ranges.begin(), scan.ranges.end(),
                                   [](float range) { return std::isfinite(range); });
    EXPECT_GT(valid, 0);
    EXPECT_LT(valid, 50);
    for (float range : scan.ranges) {
      if (std::isfinite(range)) EXPECT_NEAR(range, 2.3f, 1e-5);
    }
  }
}

TEST_F(X10Test, RawPublicationsKeepBothEchoesAndShareFilteredFrameStamps) {
  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("self_filter.enabled", true),
    rclcpp::Parameter("self_filter.laser_to_body_xyz_m", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("self_filter.laser_to_body_rpy_rad", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("self_filter.body_polygon_xy_m", std::vector<double>{-1.5, -1.5, 1.5, -1.5, 1.5, 1.5, -1.5, 1.5}),
    rclcpp::Parameter("self_filter.body_z_bounds_m", std::vector<double>{-0.1, 0.1}),
    rclcpp::Parameter("self_filter.publish_raw", true),
  };
  createDriver(parameters);
  driver->initialize();
  std::vector<sensor_msgs::msg::LaserScan> raw_scans;
  std::vector<sensor_msgs::msg::PointCloud2> raw_clouds, filtered_clouds;
  auto raw_scan_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", 10, [&raw_scans](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { raw_scans.push_back(*scan); });
  auto raw_cloud_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lslidar_point_cloud_raw", 10, [&raw_clouds](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { raw_clouds.push_back(*cloud); });
  auto filtered_cloud_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lslidar_point_cloud", 10, [&filtered_clouds](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { filtered_clouds.push_back(*cloud); });
  const auto filtered_scans = collectFrames();
  ASSERT_EQ(filtered_scans.size(), 3u);
  ASSERT_EQ(raw_scans.size(), filtered_scans.size());
  ASSERT_EQ(raw_clouds.size(), filtered_scans.size());
  ASSERT_EQ(filtered_clouds.size(), filtered_scans.size());
  for (size_t i = 0; i < filtered_scans.size(); ++i) {
    EXPECT_EQ(filtered_scans[i].header.stamp, raw_scans[i].header.stamp);
    EXPECT_EQ(filtered_scans[i].header.stamp, raw_clouds[i].header.stamp);
    EXPECT_EQ(filtered_scans[i].header.stamp, filtered_clouds[i].header.stamp);
    EXPECT_FLOAT_EQ(filtered_scans[i].scan_time, raw_scans[i].scan_time);
    EXPECT_NEAR(raw_scans[i].ranges[405], 1.1f + i * 0.1f, 1e-5);
    EXPECT_NEAR(filtered_scans[i].ranges[405], 3.2f, 1e-5);
    EXPECT_GT(raw_clouds[i].width, filtered_clouds[i].width);
    pcl::PointCloud<pcl::PointXYZI> raw_cloud, filtered_cloud;
    pcl::fromROSMsg(raw_clouds[i], raw_cloud);
    pcl::fromROSMsg(filtered_clouds[i], filtered_cloud);
    EXPECT_TRUE(std::all_of(filtered_cloud.points.begin(), filtered_cloud.points.end(),
        [](const auto &p) { return std::hypot(p.x, p.y) > 2.0; }));
    // Raw both-echo output keeps the near returns removed by the synthetic mask.
    EXPECT_TRUE(std::any_of(raw_cloud.points.begin(), raw_cloud.points.end(),
        [](const auto &p) { return std::hypot(p.x, p.y) < 1.5; }));
  }
}

TEST_F(X10Test, MotorCommandsUseFixedProtocolAndResumeAcquisition) {
  int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", ptsname(master))});
  auto actual_node = std::make_shared<rclcpp::Node>("motor_protocol_probe", options);
  LslidarX10Driver actual(actual_node);
  actual.initialize();
  for (int value : {0, 1}) {
    auto command = std::make_shared<std_msgs::msg::Int8>();
    command->data = value;
    actual.motorControl(command);
    pollfd descriptor{master, POLLIN, 0};
    ASSERT_EQ(::poll(&descriptor, 1, 1000), 1);
    std::array<uint8_t, 188> bytes{};
    ASSERT_EQ(::read(master, bytes.data(), bytes.size()), 188);
    EXPECT_EQ(bytes[0], 0xA5);
    EXPECT_EQ(bytes[1], 0x5A);
    EXPECT_EQ(bytes[2], 0x55);
    EXPECT_EQ(bytes[184], value == 0 ? 3 : 1);
    EXPECT_EQ(bytes[185], value);
    EXPECT_EQ(bytes[186], 0xFA);
    EXPECT_EQ(bytes[187], 0xFB);
    if (value == 0) EXPECT_TRUE(actual.poll());
  }
  auto source = std::make_shared<PacketSource>();
  source->packets.push_back(makePacket(0, 1500));
  actual.data_acquisition_strategy_ = source;
  ASSERT_TRUE(actual.poll());
  EXPECT_EQ(actual.points.size(), 32u);
  close(master);
}

TEST(PublicationWorkerTest, DrainsFramesInAcquisitionOrderAtDestruction) {
  std::vector<int> completed;
  {
    lslidar_driver::PublicationWorker worker;
    for (int index = 0; index < 1000; ++index) {
      worker.enqueue([&completed, index]() { completed.push_back(index); });
    }
  }
  ASSERT_EQ(completed.size(), 1000u);
  for (int index = 0; index < 1000; ++index) EXPECT_EQ(completed[index], index);
}

TEST_F(X10Test, ReassemblesFragmentedSerialPacketAndReportsDisconnect) {
  int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  auto expected = makePacket(0, 1500);
  lslidar_driver::LSIOSR serial(ptsname(master));
  std::thread writer([master, &expected]() {
    const uint8_t noise = 0x42;
    if (write(master, &noise, 1) != 1) return;
    for (size_t offset = 0; offset < 108;) {
      const size_t chunk = std::min<size_t>(7, 108 - offset);
      const ssize_t written = write(master, expected.data() + offset, chunk);
      if (written <= 0) return;
      offset += written;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  Packet packet{};
  const int noise_result = serial.getSerialData(packet);
  const int packet_size = serial.getSerialData(packet);
  writer.join();
  EXPECT_EQ(noise_result, 0);
  ASSERT_EQ(packet_size, 108);
  EXPECT_EQ(packet, expected);
  EXPECT_TRUE(driver->checkPacketValidity(packet, packet_size));
  close(master);
  EXPECT_EQ(serial.getSerialData(packet), -1);
}
}  // namespace
