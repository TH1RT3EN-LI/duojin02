// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <poll.h>
#include <thread>
#include <rclcpp/parameter_client.hpp>
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
    const int distance =
        background_distance && angle >= 8900 && angle <= 9100 ? 2300 : background_distance;
    write16(7 + group * 6, distance);
    packet[9 + group * 6] = 41;
    write16(10 + group * 6, background_distance ? 3200 : 0);
    packet[12 + group * 6] = 72;
  }
  uint8_t checksum = 0;
  for (size_t i = 0; i < 107; ++i)
    checksum += packet[i];
  packet[107] = checksum;
  return packet;
}

class PacketSource : public lslidar_driver::DataAcquisitionStrategy {
public:
  std::vector<Packet> packets;
  size_t next = 0;

  int getPacket(Packet &packet) override {
    if (next >= packets.size())
      return -1;
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
  }
};

class X10Test : public testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void createDriver(const std::vector<rclcpp::Parameter> &overrides = {}) {
    driver.reset();
    node.reset();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", HARDWARE_LIDAR_CONFIG});
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
    p.x = x;
    p.y = y;
    p.z = 0.0f;
    p.intensity = intensity;
    return p;
  }

  std::vector<sensor_msgs::msg::LaserScan> collectFrames(int angle_offset = 0, bool empty = false,
                                                         const std::string &topic = "/scan") {
    std::vector<sensor_msgs::msg::LaserScan> received;
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, 10, [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          received.push_back(*scan);
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    for (int attempt = 0; attempt < 50 && node->count_subscribers(topic) == 0; ++attempt) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto source = std::make_shared<PacketSource>();
    for (int turn = 0; turn < 4; ++turn) {
      for (int group = 0; group < 24; ++group) {
        source->packets.push_back(makePacket(angle_offset + group * 1500,
                                             angle_offset + (group + 1) * 1500,
                                             empty ? 0 : 1000 + turn * 100));
      }
    }
    driver->data_acquisition_strategy_ = source;
    for (int packet = 0; packet < 96; ++packet)
      EXPECT_TRUE(driver->poll());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.size() < 3 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.spin_some();
    return received;
  }

  rclcpp::Node::SharedPtr node;
  std::unique_ptr<OfflineX10Driver> driver;
};

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
}

TEST_F(X10Test, StartupOverridesRemainEffectiveAndRuntimeServicesRejectChanges) {
  const std::vector<rclcpp::Parameter> initial{
      rclcpp::Parameter("serial_port", "/tmp/lidar-startup-fixture"),
      rclcpp::Parameter("frame_id", "probe_laser"),
      rclcpp::Parameter("laserscan_topic", "/probe_scan"),
      rclcpp::Parameter("pointcloud_topic", "/probe_cloud"),
      rclcpp::Parameter("min_range", 1.5),
      rclcpp::Parameter("max_range", 2.5)};
  createDriver(initial);
  driver->initialize();
  auto client_node = std::make_shared<rclcpp::Node>("lidar_parameter_service_probe");
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(client_node,
                                                                node->get_fully_qualified_name());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  std::vector<std::string> names;
  std::vector<rclcpp::Parameter> changes;
  for (const auto &parameter : initial) {
    names.push_back(parameter.get_name());
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      changes.emplace_back(parameter.get_name(), parameter.as_string() + "_changed");
    else
      changes.emplace_back(parameter.get_name(), parameter.as_double() + .25);
  }
  auto before = client->get_parameters(names);
  ASSERT_EQ(executor.spin_until_future_complete(before, std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto values = before.get();
  ASSERT_EQ(values.size(), initial.size());
  for (size_t i = 0; i < initial.size(); ++i) {
    EXPECT_EQ(values[i].get_name(), initial[i].get_name());
    EXPECT_EQ(values[i].value_to_string(), initial[i].value_to_string());
  }
  auto described = client->describe_parameters(names);
  ASSERT_EQ(executor.spin_until_future_complete(described, std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto descriptors = described.get();
  ASSERT_EQ(descriptors.size(), names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(descriptors[i].name, names[i]);
    EXPECT_TRUE(descriptors[i].read_only);
  }
  auto changed = client->set_parameters(changes);
  ASSERT_EQ(executor.spin_until_future_complete(changed, std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto statuses = changed.get();
  ASSERT_EQ(statuses.size(), changes.size());
  for (const auto &status : statuses) {
    EXPECT_FALSE(status.successful);
    EXPECT_NE(status.reason.find("read-only"), std::string::npos);
  }
  auto atomic = client->set_parameters_atomically(changes);
  ASSERT_EQ(executor.spin_until_future_complete(atomic, std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(atomic.get().successful);
  auto after = client->get_parameters(names);
  ASSERT_EQ(executor.spin_until_future_complete(after, std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto unchanged = after.get();
  ASSERT_EQ(unchanged.size(), initial.size());
  for (size_t i = 0; i < initial.size(); ++i) {
    EXPECT_EQ(unchanged[i].get_name(), initial[i].get_name());
    EXPECT_EQ(unchanged[i].value_to_string(), initial[i].value_to_string());
  }
  EXPECT_EQ(node->count_publishers("/probe_scan"), 1u);
  EXPECT_EQ(node->count_publishers("/probe_cloud"), 1u);
  executor.remove_node(node);
  executor.remove_node(client_node);
  const auto received = collectFrames(0, false, "/probe_scan");
  ASSERT_EQ(received.size(), 3u);
  for (const auto &scan : received) {
    EXPECT_EQ(scan.header.frame_id, "probe_laser");
    EXPECT_FLOAT_EQ(scan.range_min, 1.5f);
    EXPECT_FLOAT_EQ(scan.range_max, 2.5f);
    EXPECT_TRUE(std::isinf(scan.ranges[270]));
    EXPECT_TRUE(std::any_of(scan.ranges.begin(), scan.ranges.end(),
                            [](float value) { return std::abs(value - 2.3f) < 1e-5f; }));
  }
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
  auto input = cloud({point(2, 0, 72), point(1, 0, 41), point(-1, 0, 11), point(-1, -0.0f, 12)});
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
      if (std::isfinite(range))
        EXPECT_NEAR(range, 2.3f, 1e-5);
    }
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
    if (value == 0)
      EXPECT_TRUE(actual.poll());
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
  for (int index = 0; index < 1000; ++index)
    EXPECT_EQ(completed[index], index);
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
    if (write(master, &noise, 1) != 1)
      return;
    for (size_t offset = 0; offset < 108;) {
      const size_t chunk = std::min<size_t>(7, 108 - offset);
      const ssize_t written = write(master, expected.data() + offset, chunk);
      if (written <= 0)
        return;
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
}
