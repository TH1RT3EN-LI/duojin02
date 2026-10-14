// SPDX-License-Identifier: Apache-2.0
// Offline replay of saved bytes through the production decoder, poll(),
// publication worker and ROS subscribers. This subclass never opens a device.
#include "lslidar_driver/lslidar_x10_driver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace lslidar_driver;

namespace {
class SavedPackets : public DataAcquisitionStrategy {
 public:
  std::vector<Packet> packets;
  size_t next{0};
  int getPacket(Packet &packet) override {
    if (next == packets.size()) return -1;
    packet = packets[next++];
    return LslidarX10Driver::packet_length;
  }
};

class ReplayDriver : public LslidarX10Driver {
 public:
  using LslidarX10Driver::LslidarX10Driver;
  using LslidarX10Driver::pointcloudToLaserscan;
  void createRosIO() override {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, qos);
    laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(san_topic_, qos);
    if (publish_raw_) {
      raw_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(raw_pointcloud_topic_, qos);
      raw_laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(raw_scan_topic_, qos);
    }
    // No serial object, SerialStrategy or motor_control subscription is created.
  }
};

SelfFilterConfig readFilter(const rclcpp::Node &node) {
  SelfFilterConfig config;
  config.enabled = node.get_parameter("self_filter.enabled").as_bool();
  config.laser_to_body_xyz_m = node.get_parameter("self_filter.laser_to_body_xyz_m").as_double_array();
  config.laser_to_body_rpy_rad = node.get_parameter("self_filter.laser_to_body_rpy_rad").as_double_array();
  config.body_polygon_xy_m = node.get_parameter("self_filter.body_polygon_xy_m").as_double_array();
  config.body_z_bounds_m = node.get_parameter("self_filter.body_z_bounds_m").as_double_array();
  config.boundary_ambiguity_m = node.get_parameter("self_filter.boundary_ambiguity_m").as_double();
  return config;
}

// Independent XY projection label with the original capture's 1 um edge
// tolerance. These labels denote model projection, not verified cable identity.
int nominalLabel(Point3 p, const std::vector<double> &vertices) {
  bool inside = false;
  double edge = std::numeric_limits<double>::infinity();
  const size_t n = vertices.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double ax = vertices[2*j], ay = vertices[2*j+1];
    const double bx = vertices[2*i], by = vertices[2*i+1];
    const double dx = bx-ax, dy = by-ay;
    const double t = std::clamp(((p.x-ax)*dx+(p.y-ay)*dy)/(dx*dx+dy*dy), 0., 1.);
    edge = std::min(edge, std::hypot(p.x-ax-t*dx, p.y-ay-t*dy));
    if ((ay > p.y) != (by > p.y) && p.x < (bx-ax)*(p.y-ay)/(by-ay)+ax) inside = !inside;
  }
  if (edge <= 1e-6) return 0;
  return inside ? 1 : -1;
}

size_t bin(const pcl::PointXYZI &p) {
  double angle = std::atan2(p.y, p.x);
  if (std::abs(angle) >= M_PI) angle = -M_PI;
  return std::min<size_t>(539, static_cast<size_t>(std::floor((angle+M_PI)/(2.*M_PI/540.))));
}

bool sameFloat(float a, float b) {
  return (std::isnan(a) && std::isnan(b)) || a == b;
}
bool samePoint(const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity;
}
void require(bool condition, const std::string &what) {
  if (!condition) throw std::runtime_error(what);
}
std::string jsonString(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c == '\n') out << "\\n";
    else out << c;
  }
  out << '"';
  return out.str();
}
template<class T> void jsonVector(std::ostream &out, const std::vector<T> &values) {
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << values[i]; }
  out << ']';
}

struct Frame {
  pcl::PointCloud<pcl::PointXYZI> raw, kept;
  sensor_msgs::msg::LaserScan scan, raw_scan;
  size_t echo0{0}, echo1{0}, removed{0}, ambiguous{0}, outside{0}, nominal_outside{0};
  size_t old_near_outside{0}, old_near_outside_kept{0}, nominal_outside_removed{0};
  size_t near_removed_far_retained_pairs{0};
};
}

int main(int argc, char **argv) {
  if (argc < 5 || argc > 7) {
    std::cerr << "usage: replay_real_lidar_crop RAW.bin HARDWARE.yaml RESULT.json FRAMES.csv [EXPECTED_FRAMES [EXPECTED_ALL_OLD_NEAR_OUTSIDE]]\n";
    return 2;
  }
  int ros_argc = 0;
  char **ros_argv = nullptr;
  rclcpp::init(ros_argc, ros_argv);
  try {
    const std::string raw_path = argv[1], config_path = argv[2];
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", config_path});
    auto node = std::make_shared<rclcpp::Node>("lslidar_driver_node", options);
    auto driver = std::make_unique<ReplayDriver>(node);
    driver->initialize();
    const SelfFilterConfig config = readFilter(*node);
    require(config.enabled, "replay requires enabled self filter");
    require(node->get_parameter("self_filter.publish_raw").as_bool(), "replay requires raw publications");
    const SelfFilter filter(config);
    const double min_range = node->get_parameter("min_range").as_double();
    const double max_range = node->get_parameter("max_range").as_double();
    std::ifstream raw_stream(raw_path, std::ios::binary);
    require(static_cast<bool>(raw_stream), "cannot open saved raw file");
    const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(raw_stream)), {});
    auto source = std::make_shared<SavedPackets>();
    for (size_t offset = 0; offset + 108 <= bytes.size();) {
      Packet packet{};
      std::copy_n(bytes.begin()+offset, 108, packet.begin());
      if (driver->checkPacketValidity(packet, 108)) {
        source->packets.push_back(packet);
        offset += 108;
      } else ++offset;
    }
    require(!source->packets.empty(), "no checksum-valid saved packets");

    // Use the real decoder to retain both echo identities before reduction.
    std::vector<Frame> frames;
    Frame current;
    std::vector<size_t> frames_after_packet;
    bool have_first_wrap = false;
    int last_angle = 0;
    size_t decoded_echoes = 0, all_positive_echoes = 0, all_old_near_outside = 0;
    size_t all_old_near_outside_kept = 0, all_nominal_outside_removed = 0;
    size_t protocol_zeroes = 0, below_sensor_floor = 0;
    std::array<float, 36000> cosine{}, sine{};
    for (size_t angle = 0; angle < 36000; ++angle) {
      const float radians = static_cast<float>(angle * M_PI / 18000.);
      cosine[angle] = std::cos(radians);
      sine[angle] = -std::sin(radians);
    }
    for (const auto &packet : source->packets) {
      driver->decodePacket(packet);
      decoded_echoes += driver->points.size();
      for (size_t group = 0; group < driver->points.size(); group += 2) {
        const int angle = driver->points[group].azimuth;
        if (std::abs(angle-last_angle) > 35000) {
          if (have_first_wrap) {
            current.raw.height = 1;
            current.raw.width = current.raw.points.size();
            current.raw.header.frame_id = node->get_parameter("frame_id").as_string();
            driver->pointcloudToLaserscan(current.raw, current.scan, &current.kept);
            driver->pointcloudToLaserscan(current.raw, current.raw_scan, nullptr, false);
            frames.push_back(std::move(current));
            current = Frame{};
          }
          have_first_wrap = true;
        }
        last_angle = angle;
        bool eligible[2]{}, removed[2]{};
        float distance[2]{};
        for (size_t echo = 0; echo < 2; ++echo) {
          const auto &firing = driver->points[group+echo];
          distance[echo] = firing.distance;
          if (firing.distance == 0) { ++protocol_zeroes; continue; }
          ++all_positive_echoes;
          pcl::PointXYZI p;
          p.x = firing.distance*cosine[angle]; p.y = firing.distance*sine[angle];
          p.z = 0.f; p.intensity = firing.intensity;
          const auto label = filter.classify({p.x,p.y,p.z});
          removed[echo] = label == SelfFilterClassification::Inside;
          const bool nominal_outside = nominalLabel(filter.toBody({p.x,p.y,p.z}), config.body_polygon_xy_m) < 0;
          const bool old_near_outside = nominal_outside && firing.distance < .3f;
          all_old_near_outside += old_near_outside;
          all_old_near_outside_kept += old_near_outside && !removed[echo];
          all_nominal_outside_removed += nominal_outside && removed[echo];
          // Match the decoded float32 representation of the configured sensor
          // limit; the protocol's exact 20 mm value must remain eligible.
          eligible[echo] = firing.distance >= static_cast<float>(min_range) &&
                           firing.distance <= static_cast<float>(max_range);
          if (!eligible[echo]) { ++below_sensor_floor; continue; }
          if (!have_first_wrap) continue;
          current.raw.points.push_back(p);
          if (echo == 0) ++current.echo0; else ++current.echo1;
          current.removed += removed[echo];
          current.ambiguous += label == SelfFilterClassification::Ambiguous;
          current.outside += label == SelfFilterClassification::Outside;
          current.nominal_outside += nominal_outside;
          current.nominal_outside_removed += nominal_outside && removed[echo];
          current.old_near_outside += old_near_outside;
          current.old_near_outside_kept += old_near_outside && !removed[echo];
        }
        if (have_first_wrap && eligible[0] && eligible[1] &&
            ((removed[0] && !removed[1] && distance[0]<distance[1]) ||
             (removed[1] && !removed[0] && distance[1]<distance[0])))
          ++current.near_removed_far_retained_pairs;
      }
      frames_after_packet.push_back(frames.size());
    }
    if (argc >= 6) require(frames.size() == std::stoul(argv[5]), "unexpected complete frame count");
    if (argc >= 7) require(all_old_near_outside == std::stoul(argv[6]), "unexpected all-packet old-gate external count");
    require(all_old_near_outside == all_old_near_outside_kept, "all-packet external near point removed");
    require(all_nominal_outside_removed == 0, "point outside nominal polygon was removed");

    std::vector<sensor_msgs::msg::LaserScan> scans, raw_scans;
    std::vector<sensor_msgs::msg::PointCloud2> clouds, raw_clouds;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const std::vector<std::string> topics{
      node->get_parameter("laserscan_topic").as_string(),
      node->get_parameter("self_filter.raw_scan_topic").as_string(),
      node->get_parameter("pointcloud_topic").as_string(),
      node->get_parameter("self_filter.raw_pointcloud_topic").as_string()};
    auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(topics[0], qos,
      [&scans](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { scans.push_back(*m); });
    auto raw_scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(topics[1], qos,
      [&raw_scans](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { raw_scans.push_back(*m); });
    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topics[2], qos,
      [&clouds](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { clouds.push_back(*m); });
    auto raw_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topics[3], qos,
      [&raw_clouds](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { raw_clouds.push_back(*m); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto discovery_deadline = std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (true) {
      executor.spin_some();
      if (std::all_of(topics.begin(), topics.end(), [&node](const auto &topic) { return node->count_subscribers(topic)>0; })) break;
      require(std::chrono::steady_clock::now()<discovery_deadline, "ROS subscriber discovery timed out");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    driver->data_acquisition_strategy_ = source;
    for (size_t packet = 0; packet < source->packets.size(); ++packet) {
      require(driver->poll(), "saved packet poll failed");
      const auto target = frames_after_packet[packet];
      if (target == 0 || (scans.size()>=target && raw_scans.size()>=target &&
                         clouds.size()>=target && raw_clouds.size()>=target)) continue;
      const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(3);
      while (scans.size()<target || raw_scans.size()<target || clouds.size()<target || raw_clouds.size()<target) {
        executor.spin_some();
        require(std::chrono::steady_clock::now()<deadline, "ROS publication delivery timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    require(!driver->poll(), "saved packet transport did not report EOF");
    driver.reset(); // Drain the real production publication worker.
    executor.spin_some();
    require(scans.size()==frames.size() && raw_scans.size()==frames.size() &&
            clouds.size()==frames.size() && raw_clouds.size()==frames.size(), "published frame count differs");
    std::ofstream rows(argv[4]);
    require(static_cast<bool>(rows), "cannot create frame CSV");
    rows << "frame,echo0,echo1,raw_points,retained_points,removed_points,ambiguous_kept,outside_kept,nominal_outside_kept,old_near_outside_kept,masked_only_nan_bins,promoted_far_bins,near_removed_far_retained_pairs,stamp_ns\n";
    size_t total_raw = 0, total_kept = 0, total_removed = 0, total_ambiguous = 0, total_outside = 0;
    size_t total_nominal_outside = 0, total_old_near = 0, total_nan = 0, total_promoted = 0;
    size_t total_pairs = 0, total_echo0 = 0, total_echo1 = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
      const auto &expected = frames[i];
      pcl::PointCloud<pcl::PointXYZI> actual_raw, actual_kept;
      pcl::fromROSMsg(raw_clouds[i], actual_raw); pcl::fromROSMsg(clouds[i], actual_kept);
      require(actual_raw.points.size()==expected.raw.points.size(), "raw publication lost echoes");
      require(actual_kept.points.size()==expected.kept.points.size(), "retained publication size differs");
      for (size_t p = 0; p < actual_raw.points.size(); ++p)
        require(samePoint(actual_raw.points[p],expected.raw.points[p]), "raw published point differs from decoded source");
      for (size_t p = 0; p < actual_kept.points.size(); ++p)
        require(samePoint(actual_kept.points[p],expected.kept.points[p]), "retained published point differs from exact mask");
      require(actual_raw.points.size()-actual_kept.points.size()==expected.removed, "classifier and publication removed counts differ");
      // Check every externally labelled near source record against the retained
      // sequence without treating duplicate coordinates as one observation.
      size_t retained_index = 0, verified_near_outside = 0;
      for (const auto &p : actual_raw.points) {
        if (filter.shouldRemove({p.x,p.y,p.z})) continue;
        require(retained_index<actual_kept.points.size() && samePoint(p,actual_kept.points[retained_index++]), "retained order/source occurrence differs");
        verified_near_outside += nominalLabel(filter.toBody({p.x,p.y,p.z}), config.body_polygon_xy_m)<0 && std::hypot(p.x,p.y)<.3;
      }
      require(verified_near_outside==expected.old_near_outside, "external near occurrence missing from retained output");
      require(expected.old_near_outside==expected.old_near_outside_kept && expected.nominal_outside_removed==0, "external point was cropped");
      require(scans[i].header.stamp==raw_scans[i].header.stamp && scans[i].header.stamp==clouds[i].header.stamp &&
              scans[i].header.stamp==raw_clouds[i].header.stamp, "raw/filtered output stamps differ");
      require(scans[i].header.frame_id==expected.raw.header.frame_id && raw_scans[i].header.frame_id==expected.raw.header.frame_id,
              "scan frame_id differs");
      require(scans[i].ranges.size()==540 && raw_scans[i].ranges.size()==540, "scan bin count differs");
      size_t nan_bins = 0, promoted = 0;
      for (size_t b = 0; b < 540; ++b) {
        require(sameFloat(scans[i].ranges[b],expected.scan.ranges[b]) &&
                sameFloat(scans[i].intensities[b],expected.scan.intensities[b]) &&
                sameFloat(raw_scans[i].ranges[b],expected.raw_scan.ranges[b]) &&
                sameFloat(raw_scans[i].intensities[b],expected.raw_scan.intensities[b]), "published scan differs from exact conversion");
        bool masked = false, retained = false;
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &p : actual_raw.points) {
          if (bin(p)!=b) continue;
          if (filter.shouldRemove({p.x,p.y,p.z})) masked = true;
          else { retained = true; nearest = std::min(nearest,std::hypot(double(p.x),double(p.y))); }
        }
        if (masked && !retained) { require(std::isnan(scans[i].ranges[b]), "self-only bin became free space"); ++nan_bins; }
        if (retained) require(std::abs(scans[i].ranges[b]-nearest)<1e-6, "nearest retained echo differs");
        if (std::isfinite(scans[i].ranges[b]) && std::isfinite(raw_scans[i].ranges[b]) &&
            scans[i].ranges[b]>raw_scans[i].ranges[b]+1e-5) ++promoted;
      }
      const int64_t stamp = rclcpp::Time(scans[i].header.stamp).nanoseconds();
      if (i) require(stamp>rclcpp::Time(scans[i-1].header.stamp).nanoseconds(), "frame timestamps are not increasing");
      rows << i << ',' << expected.echo0 << ',' << expected.echo1 << ',' << actual_raw.points.size() << ',' << actual_kept.points.size()
           << ',' << expected.removed << ',' << expected.ambiguous << ',' << expected.outside << ',' << expected.nominal_outside
           << ',' << verified_near_outside << ',' << nan_bins << ',' << promoted << ',' << expected.near_removed_far_retained_pairs << ',' << stamp << '\n';
      total_echo0 += expected.echo0; total_echo1 += expected.echo1; total_raw += actual_raw.points.size(); total_kept += actual_kept.points.size();
      total_removed += expected.removed; total_ambiguous += expected.ambiguous; total_outside += expected.outside;
      total_nominal_outside += expected.nominal_outside; total_old_near += verified_near_outside;
      total_nan += nan_bins; total_promoted += promoted; total_pairs += expected.near_removed_far_retained_pairs;
    }
    require(total_removed>0 && total_nan>0 && total_promoted>0 && total_pairs>0, "capture did not exercise required crop cases");
    std::ofstream result(argv[3]);
    require(static_cast<bool>(result), "cannot create result JSON");
    result << std::setprecision(17) << "{\n  \"status\":\"passed\",\n  \"raw_path\":" << jsonString(raw_path)
      << ",\n  \"configuration_path\":" << jsonString(config_path) << ",\n  \"scope\":\"saved bytes -> native decoder -> poll -> publication worker -> ROS subscriptions; no device\","
      << "\n  \"device_opened\":false,\n  \"motor_subscription_created\":false,\n  \"command_bytes_written\":0,"
      << "\n  \"valid_packets\":" << source->packets.size() << ",\n  \"decoded_dual_echo_records\":" << decoded_echoes
      << ",\n  \"all_nonzero_echo_records\":" << all_positive_echoes << ",\n  \"protocol_zero_records\":" << protocol_zeroes
      << ",\n  \"outside_sensor_range_nonzero_records\":" << below_sensor_floor
      << ",\n  \"complete_sweeps\":" << frames.size() << ",\n  \"raw_published_points\":" << total_raw
      << ",\n  \"raw_echo0_points\":" << total_echo0 << ",\n  \"raw_echo1_points\":" << total_echo1
      << ",\n  \"filtered_published_points\":" << total_kept << ",\n  \"removed_points\":" << total_removed
      << ",\n  \"ambiguous_kept_points\":" << total_ambiguous << ",\n  \"outside_kept_points\":" << total_outside
      << ",\n  \"nominal_projection_outside_kept_points\":" << total_nominal_outside
      << ",\n  \"nominal_projection_outside_removed_points\":0,\n  \"old_0_3_gate_external_near_points_all_packets\":" << all_old_near_outside
      << ",\n  \"old_0_3_gate_external_near_points_all_packets_retained_by_geometry\":" << all_old_near_outside_kept
      << ",\n  \"old_0_3_gate_external_near_points_complete_sweeps_published\":" << total_old_near
      << ",\n  \"self_only_bins_published_nan\":" << total_nan << ",\n  \"bins_promoted_to_retained_far_echo\":" << total_promoted
      << ",\n  \"same_group_near_removed_far_retained_pairs\":" << total_pairs
      << ",\n  \"min_range_m\":" << min_range << ",\n  \"max_range_m\":" << max_range
      << ",\n  \"boundary_ambiguity_m\":" << config.boundary_ambiguity_m << ",\n  \"laser_to_body_xyz_m\":";
    jsonVector(result,config.laser_to_body_xyz_m); result << ",\n  \"laser_to_body_rpy_rad\":"; jsonVector(result,config.laser_to_body_rpy_rad);
    result << ",\n  \"body_polygon_xy_m\":"; jsonVector(result,config.body_polygon_xy_m);
    result << ",\n  \"body_z_bounds_m\":"; jsonVector(result,config.body_z_bounds_m);
    result << ",\n  \"limitations\":[\"Outside means nominal XY projection, not verified cable identity.\",\"Heading and optical height are not physically calibrated.\",\"30 mm boundary band is a keep policy, not measured error.\",\"Head and tail partial sweeps are checked pointwise but not published as complete scans.\",\"Playback timestamps are host execution time, not original sensor acquisition time.\",\"ROS delivery and exact-mask consistency do not establish live navigation accuracy.\"]\n}\n";
    require(static_cast<bool>(result) && static_cast<bool>(rows), "result file write failed");
    std::cout << "REAL_LIDAR_CROP_REPLAY_PASSED complete_sweeps=" << frames.size()
      << " raw_points=" << total_raw << " removed=" << total_removed
      << " preserved_complete_external_near=" << total_old_near << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "REAL_LIDAR_CROP_REPLAY_FAILED " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
