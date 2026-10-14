

#include "duojin02_navigation/path_target.hpp"
#include "path_target_reference.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <time.h>
using namespace duojin02_navigation;

static double cpuNs() {
  timespec t{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t))
    throw std::runtime_error("thread clock failed");
  return static_cast<double>(t.tv_sec) * 1e9 + t.tv_nsec;
}

static void hashValue(uint64_t &hash, uint64_t bits) {
  for (int i = 0; i < 8; ++i) {
    hash ^= (bits >> (i * 8)) & 255;
    hash *= 1099511628211ULL;
  }
}

static uint64_t bits(double value) {
  uint64_t result;
  std::memcpy(&result, &value, sizeof(value));
  return result;
}

static void hashPose(uint64_t &hash, const geometry_msgs::msg::PoseStamped &p) {
  for (double d : {p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x,
                   p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w})
    hashValue(hash, bits(d));
  hashValue(hash, p.header.stamp.sec);
  hashValue(hash, p.header.stamp.nanosec);
  for (unsigned char c : p.header.frame_id) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Usage: baseline|candidate point-count reuse|replace-every-cycle corpus-out\n";
    return 2;
  }
  const std::string arm = argv[1], mode = argv[3];
  const int count = std::stoi(argv[2]);
  if ((arm != "baseline" && arm != "candidate") ||
      (mode != "reuse" && mode != "replace-every-cycle") || count < 2 || count > 10000)
    return 2;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  std::ofstream corpus(argv[4], std::ios::binary);
  const uint64_t length = count;
  corpus.write(reinterpret_cast<const char *>(&length), sizeof(length));
  for (int i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = i * .01;
    p.pose.position.y = .2 * std::sin(i * .015);
    tf2::Quaternion q;
    q.setRPY(0, 0, .15 * std::cos(i * .015));
    p.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(p);
    for (double v : {p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x,
                     p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w})
      corpus.write(reinterpret_cast<const char *>(&v), sizeof(v));
  }
  corpus.close();
  if (!corpus)
    throw std::runtime_error("corpus write failed");
  PathTarget prepared;
  nav_msgs::msg::Path retained;

  const double prep_begin = cpuNs();
  if (arm == "candidate")
    prepared.setPath(path);
  else
    retained = path;
  const double prep_cpu = cpuNs() - prep_begin;
  const int cycles = std::max(2000, 20000000 / count), warmup = 100;
  uint64_t checksum = 1469598103934665603ULL;
  size_t accepted = 0;
  std::vector<double> cpu, wall;
  cpu.reserve(cycles);
  wall.reserve(cycles);
  for (int iteration = -warmup; iteration < cycles; ++iteration) {
    const int n = iteration + warmup, index = (n * 37) % (count - 1);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "map";
    tf.header.stamp.sec = 1000 + n / 100;
    tf.header.stamp.nanosec = (n % 100) * 10000000;
    tf.transform.translation.x = .001 * (n % 17);
    tf.transform.translation.y = .001 * (n % 13);
    tf2::Quaternion q;
    q.setRPY(0, 0, .002 * (n % 11));
    tf.transform.rotation = tf2::toMsg(q);
    geometry_msgs::msg::PoseStamped pose;
    tf2::doTransform(path.poses[index], pose, tf);
    pose.pose.position.y += .005 * (n % 5 - 2);
    geometry_msgs::msg::Twist speed;
    speed.linear.x = .2 + .01 * (n % 7);
    speed.linear.y = .02 * (n % 3);
    geometry_msgs::msg::PoseStamped target;
    const auto begin = std::chrono::steady_clock::now();
    const double cpu_begin = cpuNs();
    if (mode == "replace-every-cycle") {
      if (arm == "candidate")
        prepared.setPath(path);
      else
        retained = path;
    }
    const bool ok = arm == "candidate" ? prepared.select(pose, tf, speed, target)
                                       : referencePathTarget(retained, pose, tf, speed, target);
    const double cpu_elapsed = cpuNs() - cpu_begin;
    const double wall_elapsed =
        std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - begin).count();
    if (iteration >= 0) {
      cpu.push_back(cpu_elapsed);
      wall.push_back(wall_elapsed);
      accepted += ok;
      hashValue(checksum, ok);
      hashPose(checksum, target);
    }
  }
  auto stats = [](std::vector<double> values) {
    double sum = 0;
    for (double v : values)
      sum += v;
    std::sort(values.begin(), values.end());
    const size_t p95 = static_cast<size_t>(std::ceil(values.size() * .95)) - 1;
    return std::vector<double>{sum / values.size() / 1e6, values[p95] / 1e6, values.back() / 1e6};
  };
  const auto c = stats(cpu), w = stats(wall);
  std::cout << std::setprecision(12) << "{\"arm\":\"" << arm << "\",\"mode\":\"" << mode
            << "\",\"points\":" << count << ",\"cycles\":" << cycles << ",\"warmup\":" << warmup
            << ",\"prep_cpu_ms\":" << prep_cpu / 1e6 << ",\"cpu_mean_ms\":" << c[0]
            << ",\"cpu_p95_ms\":" << c[1] << ",\"cpu_max_ms\":" << c[2]
            << ",\"wall_mean_ms\":" << w[0] << ",\"wall_p95_ms\":" << w[1]
            << ",\"wall_max_ms\":" << w[2] << ",\"accepted\":" << accepted
            << ",\"output_fingerprint\":" << checksum << "}\n";
}
