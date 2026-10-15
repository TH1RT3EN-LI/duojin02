#pragma once

#include "duojin02_navigation/sweep.hpp"
#include <array>
#include <cstdint>

namespace duojin02_navigation {

inline constexpr std::array<double, 8> brakingScales{1., .8, .6, .4, .25, .15, .08, .04};

// Preserve requested first, followed by the three measured-to-requested
// transitions. A safe later transition must never hide an unsafe earlier one.
inline std::array<Velocity, 4> brakingCandidateVelocities(const Velocity &requested,
                                                         const Velocity &measured) {
  std::array<Velocity, 4> result{requested};
  for (size_t i = 1; i < result.size(); ++i) {
    const double f = .25 * i;
    result[i] = {measured.x + f * (requested.x - measured.x),
                 measured.y + f * (requested.y - measured.y),
                 measured.yaw + f * (requested.yaw - measured.yaw)};
  }
  return result;
}

struct CenteringCandidate {
  Pose pose;
  double offset;
};

inline std::array<CenteringCandidate, 7> centeringCandidates(const Pose &start,
                                                          const Pose &target,
                                                          double lookahead) {
  const double dx = target.x - start.x, dy = target.y - start.y;
  const double length = std::hypot(dx, dy);
  const double nx = length > 0 ? -dy / length : 0;
  const double ny = length > 0 ? dx / length : 0;
  constexpr std::array<double, 7> offsets{0., -.01, .01, -.02, .02, -.03, .03};
  std::array<CenteringCandidate, 7> result;
  for (size_t i = 0; i < offsets.size(); ++i)
    result[i] = {{start.x + lookahead * dx + nx * offsets[i],
                  start.y + lookahead * dy + ny * offsets[i], target.yaw}, offsets[i]};
  return result;
}

struct ControllerGeometryStatistics {
  uint64_t cycles{0}, compute_failures{0}, exceptions{0}, fault_latched_stops{0};
  uint64_t direct_commands{0}, tracking_commands{0}, native_mppi_calls{0};
  uint64_t direct_cpu_batches{0}, direct_cuda_batches{0};
  uint64_t centering_cpu_batches{0}, centering_cuda_batches{0}, ranking_cpu_rechecks{0};
  uint64_t braking_cpu_sweeps{0}, braking_cpu_batches{0}, braking_cuda_batches{0};
  uint64_t braking_batched_poses{0};
  uint64_t braking_measured_stops{0}, braking_first_scale_accepts{0};
  uint64_t braking_final_cpu_candidates{0}, braking_final_cpu_rejections{0};
  double last_cycle_ms{0}, total_cycle_ms{0}, max_cycle_ms{0};
  // Bounded storage: quarter-millisecond buckets through 250 ms, then overflow.
  // Reported quantiles are bucket upper bounds, not interpolated exact timings.
  std::array<uint64_t, 1001> cycle_histogram{};

  void recordCycle(double ms) {
    ++cycles;
    last_cycle_ms = ms;
    total_cycle_ms += ms;
    max_cycle_ms = std::max(max_cycle_ms, ms);
    const auto bucket = static_cast<size_t>(std::min(1000., std::floor(ms * 4)));
    ++cycle_histogram[bucket];
  }

  double cycleQuantileUpperBound(double fraction) const {
    if (!cycles)
      return 0;
    const uint64_t rank = std::max<uint64_t>(1, static_cast<uint64_t>(std::ceil(fraction * cycles)));
    uint64_t count = 0;
    for (size_t i = 0; i < cycle_histogram.size(); ++i) {
      count += cycle_histogram[i];
      if (count >= rank)
        return i == 1000 ? max_cycle_ms : .25 * (i + 1);
    }
    return max_cycle_ms;
  }
};

inline double localSnapshotExtent(const Polygon &polygon, double forward, double reverse,
                                  double lateral, const Velocity &measured, double resolution) {
  double radius = 0;
  for (const auto &p : polygon)
    radius = std::max(radius, std::hypot(p.x, p.y));
  const double configured = std::hypot(std::max(forward, reverse), lateral);
  const double v = std::max(configured, std::hypot(measured.x, measured.y));
  return radius + std::max(.45, .2 * v + v * v / (2 * .4)) + .014 + resolution * std::sqrt(2.0) / 2;
}

inline double footprintClearance(const Pose &pose, const Polygon &polygon,
                                 const std::vector<Point> &points, double cap = .080) {
  double min_x = 1e100, min_y = 1e100, max_x = -1e100, max_y = -1e100;
  for (const auto &v : polygon) {
    min_x = std::min(min_x, v.x);
    min_y = std::min(min_y, v.y);
    max_x = std::max(max_x, v.x);
    max_y = std::max(max_y, v.y);
  }
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  double result = cap;
  for (const auto &p : points) {
    const double dx = p.x - pose.x, dy = p.y - pose.y;
    Point q{c * dx + s * dy, -s * dx + c * dy, p.radius};
    const double padding = cap + p.radius;
    if (q.x < min_x - padding || q.x > max_x + padding || q.y < min_y - padding ||
        q.y > max_y + padding)
      continue;
    bool inside = true;
    double distance = 1e100;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
      if ((b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x) < -1e-10)
        inside = false;
      distance = std::min(distance, distanceToSegment(q, a, b));
    }
    result = std::min(result, (inside ? -distance : distance) - p.radius);
  }
  return result;
}
}
