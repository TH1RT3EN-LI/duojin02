#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace duojin02_navigation {
struct Point {
  double x, y, radius{0.0};
};

struct Pose {
  double x{0}, y{0}, yaw{0};
};

struct Velocity {
  double x{0}, y{0}, yaw{0};
};

using Polygon = std::vector<Point>;

inline Point transform(const Point &p, const Pose &pose) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y, p.radius};
}

inline double distanceToSegment(const Point &p, const Point &a, const Point &b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double t =
      std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
  return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}

inline bool touches(const Point &p, const Polygon &polygon, double margin) {
  bool inside = true;
  double distance = 1e10;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
    if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < -1e-10)
      inside = false;
    distance = std::min(distance, distanceToSegment(p, a, b));
  }
  return inside || distance <= margin + p.radius;
}

inline bool collision(const Pose &pose, const Polygon &polygon, const std::vector<Point> &points,
                      double margin) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  double min_x = 1e100, min_y = 1e100, max_x = -1e100, max_y = -1e100;
  for (const auto &vertex : polygon) {
    min_x = std::min(min_x, vertex.x);
    min_y = std::min(min_y, vertex.y);
    max_x = std::max(max_x, vertex.x);
    max_y = std::max(max_y, vertex.y);
  }
  for (const auto &p : points) {
    const double dx = p.x - pose.x, dy = p.y - pose.y;
    Point local{c * dx + s * dy, -s * dx + c * dy, p.radius};
    const double padding = margin + p.radius;
    if (local.x < min_x - padding || local.x > max_x + padding || local.y < min_y - padding ||
        local.y > max_y + padding)
      continue;
    if (touches(local, polygon, margin))
      return true;
  }
  return false;
}

inline Pose advance(Pose pose, const Velocity &v, double dt) {
  const double mid = pose.yaw + v.yaw * dt / 2;
  pose.x += (std::cos(mid) * v.x - std::sin(mid) * v.y) * dt;
  pose.y += (std::sin(mid) * v.x + std::cos(mid) * v.y) * dt;
  pose.yaw += v.yaw * dt;
  return pose;
}

inline double approachZero(double v, double amount) {
  return std::copysign(std::max(0.0, std::abs(v) - amount), v);
}

inline bool brakingSweep(Pose pose, Velocity v, const Polygon &polygon,
                         const std::vector<Point> &points, double margin, double reaction,
                         double deceleration, double angular_deceleration,
                         double spatial_step = 0.004) {
  double radius = 0;
  for (const auto &vertex : polygon)
    radius = std::max(radius, std::hypot(vertex.x, vertex.y));
  const double speed = std::hypot(v.x, v.y), corner_speed = speed + radius * std::abs(v.yaw);
  const double dt = std::min(0.02, spatial_step / std::max(corner_speed, 0.001));
  const double end =
      reaction + std::max(speed / deceleration, std::abs(v.yaw) / angular_deceleration) + dt;

  const double sampling_margin = corner_speed * dt / 2;
  std::vector<Pose> poses;
  double min_x = 1e100, min_y = 1e100, max_x = -1e100, max_y = -1e100;
  for (double t = 0; t <= end; t += dt) {
    poses.push_back(pose);
    for (const auto &vertex : polygon) {
      const auto p = transform(vertex, pose);
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }
    pose = advance(pose, v, dt);
    if (t >= reaction) {
      const double current = std::hypot(v.x, v.y);
      const double factor = current > 0 ? std::max(0.0, 1 - deceleration * dt / current) : 0;
      v.x *= factor;
      v.y *= factor;
      v.yaw = approachZero(v.yaw, angular_deceleration * dt);
    }
  }

  std::vector<Point> nearby;
  for (const auto &p : points) {
    const double padding = margin + sampling_margin + p.radius;
    if (p.x >= min_x - padding && p.x <= max_x + padding && p.y >= min_y - padding &&
        p.y <= max_y + padding)
      nearby.push_back(p);
  }
  for (const auto &sample : poses)
    if (collision(sample, polygon, nearby, margin + sampling_margin))
      return true;
  return false;
}
}
