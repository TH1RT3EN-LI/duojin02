#pragma once

#include "duojin02_navigation/sweep.hpp"
#include <cstddef>
#include <limits>
#include <type_traits>

namespace duojin02_navigation {
struct MapBounds {
  double min_x, min_y, max_x, max_y;
};

inline double filledQueryExtent(double radius, double resolution, double margin) {
  return std::max(radius + resolution, radius + resolution * std::sqrt(2.0) / 2 + margin);
}

inline bool footprintInsideBounds(const Pose &pose, const Polygon &polygon, const MapBounds &bounds,
                                  double margin) {
  if (polygon.size() < 3 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw) || !std::isfinite(margin) || margin < 0 ||
      !std::isfinite(bounds.min_x) || !std::isfinite(bounds.min_y) ||
      !std::isfinite(bounds.max_x) || !std::isfinite(bounds.max_y) ||
      bounds.min_x >= bounds.max_x || bounds.min_y >= bounds.max_y)
    return false;
  for (const auto &vertex : polygon) {
    const auto p = transform(vertex, pose);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x - margin < bounds.min_x ||
        p.y - margin < bounds.min_y || p.x + margin > bounds.max_x || p.y + margin > bounds.max_y)
      return false;
  }
  return true;
}

struct PlannerMapSnapshot {
  unsigned int width{0}, height{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  Polygon footprint;
  bool allow_unknown{false}, ready{false};
  std::vector<unsigned char> costs;

  bool matches(unsigned int w, unsigned int h, double r, double ox, double oy, const Polygon &p,
               bool unknown, const unsigned char *data) const {
    if (!ready || width != w || height != h || resolution != r || origin_x != ox ||
        origin_y != oy || allow_unknown != unknown || footprint.size() != p.size() ||
        costs.size() != static_cast<size_t>(w) * h)
      return false;
    for (size_t i = 0; i < p.size(); ++i)
      if (footprint[i].x != p[i].x || footprint[i].y != p[i].y ||
          footprint[i].radius != p[i].radius)
        return false;
    return std::equal(costs.begin(), costs.end(), data);
  }

  void capture(unsigned int w, unsigned int h, double r, double ox, double oy, const Polygon &p,
               bool unknown, const unsigned char *data) {
    width = w;
    height = h;
    resolution = r;
    origin_x = ox;
    origin_y = oy;
    footprint = p;
    allow_unknown = unknown;
    costs.assign(data, data + static_cast<size_t>(w) * h);
    ready = true;
  }

  void clear() {
    ready = false;
    costs.clear();
    footprint.clear();
  }
};

inline bool onPoseSegment(const Pose &pose, const Pose &a, const Pose &b, double f,
                          double epsilon = 1e-9) {
  const double dyaw = std::remainder(b.yaw - a.yaw, 2 * M_PI);
  return std::hypot(pose.x - a.x - f * (b.x - a.x), pose.y - a.y - f * (b.y - a.y)) <= epsilon &&
         std::abs(std::remainder(pose.yaw - a.yaw - f * dyaw, 2 * M_PI)) <= epsilon;
}

struct SoftCostPoint {
  Point point;
  unsigned char cost;
};

class FilledSoftCost {
public:
  FilledSoftCost(const Pose &pose, const Polygon &polygon, double margin)
      : pose_(pose), polygon_(polygon), margin_(margin), c_(std::cos(pose.yaw)),
        s_(std::sin(pose.yaw)) {
    for (const auto &vertex : polygon_) {
      min_x_ = std::min(min_x_, vertex.x);
      min_y_ = std::min(min_y_, vertex.y);
      max_x_ = std::max(max_x_, vertex.x);
      max_y_ = std::max(max_y_, vertex.y);
    }
  }

  bool include(const SoftCostPoint &cell) {
    if (cell.cost <= cost_ || cell.cost > 253)
      return cost_ == 253;
    const auto &p = cell.point;
    const double dx = p.x - pose_.x, dy = p.y - pose_.y;
    const Point local{c_ * dx + s_ * dy, -s_ * dx + c_ * dy, p.radius};
    const double padding = margin_ + p.radius;
    if (local.x < min_x_ - padding || local.x > max_x_ + padding || local.y < min_y_ - padding ||
        local.y > max_y_ + padding)
      return false;
    if (touches(local, polygon_, margin_))
      cost_ = cell.cost;
    return cost_ == 253;
  }

  unsigned char cost() const {
    return cost_;
  }

private:
  Pose pose_;

  const Polygon &polygon_;
  double margin_, c_, s_;
  double min_x_{1e100}, min_y_{1e100}, max_x_{-1e100}, max_y_{-1e100};
  unsigned char cost_{0};
};

template <class HardPathValid, class SoftPoseCost>
bool headingOnlyPathAllowed(const std::vector<Pose> &reference, const std::vector<Pose> &fixed,
                            double radius, HardPathValid hard_path_valid, SoftPoseCost soft_cost) {
  static_assert(std::is_integral_v<decltype(soft_cost(Pose{}))>,
                "Heading-only dominance requires integer cell costs");
  if (reference.empty() || reference.size() != fixed.size() || !std::isfinite(radius) || radius < 0)
    return false;
  for (size_t i = 0; i < reference.size(); ++i) {
    const auto &a = reference[i], &b = fixed[i];
    if (a.x != b.x || a.y != b.y || !std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(a.yaw) || !std::isfinite(b.yaw))
      return false;
  }
  if (!hard_path_valid(fixed))
    return false;
  for (size_t i = 1; i < reference.size(); ++i) {
    const auto &a = reference[i - 1], &b = reference[i];
    const double dx = b.x - a.x, dy = b.y - a.y, ds = std::hypot(dx, dy);
    const double ref_turn = std::remainder(b.yaw - a.yaw, 2 * M_PI);
    const double fixed_turn = std::remainder(fixed[i].yaw - fixed[i - 1].yaw, 2 * M_PI);
    const double steps_raw = std::ceil(
        std::max(ds + radius * std::abs(ref_turn), ds + radius * std::abs(fixed_turn)) / .004);
    if (!std::isfinite(steps_raw) || steps_raw > std::numeric_limits<int>::max())
      return false;
    if (ds == 0)
      continue;
    const int steps = std::max(1, static_cast<int>(steps_raw));
    for (int j = 0; j <= steps; ++j) {
      const double f = static_cast<double>(j) / steps;
      const double x = a.x + f * dx, y = a.y + f * dy;
      const auto ref_cost = soft_cost(Pose{x, y, a.yaw + f * ref_turn});
      const auto fixed_cost = soft_cost(Pose{x, y, fixed[i - 1].yaw + f * fixed_turn});
      if constexpr (std::is_signed_v<decltype(ref_cost)>)
        if (ref_cost < 0 || fixed_cost < 0)
          return false;
      if (ref_cost > 253 || fixed_cost > 253 || fixed_cost > ref_cost)
        return false;
    }
  }
  return true;
}
}
