// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lslidar_driver {

struct Point2 { double x, y; };
struct Point3 { double x, y, z; };

// A verified solid cross-section, not the navigation footprint or its inflation.
// The prism and transform share one fixed body coordinate frame, in metres.
// Geometric inclusion is only a self-return hypothesis if that supplied model
// represents real material at the scan plane. A collision proxy can over-mask.
struct SelfFilterConfig {
  bool enabled{false};
  std::vector<double> laser_to_body_xyz_m;
  std::vector<double> laser_to_body_rpy_rad;
  std::vector<double> body_polygon_xy_m;
  std::vector<double> body_z_bounds_m;
  double boundary_ambiguity_m{0.0};
};

enum class SelfFilterClassification { Outside, Inside, Ambiguous };

// Dependency-free geometry. R = Rz(yaw) Ry(pitch) Rx(roll), p_body = R p_laser + t.
// Only points strictly inside the prism and farther than the ambiguity band
// from every surface are removed. Unknown or boundary points remain observable.
class SelfFilter {
public:
  explicit SelfFilter(const SelfFilterConfig &config = {}) : config_(config) {
    if (!std::isfinite(config_.boundary_ambiguity_m) || config_.boundary_ambiguity_m < 0.0)
      throw std::invalid_argument("self_filter.boundary_ambiguity_m must be finite and nonnegative");
    if (!config_.enabled) return;
    requireVector(config_.laser_to_body_xyz_m, 3, "laser_to_body_xyz_m");
    requireVector(config_.laser_to_body_rpy_rad, 3, "laser_to_body_rpy_rad");
    requireVector(config_.body_z_bounds_m, 2, "body_z_bounds_m");
    if (config_.body_z_bounds_m[0] >= config_.body_z_bounds_m[1])
      throw std::invalid_argument("self_filter.body_z_bounds_m must have lower < upper");
    if (config_.body_polygon_xy_m.size() < 6 || config_.body_polygon_xy_m.size() % 2 != 0)
      throw std::invalid_argument("self_filter.body_polygon_xy_m needs at least three XY vertices");
    for (double coordinate : config_.body_polygon_xy_m)
      if (!std::isfinite(coordinate))
        throw std::invalid_argument("self_filter.body_polygon_xy_m must be finite");
    for (size_t i = 0; i < config_.body_polygon_xy_m.size(); i += 2)
      polygon_.push_back({config_.body_polygon_xy_m[i], config_.body_polygon_xy_m[i + 1]});
    validatePolygon();
    const auto &rpy = config_.laser_to_body_rpy_rad;
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    rotation_ = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                  sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                  -sp, cp * sr, cp * cr}};
  }

  bool enabled() const { return config_.enabled; }

  Point3 toBody(const Point3 &point) const {
    if (!enabled()) return point;
    const auto &t = config_.laser_to_body_xyz_m;
    return {rotation_[0] * point.x + rotation_[1] * point.y + rotation_[2] * point.z + t[0],
            rotation_[3] * point.x + rotation_[4] * point.y + rotation_[5] * point.z + t[1],
            rotation_[6] * point.x + rotation_[7] * point.y + rotation_[8] * point.z + t[2]};
  }

  SelfFilterClassification classify(const Point3 &laser_point) const {
    if (!enabled()) return SelfFilterClassification::Outside;
    const Point3 p = toBody(laser_point);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      return SelfFilterClassification::Ambiguous;
    const double low = config_.body_z_bounds_m[0], high = config_.body_z_bounds_m[1];
    const double band = config_.boundary_ambiguity_m;
    bool inside_xy = false;
    double edge_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0, j = polygon_.size() - 1; i < polygon_.size(); j = i++) {
      const auto &a = polygon_[j];
      const auto &b = polygon_[i];
      edge_distance = std::min(edge_distance, pointSegmentDistance({p.x, p.y}, a, b));
      if ((a.y > p.y) != (b.y > p.y) &&
          p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
        inside_xy = !inside_xy;
    }
    // The numerical guard preserves exact boundaries even when band == 0.
    const double guard = 64.0 * std::numeric_limits<double>::epsilon() *
                         std::max({1.0, std::abs(p.x), std::abs(p.y), std::abs(p.z)});
    const bool z_near = p.z >= low - band - guard && p.z <= high + band + guard;
    if (!z_near || (!inside_xy && edge_distance > band + guard))
      return SelfFilterClassification::Outside;
    if (edge_distance <= band + guard || p.z <= low + band + guard || p.z >= high - band - guard)
      return SelfFilterClassification::Ambiguous;
    return SelfFilterClassification::Inside;
  }

  bool shouldRemove(const Point3 &point) const {
    return classify(point) == SelfFilterClassification::Inside;
  }

private:
  static void requireVector(const std::vector<double> &values, size_t size, const char *name) {
    if (values.size() != size || !std::all_of(values.begin(), values.end(),
                                           [](double value) { return std::isfinite(value); }))
      throw std::invalid_argument(std::string("self_filter.") + name + " requires explicit finite coordinates");
  }
  static double cross(Point2 a, Point2 b, Point2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }
  static double pointSegmentDistance(Point2 p, Point2 a, Point2 b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
  }
  static bool onSegment(Point2 a, Point2 b, Point2 p) {
    return cross(a, b, p) == 0.0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
           p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
  }
  static bool segmentsIntersect(Point2 a, Point2 b, Point2 c, Point2 d) {
    const double ca = cross(a, b, c), cb = cross(a, b, d);
    const double cc = cross(c, d, a), cd = cross(c, d, b);
    if (((ca > 0.0 && cb < 0.0) || (ca < 0.0 && cb > 0.0)) &&
        ((cc > 0.0 && cd < 0.0) || (cc < 0.0 && cd > 0.0))) return true;
    return onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
  }
  void validatePolygon() const {
    double area2 = 0.0;
    for (size_t i = 0; i < polygon_.size(); ++i) {
      const Point2 a = polygon_[i], b = polygon_[(i + 1) % polygon_.size()];
      if (std::hypot(a.x - b.x, a.y - b.y) <= 1e-12)
        throw std::invalid_argument("self_filter polygon has duplicate adjacent vertices");
      area2 += a.x * b.y - b.x * a.y;
      for (size_t j = i + 1; j < polygon_.size(); ++j) {
        if (j == i + 1 || (i == 0 && j == polygon_.size() - 1)) continue;
        if (segmentsIntersect(a, b, polygon_[j], polygon_[(j + 1) % polygon_.size()]))
          throw std::invalid_argument("self_filter polygon must be simple without self intersections");
      }
      // Adjacent edges may be collinear, but may not reverse and overlap.
      const Point2 c = polygon_[(i + 2) % polygon_.size()];
      if (cross(a, b, c) == 0.0 && (a.x - b.x) * (c.x - b.x) + (a.y - b.y) * (c.y - b.y) > 0.0)
        throw std::invalid_argument("self_filter polygon has overlapping adjacent edges");
    }
    if (!std::isfinite(area2) || std::abs(area2) <= 1e-12)
      throw std::invalid_argument("self_filter polygon must have nonzero finite area");
  }

  SelfFilterConfig config_;
  std::vector<Point2> polygon_;
  std::array<double, 9> rotation_{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
};

}  // namespace lslidar_driver
