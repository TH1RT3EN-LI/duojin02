// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__GLOBAL_ACQUISITION_HPP_
#define NAV2_AMCL__GLOBAL_ACQUISITION_HPP_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>
#include "nav2_amcl/map/map.hpp"
namespace nav2_amcl
{
struct AcquisitionPose {double x{}, y{}, yaw{};};
struct AcquisitionPoint {double x{}, y{};};
struct AcquisitionCandidate {AcquisitionPose pose; double score{};};
struct AcquisitionConfig
{
  // Explicit finite domain; no continuous SE(2)/PF/physical-truth certificate.
  double xy_step{0.05}, yaw_step{0.00545415391248228};
  double final_sigma{0.05}, min_score{0.65}, min_margin{0.05};
  double mode_translation{0.4}, mode_yaw{0.2617993877991494};
  int confirmations{3}, root_block_power{4}, root_angle_power{4};
  uint64_t max_evaluations{3000000}, max_memory_bytes{512ULL * 1024 * 1024};
  // Optional user-supplied map-frame center; all yaw values remain candidates.
  // The square follows odometry from the first scan, not an unchanging robot pose.
  bool local_domain{false};
  AcquisitionPose local_center{};
  double local_xy_half_width{2.0};
};
struct AcquisitionResult
{
  std::string status{"idle"};
  bool acquired{false}, searched{false}, exhaustive_lattice{false};
  bool search_complete{false}, certificate_valid{false};
  int confirmations{}, observed_scans{};
  int64_t evidence_stamp_ns{};
  uint64_t evaluations{}, frontier_size{}, lattice_poses{}, memory_bytes{};
  double best_score{}, runner_up_score{}, margin{}, competitor_upper_bound{1};
  double certified_gap{}, lattice_xy_step{}, lattice_yaw_step{};
  std::vector<AcquisitionCandidate> modes;
};
// Canonical integer-cell lattice: endpoints are rounded once per yaw, then
// integer translations are added. This intentionally avoids world-origin
// cancellation; it is not bitwise identical to arbitrary world-coordinate PF
// scoring. Every selected beam uses the same final sigma at every search level.
class GlobalAcquisition
{
public:
  explicit GlobalAcquisition(AcquisitionConfig config = {}) : config_(config)
  {
    if (!std::isfinite(config.xy_step) || config.xy_step <= 0 ||
      !std::isfinite(config.yaw_step) || config.yaw_step <= 0 || config.yaw_step > M_PI ||
      !std::isfinite(config.final_sigma) || config.final_sigma <= 0 ||
      !std::isfinite(config.min_score) || config.min_score <= 0 || config.min_score > 1 ||
      !std::isfinite(config.min_margin) || config.min_margin <= 0 || config.min_margin > 1 ||
      !std::isfinite(config.mode_translation) || config.mode_translation <= 0 ||
      !std::isfinite(config.mode_yaw) || config.mode_yaw <= 0 || config.mode_yaw > M_PI ||
      config.confirmations < 2 || !config.max_evaluations || config.max_memory_bytes < 1024 ||
      config.root_block_power < 0 || config.root_block_power > 10 ||
      config.root_angle_power < 0 || config.root_angle_power > 10 ||
      (config.local_domain && (!std::isfinite(config.local_center.x) ||
      !std::isfinite(config.local_center.y) || !std::isfinite(config.local_center.yaw) ||
      !std::isfinite(config.local_xy_half_width) || config.local_xy_half_width <= 0)))
    {throw std::invalid_argument("invalid global acquisition configuration");}
  }
  const AcquisitionResult & result() const {return result_;}
  AcquisitionResult observe(const map_t & map, const std::vector<AcquisitionPoint> & points,
    AcquisitionPose odom, int64_t source_stamp_ns)
  {
    const int previous_confirmations = result_.confirmations, observed = result_.observed_scans;
    result_ = {}; result_.observed_scans = observed; result_.evidence_stamp_ns = source_stamp_ns;
    if (source_stamp_ns <= last_stamp_ns_) {
      result_.confirmations = previous_confirmations;
      result_.status = "duplicate_or_nonmonotonic_source"; return result_;
    }
    last_stamp_ns_ = source_stamp_ns;
    if (!valid(map, points, odom)) {
      have_best_ = false; result_.status = "invalid_or_insufficient_support"; return result_;
    }
    if (config_.local_domain && !have_local_odom_) {
      local_odom_ = odom;
      have_local_odom_ = true;
    }
    current_center_ = config_.local_domain ? propagate(config_.local_center, local_odom_, odom) : AcquisitionPose{};
    ++result_.observed_scans; result_.searched = true;
    const auto predicted = have_best_ ? propagate(last_best_, last_odom_, odom) : AcquisitionPose{};
    try {
      search(map, points, have_best_ ? &predicted : nullptr);
    } catch (const std::bad_alloc &) {
      result_.status = "search_allocation_failed";
      result_.search_complete = result_.exhaustive_lattice = result_.certificate_valid = false;
      result_.competitor_upper_bound = 1;
    }
    if (!result_.search_complete || result_.modes.empty()) {have_best_ = false; return result_;}
    const bool continuous = have_best_ && sameMode(predicted, result_.modes.front().pose);
    if (result_.certificate_valid) {
      result_.confirmations = continuous ? previous_confirmations + 1 : 1;
      result_.status = "confirming";
      if (result_.confirmations >= config_.confirmations) {
        result_.acquired = true; result_.status = "acquired_engineering_gate";
      }
    }
    last_best_ = result_.modes.front().pose; last_odom_ = odom; have_best_ = true;
    return result_;
  }
  static double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
  static AcquisitionPose propagate(AcquisitionPose p, AcquisitionPose old_odom, AcquisitionPose new_odom)
  {
    const double yaw = p.yaw - old_odom.yaw, dx = new_odom.x - old_odom.x, dy = new_odom.y - old_odom.y;
    return {p.x + std::cos(yaw) * dx - std::sin(yaw) * dy,
      p.y + std::sin(yaw) * dx + std::cos(yaw) * dy, wrap(p.yaw + wrap(new_odom.yaw - old_odom.yaw))};
  }
private:
  struct Offset {int x, y;};
  struct Node {int x, y, level, angle, angle_level; double upper;};
  struct Envelope {int min_x, max_x, min_y, max_y;};
  struct Worse
  {
    bool operator()(const Node & a, const Node & b) const
    {
      if (a.upper != b.upper) {return a.upper < b.upper;}
      if (a.level != b.level) {return a.level > b.level;}
      if (a.angle_level != b.angle_level) {return a.angle_level > b.angle_level;}
      if (a.angle != b.angle) {return a.angle > b.angle;}
      if (a.y != b.y) {return a.y > b.y;}
      return a.x > b.x;
    }
  };
  struct NodeQueue : std::priority_queue<Node, std::vector<Node>, Worse>
  {
    size_t capacity() const {return this->c.capacity();}
    void reserve(size_t count) {this->c.reserve(count);}
  };
  AcquisitionConfig config_;
  AcquisitionResult result_;
  int64_t last_stamp_ns_{std::numeric_limits<int64_t>::min()};
  AcquisitionPose last_odom_, last_best_;
  bool have_best_{false};
  AcquisitionPose local_odom_, current_center_;
  bool have_local_odom_{false};
  int local_min_x_{}, local_max_x_{}, local_min_y_{}, local_max_y_{};
  int stride_{}, angles_{}, nx_{}, ny_{}, width_{}, height_{};
  std::vector<std::vector<float>> pyramid_;
  std::vector<Offset> offsets_;
  std::vector<std::vector<Envelope>> yaw_envelopes_;
  std::vector<std::vector<uint32_t>> yaw_spans_;
  std::vector<uint32_t> free_prefix_;
  uint64_t fixed_memory_{};
  size_t beams_{};
  int best_x_{}, best_y_{}, best_a_{};
  static bool valid(const map_t & m, const std::vector<AcquisitionPoint> & points, const AcquisitionPose & o)
  {
    if (!m.cells || m.size_x <= 0 || m.size_y <= 0 ||
      m.size_x > (1 << 20) || m.size_y > (1 << 20) ||
      static_cast<uint64_t>(m.size_x) * m.size_y > UINT32_MAX ||
      !std::isfinite(m.scale) || m.scale < 1e-8 || !std::isfinite(m.origin_x) || !std::isfinite(m.origin_y) ||
      !std::isfinite(m.origin_x - m.size_x * m.scale) ||
      !std::isfinite(m.origin_x + m.size_x * m.scale) ||
      !std::isfinite(m.origin_y - m.size_y * m.scale) ||
      !std::isfinite(m.origin_y + m.size_y * m.scale) ||
      points.size() < 12 || points.size() > 1000000 || !std::isfinite(o.x) || !std::isfinite(o.y) ||
      !std::isfinite(o.yaw)) {return false;}
    for (const auto & p : points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        std::abs(p.x / m.scale) > 1e8 || std::abs(p.y / m.scale) > 1e8) {return false;}
    }
    return true;
  }
  bool sameMode(AcquisitionPose a, AcquisitionPose b) const
  {
    return std::hypot(a.x - b.x, a.y - b.y) < config_.mode_translation &&
      std::abs(wrap(a.yaw - b.yaw)) < config_.mode_yaw;
  }
  double angle(int a) const {return -M_PI + 2 * M_PI * a / angles_;}
  AcquisitionPose pose(const map_t & m, int x, int y, int a) const
  {return {MAP_WXGX((&m), x * stride_), MAP_WYGY((&m), y * stride_), angle(a)};}
  double field(double distance) const
  {
    if (!std::isfinite(distance) || distance < 0) {return 0;}
    const double ratio = distance / config_.final_sigma;
    return ratio >= 4 ? 0 : std::exp(-0.5 * ratio * ratio);
  }
  uint32_t freeCount(int x, int y, int side) const
  {
    const int x1 = std::min(nx_, x + side), y1 = std::min(ny_, y + side);
    if (x >= x1 || y >= y1) {return 0;}
    const size_t pitch = static_cast<size_t>(nx_) + 1;
    return free_prefix_[y1 * pitch + x1] - free_prefix_[y * pitch + x1] -
      free_prefix_[y1 * pitch + x] + free_prefix_[y * pitch + x];
  }
  bool prepare(const map_t & m, const std::vector<AcquisitionPoint> & points)
  {
    // Tolerance avoids float32 map resolution making nominal .05/.01 use six
    // cells. The actual stride is always reported; the domain is explicit.
    const double requested_stride = config_.xy_step / m.scale;
    if (requested_stride > INT32_MAX / 1024.0 || 2 * M_PI / config_.yaw_step > 1000000) {return false;}
    stride_ = std::max(1, static_cast<int>(std::ceil(requested_stride - 1e-6)));
    angles_ = static_cast<int>(std::ceil(2 * M_PI / config_.yaw_step - 1e-10));
    width_ = m.size_x; height_ = m.size_y; beams_ = points.size();
    nx_ = (width_ - 1) / stride_ + 1; ny_ = (height_ - 1) / stride_ + 1;
    local_min_x_ = 0; local_max_x_ = nx_ - 1;
    local_min_y_ = 0; local_max_y_ = ny_ - 1;
    if (config_.local_domain) {
      const double grid_x = ((current_center_.x - m.origin_x) / m.scale + m.size_x / 2) / stride_;
      const double grid_y = ((current_center_.y - m.origin_y) / m.scale + m.size_y / 2) / stride_;
      const double half = config_.local_xy_half_width / (stride_ * m.scale);
      // Clamp in floating point before conversion to avoid out-of-range casts.
      if (grid_x + half < 0 || grid_x - half > nx_ - 1 ||
        grid_y + half < 0 || grid_y - half > ny_ - 1) {
        local_max_x_ = -1;
      } else {
        local_min_x_ = static_cast<int>(std::ceil(std::max(0.0, grid_x - half)));
        local_max_x_ = static_cast<int>(std::floor(std::min(static_cast<double>(nx_ - 1), grid_x + half)));
        local_min_y_ = static_cast<int>(std::ceil(std::max(0.0, grid_y - half)));
        local_max_y_ = static_cast<int>(std::floor(std::min(static_cast<double>(ny_ - 1), grid_y + half)));
      }
    }
    result_.lattice_xy_step = stride_ * m.scale; result_.lattice_yaw_step = 2 * M_PI / angles_;
    int levels = 1;
    const uint64_t span = static_cast<uint64_t>((1 << config_.root_block_power) - 1) * stride_ + 1;
    while ((uint64_t{1} << (levels - 1)) < span) {++levels;}
    const uint64_t cells = static_cast<uint64_t>(width_) * height_;
    const uint64_t offsets = static_cast<uint64_t>(angles_) * beams_;
    fixed_memory_ = cells * levels * sizeof(float) + offsets * sizeof(Offset) +
      static_cast<uint64_t>(nx_ + 1) * (ny_ + 1) * sizeof(uint32_t) +
      static_cast<uint64_t>(levels) * sizeof(std::vector<float>);
    uint64_t yaw_groups = 0;
    for (int level = 1; level <= config_.root_angle_power; ++level) {
      yaw_groups += (static_cast<uint64_t>(angles_) + (uint64_t{1} << level) - 1) >> level;
    }
    fixed_memory_ += yaw_groups * (beams_ * sizeof(Envelope) + sizeof(uint32_t)) +
      static_cast<uint64_t>(config_.root_angle_power + 1) *
      (sizeof(std::vector<Envelope>) + sizeof(std::vector<uint32_t>));
    if (fixed_memory_ > config_.max_memory_bytes) {return false;}
    // Release retained capacities before a different beam count/map geometry.
    // Account actual capacities after construction as well as preflight sizes.
    std::vector<Offset>().swap(offsets_);
    std::vector<std::vector<Envelope>>().swap(yaw_envelopes_);
    std::vector<std::vector<uint32_t>>().swap(yaw_spans_);
    std::vector<uint32_t>().swap(free_prefix_);
    std::vector<std::vector<float>>().swap(pyramid_);
    result_.memory_bytes = fixed_memory_; offsets_.resize(offsets);
    for (int a = 0; a < angles_; ++a) {
      const double c = std::cos(angle(a)), s = std::sin(angle(a));
      for (size_t i = 0; i < beams_; ++i) {
        offsets_[static_cast<size_t>(a) * beams_ + i] = {
          static_cast<int>(std::floor((c * points[i].x - s * points[i].y) / m.scale + .5)),
          static_cast<int>(std::floor((s * points[i].x + c * points[i].y) / m.scale + .5))};
      }
    }
    yaw_envelopes_.resize(config_.root_angle_power + 1);
    yaw_spans_.resize(config_.root_angle_power + 1);
    for (int level = 1; level <= config_.root_angle_power; ++level) {
      const int group_size = 1 << level;
      const int groups = (angles_ + group_size - 1) / group_size;
      auto & envelopes = yaw_envelopes_[level];
      auto & spans = yaw_spans_[level];
      envelopes.resize(static_cast<size_t>(groups) * beams_);
      spans.assign(groups, 0);
      for (int group = 0; group < groups; ++group) {
        const int begin = group * group_size, end = std::min(angles_, begin + group_size);
        for (size_t beam = 0; beam < beams_; ++beam) {
          const auto first = offsets_[static_cast<size_t>(begin) * beams_ + beam];
          Envelope e{first.x, first.x, first.y, first.y};
          for (int angle_index = begin + 1; angle_index < end; ++angle_index) {
            const auto value = offsets_[static_cast<size_t>(angle_index) * beams_ + beam];
            e.min_x = std::min(e.min_x, value.x); e.max_x = std::max(e.max_x, value.x);
            e.min_y = std::min(e.min_y, value.y); e.max_y = std::max(e.max_y, value.y);
          }
          envelopes[static_cast<size_t>(group) * beams_ + beam] = e;
          spans[group] = std::max(spans[group], static_cast<uint32_t>(
            std::max(e.max_x - e.min_x, e.max_y - e.min_y)));
        }
      }
    }
    free_prefix_.assign(static_cast<size_t>(nx_ + 1) * (ny_ + 1), 0);
    const size_t pitch = static_cast<size_t>(nx_) + 1;
    for (int y = 0; y < ny_; ++y) {
      uint32_t row = 0;
      for (int x = 0; x < nx_; ++x) {
        row += x >= local_min_x_ && x <= local_max_x_ && y >= local_min_y_ && y <= local_max_y_ &&
          m.cells[MAP_INDEX((&m), x * stride_, y * stride_)].occ_state == -1;
        free_prefix_[(y + 1) * pitch + x + 1] = free_prefix_[y * pitch + x + 1] + row;
      }
    }
    result_.lattice_poses = static_cast<uint64_t>(freeCount(0, 0, std::max(nx_, ny_))) * angles_;
    pyramid_.resize(levels); pyramid_[0].resize(cells);
    for (size_t i = 0; i < cells; ++i) {
      const double value = field(m.cells[i].occ_dist); float value_upper = static_cast<float>(value);
      if (static_cast<double>(value_upper) < value) {
        value_upper = std::nextafter(value_upper, std::numeric_limits<float>::infinity());
      }
      pyramid_[0][i] = value_upper;
    }
    for (int level = 1; level < levels; ++level) {
      const int half = 1 << (level - 1); const auto & previous = pyramid_[level - 1];
      auto & next = pyramid_[level]; next.resize(cells);
      for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
          float value = previous[static_cast<size_t>(y) * width_ + x];
          if (x + half < width_) {value = std::max(value, previous[static_cast<size_t>(y) * width_ + x + half]);}
          if (y + half < height_) {
            value = std::max(value, previous[static_cast<size_t>(y + half) * width_ + x]);
            if (x + half < width_) {
              value = std::max(value, previous[static_cast<size_t>(y + half) * width_ + x + half]);
            }
          }
          next[static_cast<size_t>(y) * width_ + x] = value;
        }
      }
    }
    fixed_memory_ = offsets_.capacity() * sizeof(Offset) +
      free_prefix_.capacity() * sizeof(uint32_t) +
      pyramid_.capacity() * sizeof(std::vector<float>);
    for (const auto & level : pyramid_) {fixed_memory_ += level.capacity() * sizeof(float);}
    fixed_memory_ += yaw_envelopes_.capacity() * sizeof(std::vector<Envelope>) +
      yaw_spans_.capacity() * sizeof(std::vector<uint32_t>);
    for (const auto & envelopes : yaw_envelopes_) {fixed_memory_ += envelopes.capacity() * sizeof(Envelope);}
    for (const auto & spans : yaw_spans_) {fixed_memory_ += spans.capacity() * sizeof(uint32_t);}
    result_.memory_bytes = fixed_memory_;
    return fixed_memory_ <= config_.max_memory_bytes;
  }
  double rectangleUpper(int x0, int y0, int x1, int y1) const
  {
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(width_ - 1, x1); y1 = std::min(height_ - 1, y1);
    if (x0 > x1 || y0 > y1) {return 0;}
    // Four overlapping dyadic squares cover this rectangle. On un-clipped
    // square blocks their union is exactly the rectangle, substantially tighter
    // than a containing next-power-of-two square. Clipping can make an axis
    // shorter; expansion on that axis remains a conservative superset.
    int level = 0; const int span = std::max(x1 - x0 + 1, y1 - y0 + 1);
    while ((1 << (level + 1)) <= span) {++level;}
    if (level >= static_cast<int>(pyramid_.size())) {
      level = static_cast<int>(pyramid_.size()) - 1;
      const int tile_side = 1 << level;
      const auto & tiled = pyramid_[level];
      float value = 0;
      for (int y = y0; y <= y1; y += tile_side) {
        for (int x = x0; x <= x1; x += tile_side) {
          value = std::max(value, tiled[static_cast<size_t>(y) * width_ + x]);
        }
      }
      return value;
    }
    const int side = 1 << level;
    const int bx = std::max(x0, x1 - side + 1), by = std::max(y0, y1 - side + 1);
    const auto & values = pyramid_[level];
    return std::max({values[static_cast<size_t>(y0) * width_ + x0],
      values[static_cast<size_t>(y0) * width_ + bx],
      values[static_cast<size_t>(by) * width_ + x0],
      values[static_cast<size_t>(by) * width_ + bx]});
  }
  double upper(const Node & node)
  {
    ++result_.evaluations; const int side = 1 << node.level;
    const int x0 = node.x * stride_, y0 = node.y * stride_;
    const int x1 = (std::min(nx_, node.x + side) - 1) * stride_;
    const int y1 = (std::min(ny_, node.y + side) - 1) * stride_;
    double sum = 0;
    for (size_t i = 0; i < beams_; ++i) {
      if (node.angle_level == 0) {
        const auto & o = offsets_[static_cast<size_t>(node.angle) * beams_ + i];
        sum += rectangleUpper(x0 + o.x, y0 + o.y, x1 + o.x, y1 + o.y);
      } else {
        const auto & e = yaw_envelopes_[node.angle_level][
          static_cast<size_t>(node.angle >> node.angle_level) * beams_ + i];
        sum += rectangleUpper(x0 + e.min_x, y0 + e.min_y, x1 + e.max_x, y1 + e.max_y);
      }
    }
    // Nonnegative-summation gamma_n guard covers bound and leaf accumulation.
    const double ne = (beams_ + 2) * std::numeric_limits<double>::epsilon();
    const double guard = 2 * ne / (1 - ne);
    if (sum == 0) {return 0;}  // Every covered leaf term is exactly zero.
    // Sequential additions of values in [0,1] never exceed the exact integer
    // beam count, so the computed double leaf score is also bounded by one.
    return std::min(1.0, std::nextafter(sum / beams_ + guard,
      std::numeric_limits<double>::infinity()));
  }
  double leaf(const map_t & m, int x, int y, int a)
  {
    ++result_.evaluations;
    if (x < local_min_x_ || x > local_max_x_ || y < local_min_y_ || y > local_max_y_ ||
      m.cells[MAP_INDEX((&m), x * stride_, y * stride_)].occ_state != -1) {return -1;}
    double sum = 0;
    for (size_t i = 0; i < beams_; ++i) {
      const auto & o = offsets_[static_cast<size_t>(a) * beams_ + i];
      const int xx = x * stride_ + o.x, yy = y * stride_ + o.y;
      if (MAP_VALID((&m), xx, yy)) {sum += field(m.cells[MAP_INDEX((&m), xx, yy)].occ_dist);}
    }
    return sum / beams_;
  }
  bool insideBest(const Node & node) const
  {
    const int angle_end = std::min(angles_, node.angle + (1 << node.angle_level));
    for (int a = node.angle; a < angle_end; ++a) {
      if (std::abs(wrap(angle(a) - angle(best_a_))) >= config_.mode_yaw) {return false;}
    }
    const int side = 1 << node.level;
    const int xmax = std::min(nx_, node.x + side) - 1, ymax = std::min(ny_, node.y + side) - 1;
    const double dx = std::max(std::abs(node.x - best_x_), std::abs(xmax - best_x_)) * result_.lattice_xy_step;
    const double dy = std::max(std::abs(node.y - best_y_), std::abs(ymax - best_y_)) * result_.lattice_xy_step;
    return std::hypot(dx, dy) < config_.mode_translation;
  }
  void search(const map_t & m, const std::vector<AcquisitionPoint> & points, const AcquisitionPose * seed)
  {
    if (!prepare(m, points)) {result_.status = "search_memory_or_domain_budget_exceeded"; return;}
    if (!result_.lattice_poses) {
      result_.status = "no_free_space_candidate"; result_.search_complete = result_.exhaustive_lattice = true; return;
    }
    double best = -1;
    if (seed && result_.evaluations < config_.max_evaluations) {
      const double sx = (((seed->x - m.origin_x) / m.scale) + m.size_x / 2) / stride_;
      const double sy = (((seed->y - m.origin_y) / m.scale) + m.size_y / 2) / stride_;
      if (sx >= 0 && sx < nx_ && sy >= 0 && sy < ny_) {
        const int x = std::min(nx_ - 1, static_cast<int>(std::llround(sx)));
        const int y = std::min(ny_ - 1, static_cast<int>(std::llround(sy)));
        int a = static_cast<int>(std::llround((wrap(seed->yaw) + M_PI) * angles_ / (2 * M_PI)));
        a = (a % angles_ + angles_) % angles_;
        best = leaf(m, x, y, a); best_x_ = x; best_y_ = y; best_a_ = a;
      }
    }
    // The priority queue plus evaluated leaves is a complete partition of the
    // finite free-pose domain. Preserve it between maximization and competitor
    // certification: no coarse threshold, top-K, or rebuilding different evidence.
    NodeQueue frontier;
    std::vector<Node> evaluated;
    auto room = [&](bool queue_growth, bool evaluated_growth) {
        const uint64_t old_queue = frontier.capacity(), old_evaluated = evaluated.capacity();
        const uint64_t new_queue = queue_growth && frontier.size() == old_queue ?
          std::max<uint64_t>(1, old_queue * 2) : old_queue;
        const uint64_t new_evaluated = evaluated_growth && evaluated.size() == old_evaluated ?
          std::max<uint64_t>(1, old_evaluated * 2) : old_evaluated;
        // Include retained capacities and old+new buffers during reallocation.
        const uint64_t peak_queue = new_queue > old_queue ? old_queue + new_queue : old_queue;
        const uint64_t peak_evaluated = new_evaluated > old_evaluated ?
          old_evaluated + new_evaluated : old_evaluated;
        const uint64_t peak = fixed_memory_ + (peak_queue + peak_evaluated) * sizeof(Node);
        if (peak > config_.max_memory_bytes) {return false;}
        result_.memory_bytes = std::max(result_.memory_bytes, peak);
        if (new_queue > old_queue) {frontier.reserve(new_queue);}
        if (new_evaluated > old_evaluated) {evaluated.reserve(new_evaluated);}
        const uint64_t actual = fixed_memory_ +
          (frontier.capacity() + evaluated.capacity()) * sizeof(Node);
        result_.memory_bytes = std::max(result_.memory_bytes, actual);
        return actual <= config_.max_memory_bytes;
      };
    auto insert = [&](Node node) {
        if (node.angle >= angles_ || !freeCount(node.x, node.y, 1 << node.level)) {return true;}
        if (result_.evaluations >= config_.max_evaluations || !room(true, false)) {return false;}
        node.upper = upper(node); frontier.push(node); return true;
      };
    bool complete = true;
    const int side = 1 << config_.root_block_power;
    const int angle_group = 1 << config_.root_angle_power;
    for (int a = 0; a < angles_ && complete; a += angle_group) {
      for (int y = (local_min_y_ / side) * side; y <= local_max_y_ && complete; y += side) {
        for (int x = (local_min_x_ / side) * side; x <= local_max_x_; x += side) {
          if (!insert({x, y, config_.root_block_power, a, config_.root_angle_power, 1})) {complete = false; break;}
        }
      }
    }
    for (int phase = 0; phase < 2; ++phase) {
      double other = 0; AcquisitionCandidate competitor{};
      // Rounding down makes the floating cutoff conservative for best-margin.
      const double cutoff = phase ? std::nextafter(best - config_.min_margin,
        -std::numeric_limits<double>::infinity()) : best;
      if (phase) {
        for (const auto & node : evaluated) {
          if (!insideBest(node) && node.upper > other) {
            other = node.upper; competitor = {pose(m, node.x, node.y, node.angle), other};
          }
        }
      }
      while (complete && !frontier.empty() && !(phase && other > cutoff)) {
        Node node = frontier.top();
        if (node.upper <= (phase ? cutoff : best)) {break;}
        frontier.pop();
        if (phase && insideBest(node)) {continue;}
        if (node.level == 0 && node.angle_level == 0) {
          if (result_.evaluations >= config_.max_evaluations || (!phase && !room(false, true))) {
            frontier.push(node); complete = false; break;
          }
          const double value = leaf(m, node.x, node.y, node.angle);
          if (!phase) {
            node.upper = value; evaluated.push_back(node);
            if (value > best) {best = value; best_x_ = node.x; best_y_ = node.y; best_a_ = node.angle;}
          } else if (value > other) {
            other = value; competitor = {pose(m, node.x, node.y, node.angle), value};
          }
        } else if (node.angle_level > 0 && (node.level == 0 ||
          yaw_spans_[node.angle_level][node.angle >> node.angle_level] >=
          static_cast<uint32_t>(((1 << node.level) - 1) * stride_)))
        {
          const int half_angle = 1 << (node.angle_level - 1);
          for (int da = 0; da < 2; ++da) {
            if (!insert({node.x, node.y, node.level, node.angle + da * half_angle,
              node.angle_level - 1, node.upper})) {complete = false; break;}
          }
        } else {
          const int half = 1 << (node.level - 1);
          for (int dy = 0; dy < 2 && complete; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
              if (!insert({node.x + dx * half, node.y + dy * half, node.level - 1,
                node.angle, node.angle_level, node.upper})) {
                complete = false; break;
              }
            }
          }
        }
      }
      result_.frontier_size = frontier.size();
      if (!complete) {
        // A failed insertion may leave unqueued siblings or roots, hence the
        // universal upper bound, never only the smaller current heap maximum.
        result_.status = "search_budget_exceeded"; result_.search_complete = false;
        result_.exhaustive_lattice = false; result_.competitor_upper_bound = 1;
        result_.certificate_valid = false; return;
      }
      if (!phase) {
        result_.modes = {{pose(m, best_x_, best_y_, best_a_), best}}; result_.best_score = best;
        result_.search_complete = result_.exhaustive_lattice = true;
        if (best < config_.min_score) {result_.status = "unsupported"; return;}
      } else {
        result_.runner_up_score = other; if (other > 0) {result_.modes.push_back(competitor);}
        result_.competitor_upper_bound = std::max(other, frontier.empty() ? 0.0 : frontier.top().upper);
        result_.certified_gap = best - result_.competitor_upper_bound; result_.margin = result_.certified_gap;
        result_.certificate_valid = result_.competitor_upper_bound <= cutoff;
        result_.status = result_.certificate_valid ? "certified_lattice_mode" : "ambiguous";
      }
    }
  }
};
}  // namespace nav2_amcl
#endif
