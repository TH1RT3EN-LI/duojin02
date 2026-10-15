#include "geometry_internal.hpp"
#include "duojin02_navigation/tracking.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace duojin02_navigation {
#ifndef DUOJIN_HAS_CUDA
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason) {
  reason = "this build has no CUDA backend";
  return {};
}
#endif
namespace {
bool equalDouble(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

bool equalPoints(const std::vector<Point> &a, const std::vector<Point> &b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!equalDouble(a[i].x, b[i].x) || !equalDouble(a[i].y, b[i].y) ||
        !equalDouble(a[i].radius, b[i].radius))
      return false;
  return true;
}

GeometryGrid makeGrid(const Polygon &polygon, const std::vector<Point> &points,
                      GeometryBounds bounds, std::vector<Point> &ordered) {
  GeometryGrid grid;
  // Every point admitted by the CPU's expanded local AABB lies within this
  // L1 bound, including AABB corners outside the actual footprint polygon.
  grid.footprint_extent = std::max(std::abs(bounds.min_x), std::abs(bounds.max_x)) +
                          std::max(std::abs(bounds.min_y), std::abs(bounds.max_y));
  if (points.empty() || !std::isfinite(grid.footprint_extent) ||
      grid.footprint_extent > 1e6)
    return grid;
  double min_x = points.front().x, max_x = min_x;
  double min_y = points.front().y, max_y = min_y;
  for (const auto &p : points) {
    // Extreme world coordinates retain the original complete CPU predicate.
    if (std::abs(p.x) > 1e6 || std::abs(p.y) > 1e6 || p.radius > 1e6)
      return grid;
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
    grid.max_radius = std::max(grid.max_radius, p.radius);
  }
  grid.origin_x = min_x;
  grid.origin_y = min_y;
  grid.cell_size = std::max(.125, grid.footprint_extent / 2);
  constexpr uint64_t maximum_cells = 262144;
  for (;;) {
    const double width = std::floor((max_x - min_x) / grid.cell_size) + 1;
    const double height = std::floor((max_y - min_y) / grid.cell_size) + 1;
    if (width * height <= maximum_cells) {
      grid.width = static_cast<uint32_t>(width);
      grid.height = static_cast<uint32_t>(height);
      break;
    }
    grid.cell_size *= 2;
  }
  const size_t cells = static_cast<size_t>(grid.width) * grid.height;
  grid.offsets.assign(cells + 1, 0);
  const auto cell = [&](const Point &p) {
    const auto x = std::min(grid.width - 1,
                            static_cast<uint32_t>((p.x - min_x) / grid.cell_size));
    const auto y = std::min(grid.height - 1,
                            static_cast<uint32_t>((p.y - min_y) / grid.cell_size));
    return static_cast<size_t>(y) * grid.width + x;
  };
  for (const auto &p : points)
    ++grid.offsets[cell(p) + 1];
  for (size_t i = 1; i < grid.offsets.size(); ++i)
    grid.offsets[i] += grid.offsets[i - 1];
  auto next = grid.offsets;
  ordered.resize(points.size());
  for (const auto &p : points)
    ordered[next[cell(p)]++] = p;
  (void)polygon;
  return grid;
}

// Empty ranges are represented by x1/y1 < x0/y0. A false return requests the
// full scan; uncertain index arithmetic is never interpreted as clear space.
bool cellRange(const Pose &pose, double padding, const GeometryGrid &grid,
               int &x0, int &y0, int &x1, int &y1) {
  if (!grid.width || std::abs(pose.x) > 1e6 || std::abs(pose.y) > 1e6)
    return false;
  const double radius = grid.footprint_extent + 2 * (padding + grid.max_radius);
  if (!std::isfinite(radius) || radius > 1e6)
    return false;
  // This inflation dominates FP64 subtraction, trig, and bucket-boundary
  // rounding on the indexed coordinate domain. It only adds candidates.
  const double guard = 1e-7 * (1 + std::abs(pose.x) + std::abs(pose.y) + radius);
  const double low_x = std::floor((pose.x - radius - guard - grid.origin_x) / grid.cell_size);
  const double high_x = std::floor((pose.x + radius + guard - grid.origin_x) / grid.cell_size);
  const double low_y = std::floor((pose.y - radius - guard - grid.origin_y) / grid.cell_size);
  const double high_y = std::floor((pose.y + radius + guard - grid.origin_y) / grid.cell_size);
  x0 = static_cast<int>(std::clamp(low_x, 0., static_cast<double>(grid.width)));
  x1 = static_cast<int>(std::clamp(high_x, -1., static_cast<double>(grid.width - 1)));
  y0 = static_cast<int>(std::clamp(low_y, 0., static_cast<double>(grid.height)));
  y1 = static_cast<int>(std::clamp(high_y, -1., static_cast<double>(grid.height - 1)));
  return true;
}

struct QueryCells {
  int x0{0}, y0{0}, x1{-1}, y1{-1};
  bool indexed{false};
};

bool worldGate(const Point &p, const GeometryWorldBounds &bounds, double margin) {
  const double padding = margin + p.radius;
  return p.x >= bounds.min_x - padding && p.x <= bounds.max_x + padding &&
         p.y >= bounds.min_y - padding && p.y <= bounds.max_y + padding;
}

// A direct visitor retains collision's early stop without materialising a
// per-pose obstacle vector. Clearance still visits every relevant obstacle.
template <class Visitor>
void visitCandidates(const QueryCells &cells, const GeometryGrid &grid,
                     const std::vector<Point> &points, const std::vector<Point> &indexed,
                     Visitor &&visitor) {
  if (!cells.indexed) {
    for (const auto &p : points)
      if (visitor(p)) return;
  } else if (cells.x0 <= cells.x1) {
    for (int y = cells.y0; y <= cells.y1; ++y) {
      const size_t row = static_cast<size_t>(y) * grid.width;
      for (size_t i = grid.offsets[row + cells.x0]; i < grid.offsets[row + cells.x1 + 1]; ++i)
        if (visitor(indexed[i])) return;
    }
  }
}

void cpuQuery(const Pose &pose, double margin, double cap, const GeometryWorldBounds *world_bounds,
              const Polygon &polygon, GeometryBounds bounds, const QueryCells &cells,
              const GeometryGrid &grid, const std::vector<Point> &points,
              const std::vector<Point> &indexed, uint8_t &hit, double &clearance) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  hit = 0;
  clearance = cap;
  visitCandidates(cells, grid, points, indexed, [&](const Point &p) {
    if (world_bounds && !worldGate(p, *world_bounds, margin))
      return false;
    const double dx = p.x - pose.x, dy = p.y - pose.y;
    const Point q{c * dx + s * dy, -s * dx + c * dy, p.radius};
    const double padding = margin + p.radius;
    // Use the original CPU's negative rejection comparisons, including its
    // IEEE behaviour for overflowed intermediate coordinates.
    if (!hit && !(q.x < bounds.min_x - padding || q.x > bounds.max_x + padding ||
                  q.y < bounds.min_y - padding || q.y > bounds.max_y + padding))
      hit = touches(q, polygon, margin);
    if (cap >= 0) {
      const double cp = cap + p.radius;
      if (!(q.x < bounds.min_x - cp || q.x > bounds.max_x + cp ||
            q.y < bounds.min_y - cp || q.y > bounds.max_y + cp)) {
        bool inside = true;
        double distance = 1e100;
        for (size_t i = 0; i < polygon.size(); ++i) {
          const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
          if ((b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x) < -1e-10)
            inside = false;
          distance = std::min(distance, distanceToSegment(q, a, b));
        }
        clearance = std::min(clearance, (inside ? -distance : distance) - p.radius);
      }
    }
    return cap < 0 && hit;
  });
}
}

struct GeometryCompute::Impl {
  GeometryOptions options;
  GeometryStatistics stats;
  Polygon polygon;
  std::vector<Point> points, ordered_points;
  GeometryGrid grid;
  GeometryBounds bounds{};
  bool scene_set{false};
  uint64_t epoch{0};
  std::unique_ptr<CudaExecutor> cuda;
  std::string reason;
  bool failed{false};

  explicit Impl(GeometryOptions o) : options(o) {
    if (o.use_gpu)
      cuda = makeCudaExecutor(reason);
    else
      reason = "use_gpu=false";
  }
};

GeometryCompute::GeometryCompute(GeometryOptions options) : impl_(new Impl(options)) {}
GeometryCompute::~GeometryCompute() = default;
GeometryCompute::GeometryCompute(GeometryCompute &&) noexcept = default;
GeometryCompute &GeometryCompute::operator=(GeometryCompute &&) noexcept = default;

void GeometryCompute::setScene(const Polygon &polygon, const std::vector<Point> &points) {
  if (impl_->failed)
    throw ComputeFailure("CUDA geometry fault latched; reconfigure required");
  if (polygon.size() < 3 || polygon.size() > 64)
    throw ComputeFailure("Invalid GPU geometry footprint size");
  double area = 0;
  GeometryBounds bounds{1e100, 1e100, -1e100, -1e100};
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || std::hypot(b.x - a.x, b.y - a.y) < 1e-12)
      throw ComputeFailure("GPU geometry requires finite, distinct footprint vertices");
    area += a.x * b.y - a.y * b.x;
    for (const auto &p : polygon)
      if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < -1e-12)
        throw ComputeFailure("GPU geometry requires a convex CCW footprint");
    bounds.min_x = std::min(bounds.min_x, a.x);
    bounds.min_y = std::min(bounds.min_y, a.y);
    bounds.max_x = std::max(bounds.max_x, a.x);
    bounds.max_y = std::max(bounds.max_y, a.y);
  }
  if (!std::isfinite(area) || area <= 1e-12)
    throw ComputeFailure("Degenerate or clockwise GPU geometry footprint");
  for (const auto &p : points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.radius) || p.radius < 0)
      throw ComputeFailure("Invalid GPU obstacle geometry");
  if (impl_->scene_set && equalPoints(impl_->polygon, polygon) && equalPoints(impl_->points, points)) {
    ++impl_->stats.scene_reuses;
    return;
  }
  Polygon owned_polygon = polygon;
  std::vector<Point> owned_points = points, ordered;
  auto grid = makeGrid(owned_polygon, owned_points, bounds, ordered);
  impl_->polygon = std::move(owned_polygon);
  impl_->points = std::move(owned_points);
  impl_->ordered_points = std::move(ordered);
  impl_->grid = std::move(grid);
  impl_->bounds = bounds;
  impl_->scene_set = true;
  ++impl_->epoch;
  ++impl_->stats.scene_updates;
}

GeometryResult GeometryCompute::evaluate(const std::vector<Pose> &poses, double margin) {
  if (!std::isfinite(margin) || margin < 0)
    throw ComputeFailure("Invalid geometry margin");
  return compute(poses, std::vector<double>(poses.size(), margin), -1);
}
GeometryResult GeometryCompute::evaluate(const std::vector<Pose> &poses,
                                         const std::vector<double> &margins) {
  return compute(poses, margins, -1);
}
GeometryResult GeometryCompute::evaluate(const std::vector<Pose> &poses,
                                         const std::vector<double> &margins,
                                         const std::vector<GeometryWorldBounds> &world_bounds) {
  if (world_bounds.size() != poses.size())
    throw ComputeFailure("Geometry world bounds cardinality mismatch");
  return compute(poses, margins, -1, world_bounds);
}
GeometryResult GeometryCompute::clearance(const std::vector<Pose> &poses, double cap) {
  return evaluateWithClearance(poses, 0, cap);
}
GeometryResult GeometryCompute::evaluateWithClearance(const std::vector<Pose> &poses,
                                                       double margin, double cap) {
  if (!std::isfinite(margin) || margin < 0)
    throw ComputeFailure("Invalid geometry margin");
  if (!std::isfinite(cap) || cap < 0)
    throw ComputeFailure("Invalid geometry clearance cap");
  return compute(poses, std::vector<double>(poses.size(), margin), cap);
}

GeometryResult GeometryCompute::compute(const std::vector<Pose> &poses,
                                        const std::vector<double> &margins, double cap,
                                        const std::vector<GeometryWorldBounds> &world_bounds) {
  const auto begin = std::chrono::steady_clock::now();
  if (!impl_->scene_set)
    throw ComputeFailure("Geometry scene not set");
  if (impl_->failed)
    throw ComputeFailure("CUDA geometry fault latched; reconfigure required");
  if (margins.size() != poses.size())
    throw ComputeFailure("Geometry margin cardinality mismatch");
  for (double margin : margins)
    if (!std::isfinite(margin) || margin < 0)
      throw ComputeFailure("Invalid geometry margin");
  for (const auto &b : world_bounds)
    if (!std::isfinite(b.min_x) || !std::isfinite(b.min_y) || !std::isfinite(b.max_x) ||
        !std::isfinite(b.max_y) || b.min_x > b.max_x || b.min_y > b.max_y)
      throw ComputeFailure("Invalid geometry world bounds");
  for (const auto &p : poses)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
      throw ComputeFailure("Invalid query pose");
  GeometryResult result;
  result.collisions.resize(poses.size());
  result.scene_version = impl_->epoch;
  if (cap >= 0) {
    result.clearances.resize(poses.size(), cap);
    result.clearance_error_bounds.resize(poses.size(), 0);
    ++impl_->stats.clearance_batches;
  }
  const auto &polygon = impl_->polygon;
  const auto &points = impl_->points;
  const auto &indexed = impl_->grid.width ? impl_->ordered_points : points;
  bool indexed_batch = false, full_scan_batch = false;
  std::vector<QueryCells> ranges(poses.size());
  uint64_t candidate_work = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
    auto &cells = ranges[i];
    cells.indexed = cellRange(poses[i], std::max(margins[i], cap), impl_->grid,
                              cells.x0, cells.y0, cells.x1, cells.y1);
    uint64_t count = 0;
    if (cells.indexed) {
      indexed_batch = true;
      if (cells.x0 <= cells.x1)
        for (int y = cells.y0; y <= cells.y1; ++y) {
          const size_t row = static_cast<size_t>(y) * impl_->grid.width;
          count += impl_->grid.offsets[row + cells.x1 + 1] - impl_->grid.offsets[row + cells.x0];
        }
    } else {
      full_scan_batch = true;
      count = points.size();
    }
    candidate_work += std::min(count, std::numeric_limits<uint64_t>::max() - candidate_work);
  }
  impl_->stats.last_candidate_work = candidate_work;
  const bool cuda_possible = willUseCuda(poses.size());
  const bool use_cuda = cuda_possible && candidate_work >= impl_->options.minimum_work;
  if (cuda_possible && !use_cuda)
    ++impl_->stats.candidate_cpu_dispatches;
  if (use_cuda) {
    std::vector<PoseFrame> frames;
    frames.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto &p = poses[i];
      frames.push_back({p.x, p.y, std::cos(p.yaw), std::sin(p.yaw), margins[i],
                         world_bounds.empty() ? GeometryWorldBounds{} : world_bounds[i],
                         world_bounds.empty() ? 0U : 1U});
    }
    DeviceGeometryBatch batch;
    try {
      batch = impl_->cuda->geometry(polygon, impl_->bounds, indexed, impl_->grid,
                                    impl_->epoch, frames, cap);
    } catch (const std::exception &e) {
      impl_->failed = true;
      throw ComputeFailure(e.what());
    }
    if (batch.results.size() != poses.size()) {
      impl_->failed = true;
      throw ComputeFailure("CUDA result cardinality mismatch");
    }
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto &raw = batch.results[i];
      if (raw.uncertain) {
        double score;
        cpuQuery(poses[i], margins[i], cap, world_bounds.empty() ? nullptr : &world_bounds[i],
                  polygon, impl_->bounds, QueryCells{}, impl_->grid, points, indexed,
                  result.collisions[i], score);
        ++impl_->stats.boundary_rechecks;
        if (cap >= 0) {
          result.clearances[i] = score;
          ++impl_->stats.clearance_rechecks;
        }
      } else {
        result.collisions[i] = raw.collision != 0;
        if (cap >= 0) {
          result.clearances[i] = raw.clearance;
          result.clearance_error_bounds[i] = raw.clearance_error;
        }
      }
    }
    if (batch.scene_uploaded) {
      ++impl_->stats.scene_uploads;
      impl_->stats.scene_upload_bytes += batch.uploaded_bytes;
    }
    result.used_cuda = true;
    ++impl_->stats.cuda_batches;
  } else {
    for (size_t i = 0; i < poses.size(); ++i) {
      double score;
      cpuQuery(poses[i], margins[i], cap, world_bounds.empty() ? nullptr : &world_bounds[i],
                polygon, impl_->bounds, ranges[i], impl_->grid, points, indexed,
                result.collisions[i], score);
      if (cap >= 0)
        result.clearances[i] = score;
    }
    ++impl_->stats.cpu_batches;
  }
  if (indexed_batch)
    ++impl_->stats.indexed_batches;
  if (full_scan_batch)
    ++impl_->stats.full_scan_batches;
  impl_->stats.last_wall_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}

bool GeometryCompute::available() const { return impl_->cuda != nullptr; }
bool GeometryCompute::willUseCuda(size_t pose_count) const {
  return impl_->cuda && impl_->scene_set && pose_count && !impl_->points.empty() &&
         static_cast<long double>(pose_count) * impl_->points.size() >= impl_->options.minimum_work;
}
std::string GeometryCompute::deviceName() const {
  return impl_->cuda ? impl_->cuda->name() : impl_->reason;
}
const GeometryStatistics &GeometryCompute::statistics() const { return impl_->stats; }
}
