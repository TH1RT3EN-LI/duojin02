#include "geometry_internal.hpp"
#include <chrono>
#include <cmath>
#include <limits>

namespace duojin02_navigation {
#ifndef DUOJIN_HAS_CUDA
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason) {
  reason = "this build has no CUDA backend";
  return {};
}
#endif
struct GeometryCompute::Impl {
  GeometryOptions options;
  GeometryStatistics stats;
  const Polygon *polygon{nullptr};
  const std::vector<Point> *points{nullptr};
  uint64_t epoch{0};
  std::unique_ptr<CudaExecutor> cuda;
  std::string reason;

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
  if (polygon.size() < 3 || polygon.size() > 64)
    throw ComputeFailure("Invalid GPU geometry footprint size");
  double area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || std::hypot(b.x - a.x, b.y - a.y) < 1e-12)
      throw ComputeFailure("GPU geometry requires finite, distinct footprint vertices");
    area += a.x * b.y - a.y * b.x;
    for (const auto &p : polygon)
      if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < -1e-12)
        throw ComputeFailure("GPU geometry requires a convex CCW footprint");
  }
  if (!std::isfinite(area) || area <= 1e-12)
    throw ComputeFailure("Degenerate or clockwise GPU geometry footprint");
  for (const auto &p : points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.radius) || p.radius < 0)
      throw ComputeFailure("Invalid GPU obstacle geometry");
  impl_->polygon = &polygon;
  impl_->points = &points;
  ++impl_->epoch;
}

GeometryResult GeometryCompute::evaluate(const std::vector<Pose> &poses, double margin) {
  const auto begin = std::chrono::steady_clock::now();
  if (!impl_->polygon || !impl_->points)
    throw ComputeFailure("Geometry scene not set");
  if (!std::isfinite(margin) || margin < 0)
    throw ComputeFailure("Invalid geometry margin");
  for (const auto &p : poses)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
      throw ComputeFailure("Invalid query pose");
  GeometryResult result;
  result.collisions.resize(poses.size());
  const auto &polygon = *impl_->polygon;
  const auto &points = *impl_->points;
  if (willUseCuda(poses.size())) {
    std::vector<PoseFrame> frames;
    frames.reserve(poses.size());
    for (const auto &p : poses)
      frames.push_back({p.x, p.y, std::cos(p.yaw), std::sin(p.yaw)});
    std::vector<DeviceGeometryResult> raw;
    try {
      raw = impl_->cuda->geometry(polygon, points, impl_->epoch, frames, margin);
    } catch (const std::exception &e) {
      throw ComputeFailure(e.what());
    }
    if (raw.size() != poses.size())
      throw ComputeFailure("CUDA result cardinality mismatch");
    for (size_t i = 0; i < poses.size(); ++i) {
      if (raw[i].uncertain) {
        result.collisions[i] = collision(poses[i], polygon, points, margin);
        ++impl_->stats.boundary_rechecks;
      } else
        result.collisions[i] = raw[i].collision != 0;
    }
    result.used_cuda = true;
    ++impl_->stats.cuda_batches;
  } else {
    for (size_t i = 0; i < poses.size(); ++i)
      result.collisions[i] = collision(poses[i], polygon, points, margin);
    ++impl_->stats.cpu_batches;
  }
  impl_->stats.last_wall_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}

bool GeometryCompute::available() const {
  return impl_->cuda != nullptr;
}

bool GeometryCompute::willUseCuda(size_t pose_count) const {
  return impl_->cuda && impl_->points && pose_count && !impl_->points->empty() &&
         static_cast<long double>(pose_count) * impl_->points->size() >=
             impl_->options.minimum_work;
}

std::string GeometryCompute::deviceName() const {
  return impl_->cuda ? impl_->cuda->name() : impl_->reason;
}

const GeometryStatistics &GeometryCompute::statistics() const {
  return impl_->stats;
}
}
