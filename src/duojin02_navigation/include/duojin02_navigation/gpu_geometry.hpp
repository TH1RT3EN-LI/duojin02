#pragma once

#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace duojin02_navigation {
class ComputeFailure : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct GeometryOptions {
  bool use_gpu{true};
  uint64_t minimum_work{262144};
};

struct GeometryWorldBounds {
  double min_x, min_y, max_x, max_y;
};

struct GeometryStatistics {
  uint64_t cpu_batches{0}, cuda_batches{0}, boundary_rechecks{0};
  uint64_t scene_updates{0}, scene_reuses{0}, scene_uploads{0}, scene_upload_bytes{0};
  uint64_t clearance_batches{0}, clearance_rechecks{0};
  uint64_t indexed_batches{0}, full_scan_batches{0};
  uint64_t last_candidate_work{0}, candidate_cpu_dispatches{0};
  double last_wall_ms{0};
};

struct GeometryResult {
  std::vector<uint8_t> collisions;
  // Populated by clearance()/evaluateWithClearance(). Same signed, capped
  // distance as footprintClearance(), including its expanded-AABB gate.
  std::vector<double> clearances;
  // Arithmetic guards for ranking: overlapping intervals require CPU ranking.
  // These describe numerical screening, not physical obstacle uncertainty.
  std::vector<double> clearance_error_bounds;
  bool used_cuda{false};
  uint64_t scene_version{0};
};

class GeometryCompute {
public:
  explicit GeometryCompute(GeometryOptions options = {});
  ~GeometryCompute();
  GeometryCompute(GeometryCompute &&) noexcept;
  GeometryCompute &operator=(GeometryCompute &&) noexcept;
  GeometryCompute(const GeometryCompute &) = delete;
  GeometryCompute &operator=(const GeometryCompute &) = delete;
  // Copies the snapshot. Mutating/destroying caller storage is safe; changed
  // content requires another setScene(). Identical content reuses its version.
  void setScene(const Polygon &polygon, const std::vector<Point> &points);
  GeometryResult evaluate(const std::vector<Pose> &poses, double margin);
  GeometryResult evaluate(const std::vector<Pose> &poses, const std::vector<double> &margins);
  // Restrict each pose to its trajectory's original world-AABB broad phase.
  GeometryResult evaluate(const std::vector<Pose> &poses, const std::vector<double> &margins,
                           const std::vector<GeometryWorldBounds> &world_bounds);
  // Also returns collisions at zero margin. Rank using CPU when clearance
  // error intervals overlap (including the caller's additive ranking terms).
  GeometryResult clearance(const std::vector<Pose> &poses, double cap = .080);
  GeometryResult evaluateWithClearance(const std::vector<Pose> &poses, double margin,
                                       double cap = .080);
  bool available() const;
  // Upper-bound precheck using total obstacle count. evaluate() may still
  // select indexed CPU when the actual neighbouring candidate work is small.
  // minimum_work=0 forces CUDA for every non-empty scene/query batch.
  bool willUseCuda(size_t pose_count) const;
  std::string deviceName() const;
  const GeometryStatistics &statistics() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  GeometryResult compute(const std::vector<Pose> &poses, const std::vector<double> &margins,
                         double clearance_cap,
                         const std::vector<GeometryWorldBounds> &world_bounds = {});
};
}
