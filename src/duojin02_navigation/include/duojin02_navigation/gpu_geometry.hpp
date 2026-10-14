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

struct GeometryStatistics {
  uint64_t cpu_batches{0}, cuda_batches{0}, boundary_rechecks{0};
  double last_wall_ms{0};
};

struct GeometryResult {
  std::vector<uint8_t> collisions;
  bool used_cuda{false};
};

class GeometryCompute {
public:
  explicit GeometryCompute(GeometryOptions options = {});
  ~GeometryCompute();
  GeometryCompute(GeometryCompute &&) noexcept;
  GeometryCompute &operator=(GeometryCompute &&) noexcept;
  GeometryCompute(const GeometryCompute &) = delete;
  GeometryCompute &operator=(const GeometryCompute &) = delete;
  void setScene(const Polygon &polygon, const std::vector<Point> &points);
  GeometryResult evaluate(const std::vector<Pose> &poses, double margin);
  bool available() const;
  bool willUseCuda(size_t pose_count) const;
  std::string deviceName() const;
  const GeometryStatistics &statistics() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
