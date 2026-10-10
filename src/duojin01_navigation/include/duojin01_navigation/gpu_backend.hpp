#pragma once
#include "duojin01_navigation/tracking.hpp"
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace duojin01_navigation {
enum class ComputeBackend {Cpu, Cuda, Auto};
class ComputeFailure:public std::runtime_error {public:using std::runtime_error::runtime_error;};
ComputeBackend parseComputeBackend(const std::string &name);
struct ComputeOptions {
  ComputeBackend backend{ComputeBackend::Cpu};
  uint64_t minimum_work{262144};
  double deadline_ms{40.0};
};
struct ComputeStatistics {
  uint64_t cpu_batches{0},cuda_batches{0},boundary_rechecks{0};
  double last_wall_ms{0};
};
struct GeometryResult {
  std::vector<uint8_t> collisions;
  std::vector<double> clearances;
  bool used_cuda{false};
};

// Scene references must remain immutable and alive until the next setScene.
// A scene is uploaded lazily at most once per epoch. No ROS or map mutex is
// owned by this component. Results are returned only after completion.
class GeometryCompute {
 public:
  explicit GeometryCompute(ComputeOptions options={});
  ~GeometryCompute();
  GeometryCompute(GeometryCompute &&) noexcept;
  GeometryCompute &operator=(GeometryCompute &&) noexcept;
  GeometryCompute(const GeometryCompute &)=delete;
  GeometryCompute &operator=(const GeometryCompute &)=delete;
  void setScene(const Polygon &polygon,const std::vector<Point> &points);
  GeometryResult evaluate(const std::vector<Pose> &poses,double margin,
                          bool clearance=false,double cap=.080);
  bool available() const;
  bool willUseCuda(size_t pose_count) const;
  std::string deviceName() const;
  const ComputeStatistics &statistics() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
