#pragma once
#include "duojin01_navigation/gpu_backend.hpp"
#include "duojin01_navigation/local_grid_snapshot.hpp"
#include "duojin01_navigation/velocity_limits.hpp"

namespace duojin01_navigation {
struct SamplingSettings {
  size_t batch_size{1600},time_steps{56},support_points{8};
  double dt{.05},temperature{.3},gamma{.015};
  Velocity noise{.15,.12,.18};
  double acceleration{.4},angular_acceleration{.8},margin{.010};
  double path_weight{12.},goal_weight{8.},yaw_weight{1.},cost_weight{2.};
  uint64_t seed{20261010};
  bool path_primitives{false};
  double primitive_fraction{.25};
};
struct SamplingResult {
  Velocity command;
  bool valid{false},used_cuda{false},used_best_trajectory{false};
  size_t collision_free{0};
  size_t primitive_samples{0},primitive_collision_free{0};
  double wall_ms{0};
  std::vector<double> costs;
  std::vector<Velocity> sequence;
};
// Optional replacement optimizer, not a bitwise port of Nav2's critic stack.
// CPU and CUDA variants implement the same low-dimensional time-interpolated
// sampling algorithm. The existing controller's final braking checks still run.
class SamplingCompute {
 public:
  SamplingCompute(SamplingSettings settings={},ComputeOptions options={});
  ~SamplingCompute();
  void reset();
  SamplingResult optimize(const LocalGridSnapshot &map,const Polygon &polygon,
    const std::vector<Pose> &path,const Pose &start,const Velocity &measured,
    const OmniVelocityBounds &bounds);
  bool available() const;
  std::string deviceName() const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
