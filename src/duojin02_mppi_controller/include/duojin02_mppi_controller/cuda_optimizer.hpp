// Copyright 2026 Duojin contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
namespace duojin_mppi {
struct OptimizerStatistics {
  uint64_t cuda_noise_batches{}, cuda_rollout_batches{}, cuda_update_batches{},
    cuda_critic_batches{}, cpu_rollout_batches{}, cpu_update_batches{}, failures{};
};
class OptimizerComputeError : public std::runtime_error {
public: using std::runtime_error::runtime_error;
};
struct CoreSettings {
  size_t batch{}, steps{};
  float dt{}, temperature{}, gamma{}, std_vx{}, std_vy{}, std_wz{};
  float vx_min{}, vx_max{}, vy_max{}, wz_max{}, min_turning_radius{};
  bool holonomic{};
};
struct CoreBuffers {
  float *cvx{}, *cvy{}, *cwz{}, *vx{}, *vy{}, *wz{}, *x{}, *y{}, *yaw{};
  float *ux{}, *uy{}, *uw{}, *cost{};
};
struct CorePose {float x{}, y{}, yaw{}, vx{}, vy{}, wz{};};
// Backend owns a private stream and persistent buffers. Counters advance only
// after successful synchronization/readback. A device error is never retried on CPU.
class CudaOptimizerCore {
public:
  CudaOptimizerCore();
  ~CudaOptimizerCore();
  CudaOptimizerCore(const CudaOptimizerCore &) = delete;
  CudaOptimizerCore &operator=(const CudaOptimizerCore &) = delete;
  bool available() const;
  const std::string &deviceName() const;
  void rollout(const CoreSettings &, CoreBuffers, CorePose, uint64_t seed, uint64_t epoch,
               const float *nx=nullptr, const float *ny=nullptr, const float *nw=nullptr);
  void update(const CoreSettings &, CoreBuffers);
  void scoreCritic(unsigned kind, float weight, unsigned power, float x, float y,
    float min_velocity, float max_velocity, float radius, float dt, float *costs);
  const OptimizerStatistics &statistics() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// Shared counter-based normal generator, used only when optimizer_seed >= 0.
// It is independent of worker scheduling and has an identical integer stream on CPU/CUDA.
float deterministicNormal(uint64_t seed, uint64_t epoch, uint64_t index, unsigned axis);
void fillDeterministicNoises(float *nx, float *ny, float *nw, size_t n,
  float sx, float sy, float sw, uint64_t seed, uint64_t epoch);
}
