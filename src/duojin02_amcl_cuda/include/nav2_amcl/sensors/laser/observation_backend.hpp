// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef DUOJIN02_AMCL_OBSERVATION_BACKEND_HPP_
#define DUOJIN02_AMCL_OBSERVATION_BACKEND_HPP_
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf.hpp"
namespace nav2_amcl {
class LaserData;
struct ObservationSettings {
  bool probability_model{false};
  bool beam_skip{false};
  bool endpoints_in_base_frame{false};
  double likelihood_temperature_beta{1.0};
  double z_hit{}, z_rand{}, sigma_hit{};
  double beam_skip_distance{}, beam_skip_threshold{}, beam_skip_error_threshold{};
  int max_beams{};
};
struct ObservationMap {
  int width{}, height{};
  double origin_x{}, origin_y{}, scale{}, max_distance{};
};
struct ObservationPose { double x, y, yaw; };
struct ObservationBeam { double range, bearing; double endpoint_x{}, endpoint_y{}; bool included{false}; };
struct ObservationStats {
  uint64_t cuda_updates{}, cpu_updates{}, failures{}, map_uploads{}, boundary_fallbacks{};
  uint64_t particles{}, beams{};
  double total_us{}, last_us{};
  uint64_t raw_beams{}, valid_beams{}, effective_beams{};
  int diagnostic_particle{-1};
  double distance_p50{}, distance_p90{}, distance_p99{}, random_floor_fraction{};
  double log_factor_min{}, log_factor_max{}, likelihood_temperature_beta{1.0};
  std::string effective_source_indices, excluded_source_reasons;
};
class DeviceObservation {
public:
  virtual ~DeviceObservation() = default;
  virtual void map(const ObservationMap &, const std::vector<double> &) = 0;
  // false signals near-cell-boundary arithmetic requiring the native CPU oracle.
  virtual bool score(const ObservationSettings &, const ObservationPose *, int,
    const ObservationBeam *, int, double, bool, double *, int *) = 0;
};
std::unique_ptr<DeviceObservation> makeDeviceObservation();
class ObservationBackend {
public:
  ObservationBackend(const std::string & mode, int min_particles, int log_every, map_t * map);
  ~ObservationBackend();
  static void validate(const std::string & mode);
  bool score(const ObservationSettings &, const pf_vector_t &, LaserData *, pf_sample_set_t *, double &);
  void recordCpu(double duration_us, int particles, int beams);
  void recordDiagnostics(const ObservationSettings &, const pf_vector_t &, const LaserData *,
    const pf_sample_set_t *, const std::vector<int> & effective_indices, const double * log_factors);
  const ObservationStats & stats() const { return stats_; }
private:
  map_t * map_;
  std::string mode_;
  int min_particles_, log_every_;
  bool disabled_{false};
  std::unique_ptr<DeviceObservation> device_;
  ObservationStats stats_;
  std::vector<ObservationPose> poses_;
  std::vector<ObservationBeam> beams_;
  std::vector<double> factors_;
  void report() const;
};
}  // namespace nav2_amcl
#endif
