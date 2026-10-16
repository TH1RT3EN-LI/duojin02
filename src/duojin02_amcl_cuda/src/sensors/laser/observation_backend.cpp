// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/sensors/laser/observation_backend.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
namespace nav2_amcl {
#ifndef DUOJIN02_AMCL_HAS_CUDA
std::unique_ptr<DeviceObservation> makeDeviceObservation() {
  throw std::runtime_error("AMCL was built without CUDA");
}
#endif
void ObservationBackend::validate(const std::string & mode) {
  if (mode != "cpu" && mode != "auto" && mode != "cuda") {
    throw std::invalid_argument("observation_backend must be cpu, auto, or cuda");
  }
  if (mode == "cuda") { auto check = makeDeviceObservation(); }
}
ObservationBackend::ObservationBackend(const std::string & mode, int min_particles, int log_every, map_t * map)
: mode_(mode), min_particles_(min_particles), log_every_(log_every) {
  validate(mode == "cuda" ? "auto" : mode);  // strict availability checked below once
  if (min_particles < 0 || log_every < 0) { throw std::invalid_argument("AMCL backend counters/threshold must be nonnegative"); }
  if (mode == "cpu") { return; }
  try {
    device_ = makeDeviceObservation();
    ObservationMap metadata{map->size_x, map->size_y, map->origin_x, map->origin_y,
      map->scale, map->max_occ_dist};
    std::vector<double> distances(static_cast<size_t>(map->size_x) * map->size_y);
    for (size_t i=0; i<distances.size(); ++i) { distances[i]=map->cells[i].occ_dist; }
    device_->map(metadata, distances);
    ++stats_.map_uploads;
  } catch (const std::exception & e) {
    ++stats_.failures; disabled_=true; device_.reset();
    if (mode_ == "cuda") { throw; }
    std::fprintf(stderr, "AMCL observation auto fallback: %s\n", e.what());
  }
}
ObservationBackend::~ObservationBackend() { report(); }
void ObservationBackend::report() const {
  std::fprintf(stderr,
    "AMCL_OBSERVATION_STATS backend=%s cuda_updates=%llu cpu_updates=%llu map_uploads=%llu failures=%llu boundary_fallbacks=%llu particles=%llu beams=%llu total_us=%.3f last_us=%.3f\n",
    mode_.c_str(), static_cast<unsigned long long>(stats_.cuda_updates),
    static_cast<unsigned long long>(stats_.cpu_updates), static_cast<unsigned long long>(stats_.map_uploads),
    static_cast<unsigned long long>(stats_.failures), static_cast<unsigned long long>(stats_.boundary_fallbacks),
    static_cast<unsigned long long>(stats_.particles), static_cast<unsigned long long>(stats_.beams),
    stats_.total_us, stats_.last_us);
}
void ObservationBackend::recordCpu(double duration_us, int particles, int beams) {
  stats_.last_us=duration_us; stats_.total_us+=duration_us;
  stats_.particles=particles; stats_.beams=beams;
  ++stats_.cpu_updates;
  if (log_every_ && (stats_.cpu_updates+stats_.cuda_updates)%log_every_ == 0) { report(); }
}
bool ObservationBackend::score(const ObservationSettings & settings, const pf_vector_t & laser_pose,
  LaserData * data, pf_sample_set_t * set, double & total) {
  if (mode_ == "cpu" || disabled_ || (mode_ == "auto" && set->sample_count < min_particles_)) { return false; }
  const auto start=std::chrono::steady_clock::now();
  try {
    // Preserve upstream pose composition (including yaw normalization) on the host.
    poses_.resize(set->sample_count); factors_.resize(set->sample_count);
    for (int i=0;i<set->sample_count;++i) {
      const auto pose=pf_vector_coord_add(laser_pose, set->samples[i].pose);
      poses_[i]={pose.v[0],pose.v[1],pose.v[2]};
    }
    const int step=std::max(1, settings.probability_model ?
      static_cast<int>(std::ceil(data->range_count/static_cast<double>(settings.max_beams))) :
      (data->range_count-1)/(settings.max_beams-1));
    beams_.clear();
    for (int i=0;i<data->range_count;i+=step) { beams_.push_back({data->ranges[i][0],data->ranges[i][1]}); }
    // The upstream probabilistic beam mask has max_beams slots, even for short scans.
    const int selected=static_cast<int>(beams_.size());
    if (settings.probability_model) {
      beams_.resize(settings.max_beams, {data->range_max, 0.0});
    }
    stats_.particles=set->sample_count; stats_.beams=selected;
    if (!device_->score(settings, poses_.data(), set->sample_count, beams_.data(),
      static_cast<int>(beams_.size()), data->range_max, set->converged, factors_.data())) {
      ++stats_.boundary_fallbacks; return false;
    }
    // Commit atomically after successful device completion; no partial mutation on fallback.
    total=0.0;
    for (int i=0;i<set->sample_count;++i) {
      set->samples[i].weight *= factors_[i]; total += set->samples[i].weight;
    }
    ++stats_.cuda_updates;
    stats_.last_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    stats_.total_us+=stats_.last_us;
    if (log_every_ && (stats_.cpu_updates+stats_.cuda_updates)%log_every_ == 0) { report(); }
    return true;
  } catch (const std::exception & e) {
    ++stats_.failures;
    if (mode_ == "cuda") { throw; }
    disabled_=true; device_.reset();
    std::fprintf(stderr, "AMCL observation auto disabled after device error: %s\n", e.what());
    return false;
  }
}
}  // namespace nav2_amcl
