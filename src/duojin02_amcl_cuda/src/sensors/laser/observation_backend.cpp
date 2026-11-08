// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/sensors/laser/observation_backend.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include <chrono>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <sstream>
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
: map_(map), mode_(mode), min_particles_(min_particles), log_every_(log_every) {
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
  if (stats_.raw_beams || stats_.likelihood_temperature_beta != 1.0) {
    std::fprintf(stderr,
      "AMCL_OBSERVATION_DIAGNOSTICS raw_beams=%llu selected_beams=%llu valid_beams=%llu effective_beams=%llu beta=%.17g log_factor_min=%.17g log_factor_max=%.17g representative_particle=%d distance_p50_m=%.17g distance_p90_m=%.17g distance_p99_m=%.17g random_floor_fraction=%.17g residual_scope=highest_posterior_candidate\n",
      static_cast<unsigned long long>(stats_.raw_beams), static_cast<unsigned long long>(stats_.beams),
      static_cast<unsigned long long>(stats_.valid_beams), static_cast<unsigned long long>(stats_.effective_beams),
      stats_.likelihood_temperature_beta, stats_.log_factor_min, stats_.log_factor_max,
      stats_.diagnostic_particle, stats_.distance_p50, stats_.distance_p90, stats_.distance_p99,
      stats_.random_floor_fraction);
  }
}
void ObservationBackend::recordCpu(double duration_us, int particles, int beams) {
  stats_.last_us=duration_us; stats_.total_us+=duration_us;
  stats_.particles=particles; stats_.beams=beams;
  ++stats_.cpu_updates;
  if (log_every_ && (stats_.cpu_updates+stats_.cuda_updates)%log_every_ == 0) { report(); }
}
void ObservationBackend::recordDiagnostics(const ObservationSettings & settings,
  const pf_vector_t & laser_pose, const LaserData * data, const pf_sample_set_t * set,
  const std::vector<int> & effective_indices, const double * log_factors) {
  stats_.raw_beams = data->range_count;
  stats_.effective_beams = effective_indices.size();
  stats_.valid_beams = 0;
  const int step = std::max(1, static_cast<int>(std::ceil(
      data->range_count / static_cast<double>(settings.max_beams))));
  for (int i = 0; i < data->range_count; i += step) {
    if (data->acceptsHit(i)) {++stats_.valid_beams;}
  }
  std::ostringstream included, excluded;
  for (int i : effective_indices) {included << (included.tellp() > 0 ? "," : "") << i;}
  for (int i = 0; i < data->range_count; i += step) {
    if (!data->acceptsHit(i)) {
      const auto reason = data->selection_reasons.empty() ?
        LaserData::SelectionReason::SensorInvalid : data->selection_reasons[i];
      excluded << (excluded.tellp() > 0 ? "," : "") << i << ":" << static_cast<int>(reason);
    } else if (std::find(effective_indices.begin(), effective_indices.end(), i) == effective_indices.end()) {
      excluded << (excluded.tellp() > 0 ? "," : "") << i << ":5";  // dynamic beam-skip selection
    }
  }
  stats_.effective_source_indices = included.str().empty() ? "none" : included.str();
  stats_.excluded_source_reasons = excluded.str().empty() ? "none" : excluded.str();
  stats_.likelihood_temperature_beta = settings.likelihood_temperature_beta;
  stats_.log_factor_min = std::numeric_limits<double>::infinity();
  stats_.log_factor_max = -std::numeric_limits<double>::infinity();
  stats_.diagnostic_particle = -1;
  double best = -std::numeric_limits<double>::infinity();
  for (int j = 0; j < set->sample_count; ++j) {
    stats_.log_factor_min = std::min(stats_.log_factor_min, log_factors[j]);
    stats_.log_factor_max = std::max(stats_.log_factor_max, log_factors[j]);
    const double score = std::log(set->samples[j].weight) + log_factors[j];
    if (std::isfinite(score) && score > best) {best = score; stats_.diagnostic_particle = j;}
  }
  // Residual quantiles describe the highest posterior-weight candidate only,
  // using exactly the included raw beam indices. They are not GT residuals.
  stats_.distance_p50 = stats_.distance_p90 = stats_.distance_p99 =
    stats_.random_floor_fraction = std::numeric_limits<double>::quiet_NaN();
  if (stats_.diagnostic_particle < 0 || effective_indices.empty()) {return;}
  auto pose = set->samples[stats_.diagnostic_particle].pose;
  if (!data->endpoints) {pose = pf_vector_coord_add(laser_pose, pose);}
  std::vector<double> distances;
  int floor_count = 0;
  const double random = settings.z_rand * (1.0 / data->range_max);
  for (int i : effective_indices) {
    double x, y;
    if (data->endpoints) {
      const double c = std::cos(pose.v[2]), sn = std::sin(pose.v[2]);
      x = pose.v[0] + c * data->endpoints[i][0] - sn * data->endpoints[i][1];
      y = pose.v[1] + sn * data->endpoints[i][0] + c * data->endpoints[i][1];
    } else {
      x = pose.v[0] + data->ranges[i][0] * std::cos(pose.v[2] + data->ranges[i][1]);
      y = pose.v[1] + data->ranges[i][0] * std::sin(pose.v[2] + data->ranges[i][1]);
    }
    const double gx = MAP_GXWX(map_, x), gy = MAP_GYWY(map_, y);
    if (!std::isfinite(gx) || !std::isfinite(gy)) {continue;}
    double distance = map_->max_occ_dist;
    if (gx >= 0 && gy >= 0 && gx < map_->size_x && gy < map_->size_y) {
      distance = map_->cells[MAP_INDEX(map_, static_cast<int>(gx), static_cast<int>(gy))].occ_dist;
    }
    distances.push_back(distance);
    if (settings.z_hit * std::exp(-distance * distance /
      (2.0 * settings.sigma_hit * settings.sigma_hit)) <= random) {++floor_count;}
  }
  if (distances.empty()) {return;}
  std::sort(distances.begin(), distances.end());
  auto quantile = [&](double p) {
    const double index = p * (distances.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(index));
    const auto hi = static_cast<size_t>(std::ceil(index));
    return distances[lo] + (index - lo) * (distances[hi] - distances[lo]);
  };
  stats_.distance_p50 = quantile(0.5); stats_.distance_p90 = quantile(0.9);
  stats_.distance_p99 = quantile(0.99);
  stats_.random_floor_fraction = floor_count / static_cast<double>(distances.size());
}
bool ObservationBackend::score(const ObservationSettings & settings, const pf_vector_t & laser_pose,
  LaserData * data, pf_sample_set_t * set, double & total) {
  data->validatePrepared();
  if (mode_ == "cpu" || disabled_ || (mode_ == "auto" && set->sample_count < min_particles_)) { return false; }
  const auto start=std::chrono::steady_clock::now();
  try {
    // Preserve upstream pose composition (including yaw normalization) on the host.
    ObservationSettings actual_settings = settings;
    actual_settings.endpoints_in_base_frame = data->endpoints != nullptr;
    poses_.resize(set->sample_count); factors_.resize(set->sample_count);
    for (int i=0;i<set->sample_count;++i) {
      const auto pose=data->endpoints ? set->samples[i].pose : pf_vector_coord_add(laser_pose, set->samples[i].pose);
      poses_[i]={pose.v[0],pose.v[1],pose.v[2]};
    }
    const int step=std::max(1, settings.probability_model ?
      static_cast<int>(std::ceil(data->range_count/static_cast<double>(settings.max_beams))) :
      (data->range_count-1)/(settings.max_beams-1));
    beams_.clear();
    for (int i=0;i<data->range_count;i+=step) {
      beams_.push_back({data->ranges[i][0],data->ranges[i][1],
        data->endpoints ? data->endpoints[i][0] : 0.0,
        data->endpoints ? data->endpoints[i][1] : 0.0, data->acceptsHit(i)});
    }
    // The upstream probabilistic beam mask has max_beams slots, even for short scans.
    const int selected=static_cast<int>(beams_.size());
    if (settings.probability_model) {
      beams_.resize(settings.max_beams, {data->range_max, 0.0});
    }
    stats_.particles=set->sample_count; stats_.beams=selected;
    std::vector<int> mask(beams_.size(), 1);
    if (!device_->score(actual_settings, poses_.data(), set->sample_count, beams_.data(),
      static_cast<int>(beams_.size()), data->range_max, set->converged, factors_.data(), mask.data())) {
      ++stats_.boundary_fallbacks; return false;
    }
    // Commit atomically after successful device completion; no partial mutation on fallback.
    total=0.0;
    if (settings.probability_model) {
      std::copy(factors_.begin(), factors_.end(), set->log_likelihoods);
      set->log_likelihoods_ready = 1;
      std::vector<int> effective_indices;
      for (int b=0; b<selected; ++b) {
        const auto & beam = beams_[b];
        if (mask[b] && beam.included) {effective_indices.push_back(b * step);}
      }
      set->sensor_no_information = effective_indices.empty();
      recordDiagnostics(settings, laser_pose, data, set, effective_indices, factors_.data());
      total = 1.0;
    } else {
      set->sensor_no_information = 1;
      for (int b=0; b<selected; ++b) {
        if (beams_[b].included) {set->sensor_no_information = 0; break;}
      }
      for (int i=0;i<set->sample_count;++i) {
        set->samples[i].weight *= factors_[i]; total += set->samples[i].weight;
      }
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
