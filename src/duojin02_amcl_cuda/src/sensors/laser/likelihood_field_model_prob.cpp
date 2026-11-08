/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include <math.h>
#include <assert.h>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <limits>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

LikelihoodFieldModelProb::LikelihoodFieldModelProb(
  double z_hit, double z_rand, double sigma_hit,
  double max_occ_dist, bool do_beamskip,
  double beam_skip_distance,
  double beam_skip_threshold,
  double beam_skip_error_threshold,
  size_t max_beams, map_t * map)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  do_beamskip_ = do_beamskip;
  beam_skip_distance_ = beam_skip_distance;
  beam_skip_threshold_ = beam_skip_threshold;
  beam_skip_error_threshold_ = beam_skip_error_threshold;
  map_update_cspace(map, max_occ_dist);
}

// Determine the probability for the given pose
double
LikelihoodFieldModelProb::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  auto * self = reinterpret_cast<LikelihoodFieldModelProb *>(data->laser);
  const auto observation_start = std::chrono::steady_clock::now();
  ObservationSettings settings;
  settings.probability_model = true;
  settings.z_hit = self->z_hit_; settings.z_rand = self->z_rand_;
  settings.sigma_hit = self->sigma_hit_; settings.max_beams = self->max_beams_;
  settings.likelihood_temperature_beta = self->likelihood_temperature_beta_;
  settings.beam_skip = self->do_beamskip_;
  settings.beam_skip_distance = self->beam_skip_distance_;
  settings.beam_skip_threshold = self->beam_skip_threshold_;
  settings.beam_skip_error_threshold = self->beam_skip_error_threshold_;
  double ignored_total = 0.0;
  if (self->observation_ && self->observation_->score(
      settings, self->laser_pose_, data, set, ignored_total)) {return ignored_total;}

  const int step = std::max(1, static_cast<int>(std::ceil(
      data->range_count / static_cast<double>(self->max_beams_))));
  const bool skipping = self->do_beamskip_ && set->converged;
  const double denom = 2.0 * self->sigma_hit_ * self->sigma_hit_;
  const double random = self->z_rand_ * (1.0 / data->range_max);
  std::vector<int> agreement(self->max_beams_, 0), mask(self->max_beams_, 1);
  std::vector<double> probabilities;
  if (skipping) {probabilities.assign(static_cast<size_t>(set->sample_count) * self->max_beams_, 1.0);}
  std::vector<int> valid_indices;
  for (int i = 0; i < data->range_count; i += step) {
    if (data->acceptsHit(i)) {valid_indices.push_back(i);}
  }
  // No callback writes prior weights. pf_update_sensor combines log prior and
  // log likelihood, validates the entire update, then commits normalization.
  for (int j = 0; j < set->sample_count; ++j) {
    pf_vector_t pose = data->endpoints ? set->samples[j].pose :
      pf_vector_coord_add(self->laser_pose_, set->samples[j].pose);
    double log_score = 0.0;
    for (int i : valid_indices) {
      double x, y;
      if (data->endpoints) {
        const double c = cos(pose.v[2]), sn = sin(pose.v[2]);
        x = pose.v[0] + c * data->endpoints[i][0] - sn * data->endpoints[i][1];
        y = pose.v[1] + sn * data->endpoints[i][0] + c * data->endpoints[i][1];
      } else {
        x = pose.v[0] + data->ranges[i][0] * cos(pose.v[2] + data->ranges[i][1]);
        y = pose.v[1] + data->ranges[i][0] * sin(pose.v[2] + data->ranges[i][1]);
      }
      double distance = self->map_->max_occ_dist;
      // Guard conversion to integer map coordinates for invalid geometry.
      const double gx = floor((x - self->map_->origin_x) / self->map_->scale + 0.5) + self->map_->size_x / 2;
      const double gy = floor((y - self->map_->origin_y) / self->map_->scale + 0.5) + self->map_->size_y / 2;
      if (!std::isfinite(gx) || !std::isfinite(gy)) {log_score = NAN; continue;}
      if (gx >= 0 && gy >= 0 && gx < self->map_->size_x && gy < self->map_->size_y) {
        distance = self->map_->cells[MAP_INDEX(self->map_, static_cast<int>(gx), static_cast<int>(gy))].occ_dist;
        if (distance < self->beam_skip_distance_) {++agreement[i / step];}
      }
      const double probability = self->z_hit_ * exp(-(distance * distance) / denom) + random;
      if (skipping) {probabilities[static_cast<size_t>(j) * self->max_beams_ + i / step] = probability;}
      else {log_score += log(probability);}
    }
    set->log_likelihoods[j] = log_score;
  }
  if (skipping) {
    int skipped = 0;
    for (int b = 0; b < self->max_beams_; ++b) {
      mask[b] = agreement[b] / static_cast<double>(set->sample_count) > self->beam_skip_threshold_;
      if (!mask[b]) {++skipped;}
    }
    // Preserve legacy padded-mask and error-recovery semantics.
    if (skipped >= self->max_beams_ * self->beam_skip_error_threshold_) {
      std::fill(mask.begin(), mask.end(), 1);
    }
    for (int j = 0; j < set->sample_count; ++j) {
      double log_score = set->log_likelihoods[j];
      for (int b = 0; b < self->max_beams_; ++b) {
        if (mask[b]) {log_score += log(probabilities[static_cast<size_t>(j) * self->max_beams_ + b]);}
      }
      set->log_likelihoods[j] = log_score;
    }
  }
  std::vector<int> effective_indices;
  for (int i : valid_indices) {if (mask[i / step]) {effective_indices.push_back(i);}}
  for (int j = 0; j < set->sample_count; ++j) {
    set->log_likelihoods[j] *= self->likelihood_temperature_beta_;
  }
  set->log_likelihoods_ready = 1;
  set->sensor_no_information = effective_indices.empty();
  if (self->observation_) {
    self->observation_->recordDiagnostics(settings, self->laser_pose_, data, set,
      effective_indices, set->log_likelihoods);
    self->observation_->recordCpu(std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - observation_start).count(), set->sample_count,
      (data->range_count + step - 1) / step);
  }
  return 1.0;
}

double
LikelihoodFieldModelProb::scorePoseOnLastSupport(const LaserData & data, const pf_vector_t & input) const
{
  if (!observation_ || observation_->stats().effective_source_indices == "none") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto pose = data.endpoints ? input : pf_vector_coord_add(laser_pose_, input);
  std::istringstream indices(observation_->stats().effective_source_indices);
  double value = 0.0;
  int i;
  while (indices >> i) {
    if (i < 0 || i >= data.range_count || !data.acceptsHit(i)) {return NAN;}
    double x, y;
    if (data.endpoints) {
      const double c = cos(pose.v[2]), sn = sin(pose.v[2]);
      x = pose.v[0] + c * data.endpoints[i][0] - sn * data.endpoints[i][1];
      y = pose.v[1] + sn * data.endpoints[i][0] + c * data.endpoints[i][1];
    } else {
      x = pose.v[0] + data.ranges[i][0] * cos(pose.v[2] + data.ranges[i][1]);
      y = pose.v[1] + data.ranges[i][0] * sin(pose.v[2] + data.ranges[i][1]);
    }
    const double gx = MAP_GXWX(map_, x), gy = MAP_GYWY(map_, y);
    if (!std::isfinite(gx) || !std::isfinite(gy)) {return NAN;}
    double d = map_->max_occ_dist;
    if (gx >= 0 && gy >= 0 && gx < map_->size_x && gy < map_->size_y) {
      d = map_->cells[MAP_INDEX(map_, static_cast<int>(gx), static_cast<int>(gy))].occ_dist;
    }
    value += log(z_hit_ * exp(-d*d/(2.0*sigma_hit_*sigma_hit_)) + z_rand_*(1.0/data.range_max));
    if (indices.peek() == ',') {indices.get();}
  }
  return likelihood_temperature_beta_ * value;
}

bool
LikelihoodFieldModelProb::sensorUpdate(pf_t * pf, LaserData * data)
{
  data->validatePrepared();
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
