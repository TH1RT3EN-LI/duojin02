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

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

LikelihoodFieldModel::LikelihoodFieldModel(
  double z_hit, double z_rand, double sigma_hit,
  double max_occ_dist, size_t max_beams, map_t * map)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  map_update_cspace(map, max_occ_dist);
}

double
LikelihoodFieldModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  LikelihoodFieldModel * self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t * sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = reinterpret_cast<LikelihoodFieldModel *>(data->laser);
  const auto observation_start = std::chrono::steady_clock::now();
  ObservationSettings settings;
  settings.probability_model = false;
  settings.z_hit = self->z_hit_; settings.z_rand = self->z_rand_;
  settings.sigma_hit = self->sigma_hit_; settings.max_beams = self->max_beams_;
  if (self->observation_ && self->observation_->score(settings, self->laser_pose_, data, set, total_weight)) {
    return total_weight;
  }


  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_mult = 1.0 / data->range_max;

  step = (data->range_count - 1) / (self->max_beams_ - 1);

  if (step < 1) {
    step = 1;
  }

  total_weight = 0.0;
  int valid_beams = 0;

  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    if (!data->endpoints) {pose = pf_vector_coord_add(self->laser_pose_, pose);}

    p = 1.0;

    for (i = 0; i < data->range_count; i += step) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      if (!data->acceptsHit(i)) {continue;}

      if (j == 0) {++valid_beams;}
      pz = 0.0;

      if (data->endpoints) {
        const double c = cos(pose.v[2]), sn = sin(pose.v[2]);
        hit.v[0] = pose.v[0] + c * data->endpoints[i][0] - sn * data->endpoints[i][1];
        hit.v[1] = pose.v[1] + sn * data->endpoints[i][0] + c * data->endpoints[i][1];
      } else {
        hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
        hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);
      }

      const double gx = MAP_GXWX(self->map_, hit.v[0]);
      const double gy = MAP_GYWY(self->map_, hit.v[1]);
      if (!std::isfinite(gx) || !std::isfinite(gy)) {p = NAN; continue;}
      // Out-of-map finite endpoints do not need conversion to a bounded integer.
      int mi = 0, mj = 0;
      const bool on_map = gx >= 0 && gy >= 0 && gx < self->map_->size_x && gy < self->map_->size_y;
      if (on_map) {mi = static_cast<int>(gx); mj = static_cast<int>(gy);}

      if (!on_map) {
        z = self->map_->max_occ_dist;
      } else {
        z = self->map_->cells[MAP_INDEX(self->map_, mi, mj)].occ_dist;
      }
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      pz += self->z_rand_ * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      // Preserve the legacy cubic beam mixture.
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  set->sensor_no_information = valid_beams == 0;
  if (self->observation_) {
    self->observation_->recordCpu(std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - observation_start).count(), set->sample_count,
      (data->range_count + step - 1) / step);
  }
  return total_weight;
}


bool
LikelihoodFieldModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  data->validatePrepared();
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
