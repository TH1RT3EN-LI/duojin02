// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


#ifndef NAV2_AMCL__SENSORS__LASER__LASER_HPP_
#define NAV2_AMCL__SENSORS__LASER__LASER_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <stdexcept>
#include "nav2_amcl/sensors/laser/observation_backend.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/map/map.hpp"

namespace nav2_amcl
{

// Forward declarations
class LaserData;

/*
 * @class Laser
 * @brief Base class for laser sensor models
 */
class Laser
{
public:
  /**
   * @brief A Laser constructor
   * @param max_beams number of beams to use
   * @param map Map pointer to use
   */
  Laser(size_t max_beams, map_t * map);

  /*
   * @brief Laser destructor
   */
  virtual ~Laser();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual bool sensorUpdate(pf_t * pf, LaserData * data) = 0;

  /*
   * @brief Set the laser pose from an update
   * @param laser_pose Pose of the laser
   */
  void SetLaserPose(pf_vector_t & laser_pose);
  const pf_vector_t & laserPose() const {return laser_pose_;}
  void setLikelihoodTemperatureBeta(double beta);
  void configureObservationBackend(const std::string & mode, int min_particles, int log_every);
  const ObservationStats * observationStats() const { return observation_ ? &observation_->stats() : nullptr; }


protected:
  std::unique_ptr<ObservationBackend> observation_;
  double z_hit_;
  double z_rand_;
  double sigma_hit_;
  double likelihood_temperature_beta_{1.0};

  /*
   * @brief Reallocate weights
   * @param max_samples Max number of samples
   * @param max_obs number of observations
   */
  void reallocTempData(int max_samples, int max_obs);
  map_t * map_;
  pf_vector_t laser_pose_;
  int max_beams_;
  int max_samples_;
  int max_obs_;
  double ** temp_obs_;
};

/*
 * @class LaserData
 * @brief Class of laser data to process
 */
class LaserData
{
public:
  enum class SelectionReason : uint8_t {Included = 0, SensorInvalid = 1, NoReturn = 2,
    BelowAlgorithmMinimum = 3, AtOrAboveAlgorithmMaximum = 4};
  Laser * laser;

  /*
   * @brief LaserData constructor
   */
  LaserData() : laser(nullptr), range_count(0), range_max(0.0), ranges(nullptr), endpoints(nullptr) {}
  /*
   * @brief LaserData destructor
   */
  virtual ~LaserData() {delete[] ranges; delete[] endpoints;}

public:
  int range_count;
  double range_max;
  double(*ranges)[2];
  // Optional checked endpoints in base_ref. measurement_states/selection_reasons
  // are authoritative whenever this is a prepared scan.
  double(*endpoints)[2];
  // Prepared-measurement state is authoritative. Raw ranges remain audit data;
  // algorithm gates are a separate selection, preserving original beam indices.
  std::vector<uint8_t> measurement_states;
  std::vector<SelectionReason> selection_reasons;
  void validatePrepared() const
  {
    if (!endpoints) {
      if (!measurement_states.empty() || !selection_reasons.empty()) {
        throw std::invalid_argument("prepared measurement metadata has no endpoint geometry");
      }
      return;
    }
    if (range_count < 0 || measurement_states.size() != static_cast<size_t>(range_count) ||
      selection_reasons.size() != static_cast<size_t>(range_count) || (range_count && !ranges))
    {throw std::invalid_argument("prepared endpoints require complete beam states and selection reasons");}
    for (int i=0; i<range_count; ++i) {
      const auto state=measurement_states[i]; const auto reason=selection_reasons[i];
      if (state > 2 || static_cast<uint8_t>(reason) > 4 ||
        (state == 0 && reason != SelectionReason::SensorInvalid) ||
        (state == 2 && reason != SelectionReason::NoReturn) ||
        (state == 1 && (reason == SelectionReason::SensorInvalid || reason == SelectionReason::NoReturn)) ||
        !std::isfinite(endpoints[i][0]) || !std::isfinite(endpoints[i][1]))
      {throw std::invalid_argument("inconsistent prepared measurement validity or non-finite geometry");}
    }
  }
  bool acceptsHit(int index) const
  {
    if (endpoints) {
      if (measurement_states.size() != static_cast<size_t>(range_count) ||
        selection_reasons.size() != static_cast<size_t>(range_count))
      {throw std::invalid_argument("prepared endpoints require complete beam states and selection reasons");}
      if (measurement_states.at(index) != 1) {return false;}
      return selection_reasons.at(index) == SelectionReason::Included;
    }
    return std::isfinite(ranges[index][0]) && std::isfinite(ranges[index][1]) &&
      ranges[index][0] < range_max;
  }
};

/*
 * @class BeamModel
 * @brief Beam model laser sensor
 */
class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  double z_short_;
  double z_max_;
  double lambda_short_;
  double chi_outlier_;
};

/*
 * @class LikelihoodFieldModel
 * @brief likelihood field model laser sensor
 */
class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModel(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
};

/*
 * @class LikelihoodFieldModelProb
 * @brief likelihood prob model laser sensor
 */
class LikelihoodFieldModelProb : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModelProb(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    bool do_beamskip, double beam_skip_distance,
    double beam_skip_threshold, double beam_skip_error_threshold,
    size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);
  // Read-only same-support diagnostic; never commits a second observation.
  double scorePoseOnLastSupport(const LaserData & data, const pf_vector_t & pose) const;

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__SENSORS__LASER__LASER_HPP_
