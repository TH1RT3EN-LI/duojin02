// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/amcl_node.hpp"
#include <cstdlib>
#include <stdexcept>
namespace {
void check(bool value, const char * message) {if (!value) {throw std::runtime_error(message);}}
class MotionSpy : public nav2_amcl::MotionModel {
public:
  int calls{0};
  void initialize(double, double, double, double, double) override {}
  void odometryUpdate(pf_t *, const pf_vector_t &, const pf_vector_t &) override
  {++calls; (void)drand48();}
};
class LaserSpy : public nav2_amcl::Laser {
public:
  explicit LaserSpy(map_t * map) : Laser(120, map) {
    pf_vector_t pose = pf_vector_zero(); SetLaserPose(pose);
  }
  bool sensorUpdate(pf_t *, nav2_amcl::LaserData *) override
  {throw std::runtime_error("acquisition scan entered PF sensor likelihood");}
};
class ParameterProbe : public nav2_amcl::AmclNode {
public:
  ParameterProbe(bool enabled, const std::string & model, int beams = 120, double sigma = .025) : AmclNode(
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("global_acquisition_enabled", enabled),
      rclcpp::Parameter("laser_model_type", model), rclcpp::Parameter("max_beams", beams),
      rclcpp::Parameter("sigma_hit", sigma)})) {}
  void validate() {initParameters();}
  void checkInvalidDynamic() {
    const auto original = max_beams_;
    for (int count : {-1, 0, 1}) {
      const auto rejected = dynamicParametersCallback({rclcpp::Parameter("max_beams", count)});
      check(!rejected.successful && max_beams_ == original, "invalid max_beams mutated acquisition configuration");
    }
    const auto rejected = dynamicParametersCallback({rclcpp::Parameter("max_beams", original + 1),
      rclcpp::Parameter("sigma_hit", NAN)});
    check(!rejected.successful && max_beams_ == original, "invalid sigma rejection partially mutated configuration");
  }
};
class ResetProbe : public nav2_amcl::AmclNode {
public:
  explicit ResetProbe(bool enabled = false) : AmclNode(rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("global_acquisition_enabled", enabled)})), enabled_(enabled) {}
  void run() {
    map_ = map_alloc();
    map_->size_x = map_->size_y = 20;
    map_->scale = 0.05;
    map_->cells = static_cast<map_cell_t *>(std::calloc(400, sizeof(map_cell_t)));
    for (int i = 0; i < 400; ++i) {map_->cells[i].occ_state = -1;}
    createFreeSpaceVector();
    pf_ = pf_alloc(2, 16, 0, 0, uniformPoseGenerator);
    sigma_hit_ = .025;
    latest_tf_valid_ = first_pose_sent_ = sent_first_transform_ = true;
    estimate_contract_.recordAccepted("laser:10", 10, 12, 13, 14);
    fine_modes_valid_ = true;
    globalLocalizationCallback(nullptr, nullptr, nullptr);
    check(!estimate_contract_.valid_observation && estimate_contract_.source_id == "none" &&
      estimate_contract_.last_effective_observation_stamp_ns == 0 && !fine_modes_valid_,
      "global localization retained prior posterior provenance");
    if (enabled_) {
      check(global_acquisition_pending_ && global_acquisition_ && !latest_tf_valid_ &&
        !first_pose_sent_ && !sent_first_transform_ && !initial_pose_is_known_,
        "pending global acquisition exposed prior pose/TF as valid");
      auto motion = std::make_shared<MotionSpy>();
      motion_model_ = motion;
      lasers_.push_back(std::make_unique<LaserSpy>(map_));
      lasers_update_.push_back(true); frame_to_laser_["laser"] = 0;
      scan_geometry_config_.enabled = true;
      active_ = true; first_map_received_ = true; pf_init_ = true;
      pf_odom_pose_ = pf_vector_zero();
      d_thresh_ = a_thresh_ = .01; laser_min_range_ = 0; laser_max_range_ = 8; max_beams_ = 120;
      auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan->header.frame_id = "laser"; scan->header.stamp.sec = 1;
      scan->range_min = 0; scan->range_max = 8; scan->ranges.push_back(1.0F);
      duojin02_scan_geometry::TimedScan geometry;
      geometry.odom_from_base_reference = {1, 0, 0};
      geometry.beams.push_back(duojin02_scan_geometry::Beam::hit(0, 1000000000, 1, {0, 0}, {1, 0}));
      srand48(7721); const double expected_random = drand48(); srand48(7721);
      processLaserScan(scan, &geometry, 0, 0);
      check(motion->calls == 0 && drand48() == expected_random,
        "pending acquisition consumed motion model or PF random stream");
      check(global_acquisition_pending_ && !initial_pose_is_known_ && !latest_tf_valid_,
        "unsupported pending scan exposed a pose");
      active_ = false;
    } else {check(!global_acquisition_pending_, "compatibility path entered acquisition");}
    // Dynamic PF reinitialization and configure both enter initParticleFilter.
    pf_free(pf_); pf_ = nullptr;
    min_particles_ = 2; max_particles_ = 16;
    alpha_slow_ = alpha_fast_ = 0; pf_err_ = .05; pf_z_ = .99;
    for (int i=0; i<3; ++i) {init_pose_[i]=0; init_cov_[i]=.001;}
    estimate_contract_.recordAccepted("laser:20", 20, 22, 23, 24);
    fine_modes_valid_ = true;
    initParticleFilter();
    check(!estimate_contract_.valid_observation && !fine_modes_valid_,
      "PF recreation retained prior posterior provenance");
    check(!global_acquisition_pending_ && !global_acquisition_, "PF recreation retained acquisition");
    estimate_contract_.recordAccepted("laser:30", 30, 32, 33, 34);
    fine_modes_valid_ = true;
    if (enabled_) {globalLocalizationCallback(nullptr, nullptr, nullptr);}
    on_cleanup(rclcpp_lifecycle::State{});
    check(!estimate_contract_.valid_observation && !fine_modes_valid_,
      "cleanup retained prior posterior provenance");
    check(!global_acquisition_pending_ && !global_acquisition_, "cleanup retained acquisition");
  }
private:
  bool enabled_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  {ResetProbe node; node.run();}
  {ResetProbe node(true); node.run();}
  {ParameterProbe node(false, "beam"); node.validate();}
  {ParameterProbe node(true, "likelihood_field_prob"); node.validate(); node.checkInvalidDynamic();
    for (const char * name : {"global_acquisition_enabled", "global_acquisition_async", "global_acquisition_min_score",
      "global_acquisition_min_margin", "global_acquisition_max_evaluations"}) {
      check(node.describe_parameter(name).read_only, "global acquisition parameter is not read-only");
    }
  }
  {ParameterProbe node(true, "beam"); bool rejected = false;
    try {node.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "enabled acquisition accepted unprepared beam-model distance map");
  }
  {ParameterProbe node(true, "likelihood_field"); bool rejected = false;
    try {node.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "enabled acquisition accepted a different beam-selection contract");
  }
  for (int count : {-1, 0, 1}) {
    ParameterProbe node(true, "likelihood_field_prob", count); bool rejected = false;
    try {node.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "invalid max_beams was not rejected at startup");
  }
  {ParameterProbe node(true, "likelihood_field_prob", 120, NAN); bool rejected = false;
    try {node.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "invalid sigma_hit was not rejected at startup");
  }
  rclcpp::shutdown();
}
