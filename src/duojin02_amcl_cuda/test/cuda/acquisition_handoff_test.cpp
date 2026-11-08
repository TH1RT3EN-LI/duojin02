// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/amcl_node.hpp"
#include <cstdlib>
#include <stdexcept>

namespace
{
void check(bool value, const char * message) {if (!value) {throw std::runtime_error(message);}}
class MotionRecorder : public nav2_amcl::MotionModel
{
public:
  std::vector<pf_vector_t> deltas;
  void initialize(double, double, double, double, double) override {}
  void odometryUpdate(pf_t *, const pf_vector_t &, const pf_vector_t & delta) override
  {deltas.push_back(delta);}
};
class ControlledLaser : public nav2_amcl::LikelihoodFieldModelProb
{
public:
  explicit ControlledLaser(map_t * map)
  : LikelihoodFieldModelProb(.85, .15, .025, 2.0, false, .5, .3, .9, 120, map)
  {pf_vector_t origin = pf_vector_zero(); SetLaserPose(origin);}
  int mode{0}, calls{0};
  bool sensorUpdate(pf_t * pf, nav2_amcl::LaserData * data) override
  {
    ++calls;
    if (mode == 1) {return false;}
    if (mode == 2) {return true;}  // deliberately stale previous diagnostics
    return LikelihoodFieldModelProb::sensorUpdate(pf, data);
  }
};
class HandoffProbe : public nav2_amcl::AmclNode
{
public:
  explicit HandoffProbe(bool asynchronous, bool local = false) : AmclNode(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("global_acquisition_enabled", !local),
      rclcpp::Parameter("local_initialization_enabled", local),
      rclcpp::Parameter("local_initialization_xy_half_width", 2.0),
      rclcpp::Parameter("global_acquisition_async", asynchronous),
      rclcpp::Parameter("laser_model_type", "likelihood_field_prob"),
      rclcpp::Parameter("max_beams", 120),
      rclcpp::Parameter("sigma_hit", .025)})), asynchronous_(asynchronous), local_(local) {}

  void run()
  {
    initParameters(); initTransforms(); initPubSub();
    pose_pub_->on_activate(); particle_cloud_pub_->on_activate(); estimate_contract_pub_->on_activate();
    map_ = map_alloc(); map_->size_x = map_->size_y = 120; map_->scale = .05;
    map_->cells = static_cast<map_cell_t *>(std::calloc(120 * 120, sizeof(map_cell_t)));
    for (int y = 0; y < 120; ++y) {
      for (int x = 0; x < 120; ++x) {
        const bool occupied = x < 3 || y < 3 || x > 116 || y > 116 ||
          (x > 32 && x < 46 && y > 60 && y < 105) || (x > 80 && x < 108 && y > 25 && y < 38);
        map_->cells[MAP_INDEX(map_, x, y)].occ_state = occupied ? 1 : -1;
      }
    }
    createFreeSpaceVector();
    pf_ = pf_alloc(100, 300, 0, 0, uniformPoseGenerator);
    auto motion = std::make_shared<MotionRecorder>(); motion_model_ = motion;
    auto laser = std::make_unique<ControlledLaser>(map_); auto * control = laser.get();
    lasers_.push_back(std::move(laser)); lasers_update_.push_back(true); frame_to_laser_["laser"] = 0;
    scan_geometry_config_.enabled = true;
    active_ = true; first_map_received_ = true; tf_broadcast_ = true;
    latest_tf_valid_ = first_pose_sent_ = sent_first_transform_ = false;
    if (local_) {
      geometry_msgs::msg::PoseWithCovarianceStamped initial;
      initial.header.frame_id = "map";
      initial.header.stamp = now();
      initial.pose.pose.position.x = 1.85;
      initial.pose.pose.position.y = -.65;
      initial.pose.pose.orientation.z = std::sin(-2.7 / 2);
      initial.pose.pose.orientation.w = std::cos(-2.7 / 2);
      initial.pose.covariance[0] = initial.pose.covariance[7] = .25;
      initial.pose.covariance[35] = .25;
      handleInitialPose(initial);
      check(global_acquisition_pending_ && !initial_pose_is_known_,
        "user initial pose skipped bounded refinement");
    } else {
      globalLocalizationCallback(nullptr, nullptr, nullptr);
    }

    const int offset = asynchronous_ ? 1 : 0;
    auto emit = [&](int second, bool invalid = false) {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.frame_id = "laser"; scan->header.stamp.sec = second;
        scan->range_min = .01F; scan->range_max = 8;
        scan->angle_min = -M_PI; scan->angle_increment = 2 * M_PI / 120;
        duojin02_scan_geometry::TimedScan geometry;
        const double odom_x = second >= 4 + offset ? .25 :
          asynchronous_ && second == 4 ? .10 : 0;
        geometry.odom_from_base_reference = {odom_x, 0, 0};
        for (int i = 0; i < 120; ++i) {
          const double angle = -M_PI + 2 * M_PI * i / 120;
          const double range = map_calc_range(map_, .45, -.65, .73 + angle, 8);
          scan->ranges.push_back(invalid ? NAN : range);
          if (invalid) {
            geometry.beams.push_back(duojin02_scan_geometry::Beam::invalid(i,
              static_cast<int64_t>(second) * 1000000000, NAN,
              duojin02_scan_geometry::InvalidReason::NotANumber));
          } else {
            geometry.beams.push_back(duojin02_scan_geometry::Beam::hit(i,
              static_cast<int64_t>(second) * 1000000000, range, {0, 0},
              {range * std::cos(angle), range * std::sin(angle)}));
          }
        }
        processLaserScan(scan, &geometry, 0, 0);
      };
    auto unavailable = [&]() {
        check(!initial_pose_is_known_ && !latest_tf_valid_ && !first_pose_sent_ && !sent_first_transform_,
          "unconfirmed acquisition leaked a pose or TF");
        check(!estimate_contract_.valid_observation, "unconfirmed seed acquired observation provenance");
      };
    emit(1); unavailable(); global_acquisition_->wait();
    if (asynchronous_) {
      emit(1); unavailable();  // completed future cannot consume a duplicate scan
      check(global_acquisition_pending_, "duplicate completed an anchor confirmation");
    }
    emit(2); unavailable(); global_acquisition_->wait();
    emit(3); global_acquisition_->wait();
    if (asynchronous_) {unavailable(); emit(4);}
    check(!global_acquisition_pending_ && global_tracking_confirmation_pending_, "capture did not enter pending PF handoff");
    check(control->calls == 0, "capture reused anchor as a PF sensor observation");
    if (asynchronous_) {
      check(motion->deltas.size() == 1 && std::abs(motion->deltas[0].v[0] - .10) < 1e-12,
        "delayed anchor was not transported to its consumed scan exactly once");
    } else {check(motion->deltas.empty(), "synchronous capture consumed zero-length motion");}
    unavailable();

    const double original_threshold = d_thresh_;
    auto rejected = dynamicParametersCallback({rclcpp::Parameter("update_min_d", .99),
      rclcpp::Parameter("laser_model_type", "beam")});
    check(!rejected.successful && d_thresh_ == original_threshold && sensor_model_type_ == "likelihood_field_prob",
      "pending configuration rejection partially mutated live state");

    // The odometry reference exists, but raw laser->base TF preparation fails.
    scan_geometry_config_.enabled = false;
    emit(4 + offset);
    scan_geometry_config_.enabled = true;
    check(control->calls == 0 && motion->deltas.size() == static_cast<size_t>(1 + offset) &&
      std::abs(motion->deltas[offset].v[0] - (asynchronous_ ? .15 : .25)) < 1e-12 && pf_odom_pose_.v[0] == .25 && force_update_,
      "failed preparation did not consume motion exactly once and preserve retry");
    unavailable();
    pf_->diagnostics.status = PF_SENSOR_OK;  // stale status must not authorize a pose
    control->mode = 1; emit(5 + offset); unavailable();
    control->mode = 2; emit(6 + offset); unavailable();
    control->mode = 0; emit(7 + offset, true); unavailable();
    check(global_tracking_confirmation_pending_ && force_update_, "failed PF update disabled retry");
    emit(8 + offset);
    check(!global_tracking_confirmation_pending_ && initial_pose_is_known_ && first_pose_sent_ &&
      latest_tf_valid_ && sent_first_transform_ && estimate_contract_.valid_observation,
      "first valid fresh PF update did not authorize normal publication");
    check(estimate_contract_.last_effective_observation_stamp_ns == (8 + offset) * 1000000000LL,
      "invalid scan refreshed last valid observation");
    check(motion->deltas.size() == static_cast<size_t>(5 + offset), "static handoff retry failed to run");
    for (size_t i = 1 + offset; i < motion->deltas.size(); ++i) {
      check(std::abs(motion->deltas[i].v[0]) < 1e-12 && std::abs(motion->deltas[i].v[1]) < 1e-12 &&
        std::abs(motion->deltas[i].v[2]) < 1e-12, "handoff reapplied already consumed motion");
    }
    const auto sensor_calls = control->calls;
    emit(8 + offset);
    check(control->calls == sensor_calls, "first valid handoff scan was consumed twice");
    rejected = dynamicParametersCallback({rclcpp::Parameter("update_min_d", .99),
      rclcpp::Parameter("laser_model_type", "beam")});
    check(!rejected.successful && d_thresh_ == original_threshold,
      "post-handoff incompatible model rejection partially mutated state");
    active_ = false;
    on_cleanup(rclcpp_lifecycle::State{});
  }
private:
  bool asynchronous_;
  bool local_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  for (bool asynchronous : {false, true}) {
    auto node = std::make_shared<HandoffProbe>(asynchronous); node->run();
    auto local_node = std::make_shared<HandoffProbe>(asynchronous, true); local_node->run();
  }
  rclcpp::shutdown();
}
