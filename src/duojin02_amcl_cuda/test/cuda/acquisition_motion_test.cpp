// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/amcl_node.hpp"
#include "nav2_amcl/motion_model/omni_motion_model.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

pf_vector_t vector(const nav2_amcl::AcquisitionPose & pose)
{
  return pf_vector_t{{pose.x, pose.y, pose.yaw}};
}

struct MotionCall
{
  pf_vector_t pose, delta;
};

class MotionRecorder : public nav2_amcl::MotionModel
{
public:
  pf_t * expected_filter{};
  std::vector<MotionCall> calls;
  void initialize(double, double, double, double, double) override {}
  void odometryUpdate(
    pf_t * filter, const pf_vector_t & pose, const pf_vector_t & delta) override
  {
    check(filter == expected_filter, "handoff used the wrong particle filter");
    calls.push_back({pose, delta});
  }
};

struct Case
{
  std::string name;
  nav2_amcl::AcquisitionDispatch dispatch;
  std::vector<MotionCall> expected;
};

Case makeCase(
  std::string name, nav2_amcl::AcquisitionPose anchor,
  std::vector<nav2_amcl::AcquisitionPose> poses, std::vector<MotionCall> expected)
{
  Case value;
  value.name = std::move(name);
  value.dispatch.anchor.odom = anchor;
  value.dispatch.anchor.reference_stamp_ns = 1000000000;
  int64_t stamp = value.dispatch.anchor.reference_stamp_ns;
  for (const auto & pose : poses) {
    stamp += 50000000;
    value.dispatch.motion_history.push_back({pose, stamp});
  }
  value.expected = std::move(expected);
  return value;
}

std::vector<Case> cases()
{
  // Binary fractions make the strict > translation threshold case exact.
  // The expected calls are independently specified, not computed by the gate.
  return {
    makeCase("accumulated_small_steps_and_tail", {2, -3, 0},
      {{2.03125, -3, 0}, {2.0625, -3, 0}, {2.09375, -3, 0},
        {2.125, -3, 0}, {2.15625, -3, 0}, {2.1875, -3, 0}},
      {{{{2.15625, -3, 0}}, {{.15625, 0, 0}}},
        {{{2.1875, -3, 0}}, {{.03125, 0, 0}}}}),
    makeCase("pure_rotation_and_tail", {2, -3, 0},
      {{2, -3, .03125}, {2, -3, .0625}, {2, -3, .09375},
        {2, -3, .15625}, {2, -3, .1875}},
      {{{{2, -3, .15625}}, {{0, 0, .15625}}},
        {{{2, -3, .1875}}, {{0, 0, .03125}}}}),
    makeCase("yaw_crosses_positive_pi", {0, 0, M_PI - .0625},
      {{0, 0, -M_PI + .03125}, {0, 0, -M_PI + .09375},
        {0, 0, -M_PI + .109375}},
      {{{{0, 0, -M_PI + .09375}}, {{0, 0, .15625}}},
        {{{0, 0, -M_PI + .109375}}, {{0, 0, .015625}}}}),
    makeCase("yaw_crosses_negative_pi", {0, 0, -M_PI + .0625},
      {{0, 0, M_PI - .03125}, {0, 0, M_PI - .09375},
        {0, 0, M_PI - .109375}},
      {{{{0, 0, M_PI - .09375}}, {{0, 0, -.15625}}},
        {{{0, 0, M_PI - .109375}}, {{0, 0, -.015625}}}}),
    makeCase("forward_reverse_return_and_tail", {0, 0, 0},
      {{.25, 0, 0}, {0, 0, 0}, {-.25, 0, 0}, {0, 0, 0}, {.015625, 0, 0}},
      {{{{.25, 0, 0}}, {{.25, 0, 0}}}, {{{0, 0, 0}}, {{-.25, 0, 0}}},
        {{{-.25, 0, 0}}, {{-.25, 0, 0}}}, {{{0, 0, 0}}, {{.25, 0, 0}}},
        {{{.015625, 0, 0}}, {{.015625, 0, 0}}}}),
    makeCase("component_gate_not_euclidean_distance", {0, 0, 0},
      {{.09375, .09375, 0}, {.125, .125, 0}, {.15625, .09375, 0},
        {.1875, .125, 0}},
      {{{{.15625, .09375, 0}}, {{.15625, .09375, 0}}},
        {{{.1875, .125, 0}}, {{.03125, .03125, 0}}}}),
    makeCase("combined_motion", {1, 2, .25},
      {{1.25, 2.0625, .5}, {1.3125, 2.125, .5625},
        {1.5, 2.25, .75}, {1.515625, 2.265625, .765625}},
      {{{{1.25, 2.0625, .5}}, {{.25, .0625, .25}}},
        {{{1.5, 2.25, .75}}, {{.25, .1875, .25}}},
        {{{1.515625, 2.265625, .765625}}, {{.015625, .015625, .015625}}}}),
    makeCase("zero_final_tail", {0, 0, 0},
      {{.25, 0, 0}, {.25, 0, 0}},
      {{{{.25, 0, 0}}, {{.25, 0, 0}}}}),
    makeCase("stationary_history", {1, 2, .5},
      {{1, 2, .5}, {1, 2, .5}}, {}),
    makeCase("synchronous_empty_history", {1, 2, .5}, {}, {})
  };
}

class MotionProbe : public nav2_amcl::AmclNode
{
public:
  MotionProbe() : AmclNode(rclcpp::NodeOptions())
  {
    d_thresh_ = a_thresh_ = .125;
    pf_ = pf_alloc(50, 512, 0, 0, uniformPoseGenerator);
  }

  ~MotionProbe() override
  {
    pf_free(pf_);
    pf_ = nullptr;
  }

  void verifyCalls(const Case & value)
  {
    auto recorder = std::make_shared<MotionRecorder>();
    recorder->expected_filter = pf_;
    motion_model_ = recorder;
    // The delayed path must use its anchor, not these live node fields.
    pf_odom_pose_ = {{71, -91, 2.25}};
    force_update_ = true;
    applyAcquisitionMotion(value.dispatch);
    check(recorder->calls.size() == value.expected.size(),
      value.name + ": unexpected motion call count");
    for (size_t i = 0; i < value.expected.size(); ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        check(std::abs(recorder->calls[i].pose.v[axis] -
          value.expected[i].pose.v[axis]) < 1e-12, value.name + ": incorrect current pose");
        check(std::abs(recorder->calls[i].delta.v[axis] -
          value.expected[i].delta.v[axis]) < 1e-12, value.name + ": incorrect motion delta");
      }
    }
    check(pf_odom_pose_.v[0] == 71 && pf_odom_pose_.v[1] == -91 &&
      pf_odom_pose_.v[2] == 2.25 && force_update_,
      value.name + ": delayed motion altered the caller's live reference");
  }

  void verifyParticles(const Case & value, long seed)
  {
    auto motion = std::make_shared<nav2_amcl::OmniMotionModel>();
    motion->initialize(.1, .12, .08, .11, .09);
    motion_model_ = motion;
    resetParticles();
    srand48(seed);
    onlinePrediction(value.dispatch);
    const auto & online = pf_->sets[pf_->current_set];
    std::vector<pf_sample_t> expected(online.samples, online.samples + online.sample_count);
    const double expected_next_random = drand48();

    resetParticles();
    pf_odom_pose_ = {{71, -91, 2.25}};
    force_update_ = true;
    srand48(seed);
    applyAcquisitionMotion(value.dispatch);
    const double actual_next_random = drand48();
    const auto & delayed = pf_->sets[pf_->current_set];
    check(delayed.sample_count == static_cast<int>(expected.size()),
      value.name + ": motion changed particle count");
    for (int i = 0; i < delayed.sample_count; ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        check(delayed.samples[i].pose.v[axis] == expected[i].pose.v[axis],
          value.name + ": delayed/online particle pose differs");
      }
      check(delayed.samples[i].weight == expected[i].weight &&
        delayed.samples[i].lineage == expected[i].lineage,
        value.name + ": motion changed weights or ancestry");
    }
    check(actual_next_random == expected_next_random,
      value.name + ": different number/order of random motion draws");
  }

private:
  void resetParticles()
  {
    pf_->current_set = 0;
    auto & set = pf_->sets[0];
    set.sample_count = pf_->max_samples;
    for (int i = 0; i < set.sample_count; ++i) {
      set.samples[i].pose = {{-8 + (i % 17 - 8) * .003,
          .25 + (i % 23 - 11) * .002, -.8 + (i % 29 - 14) * .001}};
      set.samples[i].weight = 1.0 / set.sample_count;
      set.samples[i].lineage = i + 1;
    }
  }

  void onlinePrediction(const nav2_amcl::AcquisitionDispatch & dispatch)
  {
    // Use the existing online gate itself as the independent compatibility
    // oracle. The final nonzero residual is an explicit handoff extension.
    pf_odom_pose_ = vector(dispatch.anchor.odom);
    force_update_ = false;
    for (size_t i = 0; i < dispatch.motion_history.size(); ++i) {
      const auto current = vector(dispatch.motion_history[i].odom);
      pf_vector_t delta = pf_vector_zero();
      const bool selected = shouldUpdateFilter(current, delta);
      if (selected || i + 1 == dispatch.motion_history.size()) {
        if (delta.v[0] != 0 || delta.v[1] != 0 || delta.v[2] != 0) {
          motion_model_->odometryUpdate(pf_, current, delta);
          pf_odom_pose_ = current;
        }
      }
    }
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<MotionProbe>();
    for (const auto & value : cases()) {
      node->verifyCalls(value);
      for (long seed : {1L, 271828L, 918273L}) {node->verifyParticles(value, seed);}
    }
  }
  rclcpp::shutdown();
  std::cout << "acquisition motion: 10 exact call fixtures and 30 particle-exact comparisons passed\n";
}
