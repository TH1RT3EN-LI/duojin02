// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__ACQUISITION_EXECUTOR_HPP_
#define NAV2_AMCL__ACQUISITION_EXECUTOR_HPP_

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "nav2_amcl/global_acquisition.hpp"

namespace nav2_amcl
{
struct AcquisitionInput
{
  std::vector<AcquisitionPoint> points;
  AcquisitionPose odom;
  int64_t raw_stamp_ns{}, reference_stamp_ns{};
  std::string frame, selected_source_indices;
};

struct AcquisitionMotionSample
{
  AcquisitionPose odom;
  int64_t reference_stamp_ns{};
};

struct AcquisitionDispatch
{
  AcquisitionResult search;
  AcquisitionInput anchor;
  std::vector<AcquisitionMotionSample> motion_history;
  bool completed_search{false}, completed_certificate{false}, in_flight{false};
  double search_ms{};
};

// One owned worker and one immutable map per explicit acquisition request.
// Only completed anchor searches enter GlobalAcquisition::observe; arriving
// scans during a search are acknowledgements, never repeated confirmations.
// Destruction joins bounded solver work before discarding its request state.
class AcquisitionExecutor
{
public:
  static constexpr size_t kMaxMotionHistorySamples = 4096;

  AcquisitionExecutor(AcquisitionConfig config, bool asynchronous)
  : config_(config), solver_(std::make_shared<GlobalAcquisition>(config)),
    asynchronous_(asynchronous) {}

  // The node records every supported odometry sample before laser preparation,
  // including samples whose later laser preparation fails. Never throw from
  // this observation hook or silently truncate an already started history.
  void recordOdometry(AcquisitionMotionSample sample) noexcept
  {
    if (!future_.valid() || sample.reference_stamp_ns <= last_motion_stamp_) {return;}
    last_motion_stamp_ = sample.reference_stamp_ns;
    if (!motion_history_valid_) {return;}
    if (!std::isfinite(sample.odom.x) || !std::isfinite(sample.odom.y) ||
      !std::isfinite(sample.odom.yaw) ||
      motion_history_.size() >= kMaxMotionHistorySamples)
    {
      motion_history_valid_ = false;
      return;
    }
    try {
      motion_history_.push_back(sample);
    } catch (...) {
      motion_history_valid_ = false;
    }
  }

  AcquisitionDispatch advance(const map_t & map, AcquisitionInput input)
  {
    try {
      if (!solver_) {solver_ = std::make_shared<GlobalAcquisition>(config_);}
      return advanceImpl(map, std::move(input));
    } catch (...) {
      // A failed job/launch/allocation must not retain previous confirmations.
      // No worker accesses node state; join any owned work before discarding it.
      future_ = {};
      solver_.reset();
      map_.reset();
      resetMotionHistory();
      AcquisitionDispatch failure;
      failure.search.status = "worker_execution_failed";
      return failure;
    }
  }

  // Deterministic test/lifecycle barrier. Normal scan callbacks never wait.
  void wait() const {if (future_.valid()) {future_.wait();}}

private:
  AcquisitionDispatch advanceImpl(const map_t & map, AcquisitionInput input)
  {
    AcquisitionDispatch dispatch;
    if (input.reference_stamp_ns <= last_input_stamp_ ||
      (future_.valid() && input.reference_stamp_ns < last_motion_stamp_))
    {
      dispatch.search.status = "duplicate_or_nonmonotonic_source";
      dispatch.in_flight = future_.valid();
      return dispatch;
    }
    last_input_stamp_ = input.reference_stamp_ns;
    recordOdometry({input.odom, input.reference_stamp_ns});
    if (future_.valid() && (!motion_history_valid_ ||
      !historyEndsAt(input)))
    {
      // A missing, inconsistent, or truncated history cannot be replaced by
      // one anchor-to-current delta: alpha*d^2 would be a different model.
      // Joining also ensures the old worker cannot preserve confirmations.
      future_ = {};
      solver_.reset();
      resetMotionHistory();
      dispatch.search.status = "odometry_history_unavailable";
      return dispatch;
    }
    if (!map_) {
      // This is an explicit resource boundary, not a partial-map search.
      constexpr size_t max_cells = 16 * 1024 * 1024;
      if (!map.cells || map.size_x <= 0 || map.size_y <= 0 ||
        static_cast<size_t>(map.size_x) > max_cells / static_cast<size_t>(map.size_y))
      {
        dispatch.search.status = "map_snapshot_budget_exceeded";
        return dispatch;
      }
      map_ = std::make_shared<OwnedMap>(map);
    }
    if (!asynchronous_) {
      return run(solver_, map_, std::move(input));
    }
    if (future_.valid() &&
      future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      dispatch = future_.get();
      if (dispatch.anchor.reference_stamp_ns != motion_anchor_stamp_) {
        solver_.reset();
        resetMotionHistory();
        dispatch = {};
        dispatch.search.status = "odometry_history_unavailable";
        return dispatch;
      }
      dispatch.motion_history = std::move(motion_history_);
      resetMotionHistory();
      if (dispatch.search.acquired) {return dispatch;}
    } else {
      dispatch.search.status = "search_running";
    }
    if (!future_.valid()) {
      resetMotionHistory(input.reference_stamp_ns);
      future_ = std::async(std::launch::async, &AcquisitionExecutor::run,
        solver_, map_, std::move(input));
    }
    dispatch.in_flight = true;
    return dispatch;
  }

  bool historyEndsAt(const AcquisitionInput & input) const noexcept
  {
    if (motion_history_.empty()) {return false;}
    const auto & last = motion_history_.back();
    return motion_history_.front().reference_stamp_ns > motion_anchor_stamp_ &&
           last.reference_stamp_ns == input.reference_stamp_ns &&
           last.odom.x == input.odom.x && last.odom.y == input.odom.y &&
           last.odom.yaw == input.odom.yaw;
  }

  void resetMotionHistory(
    int64_t anchor_stamp = std::numeric_limits<int64_t>::min()) noexcept
  {
    motion_history_.clear();
    motion_history_valid_ = true;
    motion_anchor_stamp_ = last_motion_stamp_ = anchor_stamp;
  }

  struct OwnedMap
  {
    explicit OwnedMap(const map_t & source)
    : value(source), cells(source.cells,
        source.cells + static_cast<size_t>(source.size_x) * source.size_y)
    {value.cells = cells.data();}
    map_t value;
    std::vector<map_cell_t> cells;
  };

  static AcquisitionDispatch run(std::shared_ptr<GlobalAcquisition> solver,
    std::shared_ptr<const OwnedMap> map, AcquisitionInput input)
  {
    AcquisitionDispatch dispatch;
    const auto start = std::chrono::steady_clock::now();
    dispatch.search = solver->observe(map->value, input.points, input.odom,
      input.reference_stamp_ns);
    dispatch.search_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    dispatch.anchor = std::move(input);
    dispatch.completed_search = true;
    dispatch.completed_certificate = dispatch.search.certificate_valid;
    return dispatch;
  }

  // future is destroyed first: the worker never references an AmclNode, a live
  // map buffer, or a subsequent acquisition request.
  AcquisitionConfig config_;
  std::shared_ptr<GlobalAcquisition> solver_;
  bool asynchronous_;
  std::shared_ptr<const OwnedMap> map_;
  int64_t last_input_stamp_{std::numeric_limits<int64_t>::min()};
  std::vector<AcquisitionMotionSample> motion_history_;
  bool motion_history_valid_{true};
  int64_t motion_anchor_stamp_{std::numeric_limits<int64_t>::min()};
  int64_t last_motion_stamp_{std::numeric_limits<int64_t>::min()};
  std::future<AcquisitionDispatch> future_;
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__ACQUISITION_EXECUTOR_HPP_
