#include "duojin02_navigation/planner_checks.hpp"
#include "duojin02_navigation/finite_heading.hpp"
#include "duojin02_navigation/path_cleanup.hpp"
#include <nav2_smac_planner/smac_planner_lattice.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>

namespace duojin02_navigation {

class PreciseLattice : public nav2_smac_planner::SmacPlannerLattice {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    auto node = parent.lock();
    const auto cache_name = name + ".cache_obstacle_heuristic";
    if (!node->has_parameter(cache_name))
      node->declare_parameter(cache_name, false);
    if (node->get_parameter(cache_name).as_bool())
      throw nav2_core::PlannerException(
          "PreciseLattice requires cache_obstacle_heuristic=false; unchanged maps use its validated path cache");
    SmacPlannerLattice::configure(parent, name, tf, costmap);
    cached_ = nav_msgs::msg::Path();
    cached_map_.clear();
    parameter_epoch_ = 0;
    cache_parameter_handler_ = node->add_on_set_parameters_callback(
        [this, prefix = name + "."](const std::vector<rclcpp::Parameter> &parameters) {
          if (unsafeCacheRequested(parameters))
            return rejectUnsafeCache();
          for (const auto &p : parameters)
            if (p.get_name().rfind(prefix, 0) == 0) {
              ++parameter_epoch_;
              break;
            }
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          return result;
        });
  }

  void activate() override {
    _raw_plan_publisher->on_activate();
    auto node = _node.lock();

    _dyn_params_handler = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          if (unsafeCacheRequested(parameters))
            return rejectUnsafeCache();

          std::lock_guard<std::mutex> plan_lock(plan_mutex_);
          for (const auto &p : parameters)
            if (p.get_name().rfind(_name + ".", 0) == 0) {
              ++parameter_epoch_;
              break;
            }
          return SmacPlannerLattice::dynamicParametersCallback(parameters);
        });
  }

  void cleanup() override {
    _dyn_params_handler.reset();
    cache_parameter_handler_.reset();
    cached_ = nav_msgs::msg::Path();
    cached_map_.clear();
    SmacPlannerLattice::cleanup();
  }

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal) override {
    const auto plan_began = SteadyHeadingBudget::Clock::now();
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);

    std::unique_lock<std::mutex> settings_lock(_mutex);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    const SteadyHeadingBudget budget(plan_began, _max_planning_time);
    auto require_time = [&]() {
      if (!budget.available())
        throw nav2_core::PlannerException("PreciseLattice total planning budget exhausted");
    };
    require_time();
    Polygon footprint;
    double resolution = 0, radius = 0;
    MapBounds bounds{};
    std::vector<std::vector<Point>> lethal_rows, nonzero_rows;
    std::vector<std::vector<SoftCostPoint>> soft_rows;
    auto index_map = [&]() {
      require_time();
      footprint.clear();
      radius = 0;
      for (const auto &p : _costmap_ros->getRobotFootprint()) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
          throw nav2_core::PlannerException("Invalid filled footprint coordinate");
        footprint.push_back({p.x, p.y});
        radius = std::max(radius, std::hypot(p.x, p.y));
      }
      resolution = _costmap->getResolution();
      bounds = {_costmap->getOriginX(), _costmap->getOriginY(),
                _costmap->getOriginX() + _costmap->getSizeInCellsX() * resolution,
                _costmap->getOriginY() + _costmap->getSizeInCellsY() * resolution};
      if (footprint.size() < 3 || !std::isfinite(radius) || !std::isfinite(resolution) ||
          !std::isfinite(bounds.min_x) || !std::isfinite(bounds.min_y) ||
          !std::isfinite(bounds.max_x) || !std::isfinite(bounds.max_y) ||
          bounds.min_x >= bounds.max_x || bounds.min_y >= bounds.max_y || resolution <= 0 ||
          _costmap->getSizeInCellsX() == 0 || _costmap->getSizeInCellsY() == 0)
        throw nav2_core::PlannerException("Invalid costmap geometry or filled footprint");
      lethal_rows.assign(_costmap->getSizeInCellsY(), {});
      nonzero_rows.assign(_costmap->getSizeInCellsY(), {});
      soft_rows.assign(_costmap->getSizeInCellsY(), {});
      for (unsigned int y = 0; y < _costmap->getSizeInCellsY(); ++y)
        for (unsigned int x = 0; x < _costmap->getSizeInCellsX(); ++x) {
          const auto cost = _costmap->getCost(x, y);
          if (cost == nav2_costmap_2d::FREE_SPACE)
            continue;
          double wx = 0, wy = 0;
          _costmap->mapToWorld(x, y, wx, wy);
          const Point point{wx, wy, resolution * std::sqrt(2.0) / 2};
          nonzero_rows[y].push_back(point);
          if (cost < nav2_costmap_2d::LETHAL_OBSTACLE)
            soft_rows[y].push_back({point, cost});
          if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
              (cost == nav2_costmap_2d::NO_INFORMATION && !_allow_unknown))
            lethal_rows[y].push_back(point);
        }
      require_time();
    };
    auto snapshot_matches = [&](const PlannerMapSnapshot &snapshot) {
      return snapshot.matches(_costmap->getSizeInCellsX(), _costmap->getSizeInCellsY(), resolution,
                              _costmap->getOriginX(), _costmap->getOriginY(), footprint,
                              _allow_unknown, _costmap->getCharMap());
    };
    auto capture = [&](PlannerMapSnapshot &snapshot) {
      snapshot.capture(_costmap->getSizeInCellsX(), _costmap->getSizeInCellsY(), resolution,
                       _costmap->getOriginX(), _costmap->getOriginY(), footprint, _allow_unknown,
                       _costmap->getCharMap());
    };
    index_map();
    auto valid = [&](const Pose &pose, bool free_only = false) {
      require_time();
      if (!footprintInsideBounds(pose, footprint, bounds, .002)) {
        require_time();
        return false;
      }
      std::vector<Point> obstacles;
      const double extent = filledQueryExtent(radius, resolution, .002);
      int lx = 0, ly = 0, ux = 0, uy = 0;
      _costmap->worldToMapEnforceBounds(pose.x - extent, pose.y - extent, lx, ly);
      _costmap->worldToMapEnforceBounds(pose.x + extent, pose.y + extent, ux, uy);
      double low = 0, high = 0, unused = 0;
      _costmap->mapToWorld(lx, ly, low, unused);
      _costmap->mapToWorld(ux, ly, high, unused);
      const auto &rows = free_only ? nonzero_rows : lethal_rows;
      for (int y = ly; y <= uy; ++y) {
        const auto &row = rows[y];
        auto begin = std::lower_bound(row.begin(), row.end(), low,
                                      [](const Point &p, double x) { return p.x < x; });
        for (auto it = begin; it != row.end() && it->x <= high; ++it)
          obstacles.push_back(*it);
      }
      const bool okay = !collision(pose, footprint, obstacles, .002);
      require_time();
      return okay;
    };
    auto segment_valid = [&](const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b,
                             bool free_only = false) {
      const double yaw = tf2::getYaw(a.orientation);
      const double dyaw = std::remainder(tf2::getYaw(b.orientation) - yaw, 2 * M_PI);
      const double dx = b.position.x - a.position.x, dy = b.position.y - a.position.y;
      const double travel = std::hypot(dx, dy) + radius * std::abs(dyaw);
      int steps = 0;
      if (!finiteHeadingSteps(travel, steps))
        return false;
      for (int j = 0; j <= steps; ++j) {
        const double f = static_cast<double>(j) / steps;
        if (!valid({a.position.x + f * dx, a.position.y + f * dy, yaw + f * dyaw}, free_only))
          return false;
      }
      return true;
    };
    auto path_valid = [&](const nav_msgs::msg::Path &path, bool free_only = false) {
      if (path.poses.empty() ||
          !valid({path.poses.front().pose.position.x, path.poses.front().pose.position.y,
                  tf2::getYaw(path.poses.front().pose.orientation)},
                 free_only))
        return false;
      for (size_t i = 1; i < path.poses.size(); ++i)
        if (!segment_valid(path.poses[i - 1].pose, path.poses[i].pose, free_only))
          return false;
      return true;
    };
    auto soft_cost = [&](const Pose &pose) {
      require_time();
      FilledSoftCost cost(pose, footprint, .002);
      const double extent = filledQueryExtent(radius, resolution, .002);
      int lx = 0, ly = 0, ux = 0, uy = 0;
      _costmap->worldToMapEnforceBounds(pose.x - extent, pose.y - extent, lx, ly);
      _costmap->worldToMapEnforceBounds(pose.x + extent, pose.y + extent, ux, uy);
      double low = 0, high = 0, unused = 0;
      _costmap->mapToWorld(lx, ly, low, unused);
      _costmap->mapToWorld(ux, ly, high, unused);
      for (int y = ly; y <= uy; ++y) {
        const auto &row = soft_rows[y];
        auto begin =
            std::lower_bound(row.begin(), row.end(), low,
                             [](const SoftCostPoint &p, double x) { return p.point.x < x; });
        for (auto it = begin; it != row.end() && it->point.x <= high; ++it)
          if (cost.include(*it)) {
            require_time();
            return cost.cost();
          }
      }
      require_time();
      return cost.cost();
    };
    const double dx = goal.pose.position.x - start.pose.position.x,
                 dy = goal.pose.position.y - start.pose.position.y;
    const double yaw = tf2::getYaw(start.pose.orientation),
                 dyaw = std::remainder(tf2::getYaw(goal.pose.orientation) - yaw, 2 * M_PI);
    const double travel = std::hypot(dx, dy) + radius * std::abs(dyaw);
    if (!std::isfinite(travel))
      throw nav2_core::PlannerException("Invalid planar start or goal");
    unsigned int start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    if (!valid({start.pose.position.x, start.pose.position.y, yaw}) ||
        !valid({goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)}) ||
        !_costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !_costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
      throw nav2_core::PlannerException(
          "Filled footprint / exact start or terminal pose intersects an obstacle, unknown cell, or map boundary");

    if (travel <= 6.0 && segment_valid(start.pose, goal.pose, true)) {
      nav_msgs::msg::Path direct;
      direct.header.frame_id = _global_frame;
      direct.header.stamp = _clock->now();
      const int steps = std::max(1, static_cast<int>(std::ceil(travel / .01)));
      for (int i = 0; i <= steps; ++i) {
        const double f = static_cast<double>(i) / steps;
        auto point = start;
        point.header = direct.header;
        point.pose.position.x += f * dx;
        point.pose.position.y += f * dy;
        point.pose.orientation.x = point.pose.orientation.y = 0;
        point.pose.orientation.z = std::sin((yaw + f * dyaw) / 2);
        point.pose.orientation.w = std::cos((yaw + f * dyaw) / 2);
        direct.poses.push_back(point);
      }
      direct.poses.back() = goal;
      direct.poses.back().header = direct.header;
      require_time();
      return direct;
    }
    const auto epoch = parameter_epoch_.load();
    if (cached_.poses.size() >= 2 && cached_.header.frame_id == _global_frame &&
        goal.pose == cached_.poses.back().pose && cached_epoch_ == epoch &&
        snapshot_matches(cached_map_)) {
      size_t nearest = 0;
      double fraction = 0, distance = 1e100;
      const Pose current{start.pose.position.x, start.pose.position.y, yaw};
      geometry_msgs::msg::Pose anchor;
      for (size_t i = 0; i + 1 < cached_.poses.size(); ++i) {
        const auto &a = cached_.poses[i].pose, &b = cached_.poses[i + 1].pose;
        const double sx = b.position.x - a.position.x, sy = b.position.y - a.position.y;
        const double ayaw = tf2::getYaw(a.orientation);
        const double turn = std::remainder(tf2::getYaw(b.orientation) - ayaw, 2 * M_PI);
        const double length2 = sx * sx + sy * sy;
        const double f =
            length2 > 1e-18 ? std::clamp(((start.pose.position.x - a.position.x) * sx +
                                          (start.pose.position.y - a.position.y) * sy) /
                                             length2,
                                         0., 1.)
                            : (std::abs(turn) > 1e-12
                                   ? std::clamp(std::remainder(yaw - ayaw, 2 * M_PI) / turn, 0., 1.)
                                   : 0.);
        auto p = a;
        p.position.x += f * sx;
        p.position.y += f * sy;
        p.orientation.x = p.orientation.y = 0;
        p.orientation.z = std::sin((ayaw + f * turn) / 2);
        p.orientation.w = std::cos((ayaw + f * turn) / 2);
        const double d =
            std::hypot(p.position.x - start.pose.position.x, p.position.y - start.pose.position.y) +
            radius * std::abs(std::remainder(tf2::getYaw(p.orientation) - yaw, 2 * M_PI));
        if (d < distance) {
          nearest = i;
          fraction = f;
          distance = d;
          anchor = p;
        }
      }
      const auto &a = cached_.poses[nearest].pose, &b = cached_.poses[nearest + 1].pose;
      const bool original_segment =
          onPoseSegment(current, {a.position.x, a.position.y, tf2::getYaw(a.orientation)},
                        {b.position.x, b.position.y, tf2::getYaw(b.orientation)}, fraction);
      if (distance < .20 && (original_segment || segment_valid(start.pose, anchor, true))) {
        auto remaining = cached_;
        remaining.header.stamp = _clock->now();
        remaining.poses.erase(remaining.poses.begin(), remaining.poses.begin() + nearest + 1);
        if (!original_segment) {
          auto p = start;
          p.pose = anchor;
          remaining.poses.insert(remaining.poses.begin(), p);
        }
        remaining.poses.insert(remaining.poses.begin(), start);
        remaining.poses.back() = goal;
        for (auto &p : remaining.poses)
          p.header = remaining.header;
        if (path_valid(remaining)) {
          require_time();
          cached_ = remaining;
          if (!budget.available()) {
            cached_ = nav_msgs::msg::Path();
            cached_map_.clear();
            throw nav2_core::PlannerException(
                "PreciseLattice total planning budget exhausted before cached return");
          }
          return remaining;
        }
      }
    }

    if (_search_info.cache_obstacle_heuristic)
      throw nav2_core::PlannerException("PreciseLattice requires cache_obstacle_heuristic=false");

    nav2_smac_planner::NodeHybrid::obstacle_heuristic_lookup_table.clear();
    PlannerMapSnapshot before_native;
    capture(before_native);

    settings_lock.unlock();
    auto path = SmacPlannerLattice::createPlan(start, goal);
    settings_lock.lock();
    require_time();
    index_map();
    const auto after_epoch = parameter_epoch_.load();
    const bool same_snapshot = epoch == after_epoch && snapshot_matches(before_native);
    if (!same_snapshot) {
      cached_map_.clear();
      cached_ = nav_msgs::msg::Path();
    }
    if (path.poses.empty()) {
      require_time();
      return path;
    }
    path.poses.insert(path.poses.begin(), start);
    path.poses.push_back(goal);
    const auto native_original = path;
    size_t native_first = path.poses.size();
    for (size_t i = 1; i + 1 < path.poses.size(); ++i)
      if (path.poses[i].pose.position.x != start.pose.position.x ||
          path.poses[i].pose.position.y != start.pose.position.y) {
        native_first = i;
        break;
      }
    std::vector<Pose> states;
    for (const auto &p : path.poses)
      states.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
    auto clean = path;
    clean.poses.clear();
    for (const auto i : loopFreeIndices(states))
      clean.poses.push_back(path.poses[i]);

    if (path_valid(clean, true))
      path = clean;
    auto fixed = path;
    for (auto &p : fixed.poses)
      p.pose.orientation = start.pose.orientation;
    fixed.poses.push_back(goal);

    auto reference = states;
    reference.push_back(states.back());
    auto message_states = [&](const nav_msgs::msg::Path &message) {
      std::vector<Pose> result;
      result.reserve(message.poses.size());
      for (const auto &p : message.poses)
        result.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
      return result;
    };
    auto heading_only_allowed = [&](const nav_msgs::msg::Path &message,
                                    bool check_quaternions = false) {
      require_time();
      if (check_quaternions)
        for (const auto &p : message.poses)
          if (!finiteHeadingQuaternionValid(p.pose.orientation))
            throw nav2_core::PlannerException("Invalid quaternion in finite heading repair");
      const auto verification = finiteHeadingVerify(
          reference, message_states(message), radius, [&](const Pose &p) { return valid(p); },
          soft_cost, [&]() { return budget.available(); });
      if (verification.status == FiniteHeadingStatus::BudgetExceeded)
        throw nav2_core::PlannerException(
            "PreciseLattice total planning budget exhausted during heading verification");
      require_time();
      return verification.status == FiniteHeadingStatus::Found;
    };
    if (path_valid(fixed, true) || heading_only_allowed(fixed))
      path = fixed;
    else if (!path_valid(path)) {
      if (native_first == native_original.poses.size() ||
          native_original.poses.size() >= finiteHeadingMaxPoints)
        throw nav2_core::PlannerException("No finite heading repair input within the point limit");
      auto constant = native_original;
      for (size_t i = 1; i < constant.poses.size(); ++i)
        constant.poses[i].pose.orientation = native_original.poses[native_first].pose.orientation;
      constant.poses.front() = start;
      constant.poses.push_back(goal);
      if (heading_only_allowed(constant, true))
        path = constant;
      else {
        auto candidate = native_original;
        candidate.poses.push_back(goal);
        using Quaternion = geometry_msgs::msg::Quaternion;
        std::vector<std::vector<Quaternion>> orientations;
        orientations.reserve(reference.size());
        std::vector<std::vector<double>> choices;
        choices.reserve(reference.size());
        for (size_t i = 0; i < candidate.poses.size(); ++i) {
          require_time();
          orientations.emplace_back();
          if (i == 0)
            orientations.back().push_back(start.pose.orientation);
          else if (i + 1 == candidate.poses.size())
            orientations.back().push_back(goal.pose.orientation);
          else {
            const auto options = finiteHeadingOrientations(
                start.pose.orientation, native_original.poses[native_first].pose.orientation,
                candidate.poses[i].pose.orientation);
            orientations.back().assign(options.begin(), options.end());
          }
          choices.emplace_back();
          for (const auto &q : orientations.back()) {
            if (!finiteHeadingQuaternionValid(q))
              throw nav2_core::PlannerException("Invalid quaternion in finite heading family");
            const double candidate_yaw = tf2::getYaw(q);
            if (!std::isfinite(candidate_yaw))
              throw nav2_core::PlannerException("Invalid derived yaw in finite heading family");
            choices.back().push_back(candidate_yaw);
          }
        }
        const auto repair = finiteHeadingSearch(
            reference, choices, radius, [&](const Pose &p) { return valid(p); }, soft_cost,
            [&]() { return budget.available(); });
        if (repair.status == FiniteHeadingStatus::BudgetExceeded)
          throw nav2_core::PlannerException(
              "PreciseLattice total planning budget exhausted during finite heading search");
        if (repair.status != FiniteHeadingStatus::Found)
          throw nav2_core::PlannerException(
              repair.status == FiniteHeadingStatus::NoFeasible
                  ? "Native filled-footprint sweep invalid; no feasible path in the finite heading family"
                  : "Finite heading repair input or full verification failed");
        for (size_t i = 0; i < candidate.poses.size(); ++i)
          candidate.poses[i].pose.orientation = orientations[i][repair.choices[i]];
        candidate.poses.front() = start;
        candidate.poses.back() = goal;

        if (!heading_only_allowed(candidate, true))
          throw nav2_core::PlannerException(
              "Materialized finite heading path failed complete hard/Q verification");
        path = candidate;
      }
    }
    require_time();
    for (auto &p : path.poses)
      p.header = path.header;
    require_time();
    if (same_snapshot) {
      auto cache_path = path;
      auto cache_snapshot = before_native;
      require_time();
      cached_ = std::move(cache_path);
      cached_map_ = std::move(cache_snapshot);
      cached_epoch_ = epoch;
    }

    if (!budget.available()) {
      if (same_snapshot) {
        cached_ = nav_msgs::msg::Path();
        cached_map_.clear();
      }
      throw nav2_core::PlannerException(
          "PreciseLattice total planning budget exhausted before return");
    }
    return path;
  }

private:
  bool unsafeCacheRequested(const std::vector<rclcpp::Parameter> &parameters) const {
    for (const auto &p : parameters)
      if (p.get_name() == _name + ".cache_obstacle_heuristic" && p.as_bool())
        return true;
    return false;
  }

  rcl_interfaces::msg::SetParametersResult rejectUnsafeCache() const {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason =
        "PreciseLattice requires cache_obstacle_heuristic=false; unchanged maps use its validated path cache";
    return result;
  }

  std::mutex plan_mutex_;
  std::atomic<uint64_t> parameter_epoch_{0};
  uint64_t cached_epoch_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cache_parameter_handler_;
  PlannerMapSnapshot cached_map_;
  nav_msgs::msg::Path cached_;
};
}
PLUGINLIB_EXPORT_CLASS(duojin02_navigation::PreciseLattice, nav2_core::GlobalPlanner)
