#pragma once

#include "duojin02_navigation/planner_checks.hpp"
#include <array>
#include <chrono>

namespace duojin02_navigation {
inline constexpr size_t finiteHeadingMaxPoints = 65536;
enum class FiniteHeadingStatus {
  Found,
  NoFeasible,
  BudgetExceeded,
  InvalidInput,
  VerificationFailed
};
enum class FiniteHeadingFailure { None, NodeHard, EdgeHard, StrictQ, InvalidSteps };

struct FiniteHeadingTrace {
  FiniteHeadingFailure reason{FiniteHeadingFailure::None};
  size_t point{0}, source{0}, target{0};
  int sample{0}, steps{0};
  Pose reference{}, candidate{};
  int reference_cost{0}, candidate_cost{0};
};

struct FiniteHeadingCounts {
  size_t states{0}, expanded{0}, edges{0}, accepted{0}, hard_rejected{0}, soft_rejected{0};
  size_t hard_queries{0}, soft_queries{0}, budget_checks{0};
};

struct FiniteHeadingResult {
  FiniteHeadingStatus status{FiniteHeadingStatus::InvalidInput};
  std::vector<Pose> path;
  std::vector<size_t> choices;
  double angular_travel{0};
  size_t first_unreachable{0};
  FiniteHeadingTrace first_rejection;
  FiniteHeadingCounts counts;
};

class SteadyHeadingBudget {
public:
  using Clock = std::chrono::steady_clock;

  SteadyHeadingBudget(Clock::time_point began, double seconds) : began_(began) {
    seconds_ = std::isfinite(seconds) && seconds > 0 ? seconds : 0;
  }

  bool available() const {
    return std::chrono::duration<double>(Clock::now() - began_).count() < seconds_;
  }

private:
  Clock::time_point began_;
  double seconds_{0};
};

inline bool finiteHeadingSteps(double travel, int &steps) {
  const double raw = std::ceil(travel / .004);
  if (!std::isfinite(raw) || raw < 0 || raw >= std::numeric_limits<int>::max())
    return false;

  steps = std::max(1, static_cast<int>(raw));
  return true;
}

inline bool finiteHeadingCommonSteps(const Pose &ra, const Pose &rb, const Pose &a, const Pose &b,
                                     double radius, int &steps) {
  const double ds = std::hypot(rb.x - ra.x, rb.y - ra.y);
  const double rt = std::remainder(rb.yaw - ra.yaw, 2 * M_PI),
               ft = std::remainder(b.yaw - a.yaw, 2 * M_PI);
  return finiteHeadingSteps(std::max(ds + radius * std::abs(rt), ds + radius * std::abs(ft)),
                            steps);
}

template <class Quaternion> bool finiteHeadingQuaternionValid(const Quaternion &q) {
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         std::isfinite(norm) && norm > 0;
}

template <class Quaternion> Quaternion halfTurnWorldZ(const Quaternion &q) {
  auto result = q;
  result.x = -q.y;
  result.y = q.x;
  result.z = q.w;
  result.w = -q.z;
  return result;
}

template <class Quaternion>
std::array<Quaternion, 4> finiteHeadingOrientations(const Quaternion &start,
                                                    const Quaternion &native_first,
                                                    const Quaternion &native) {
  return {start, native_first, native, halfTurnWorldZ(native)};
}

namespace finite_heading_detail {
struct BudgetExpired {};

template <class HardPoseValid, class SoftPoseCost, class WithinBudget> class Checks {
public:
  Checks(double radius, HardPoseValid hard, SoftPoseCost soft, WithinBudget within,
         FiniteHeadingResult &result)
      : radius_(radius), hard_(hard), soft_(soft), within_(within), result_(result) {}

  void budget() {
    ++result_.counts.budget_checks;
    if (!within_())
      throw BudgetExpired{};
  }

  bool hard(const Pose &p) {
    budget();
    ++result_.counts.hard_queries;
    const bool okay = hard_(p);
    budget();
    return okay;
  }

  auto soft(const Pose &p) {
    budget();
    ++result_.counts.soft_queries;
    const auto cost = soft_(p);
    budget();
    return cost;
  }

  void remember(FiniteHeadingTrace trace) {
    if (result_.first_rejection.reason == FiniteHeadingFailure::None)
      result_.first_rejection = trace;
  }

  bool hardEdge(const Pose &a, const Pose &b, size_t edge, size_t source = 0, size_t target = 0) {
    const double turn = std::remainder(b.yaw - a.yaw, 2 * M_PI), dx = b.x - a.x, dy = b.y - a.y;
    int steps = 0;
    if (!finiteHeadingSteps(std::hypot(dx, dy) + radius_ * std::abs(turn), steps)) {
      remember({FiniteHeadingFailure::InvalidSteps, edge, source, target});
      return false;
    }

    for (size_t j = 0; j <= static_cast<size_t>(steps); ++j) {
      const double f = static_cast<double>(j) / steps;
      const Pose p{a.x + f * dx, a.y + f * dy, a.yaw + f * turn};
      if (!hard(p)) {
        remember({FiniteHeadingFailure::EdgeHard,
                  edge,
                  source,
                  target,
                  static_cast<int>(j),
                  steps,
                  {},
                  p});
        return false;
      }
    }
    return true;
  }

  bool hardPath(const std::vector<Pose> &path) {
    if (path.empty() || !hard(path.front()))
      return false;
    for (size_t i = 1; i < path.size(); ++i)
      if (!hardEdge(path[i - 1], path[i], i))
        return false;
    return true;
  }

  bool edgeQ(const Pose &ra, const Pose &rb, const Pose &a, const Pose &b, size_t edge,
             size_t source, size_t target) {
    int steps = 0;
    if (!finiteHeadingCommonSteps(ra, rb, a, b, radius_, steps)) {
      remember({FiniteHeadingFailure::InvalidSteps, edge, source, target});
      return false;
    }
    size_t calls = 0;
    decltype(soft_(Pose{})) previous{};
    Pose previous_pose{};

    return headingOnlyPathAllowed(
        {ra, rb}, {a, b}, radius_, [](const auto &) { return true; },
        [&](const Pose &p) {
          const auto cost = soft(p);
          if (calls % 2 == 0) {
            previous = cost;
            previous_pose = p;
          } else {
            bool invalid = previous > 253 || cost > 253 || cost > previous;
            if constexpr (std::is_signed_v<decltype(cost)>)
              invalid = invalid || previous < 0 || cost < 0;
            if (invalid)
              remember({FiniteHeadingFailure::StrictQ, edge, source, target,
                        static_cast<int>(calls / 2), steps, previous_pose, p,
                        static_cast<int>(previous), static_cast<int>(cost)});
          }
          ++calls;
          return cost;
        });
  }

  bool whole(const std::vector<Pose> &ref, const std::vector<Pose> &path) {
    if (ref.empty() || ref.size() != path.size() || ref.front().yaw != path.front().yaw ||
        ref.back().yaw != path.back().yaw)
      return false;
    for (size_t i = 1; i < ref.size(); ++i) {
      budget();
      int steps = 0;
      if (!finiteHeadingCommonSteps(ref[i - 1], ref[i], path[i - 1], path[i], radius_, steps)) {
        remember({FiniteHeadingFailure::InvalidSteps, i});
        return false;
      }
    }
    const bool allowed = headingOnlyPathAllowed(
        ref, path, radius_, [&](const auto &p) { return hardPath(p); },
        [&](const Pose &p) { return soft(p); });
    budget();
    return allowed;
  }

private:
  double radius_;
  HardPoseValid hard_;
  SoftPoseCost soft_;
  WithinBudget within_;
  FiniteHeadingResult &result_;
};

struct State {
  double yaw, score{std::numeric_limits<double>::infinity()};
  size_t option;
  int previous{-1};
  bool safe{false};
};
}

template <class HardPoseValid, class SoftPoseCost, class WithinBudget>
FiniteHeadingResult finiteHeadingVerify(const std::vector<Pose> &ref, const std::vector<Pose> &path,
                                        double radius, HardPoseValid hard, SoftPoseCost soft,
                                        WithinBudget within) {
  FiniteHeadingResult result;
  finite_heading_detail::Checks checks(radius, hard, soft, within, result);
  try {
    checks.budget();
    const bool allowed = checks.whole(ref, path);
    checks.budget();
    result.status = allowed ? FiniteHeadingStatus::Found : FiniteHeadingStatus::VerificationFailed;
    if (allowed)
      result.path = path;
    checks.budget();
    return result;
  } catch (const finite_heading_detail::BudgetExpired &) {
    result.status = FiniteHeadingStatus::BudgetExceeded;
    result.path.clear();
    return result;
  }
}

template <class HardPoseValid, class SoftPoseCost, class WithinBudget>
FiniteHeadingResult
finiteHeadingSearch(const std::vector<Pose> &ref, const std::vector<std::vector<double>> &options,
                    double radius, HardPoseValid hard, SoftPoseCost soft, WithinBudget within) {
  FiniteHeadingResult result;
  static_assert(std::is_integral_v<decltype(soft(Pose{}))>, "Q costs must remain integral");
  finite_heading_detail::Checks checks(radius, hard, soft, within, result);
  try {
    checks.budget();
    if (ref.size() < 2 || ref.size() > finiteHeadingMaxPoints || ref.size() != options.size() ||
        !std::isfinite(radius) || radius < 0 || options.front().size() != 1 ||
        options.back().size() != 1 || options.front()[0] != ref.front().yaw ||
        options.back()[0] != ref.back().yaw)
      return result;
    for (size_t i = 0; i < ref.size(); ++i) {
      checks.budget();
      if (!std::isfinite(ref[i].x) || !std::isfinite(ref[i].y) || !std::isfinite(ref[i].yaw) ||
          options[i].empty() || options[i].size() > 4)
        return result;
      for (const auto yaw : options[i])
        if (!std::isfinite(yaw))
          return result;
    }
    using State = finite_heading_detail::State;
    std::vector<std::vector<State>> layers;
    layers.reserve(ref.size());
    for (size_t i = 0; i < ref.size(); ++i) {
      checks.budget();
      layers.emplace_back();
      auto &current = layers.back();
      for (size_t k = 0; k < options[i].size(); ++k) {
        bool duplicate = false;
        for (const auto &s : current)
          duplicate = duplicate || s.yaw == options[i][k];
        if (duplicate)
          continue;
        current.push_back({options[i][k], std::numeric_limits<double>::infinity(), k});
        auto &s = current.back();
        ++result.counts.states;
        s.safe = checks.hard({ref[i].x, ref[i].y, s.yaw});
        if (!s.safe)
          checks.remember(
              {FiniteHeadingFailure::NodeHard, i, 0, k, 0, 0, {}, {ref[i].x, ref[i].y, s.yaw}});
      }
      if (i == 0) {
        if (current[0].safe)
          current[0].score = 0;
      } else
        for (size_t s = 0; s < layers[i - 1].size(); ++s) {
          const auto &previous = layers[i - 1][s];
          if (!std::isfinite(previous.score))
            continue;
          ++result.counts.expanded;
          for (size_t t = 0; t < current.size(); ++t) {
            checks.budget();
            auto &next = current[t];
            if (!next.safe)
              continue;
            ++result.counts.edges;
            const Pose a{ref[i - 1].x, ref[i - 1].y, previous.yaw}, b{ref[i].x, ref[i].y, next.yaw};
            if (!checks.hardEdge(a, b, i, s, t)) {
              ++result.counts.hard_rejected;
              continue;
            }
            if (!checks.edgeQ(ref[i - 1], ref[i], a, b, i, s, t)) {
              ++result.counts.soft_rejected;
              continue;
            }
            ++result.counts.accepted;
            const double score = previous.score + std::abs(std::remainder(b.yaw - a.yaw, 2 * M_PI));
            if (score < next.score ||
                (score == next.score && static_cast<int>(s) < next.previous)) {
              next.score = score;
              next.previous = static_cast<int>(s);
            }
          }
        }
      bool reachable = false;
      for (const auto &s : current)
        reachable = reachable || std::isfinite(s.score);
      if (!reachable) {
        checks.budget();
        result.status = FiniteHeadingStatus::NoFeasible;
        result.first_unreachable = i;
        return result;
      }
    }
    result.path = ref;
    result.choices.resize(ref.size());
    int state = 0;
    result.angular_travel = layers.back()[0].score;
    for (size_t i = ref.size(); i-- > 0;) {
      checks.budget();
      const auto &s = layers[i][state];
      result.path[i].yaw = s.yaw;
      result.choices[i] = s.option;
      state = s.previous;
    }
    result.status = checks.whole(ref, result.path) ? FiniteHeadingStatus::Found
                                                   : FiniteHeadingStatus::VerificationFailed;
    checks.budget();
    return result;
  } catch (const finite_heading_detail::BudgetExpired &) {
    result.status = FiniteHeadingStatus::BudgetExceeded;
    result.path.clear();
    result.choices.clear();
    return result;
  }
}
}
