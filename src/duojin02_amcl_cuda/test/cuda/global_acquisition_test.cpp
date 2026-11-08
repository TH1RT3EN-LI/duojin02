// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/global_acquisition.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
using nav2_amcl::AcquisitionPoint;
using nav2_amcl::AcquisitionPose;
using nav2_amcl::GlobalAcquisition;
namespace
{
void check(bool condition, const char * message)
{if (!condition) {throw std::runtime_error(message);}}
using Map = std::unique_ptr<map_t, decltype(&map_free)>;
Map makeMap(bool asymmetric, double ox = 0, double oy = 0)
{
  Map map(map_alloc(), map_free);
  map->size_x = map->size_y = 120;
  map->scale = .05; map->origin_x = ox; map->origin_y = oy;
  map->cells = static_cast<map_cell_t *>(std::calloc(120 * 120, sizeof(map_cell_t)));
  for (int y = 0; y < 120; ++y) {
    for (int x = 0; x < 120; ++x) {
      const bool boundary = x < 3 || y < 3 || x > 116 || y > 116;
      const bool object1 = asymmetric && x > 32 && x < 46 && y > 60 && y < 105;
      const bool object2 = asymmetric && x > 80 && x < 108 && y > 25 && y < 38;
      map->cells[MAP_INDEX(map.get(), x, y)].occ_state = boundary || object1 || object2 ? 1 : -1;
    }
  }
  map_update_cspace(map.get(), 2.0);
  return map;
}
std::vector<AcquisitionPoint> scan(const map_t & map, AcquisitionPose pose)
{
  std::vector<AcquisitionPoint> points;
  for (int beam = 0; beam < 120; ++beam) {
    const double angle = -M_PI + beam * 2 * M_PI / 120;
    const double range = map_calc_range(const_cast<map_t *>(&map), pose.x, pose.y,
      pose.yaw + angle, 8.0);
    if (range < 8.0) {points.push_back({range * std::cos(angle), range * std::sin(angle)});}
  }
  return points;
}
void uniqueScene(double ox, double oy, AcquisitionPose truth)
{
  auto map = makeMap(true, ox, oy);
  GlobalAcquisition acquisition;
  nav2_amcl::AcquisitionResult result;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 4 && !result.acquired; ++i) {
    result = acquisition.observe(*map, scan(*map, truth), {0, 0, 0}, 10 + i);
    std::cout << "unique " << i << " " << result.status << " " << result.best_score << " " <<
      result.margin << " modes=" << result.modes.size() << '\n';
  }
  check(result.acquired, "asymmetric scene did not acquire");
  check(result.search_complete && result.certificate_valid &&
    result.best_score - result.competitor_upper_bound >= .05,
    "acquisition did not close the complete competitor bound");
  const auto actual = result.modes.front().pose;
  check(std::hypot(actual.x - truth.x, actual.y - truth.y) < .06,
    "global translation recovery failed");
  check(std::abs(GlobalAcquisition::wrap(actual.yaw - truth.yaw)) < .03,
    "global rotation recovery failed");
  std::cout << "unique elapsed_ms=" << std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count() << " error_m=" <<
    std::hypot(actual.x - truth.x, actual.y - truth.y) << '\n';
}
}
int main()
{
  uniqueScene(0, 0, {.45, -.65, .73});
  uniqueScene(17, -12, {17.45, -12.65, .73});
  uniqueScene(0, 0, {-1.5, -.6, -2.7});
  auto symmetric = makeMap(false);
  GlobalAcquisition ambiguous;
  for (int i = 0; i < 5; ++i) {
    const auto result = ambiguous.observe(*symmetric, scan(*symmetric, {.4, -.3, .43}),
      {0, 0, 0}, 100 + i);
    check(!result.acquired, "symmetric map falsely declared unique initialization");
    check(result.status == "ambiguous", "symmetry was not preserved as ambiguity");
    check(result.search_complete && !result.certificate_valid &&
      result.competitor_upper_bound >= result.best_score - .05,
      "symmetry discarded an unresolved competitor");
  }
  auto asymmetric = makeMap(true);
  {
    // A 1.4 m offset and a large yaw error are corrected inside the user domain.
    nav2_amcl::AcquisitionConfig local;
    local.local_domain = true;
    local.local_center = {1.85, -.65, -2.7};
    local.local_xy_half_width = 2.0;
    GlobalAcquisition bounded_local(local);
    nav2_amcl::AcquisitionResult result;
    for (int i = 0; i < 4 && !result.acquired; ++i) {
      result = bounded_local.observe(*asymmetric, scan(*asymmetric, {.45, -.65, .73}),
        {0, 0, 0}, 200 + i);
    }
    check(result.acquired, "bounded initial pose did not recover");
    check(std::hypot(result.modes.front().pose.x - .45, result.modes.front().pose.y + .65) < .06,
      "bounded initial translation recovery failed");
    check(std::abs(GlobalAcquisition::wrap(result.modes.front().pose.yaw - .73)) < .03,
      "bounded full-circle heading recovery failed");

    // No cell in a domain outside the map may leak an old/zero candidate.
    local.local_center = {100, 100, 0};
    GlobalAcquisition outside(local);
    const auto empty = outside.observe(*asymmetric, scan(*asymmetric, {.45, -.65, .73}), {0, 0, 0}, 300);
    check(!empty.acquired && empty.status == "no_free_space_candidate" && empty.modes.empty(),
      "out-of-map local domain returned a candidate");
  }
  GlobalAcquisition duplicate;
  const auto points = scan(*asymmetric, {.45, -.65, .73});
  const auto first = duplicate.observe(*asymmetric, points, {0, 0, 0}, 100);
  check(first.confirmations == 1, "test did not produce first confirmation");
  for (int i = 0; i < 5; ++i) {
    const auto result = duplicate.observe(*asymmetric, points, {0, 0, 0}, 100);
    check(!result.acquired && result.confirmations == 1 && result.observed_scans == 1,
      "duplicate source incremented confirmation");
    check(!result.certificate_valid && !result.search_complete,
      "duplicate source retained a stale certificate");
  }
  GlobalAcquisition unsupported;
  const std::vector<AcquisitionPoint> missing(24, {100, 100});
  for (int i = 0; i < 4; ++i) {
    check(!unsupported.observe(*asymmetric, missing, {0, 0, 0}, i).acquired,
      "unsupported scan acquired");
  }
  const auto invalid = unsupported.observe(*asymmetric, {{NAN, 0}}, {0, 0, 0}, 100);
  check(invalid.status == "invalid_or_insufficient_support", "invalid scan accepted");
  nav2_amcl::AcquisitionConfig config;
  config.max_evaluations = 1;
  GlobalAcquisition bounded(config);
  const auto budget = bounded.observe(*asymmetric, points, {0, 0, 0}, 100);
  check(!budget.acquired && budget.status == "search_budget_exceeded" &&
    !budget.exhaustive_lattice && !budget.certificate_valid &&
    budget.evaluations <= config.max_evaluations && budget.competitor_upper_bound == 1,
    "truncated search silently acquired");
  config.max_memory_bytes = 1024;
  GlobalAcquisition memory_bounded(config);
  const auto memory = memory_bounded.observe(*asymmetric, points, {0, 0, 0}, 100);
  check(!memory.acquired && !memory.certificate_valid &&
    memory.status == "search_memory_or_domain_budget_exceeded",
    "memory budget failed open");
  // Every distinct scan rebuilds the complete domain. Strong prior support
  // cannot carry a confirmation over a new unsupported observation.
  const auto changed = duplicate.observe(*asymmetric, missing, {0, 0, 0}, 101);
  check(changed.searched && changed.confirmations == 0 && !changed.certificate_valid &&
    !changed.acquired && changed.evidence_stamp_ns == 101,
    "new source reused an earlier scan certificate");
  const auto propagated = GlobalAcquisition::propagate({2, 3, M_PI / 2}, {1, 1, 0}, {2, 1, .2});
  check(std::abs(propagated.x - 2) < 1e-12 && std::abs(propagated.y - 4) < 1e-12 &&
    std::abs(propagated.yaw - M_PI / 2 - .2) < 1e-12, "SE2 propagation wrong");
  std::cout << "global acquisition component checks passed\n";
}
