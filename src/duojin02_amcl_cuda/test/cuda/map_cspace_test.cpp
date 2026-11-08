// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
// Independent oracle: enumerate all occupied centers for every cell. This test
// deliberately does not use separable transforms or a parabola envelope.
#include "nav2_amcl/map/map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct Counts
{
  uint64_t maps{}, cells{}, failed_maps{}, failed_cells{};
  double maximum_error{};
} counts;

void check(map_t & map, double cap, const std::string & name)
{
  const size_t size = static_cast<size_t>(map.size_x) * map.size_y;
  std::vector<std::pair<int, int>> obstacles;
  std::vector<int> original_states;
  original_states.reserve(size);
  for (int y = 0; y < map.size_y; ++y) {
    for (int x = 0; x < map.size_x; ++x) {
      const int state = map.cells[static_cast<size_t>(y) * map.size_x + x].occ_state;
      original_states.push_back(state);
      if (state == 1) {obstacles.emplace_back(x, y);}
    }
  }
  map_update_cspace(&map, cap);
  bool failed = map.max_occ_dist != cap;
  for (int y = 0; y < map.size_y; ++y) {
    for (int x = 0; x < map.size_x; ++x) {
      const size_t index = static_cast<size_t>(y) * map.size_x + x;
      int64_t best_square = std::numeric_limits<int64_t>::max();
      for (const auto & obstacle : obstacles) {
        const int64_t dx = static_cast<int64_t>(x) - obstacle.first;
        const int64_t dy = static_cast<int64_t>(y) - obstacle.second;
        best_square = std::min(best_square, dx * dx + dy * dy);
      }
      const double expected = obstacles.empty() ? cap : std::min(cap, static_cast<double>(
        std::sqrt(static_cast<long double>(best_square)) *
        static_cast<long double>(map.scale)));
      const auto & cell = map.cells[index];
      const double error = std::abs(cell.occ_dist - expected);
      const double tolerance = 16 * std::numeric_limits<double>::epsilon() *
        std::max(std::abs(expected), map.scale);
      const bool bad = !std::isfinite(cell.occ_dist) || cell.occ_dist < 0 ||
        cell.occ_dist > cap || cell.occ_state != original_states[index] ||
        error > tolerance || (expected == 0.0 && cell.occ_dist != 0.0);
      if (std::isfinite(error)) {counts.maximum_error = std::max(counts.maximum_error, error);}
      if (bad) {
        if (counts.failed_cells < 8) {
          std::cerr << std::setprecision(17) << name << ": cell(" << x << "," << y <<
            ") actual=" << cell.occ_dist << " expected=" << expected <<
            " scale=" << map.scale << " cap=" << cap << "\n";
        }
        ++counts.failed_cells;
        failed = true;
      }
    }
  }
  ++counts.maps;
  counts.cells += size;
  counts.failed_maps += failed;
}

struct OwnedMap
{
  std::vector<map_cell_t> cells;
  map_t map{};
  OwnedMap(int width, int height, double scale, int state = -1)
  : cells(static_cast<size_t>(width) * height)
  {
    map.size_x = width; map.size_y = height; map.scale = scale; map.cells = cells.data();
    // An EDT must not depend on the chosen world-coordinate gauge.
    map.origin_x = 1e6 + .125; map.origin_y = -1e6 - .375;
    for (auto & cell : cells) {
      cell.occ_state = state; cell.occ_dist = std::numeric_limits<double>::quiet_NaN();
    }
  }
  void obstacle(int x, int y) {cells[static_cast<size_t>(y) * map.size_x + x].occ_state = 1;}
};
}  // namespace

int main()
{
  // Recorded old queue/mark-on-enqueue counterexample. At cell (2,8), the old
  // implementation reports .03 instead of sqrt(8)*.01.
  {
    OwnedMap map(7, 21, .01);
    for (const int index : {19, 36, 37, 46, 69, 82, 85, 87, 90, 110, 115, 116, 126, 131, 138, 146}) {
      map.cells[index].occ_state = 1;
    }
    check(map.map, 2.0, "known-enqueue-counterexample");
  }
  // floor(cap/scale) cannot be used as the radial cutoff. sqrt(2)*scale
  // is below a 1.5-cell cap and must be represented, not replaced by cap.
  {
    OwnedMap map(3, 3, .1);
    map.obstacle(0, 0);
    check(map.map, .15, "fractional-radius");
  }
  for (const auto & shape : {std::pair<int, int>{1, 1}, {1, 39}, {41, 1}, {8, 11}}) {
    for (const int state : {-1, 0, 1}) {
      OwnedMap map(shape.first, shape.second, .03, state);
      check(map.map, 0, "zero-cap");
      check(map.map, .037, "uniform-fractional-cap");
      check(map.map, 1.207, "uniform-large-cap");
    }
  }
  // Squared axis coordinates exceed int32 even with a small, cheap cap. Both
  // axes are exercised. There are only two obstacles, so the independent
  // brute-force oracle stays linear for these particular test fixtures.
  for (const auto & shape : {std::pair<int, int>{100003, 1}, {1, 100003}}) {
    OwnedMap map(shape.first, shape.second, .00001);
    map.obstacle(0, 0); map.obstacle(shape.first - 1, shape.second - 1);
    check(map.map, .000105, "long-axis-int64-squares");
  }
  {
    OwnedMap map(17, 13, .07);
    auto * const address = map.map.cells;
    map.obstacle(0, 0); map.obstacle(16, 12);
    check(map.map, .831, "same-address-initial");
    for (auto & cell : map.cells) {cell.occ_state = 0;}
    check(map.map, .831, "same-address-remove-all");
    map.obstacle(8, 6); map.cells[1].occ_state = -1;
    check(map.map, .099, "same-address-add-obstacle");
    map.map.scale = static_cast<double>(.01f);
    check(map.map, .217, "same-address-change-scale");
    check(map.map, 0, "same-address-zero-cap");
    for (auto & cell : map.cells) {cell.occ_state = 1;}
    check(map.map, .217, "same-address-all-occupied");
    if (map.map.cells != address) {
      std::cerr << "map_update_cspace changed the caller-owned cell pointer\n";
      return 2;
    }
  }
  std::mt19937 rng(20261020);
  const double scales[] = {.01, static_cast<double>(.01f), .03, .07, .5, 1.0, 1.3};
  const double cap_cells[] = {0, .3, .9, 1.01, 1.5, 2.3, 7.7, 31.01};
  for (int trial = 0; trial < 800; ++trial) {
    const int width = 1 + rng() % 29, height = 1 + rng() % 31;
    OwnedMap map(width, height, scales[trial % 7]);
    for (auto & cell : map.cells) {
      const unsigned choice = rng() % 10;
      cell.occ_state = choice < 5 ? -1 : (choice < 8 ? 0 : 1);
    }
    const double cap = map.map.scale * cap_cells[trial % 8];
    check(map.map, cap, "random-tristate-" + std::to_string(trial));
    if (trial % 17 == 0) {
      for (auto & cell : map.cells) {cell.occ_state = rng() % 2 ? 0 : -1;}
      check(map.map, cap, "random-remove-obstacles-" + std::to_string(trial));
    }
  }
  std::cout << std::setprecision(17) <<
    "{\"oracle\":\"all_occupied_centers_brute_force_int64_squared_long_double_sqrt\","
    "\"maps\":" << counts.maps << ",\"cells\":" << counts.cells <<
    ",\"failed_maps\":" << counts.failed_maps << ",\"failed_cells\":" <<
    counts.failed_cells << ",\"maximum_absolute_error_m\":" << counts.maximum_error <<
    ",\"passed\":" << (counts.failed_maps ? "false" : "true") << "}\n";
  return counts.failed_maps ? 1 : 0;
}
