// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
// Independent exhaustive yaw-block oracle: score every finite-domain leaf,
// verify xy/yaw envelopes, tail groups, whole-mode competition, and fail-closed
// resource/mutable-map contracts. This does not claim continuous pose accuracy.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>
#include <random>
#define private public
#include "nav2_amcl/global_acquisition.hpp"
#undef private

using namespace nav2_amcl;
void require(bool condition, const char *message)
{if (!condition) {throw std::runtime_error(message);}}

double independentScore(const map_t &m, const std::vector<AcquisitionPoint> &points,
  const AcquisitionConfig &cfg, int gx, int gy, double yaw)
{
  if (m.cells[gx + gy * m.size_x].occ_state != -1) {return -1;}
  double sum = 0;
  for (auto p : points) {
    int x = gx + static_cast<int>(std::floor(
      (std::cos(yaw) * p.x - std::sin(yaw) * p.y) / m.scale + .5));
    int y = gy + static_cast<int>(std::floor(
      (std::sin(yaw) * p.x + std::cos(yaw) * p.y) / m.scale + .5));
    if (x < 0 || y < 0 || x >= m.size_x || y >= m.size_y) {continue;}
    const double d = m.cells[x + y * m.size_x].occ_dist;
    if (!std::isfinite(d) || d < 0 || d >= 4 * cfg.final_sigma) {continue;}
    sum += std::exp(-.5 * d * d / (cfg.final_sigma * cfg.final_sigma));
  }
  return sum / points.size();
}

void checkYawDomain()
{
  std::mt19937 rng(20261020);
  uint64_t poses = 0, bounds = 0, bound_members = 0, certificates = 0, ambiguous = 0;
  double worst_bound_violation = 0, worst_best_error = 0, worst_competitor_violation = 0;
  for (int trial = 0; trial < 150; ++trial) {
    map_t m{};
    m.size_x = 7 + rng() % 9; m.size_y = 7 + rng() % 7;
    m.scale = trial % 2 ? .05 : .1; m.origin_x = 17; m.origin_y = -12;
    std::vector<map_cell_t> cells(m.size_x * m.size_y); m.cells = cells.data();
    for (auto &cell : cells) {
      const int choice = rng() % 6;
      cell.occ_state = choice < 4 ? -1 : (choice == 4 ? 0 : 1);
      const int kind = rng() % 20;
      cell.occ_dist = kind == 0 ? NAN : kind == 1 ? -.01 : kind == 2 ? INFINITY :
        (rng() % 100) * .001;
    }
    cells[0].occ_state = -1;
    std::vector<AcquisitionPoint> points;
    for (int i = 0; i < 12 + trial % 17; ++i) {
      points.push_back({static_cast<int>(rng() % 101) * .01 - .5,
        static_cast<int>(rng() % 101) * .01 - .5});
    }
    AcquisitionConfig cfg;
    cfg.xy_step = trial % 3 ? .11 : .05;
    cfg.yaw_step = .47; cfg.mode_translation = .25; cfg.mode_yaw = .8;
    cfg.min_score = .10; cfg.min_margin = .02; cfg.root_block_power = trial % 5; cfg.root_angle_power = 3 + trial % 2;
    GlobalAcquisition subject(cfg);
    require(subject.prepare(m, points), "prepare unexpectedly failed");
    struct Candidate {int x, y, a; double value;};
    std::vector<Candidate> brute;
    double true_best = -1;
    for (int a = 0; a < subject.angles_; ++a) {
      const double yaw = -M_PI + 2 * M_PI * a / subject.angles_;
      for (int y = 0; y < subject.ny_; ++y) {
        for (int x = 0; x < subject.nx_; ++x) {
          const double s = independentScore(m, points, cfg, x * subject.stride_,
            y * subject.stride_, yaw);
          require(std::abs(s - subject.leaf(m, x, y, a)) < 1e-14,
            "leaf score differs from independent full-beam objective");
          if (s >= 0) {brute.push_back({x,y,a,s}); true_best = std::max(true_best, s);}
          ++poses;
        }
      }
      for (int level = 0; level <= cfg.root_block_power; ++level) {
        const int side = 1 << level;
        for (int y = 0; y < subject.ny_; y += side) {
          for (int x = 0; x < subject.nx_; x += side) {
            const double upper = subject.upper({x,y,level,a,0,0});
            for (const auto &c : brute) {
              if (c.a != a || c.x < x || c.x >= x + side || c.y < y || c.y >= y + side) {continue;}
              worst_bound_violation = std::max(worst_bound_violation, c.value - upper);
              require(c.value <= upper, "native block upper bound is inadmissible");
              ++bound_members;
            }
            ++bounds;
          }
        }
      }
    }
    // Exhaust all tested dyadic yaw blocks, including the incomplete last group.
    for (int angle_level = 1; angle_level <= cfg.root_angle_power; ++angle_level) {
      const int angle_width = 1 << angle_level;
      for (int a = 0; a < subject.angles_; a += angle_width) {
        for (int level = 0; level <= cfg.root_block_power; ++level) {
          const int side = 1 << level;
          for (int y = 0; y < subject.ny_; y += side) {
            for (int x = 0; x < subject.nx_; x += side) {
              const double upper = subject.upper({x,y,level,a,angle_level,0});
              for (const auto & c : brute) {
                if (c.a < a || c.a >= a + angle_width || c.x < x || c.x >= x + side ||
                  c.y < y || c.y >= y + side) {continue;}
                worst_bound_violation = std::max(worst_bound_violation, c.value - upper);
                require(c.value <= upper, "yaw block upper bound is inadmissible");
                ++bound_members;
              }
              ++bounds;
            }
          }
        }
      }
    }
    const auto result = subject.observe(m, points, {0,0,0}, 100);
    require(result.search_complete && !result.modes.empty(), "small-domain search incomplete");
    worst_best_error = std::max(worst_best_error, std::abs(result.best_score - true_best));
    require(std::abs(result.best_score - true_best) < 1e-14, "native best is not exhaustive best");
    double true_competitor = 0;
    for (auto c : brute) {
      const double xy = std::hypot(c.x - subject.best_x_, c.y - subject.best_y_) *
        result.lattice_xy_step;
      const double da = std::abs(GlobalAcquisition::wrap(
        subject.angle(c.a) - subject.angle(subject.best_a_)));
      if (xy >= cfg.mode_translation || da >= cfg.mode_yaw) {
        true_competitor = std::max(true_competitor, c.value);
      }
    }
    if (result.best_score >= cfg.min_score) {
      worst_competitor_violation = std::max(worst_competitor_violation,
        true_competitor - result.competitor_upper_bound);
      require(true_competitor <= result.competitor_upper_bound + 1e-14,
        "competitor upper bound omitted an outside mode");
      if (result.certificate_valid) {
        require(result.best_score - true_competitor >= cfg.min_margin - 1e-14,
          "false finite-domain certificate");
        ++certificates;
      } else {++ambiguous;}
    }
    const auto duplicate = subject.observe(m, points, {0,0,0}, 100);
    require(!duplicate.certificate_valid && !duplicate.search_complete && !duplicate.acquired &&
      duplicate.observed_scans == result.observed_scans &&
      duplicate.confirmations == result.confirmations, "duplicate reuses new-frame evidence");
    const auto invalid = subject.observe(m, {{NAN, 0}}, {0,0,0}, 101);
    require(!invalid.certificate_valid && invalid.confirmations == 0 &&
      !invalid.search_complete && !invalid.acquired, "invalid support reused certificate");
    AcquisitionConfig memory_cfg = cfg;
    memory_cfg.max_memory_bytes = subject.fixed_memory_ + sizeof(GlobalAcquisition::Node);
    GlobalAcquisition memory_bounded(memory_cfg);
    const auto memory_result = memory_bounded.observe(m, points, {0,0,0}, 100);
    require(!memory_result.certificate_valid && !memory_result.acquired &&
      !memory_result.search_complete && memory_result.memory_bytes <= memory_cfg.max_memory_bytes,
      "frontier allocation exceeded the owned-buffer budget");
    cfg.max_evaluations = 1;
    GlobalAcquisition bounded(cfg);
    const auto budget = bounded.observe(m, points, {0,0,0}, 100);
    require(!budget.certificate_valid && !budget.acquired && !budget.search_complete &&
      budget.evaluations <= 1, "budget does not fail closed");
  }
  std::cout << "{\n\"random_maps\":150,\n\"leaf_checks\":" << poses <<
    ",\n\"native_block_bounds\":" << bounds << ",\n\"covered_candidate_checks\":" <<
    bound_members << ",\n\"certificates_checked\":" << certificates <<
    ",\n\"ambiguous_decisions_checked\":" << ambiguous <<
    ",\n\"worst_bound_violation\":" << worst_bound_violation <<
    ",\n\"worst_best_error\":" << worst_best_error <<
    ",\n\"worst_competitor_violation\":" << worst_competitor_violation <<
    ",\n\"passed\":true\n}\n";
}

void checkResourceAndMutationContracts()
{
  std::mt19937 rng(202610201);
  uint64_t budget_cases = 0, memory_cases = 0, mutation_cases = 0;
  for (int trial = 0; trial < 24; ++trial) {
    map_t map{}; map.size_x = 8 + trial % 5; map.size_y = 9 + trial % 3;
    map.scale = .07; map.origin_x = 1.1; map.origin_y = -3.3;
    std::vector<map_cell_t> cells(map.size_x * map.size_y); map.cells = cells.data();
    for (auto & c : cells) {c.occ_state = (rng() % 5) ? -1 : 1; c.occ_dist = (rng() % 80) * .001;}
    cells[0].occ_state = -1;
    std::vector<AcquisitionPoint> points;
    for (int i = 0; i < 12 + trial % 5; ++i) {
      points.push_back({(static_cast<int>(rng() % 50) - 25) * .01,
        (static_cast<int>(rng() % 50) - 25) * .01});
    }
    AcquisitionConfig cfg; cfg.xy_step = .07; cfg.yaw_step = .47;
    cfg.root_block_power = trial % 5; cfg.root_angle_power = trial % 2 ? 3 : 4;
    cfg.min_score = .1; cfg.min_margin = .02;
    GlobalAcquisition full(cfg); const auto exact = full.observe(map, points, {}, 1);
    require(exact.search_complete, "control full search not complete");
    const std::vector<uint64_t> budgets{1,2,3,7,16,31,64,128,256,512,
      std::max<uint64_t>(1,exact.evaluations-1), exact.evaluations};
    for (uint64_t cap : budgets) {
      auto bounded_cfg = cfg; bounded_cfg.max_evaluations = cap;
      GlobalAcquisition bounded(bounded_cfg); auto value = bounded.observe(map, points, {}, 1);
      require(value.evaluations <= cap, "budget accounting exceeded");
      if (value.search_complete) {
        require(value.best_score == exact.best_score, "budget changed completed best");
        if (value.certificate_valid) {
          require(exact.certificate_valid, "budget manufactured certificate");
        }
      } else {
        require(!value.acquired && !value.certificate_valid && value.confirmations == 0 &&
          value.competitor_upper_bound == 1, "budget incomplete did not fail closed");
      }
      ++budget_cases;
    }
    auto memory_cfg = cfg; memory_cfg.max_memory_bytes = full.fixed_memory_ + sizeof(GlobalAcquisition::Node);
    GlobalAcquisition memory(memory_cfg); const auto memory_value = memory.observe(map, points, {}, 1);
    require(!memory_value.search_complete && !memory_value.certificate_valid &&
      !memory_value.acquired && memory_value.memory_bytes <= memory_cfg.max_memory_bytes,
      "memory incomplete did not fail closed"); ++memory_cases;

    // The same cells pointer remains valid while content/geometry changes.
    // Compare each changed objective with a newly constructed solver.
    GlobalAcquisition reused(cfg); int64_t stamp = 10;
    reused.observe(map, points, {}, stamp++);
    auto compare_fresh = [&]() {
      const auto actual = reused.observe(map, points, {}, stamp++);
      GlobalAcquisition fresh(cfg); const auto expected = fresh.observe(map, points, {}, 1);
      require(actual.best_score == expected.best_score &&
        actual.lattice_poses == expected.lattice_poses &&
        actual.search_complete == expected.search_complete &&
        actual.certificate_valid == expected.certificate_valid,
        "same-address changed map retained stale objective"); ++mutation_cases;
    };
    for (auto & c : cells) {c.occ_dist = .025;} compare_fresh();
    for (auto & c : cells) {c.occ_dist = .04; c.occ_state = -1;} compare_fresh();
    map.scale = .11; map.origin_x += 1.7; map.origin_y -= 2.1; compare_fresh();
    for (auto & c : cells) {c.occ_state = 1;} compare_fresh();
    for (auto & c : cells) {c.occ_state = -1;} map.size_x -= 1; compare_fresh();
  }
  std::cout << "{\"budget_cases\":" << budget_cases << ",\"memory_cases\":" << memory_cases <<
    ",\"same_address_mutations\":" << mutation_cases << ",\"passed\":true}\n";
}

int main()
{
  checkYawDomain();
  checkResourceAndMutationContracts();
  return 0;
}
