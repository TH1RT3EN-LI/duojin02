// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
// Independent exhaustive finite-domain oracle: score all candidates directly,
// check every tested block bound and the complete outside-mode competitor set.
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

int main()
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
    cfg.min_score = .10; cfg.min_margin = .02; cfg.root_block_power = trial % 5;
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
    {
      AcquisitionConfig local_cfg = cfg;
      local_cfg.local_domain = true;
      local_cfg.local_center = {m.origin_x + .07, m.origin_y - .03, 2.3};
      local_cfg.local_xy_half_width = .16;
      GlobalAcquisition local(local_cfg);
      const auto actual = local.observe(m, points, {0, 0, 0}, 200);
      double local_best = -1;
      std::vector<Candidate> inside;
      for (const auto & candidate : brute) {
        const double x = MAP_WXGX((&m), candidate.x * subject.stride_);
        const double y = MAP_WYGY((&m), candidate.y * subject.stride_);
        if (std::abs(x - local_cfg.local_center.x) <= local_cfg.local_xy_half_width &&
          std::abs(y - local_cfg.local_center.y) <= local_cfg.local_xy_half_width) {
          inside.push_back(candidate);
          local_best = std::max(local_best, candidate.value);
        }
      }
      if (inside.empty()) {
        require(!actual.acquired && actual.status == "no_free_space_candidate",
          "empty bounded domain accepted");
      } else {
        require(actual.search_complete && !actual.modes.empty() &&
          std::abs(actual.best_score - local_best) < 1e-14,
          "bounded best differs from exhaustive local oracle");
        double other = 0;
        for (const auto & candidate : inside) {
          const double xy = std::hypot(candidate.x - local.best_x_, candidate.y - local.best_y_) *
            actual.lattice_xy_step;
          const double yaw = std::abs(GlobalAcquisition::wrap(
            local.angle(candidate.a) - local.angle(local.best_a_)));
          if (xy >= local_cfg.mode_translation || yaw >= local_cfg.mode_yaw) {
            other = std::max(other, candidate.value);
          }
        }
        if (actual.best_score >= local_cfg.min_score) {
          require(other <= actual.competitor_upper_bound + 1e-14,
            "bounded domain omitted a competing mode");
        }
      }
    }
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
