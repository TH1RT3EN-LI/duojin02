// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/pf/posterior_snapshot.hpp"
#include "nav2_amcl/estimate_contract.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace {
void check(bool value, const char * reason) {if (!value) {throw std::runtime_error(reason);}}
pf_vector_t zero(void *) {return {{0.0, 0.0, 0.0}};}
void seedPosterior(pf_t * pf) {
  auto & set = pf->sets[pf->current_set];
  set.sample_count = pf->max_samples;
  for (int i = 0; i < set.sample_count; ++i) {
    set.samples[i].pose = {{i < set.sample_count / 2 ? -.04 : .04, 0.01, .001}};
    set.samples[i].weight = 1.0 / set.sample_count;
    set.samples[i].lineage = i + 1;
  }
  // Deliberately do not update the KD-tree: production motion has changed poses.
}
}
int main() {
  using namespace nav2_amcl;
  const double precise = std::nextafter(1.0, 2.0);
  check(std::stod(diagnosticNumber(precise)) == precise, "diagnostic lost binary64 coordinates");
  EstimateContract c;
  c.recordAccepted("laser:10", 10, 13, 15, 17);
  c.recordAccepted("laser:10", 10, 23, 25, 27);
  check(c.duplicate_source_consumed && c.first_available_steady_ns == 17 &&
    c.received_ros_stamp_ns == 13 && c.last_effective_observation_stamp_ns == 10,
    "duplicate source refreshed observation age");
  c.recordAccepted("old_laser:9", 9, 23, 25, 27);
  check(c.nonmonotonic_source && c.first_available_steady_ns == 17, "out of order source refreshed age");
  c.recordAccepted("laser:11", 11, 33, 35, 37);
  check(!c.duplicate_source_consumed && c.first_available_steady_ns == 37,
    "new accepted source did not advance availability");
  c.resetObservation(); check(!c.valid_observation && c.source_id == "none", "reset stale observation");
  pf_t * pf = pf_alloc(700, 3000, 0.0, 0.0, zero);
  pf_init_model(pf, zero, nullptr);
  std::vector<double> resampled_means;
  double squares = 0.0;
  for (int trial = 0; trial < 100; ++trial) {
    seedPosterior(pf);
    srand48(10000 + trial);
    const auto snapshot = buildPosteriorSnapshot(pf);
    check(snapshot->clusters.size() == 1, "E1 changed coarse mode rule");
    const auto fixed_mean = snapshot->clusters[0].mean.v[0];
    check(std::abs(fixed_mean) < 1e-15, "weighted pre-resample output is not fixed");
    check(std::abs(snapshot->clusters[0].covariance.m[0][0] - .0016) < 1e-15,
      "weighted covariance not refreshed");
    pf_update_resample(pf, nullptr);
    const auto & after = pf->sets[pf->current_set];
    resampled_means.push_back(after.mean.v[0]); squares += after.mean.v[0] * after.mean.v[0];
    std::vector<pf_sample_t> actual(after.samples, after.samples + after.sample_count);
    const double next_random = drand48();
    const int actual_count = after.sample_count;
    // Resampling the same posterior with the same seed without the snapshot must
    // produce the exact same next-step representation and the same next random.
    seedPosterior(pf); srand48(10000 + trial); pf_update_resample(pf, nullptr);
    const auto & control = pf->sets[pf->current_set];
    check(control.sample_count == actual_count, "snapshot changed KLD sample count");
    for (int i = 0; i < control.sample_count; ++i) {
      for (int j = 0; j < 3; ++j) {
        check(control.samples[i].pose.v[j] == actual[i].pose.v[j], "snapshot changed RNG draws");
      }
      check(control.samples[i].weight == actual[i].weight &&
        control.samples[i].lineage == actual[i].lineage, "snapshot changed posterior particles");
    }
    check(drand48() == next_random, "snapshot consumed RNG state");
    // Overwrite both PF buffers: the snapshot must retain independent ownership.
    seedPosterior(pf);
    check(snapshot->clusters[0].mean.v[0] == fixed_mean, "snapshot aliases PF buffer");
  }
  check(squares > 0.0, "control did not expose post-resample noise");
  pf_free(pf);
  std::cout << "{\"passed\":true,\"trials\":100,\"snapshot_rng_path_exact\":true,"
    "\"snapshot_mean_rms_m\":0,\"post_resample_mean_rms_m\":" << std::sqrt(squares / 100)
    << ",\"scope\":\"synthetic PF contract; no physical accuracy claim\"}\n";
}
