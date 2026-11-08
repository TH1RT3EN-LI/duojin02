// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
namespace {
void check(bool value, const char * message) {if (!value) {throw std::runtime_error(message);}}
void near(double actual, double expected, double tolerance, const char * message) {
  check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
pf_vector_t origin(void *) {return {{0.1, 0.1, 0.01}};}
struct Delete {void operator()(pf_t * pf) const {pf_free(pf);}};
using Pf = std::unique_ptr<pf_t, Delete>;
Pf filter(int count = 4, double slow = 0.0, double fast = 0.0) {
  Pf pf(pf_alloc(2, count, slow, fast, origin)); pf_init_model(pf.get(), origin, nullptr);
  return pf;
}
struct Scores {std::vector<double> values; bool no_information{false};};
double sensor(void * data, pf_sample_set_t * set) {
  auto & scores = *static_cast<Scores *>(data);
  for (int i = 0; i < set->sample_count; ++i) {set->log_likelihoods[i] = scores.values[i];}
  set->log_likelihoods_ready = 1;
  set->sensor_no_information = scores.no_information;
  return 1.0;
}
void normalization() {
  auto pf = filter(); auto & set = pf->sets[0];
  Scores scores{{-10000.0, -10001.0, -10002.0, -10003.0}};
  pf_update_sensor(pf.get(), sensor, &scores);
  const double sum = 1 + std::exp(-1.0) + std::exp(-2.0) + std::exp(-3.0);
  near(set.samples[0].weight, 1 / sum, 1e-14, "extreme log normalization");
  near(pf->diagnostics.log_evidence, -10000.0 + std::log(sum / 4), 2e-12, "log evidence");
  near(pf->diagnostics.prior_ess, 4, 1e-14, "prior ESS captured after observation");
  near(pf->diagnostics.prior_max_weight, .25, 1e-14, "prior maximum weight");
  near(pf->diagnostics.posterior_max_weight, 1/sum, 1e-14, "posterior maximum weight");
  check(pf->diagnostics.posterior_entropy>0 && pf->diagnostics.posterior_entropy<std::log(4.),
    "posterior entropy bounds");
  check(std::isfinite(pf->log_w_slow) && pf->w_slow == 0, "recovery stored only underflowed linear value");
  check(pf->diagnostics.ess > 1 && pf->diagnostics.ess < 4, "ESS bounds");
  check(pf_should_resample(pf.get(), 0) == 1, "default resampling policy");
  check(pf_should_resample(pf.get(), .1) == 0, "optional ESS gate");
  check(pf_should_resample(pf.get(), 1.1) == -1, "invalid ESS gate");
  std::vector<double> prior;
  for (int i = 0; i < 4; ++i) {prior.push_back(set.samples[i].weight);}
  for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    scores.values = {0, 0, bad, 0}; pf_update_sensor(pf.get(), sensor, &scores);
    check(pf->diagnostics.status == PF_SENSOR_INVALID_WEIGHT, "invalid score accepted");
    for (int i = 0; i < 4; ++i) {check(set.samples[i].weight == prior[i], "rejection changed prior");}
  }
  scores.values.assign(4, -std::numeric_limits<double>::infinity());
  pf_update_sensor(pf.get(), sensor, &scores);
  check(pf->diagnostics.status == PF_SENSOR_ZERO_SUPPORT, "all minus infinity hidden");
  for (int i = 0; i < 4; ++i) {check(set.samples[i].weight == prior[i], "zero support changed prior");}
  const double old_average = pf->log_w_slow;
  scores.values.assign(4, 0); scores.no_information = true;
  pf_update_sensor(pf.get(), sensor, &scores);
  check(pf->diagnostics.status == PF_SENSOR_NO_INFORMATION, "empty observation not distinguished");
  check(pf->log_w_slow == old_average, "no information trained recovery");
  set.samples[0].weight = 0;
  scores.values = {10000, -10000, -10000, -10000}; scores.no_information = false;
  pf_update_sensor(pf.get(), sensor, &scores);
  check(set.samples[0].weight == 0, "zero prior acquired mass");
}
void recovery() {
  auto pf = filter(4, .1, .9);
  Scores scores{{-10000, -10000, -10000, -10000}};
  pf_update_sensor(pf.get(), sensor, &scores);
  const double old = pf->log_w_slow;
  scores.values.assign(4, -10010); pf_update_sensor(pf.get(), sensor, &scores);
  near(pf->log_w_slow, old + std::log(.9 + .1 * std::exp(-10.0)), 2e-12, "slow log EMA");
  near(pf->log_w_fast, old + std::log(.1 + .9 * std::exp(-10.0)), 2e-12, "fast log EMA");
  check(pf->log_w_fast < pf->log_w_slow, "underflow erased recovery contrast");
  srand48(6001); pf_update_resample(pf.get(), nullptr);
  check(pf->diagnostics.random_injections > 0, "underflow prevented recovery injection");
}
void covariance() {
  auto pf = filter(); auto & set = pf->sets[0];
  for (int i = 0; i < 4; ++i) {
    const double d = (i - 1.5) * .02;
    set.samples[i].pose = {{1000000 + d, 1000000 + 2 * d, .4 + 3 * d}};
    set.samples[i].weight = .25;
  }
  pf_recompute_stats(pf.get());
  check(set.cluster_count == 1, "test cloud unexpectedly split");
  const auto & cov = set.clusters[0].cov;
  near(cov.m[0][0], .0005, 3e-12, "centered variance at large origin");
  near(cov.m[0][1], .001, 3e-12, "xy cross covariance");
  near(cov.m[0][2], .0015, 3e-12, "x-yaw cross covariance");
  near(cov.m[1][2], .003, 3e-12, "y-yaw cross covariance");
  near(cov.m[2][2], .0045, 1e-14, "local yaw covariance");
  // Move particles without resampling: stale KD-tree bins must be rebuilt.
  for (int i = 0; i < 4; ++i) {set.samples[i].pose.v[0] += 10;}
  pf_recompute_stats(pf.get());
  near(set.mean.v[0], 1000010, 1e-9, "no-resample weighted mean is stale");
  // A cluster carrying half the probability mass has the same conditional
  // variance as the corresponding full-mass two-point cloud.
  for (int i = 0; i < 4; ++i) {
    set.samples[i].pose = {{i < 2 ? .1 : 5.1, .1, i % 2 ? .04 : .02}};
    set.samples[i].weight = .25;
  }
  pf_recompute_stats(pf.get()); check(set.cluster_count == 2, "two-mode fixture");
  for (int i = 0; i < 2; ++i) {
    near(set.clusters[i].cov.m[2][2], .0001, 1e-14, "cluster mass polluted yaw variance");
    near(set.clusters[i].weight, .5, 1e-14, "cluster probability mass");
  }
  // The overall narrow chart correctly crosses the +/-pi representation cut.
  set.samples[0].pose.v[2] = M_PI - .02; set.samples[1].pose.v[2] = -M_PI + .02;
  set.samples[2].pose.v[2] = M_PI - .02; set.samples[3].pose.v[2] = -M_PI + .02;
  pf_recompute_stats(pf.get());
  near(set.cov.m[2][2], .0004, 1e-14, "yaw representation branch cut");
}
void kld_and_lineage() {
  auto pf = filter(64);
  Scores collapse; collapse.values.assign(64, -INFINITY); collapse.values[0]=0;
  pf_update_sensor(pf.get(), sensor, &collapse);
  near(pf->diagnostics.prior_ess,64,1e-14,"collapse prior ESS");
  near(pf->diagnostics.ess,1,1e-14,"collapse posterior ESS");
  near(pf->diagnostics.posterior_max_weight,1,1e-14,"collapse maximum weight");
  near(pf->diagnostics.posterior_entropy,0,1e-14,"collapse posterior entropy");
  pf_recompute_stats(pf.get()); srand48(42); pf_update_resample(pf.get(), nullptr);
  check(pf->diagnostics.resample_output_count == 64, "legacy single-bin cap changed");
  check(pf->diagnostics.stop_reason == PF_RESAMPLE_SINGLE_BIN_CAP, "single-bin stop reason");
  check(pf->diagnostics.unique_parents == 1 && pf->diagnostics.distinct_lineages == 1, "lineage counting");
  check(pf->diagnostics.second_bin_sample == 0, "single bin falsely found second bin");
  near(pf->diagnostics.duplicate_fraction, 63.0 / 64, 1e-14, "duplicate fraction");
  for (int i = 0; i < 64; ++i) {
    near(pf->sets[pf->current_set].samples[i].weight, 1.0 / 64, 1e-14, "resampled normalization");
  }
  near(pf_kld_probability_to_z(.99), 2.3263478740408408, 2e-14, "explicit KLD confidence quantile");
  near(pf_kld_probability_to_z(.5), 0, 1e-14, "median KLD quantile");
  check(std::isnan(pf_kld_probability_to_z(1.0)), "invalid confidence accepted");
  // Keep the legacy strict 'sample_count > limit' stopping boundary.
  auto split = filter(64); auto & cloud = split->sets[0];
  split->pop_err = 100; split->pop_z = .99;
  for (int i = 0; i < 64; ++i) {cloud.samples[i].pose.v[0] = i % 2 ? .1 : 2.1;}
  pf_recompute_stats(split.get()); srand48(91); pf_update_resample(split.get(), nullptr);
  check(split->diagnostics.occupied_bins == 2 && split->diagnostics.kld_limit == 2, "KLD split fixture");
  check(split->diagnostics.resample_output_count > split->diagnostics.kld_limit, "legacy KLD strict stop changed");
  check(split->diagnostics.second_bin_sample > 1, "second-bin discovery diagnostic");
  check(split->diagnostics.stop_reason == PF_RESAMPLE_KLD_LIMIT, "KLD stop reason");
  auto broad = filter(64);
  for (int i = 0; i < 64; ++i) {broad->sets[0].samples[i].pose.v[2] = -M_PI + (i + .5) * 2 * M_PI / 64;}
  pf_recompute_stats(broad.get());
  check(broad->sets[0].cluster_count == 1, "broad connected angular fixture");
  check(!broad->sets[0].angular_chart_valid && !broad->sets[0].clusters[0].angular_chart_valid,
    "broad angular support accepted as narrow local chart");
}
}
int main() {
  try {normalization(); recovery(); covariance(); kld_and_lineage();
    std::cout << "AMCL_PF_NUMERICS_RESULT log_weights=stable rejection=atomic recovery=log_ema covariance=full_local_chart KLD=legacy lineage=recorded\n";
    return 0;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
