// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__PF__POSTERIOR_SNAPSHOT_HPP_
#define NAV2_AMCL__PF__POSTERIOR_SNAPSHOT_HPP_

#include <memory>
#include <vector>
#include "nav2_amcl/pf/pf.hpp"

namespace nav2_amcl
{
struct PosteriorCluster
{
  double weight;
  pf_vector_t mean;
  pf_matrix_t covariance;
  bool angular_chart_valid;
};

// A value copy with no particle, KD-tree or cluster-array pointers. Consumers
// receive const ownership before either PF buffer may be reused by resampling.
struct PosteriorSnapshot
{
  std::vector<PosteriorCluster> clusters;
  int sample_count{0};
  unsigned long long observation_update{0};
};

inline std::shared_ptr<const PosteriorSnapshot> buildPosteriorSnapshot(pf_t * pf)
{
  // Motion changes both histogram membership and weighted moments. Refresh both,
  // without changing poses, weights, sensor likelihoods or any RNG state.
  pf_recompute_stats(pf);
  auto result = std::make_shared<PosteriorSnapshot>();
  const auto & set = pf->sets[pf->current_set];
  result->sample_count = set.sample_count;
  result->observation_update = pf->diagnostics.update_count;
  result->clusters.reserve(set.cluster_count);
  for (int i = 0; i < set.cluster_count; ++i) {
    const auto & cluster = set.clusters[i];
    result->clusters.push_back({cluster.weight, cluster.mean, cluster.cov,
      cluster.angular_chart_valid != 0});
  }
  return result;
}
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__PF__POSTERIOR_SNAPSHOT_HPP_
