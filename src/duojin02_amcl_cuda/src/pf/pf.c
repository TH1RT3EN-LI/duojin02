/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <float.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

#include "nav2_amcl/portable_utils.hpp"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t * pf, int k);
static double pf_resample_unclipped_limit(pf_t * pf, int k);
static int pf_compare_lineage(const void * a, const void * b)
{
  unsigned long long x = *(const unsigned long long *)a;
  unsigned long long y = *(const unsigned long long *)b;
  return (x > y) - (x < y);
}


// Create a new filter
pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn)
{
  int i, j;
  pf_t * pf;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  srand48(time(NULL));

  pf = calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;

  pf->current_set = 0;
  for (j = 0; j < 2; j++) {
    set = pf->sets + j;

    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));
    set->log_likelihoods = calloc(max_samples, sizeof(double));
    set->sensor_prior_weights = calloc(max_samples, sizeof(double));

    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
      sample->lineage = ++pf->next_lineage;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;
  pf->log_w_slow = pf->log_w_fast = -INFINITY;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t * pf)
{
  int i;

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
    free(pf->sets[i].log_likelihoods);
    free(pf->sets[i].sensor_prior_weights);
  }
  free(pf);
}

// Initialize the filter using a guassian
void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  pf_pdf_gaussian_t * pdf;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  pdf = pf_pdf_gaussian_alloc(mean, cov);

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->lineage = ++pf->next_lineage;
    sample->pose = pf_pdf_gaussian_sample(pdf);

    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  pf->log_w_slow = pf->log_w_fast = -INFINITY;

  pf_pdf_gaussian_free(pdf);

  pf_cluster_stats(pf, set);

  pf_init_converged(pf);
}


// Initialize the filter using some model
void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->lineage = ++pf->next_lineage;
    sample->pose = (*init_fn)(init_data);

    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  pf->log_w_slow = pf->log_w_fast = -INFINITY;

  pf_cluster_stats(pf, set);

  pf_init_converged(pf);
}

void pf_init_converged(pf_t * pf)
{
  pf_sample_set_t * set;
  set = pf->sets + pf->current_set;
  set->converged = 0;
  pf->converged = 0;
}

int pf_update_converged(pf_t * pf)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
      fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold)
    {
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  set->converged = 1;
  pf->converged = 1;
  return 1;
}


// Stable logaddexp and the legacy exponential moving average in log space.
static double pf_log_add(double a, double b)
{
  if (a == -INFINITY) {return b;}
  if (b == -INFINITY) {return a;}
  double m = fmax(a, b);
  return m + log(exp(a - m) + exp(b - m));
}
static double pf_log_average(double previous, double value, double alpha)
{
  if (previous == -INFINITY) {return value;}
  if (alpha == 0.0) {return previous;}
  if (alpha == 1.0) {return value;}
  return pf_log_add(log1p(-alpha) + previous, log(alpha) + value);
}
const char * pf_sensor_status_name(pf_sensor_status_t status)
{
  switch (status) {
    case PF_SENSOR_OK: return "ok";
    case PF_SENSOR_NO_INFORMATION: return "no_information";
    case PF_SENSOR_INVALID_WEIGHT: return "invalid_weight";
    case PF_SENSOR_ZERO_SUPPORT: return "zero_support";
    default: return "unknown";
  }
}
const char * pf_resample_stop_name(pf_resample_stop_t reason)
{
  switch (reason) {
    case PF_RESAMPLE_SINGLE_BIN_CAP: return "single_bin_cap";
    case PF_RESAMPLE_KLD_LIMIT: return "kld_limit";
    case PF_RESAMPLE_MAX_CAP: return "max_cap";
    default: return "none";
  }
}
double pf_kld_probability_to_z(double probability)
{
  if (!isfinite(probability) || probability <= 0.0 || probability >= 1.0) {return NAN;}
  // Invert the normal CDF, using its lower tail to avoid upper-tail cancellation.
  double p = probability <= 0.5 ? probability : 1.0 - probability;
  double lo = -40.0, hi = 0.0;
  for (int j = 0; j < 100; ++j) {
    double mid = (lo + hi) / 2.0;
    if (0.5 * erfc(-mid / sqrt(2.0)) < p) {lo = mid;} else {hi = mid;}
  }
  double z = (lo + hi) / 2.0;
  return probability <= 0.5 ? z : -z;
}
int pf_should_resample(const pf_t * pf, double ratio)
{
  if (!isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {return -1;}
  return ratio == 0.0 || pf->diagnostics.ess < ratio * pf->sets[pf->current_set].sample_count;
}

// Update sensor weights only after all log weights have been validated. A rejected
// observation retains the prior and does not train the recovery averages.
void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data)
{
  pf_sample_set_t * set = pf->sets + pf->current_set;
  const int n = set->sample_count;
  if (n <= 0) {return;}
  double * prior = set->sensor_prior_weights;
  double * logs = set->log_likelihoods;
  assert(prior && logs);
  pf_sensor_status_t status = PF_SENSOR_OK;
  double maximum = -INFINITY, minimum = INFINITY;
  double prior_sum = 0.0, prior_square_sum = 0.0, prior_max = 0.0;
  for (int i = 0; i < n; ++i) {
    prior[i] = set->samples[i].weight;
    if (!isfinite(prior[i]) || prior[i] < 0.0) {status = PF_SENSOR_INVALID_WEIGHT;}
    prior_sum += prior[i]; prior_square_sum += prior[i] * prior[i];
    prior_max = fmax(prior_max, prior[i]);
  }
  pf->diagnostics.observation_input_count = n;
  pf->diagnostics.prior_ess = prior_square_sum > 0.0 ?
    prior_sum * prior_sum / prior_square_sum : 0.0;
  pf->diagnostics.prior_max_weight = prior_sum > 0.0 ? prior_max / prior_sum : 0.0;
  set->log_likelihoods_ready = 0;
  set->sensor_no_information = 0;
  if (status == PF_SENSOR_OK) {
    (void)(*sensor_fn)(sensor_data, set);
    for (int i = 0; i < n; ++i) {
      if (set->log_likelihoods_ready) {
        double likelihood = set->log_likelihoods[i];
        if (isnan(likelihood) || likelihood == INFINITY) {status = PF_SENSOR_INVALID_WEIGHT;}
        logs[i] = prior[i] == 0.0 ? -INFINITY : log(prior[i]) + likelihood;
      } else {
        double weight = set->samples[i].weight;
        if (!isfinite(weight) || weight < 0.0) {status = PF_SENSOR_INVALID_WEIGHT;}
        logs[i] = weight == 0.0 ? -INFINITY : log(weight);
      }
      if (isnan(logs[i]) || logs[i] == INFINITY) {status = PF_SENSOR_INVALID_WEIGHT;}
      maximum = fmax(maximum, logs[i]);
      minimum = fmin(minimum, logs[i]);
    }
  }
  if (status == PF_SENSOR_OK && maximum == -INFINITY) {status = PF_SENSOR_ZERO_SUPPORT;}
  double log_evidence = NAN;
  if (status == PF_SENSOR_OK) {
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {sum += exp(logs[i] - maximum);}
    log_evidence = maximum + log(sum);
    for (int i = 0; i < n; ++i) {set->samples[i].weight = exp(logs[i] - maximum) / sum;}
    if (set->sensor_no_information) {
      status = PF_SENSOR_NO_INFORMATION;
    } else {
      const double log_average = log_evidence - log((double)n);
      pf->log_w_slow = pf_log_average(pf->log_w_slow, log_average, pf->alpha_slow);
      pf->log_w_fast = pf_log_average(pf->log_w_fast, log_average, pf->alpha_fast);
      // Compatibility/debug fields only; recovery uses the stable log values.
      pf->w_slow = exp(pf->log_w_slow);
      pf->w_fast = exp(pf->log_w_fast);
    }
  } else {
    for (int i = 0; i < n; ++i) {set->samples[i].weight = prior[i];}
    ++pf->diagnostics.rejected_updates;
    fprintf(stderr, "AMCL_SENSOR_REJECTED update=%llu status=%s prior_preserved=1\n",
      pf->diagnostics.update_count + 1, pf_sensor_status_name(status));
  }
  double sum = 0.0, square_sum = 0.0, posterior_max = 0.0, entropy = 0.0;
  for (int i = 0; i < n; ++i) {
    double w = set->samples[i].weight;
    sum += w; square_sum += w * w;
    posterior_max = fmax(posterior_max, w);
    if (w > 0.0) {entropy -= w * log(w);}
  }
  ++pf->diagnostics.update_count;
  pf->diagnostics.status = status;
  pf->diagnostics.log_evidence = log_evidence;
  pf->diagnostics.log_weight_min = minimum;
  pf->diagnostics.log_weight_max = maximum;
  pf->diagnostics.ess = square_sum > 0.0 ? sum * sum / square_sum : 0.0;
  pf->diagnostics.posterior_max_weight = sum > 0.0 ? posterior_max / sum : 0.0;
  pf->diagnostics.posterior_entropy = sum > 0.0 ? entropy / sum + log(sum) : NAN;
}


// Resample the distribution
void pf_update_resample(pf_t * pf, void * random_pose_data)
{
  int i;
  double total;
  pf_sample_set_t * set_a, * set_b;
  pf_sample_t * sample_a, * sample_b;

  double * c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  int * parent_counts = calloc(set_a->sample_count, sizeof(int));
  unsigned long long * lineages = malloc(pf->max_samples * sizeof(unsigned long long));
  assert(c && parent_counts && lineages);
  pf_diagnostics_t * diagnostics = &pf->diagnostics;
  diagnostics->resample_input_count = set_a->sample_count;
  diagnostics->resample_input_ess = diagnostics->ess;
  diagnostics->second_bin_sample = 0;
  diagnostics->unique_parents = diagnostics->random_injections = 0;
  diagnostics->stop_reason = PF_RESAMPLE_MAX_CAP;
  int valid_cdf = 1;
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++) {
    if (!isfinite(set_a->samples[i].weight) || set_a->samples[i].weight < 0.0) {valid_cdf = 0;}
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  // Normalize the CDF itself: a trailing zero-mass sample must not acquire a
  // roundoff interval, and r in [0,1) must always have support.
  const double cdf_mass = c[set_a->sample_count];
  if (!valid_cdf || !isfinite(cdf_mass) || cdf_mass <= 0.0) {
    fprintf(stderr, "AMCL_RESAMPLE_REJECTED invalid_cdf prior_preserved=1\n");
    free(c); free(parent_counts); free(lineages); return;
  }
  for (i = 1; i <= set_a->sample_count; ++i) {c[i] /= cdf_mass;}

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  w_diff = 0.0;
  if (isfinite(pf->log_w_slow) && isfinite(pf->log_w_fast) &&
    pf->log_w_fast < pf->log_w_slow)
  {
    w_diff = -expm1(pf->log_w_fast - pf->log_w_slow);
  }

  // Multinomial sampling supports the variable sample count required by KLD.
  while (set_b->sample_count < pf->max_samples) {
    sample_b = set_b->samples + set_b->sample_count++;

    if (drand48() < w_diff) {
      sample_b->pose = (pf->random_pose_fn)(random_pose_data);
      sample_b->lineage = ++pf->next_lineage;
      ++diagnostics->random_injections;
    } else {
      // Naive discrete event sampler
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) {
          break;
        }
      }
      assert(i < set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      sample_b->pose = sample_a->pose;
      sample_b->lineage = sample_a->lineage;
      if (parent_counts[i]++ == 0) {++diagnostics->unique_parents;}
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    lineages[set_b->sample_count - 1] = sample_b->lineage;
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
    if (!diagnostics->second_bin_sample && set_b->kdtree->leaf_count >= 2) {
      diagnostics->second_bin_sample = set_b->sample_count;
    }

    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count)) {
      diagnostics->stop_reason = PF_RESAMPLE_KLD_LIMIT;
      break;
    }
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  if (w_diff > 0.0) {
    pf->w_slow = pf->w_fast = 0.0;
    pf->log_w_slow = pf->log_w_fast = -INFINITY;
  }


  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  pf_cluster_stats(pf, set_b);

  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  diagnostics->resample_output_count = set_b->sample_count;
  diagnostics->occupied_bins = set_b->kdtree->leaf_count;
  diagnostics->kld_limit = pf_resample_limit(pf, diagnostics->occupied_bins);
  diagnostics->kld_unclipped_limit = pf_resample_unclipped_limit(pf, diagnostics->occupied_bins);
  if (diagnostics->occupied_bins <= 1) {diagnostics->stop_reason = PF_RESAMPLE_SINGLE_BIN_CAP;}
  const int inherited = set_b->sample_count - diagnostics->random_injections;
  diagnostics->duplicate_fraction = inherited ? 1.0 - diagnostics->unique_parents / (double)inherited : 0.0;
  qsort(lineages, set_b->sample_count, sizeof(unsigned long long), pf_compare_lineage);
  diagnostics->distinct_lineages = set_b->sample_count ? 1 : 0;
  for (i = 1; i < set_b->sample_count; ++i) {
    if (lineages[i] != lineages[i - 1]) {++diagnostics->distinct_lineages;}
  }
  ++diagnostics->resample_count;
  // Keep diagnostics.ess tied to the last observation; the uniform output ESS
  // is exactly resample_output_count, and resample_input_ess records this event.
  free(c); free(parent_counts); free(lineages);
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
static double pf_resample_unclipped_limit(pf_t * pf, int k)
{
  if (k <= 1) {return INFINITY;}
  const double b = 2.0 / (9.0 * (k - 1));
  const double x = 1.0 - b + sqrt(b) * pf->pop_z;
  return ceil((k - 1) / (2.0 * pf->pop_err) * x * x * x);
}
int pf_resample_limit(pf_t * pf, int k)
{
  if (k <= 1) {return pf->max_samples;}
  const double n = pf_resample_unclipped_limit(pf, k);
  if (n < pf->min_samples) {return pf->min_samples;}
  if (!isfinite(n) || n > pf->max_samples) {return pf->max_samples;}
  return (int)n;
}

static double pf_wrap(double angle) {return atan2(sin(angle), cos(angle));}
void pf_recompute_stats(pf_t * pf)
{
  pf_sample_set_t * set = pf->sets + pf->current_set;
  pf_kdtree_clear(set->kdtree);
  for (int i = 0; i < set->sample_count; ++i) {
    pf_kdtree_insert(set->kdtree, set->samples[i].pose, set->samples[i].weight);
  }
  pf_cluster_stats(pf, set);
  pf_update_converged(pf);
}

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set)
{
  (void)pf;
  pf_kdtree_cluster(set->kdtree);
  set->cluster_count = 0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  double total = 0.0, cosine = 0.0, sine = 0.0;
  for (int i = 0; i < set->cluster_max_count; ++i) {
    pf_cluster_t * cluster = set->clusters + i;
    cluster->count = 0; cluster->weight = 0.0;
    cluster->mean = pf_vector_zero(); cluster->cov = pf_matrix_zero();
    for (int j = 0; j < 4; ++j) {cluster->m[j] = 0.0;}
  }
  // Choose circular anchors, then use one explicit local unwrapped chart for
  // each random variable. All three covariance coordinates use these same means.
  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    int cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0 && cidx < set->cluster_max_count);
    if (cidx + 1 > set->cluster_count) {set->cluster_count = cidx + 1;}
    pf_cluster_t * cluster = set->clusters + cidx;
    ++cluster->count;
    const double w = sample->weight;
    cluster->weight += w; total += w;
    cluster->m[0] += w * sample->pose.v[0];
    cluster->m[1] += w * sample->pose.v[1];
    cluster->m[2] += w * cos(sample->pose.v[2]);
    cluster->m[3] += w * sin(sample->pose.v[2]);
    set->mean.v[0] += w * sample->pose.v[0];
    set->mean.v[1] += w * sample->pose.v[1];
    cosine += w * cos(sample->pose.v[2]); sine += w * sin(sample->pose.v[2]);
  }
  if (total <= 0.0) {set->angular_chart_valid = 0; return;}
  const double anchor = atan2(sine, cosine);
  for (int i = 0; i < set->cluster_count; ++i) {
    pf_cluster_t * cluster = set->clusters + i;
    cluster->angular_chart_valid = cluster->weight > 0.0;
    if (cluster->weight <= 0.0) {continue;}
    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->m[0] = atan2(cluster->m[3], cluster->m[2]);
    cluster->m[1] = INFINITY; cluster->m[2] = -INFINITY;
  }
  set->mean.v[0] /= total; set->mean.v[1] /= total;
  double yaw_offset = 0.0, chart_min = INFINITY, chart_max = -INFINITY;
  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    if (sample->weight <= 0.0) {continue;}
    pf_cluster_t * cluster = set->clusters + pf_kdtree_get_cluster(set->kdtree, sample->pose);
    double offset = pf_wrap(sample->pose.v[2] - cluster->m[0]);
    cluster->mean.v[2] += sample->weight * offset / cluster->weight;
    cluster->m[1] = fmin(cluster->m[1], offset); cluster->m[2] = fmax(cluster->m[2], offset);
    offset = pf_wrap(sample->pose.v[2] - anchor);
    yaw_offset += sample->weight * offset / total;
    chart_min = fmin(chart_min, offset); chart_max = fmax(chart_max, offset);
  }
  set->mean.v[2] = pf_wrap(anchor + yaw_offset);
  set->angular_chart_valid = chart_max - chart_min < M_PI;
  for (int i = 0; i < set->cluster_count; ++i) {
    pf_cluster_t * cluster = set->clusters + i;
    cluster->mean.v[2] = pf_wrap(cluster->m[0] + cluster->mean.v[2]);
    cluster->angular_chart_valid = cluster->weight > 0.0 && cluster->m[2] - cluster->m[1] < M_PI;
  }
  // Centered second pass avoids cancellation from large world coordinates and
  // includes x-y, x-yaw, and y-yaw cross terms with normalized cluster mass.
  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    if (sample->weight <= 0.0) {continue;}
    pf_cluster_t * cluster = set->clusters + pf_kdtree_get_cluster(set->kdtree, sample->pose);
    double local[3], overall[3];
    for (int j = 0; j < 3; ++j) {
      local[j] = sample->pose.v[j] - cluster->mean.v[j];
      overall[j] = sample->pose.v[j] - set->mean.v[j];
    }
    local[2] = pf_wrap(local[2]); overall[2] = pf_wrap(overall[2]);
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        cluster->cov.m[j][k] += sample->weight / cluster->weight * local[j] * local[k];
        set->cov.m[j][k] += sample->weight / total * overall[j] * overall[k];
      }
    }
  }
}


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(
  pf_t * pf, int clabel, double * weight,
  pf_vector_t * mean, pf_matrix_t * cov)
{
  pf_sample_set_t * set;
  pf_cluster_t * cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count) {
    return 0;
  }
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}
