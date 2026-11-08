# AMCL P4/P5 implementation contract

These changes live only in `duojin02_amcl_cuda`, including its host particle-filter
and native fallback/oracle. They implement numerical contracts and diagnostics;
they do not establish a new localization ATE or calibrate a new noise model.

- The probabilistic likelihood-field callback and CUDA combiner return log
  likelihoods. The PF combines each with the log prior and normalizes by
  log-sum-exp, committing only after every weight is valid. Zero prior stays zero.
  NaN, positive infinity and all-negative-infinity observations retain the motion
  prior and print `AMCL_SENSOR_REJECTED`. `no_information` is a separate status for
  a scan with no included valid beams. Rejected/no-information updates do not
  train recovery averages.
- Recovery exponential averages are stored in log space, retaining the legacy
  averaging definition (`logZ - log N`) and alpha endpoints. `w_slow/w_fast` remain
  derived compatibility fields; their possible underflow does not control recovery.
- `likelihood_temperature_beta` defaults to 1; finite `0 < beta <= 1` is required.
  It scales log likelihood alone, preserving the prior and selected beam indices.
  The other observation models do not use this temperature.
- KLD still uses independent multinomial draws, the single-bin maximum-sample
  branch, and the legacy strict `sample_count > limit` boundary. `pf_z` retains its
  numerical-standard-normal-quantile meaning. An explicit confidence probability
  can be converted by `pf_kld_probability_to_z`; parameter conflict handling lives
  in the node. ESS gating is optional and defaults to disabled. The CDF is scaled
  by its actual mass so trailing zero-weight particles receive no artificial
  floating-point tail interval.
- PF diagnostics retain normalization status, log evidence/range, ESS, pre/post
  resampling N, occupied bins, second-bin discovery index, raw and clipped KLD
  limit, stop reason, immediate unique parents, inherited duplicate fraction,
  persistent distinct lineages, and random recovery samples. A raw limit of
  infinity is intentional for the undefined single-bin KLD expression.
- `pf_recompute_stats` rebuilds the KD-tree from current particles/weights before
  clustering when a node skips resampling. Means and complete 3x3 covariance use
  the same normalized cluster and local unwrapped angular chart, with centered
  second moments to avoid cancellation at large coordinates. Chart spans of pi
  or more are explicitly invalid; node publication must reject that chart.
- Observation diagnostics report raw/selected/valid/included beam counts, beta,
  log-likelihood range, and distance p50/p90/p99/random-floor fraction for the
  highest posterior-weight candidate. The residual scope is explicit; these
  distances are neither GT error nor a calibrated covariance, and their local
  map dependence remains. Quantiles use linear interpolation. A random-floor
  beam means its hit mixture term is no greater than its random mixture term.
- Prepared `LaserData::endpoints` holds checked endpoints in the reference base.
  `measurement_states` retains Invalid/Hit/NoReturn; `selection_reasons` separately
  records sensor-invalid, no-return and algorithm min/max gates. Both native
  likelihood-field paths and CUDA consume this selection. Invalid measurements
  have no consumable geometry in the shared representation; only a state-guarded
  finite placeholder crosses the contiguous native ABI. Raw ranges remain audit
  data and cannot silently recreate a rigid scan.
- Each update records the prior ESS and maximum weight before observation, the
  posterior maximum weight/entropy before resampling, largest cluster mass and
  an explicit single-parent/point-mass diversity status. ESS after resampling is
  still reported separately. These are diagnostics; no artificial covariance
  floor, extra particles or likelihood temperature change conceals collapse.
- Beam diagnostics retain original effective indices and exclusion reasons.
  The v2 geometry contract declares its sensor no-return policy and explicitly
  marks clock and acquisition order as uncalibrated assumptions, even when a
  caller supplies a custom mapping label/source. A compensated v1 map requires
  rebuilding; only the historical rigid contract is explicitly compatible.

`amcl_pf_numerics` exercises extreme log weights, atomic NaN/+infinity/all-zero
support rejection, zero priors, no-information status, recovery below linear
underflow, ESS validation, complete covariance and cluster normalization, branch
cuts/wide chart rejection, no-resampling KD-tree rebuilding, CDF zero tails,
lineage counting, strict KLD stopping, and explicit confidence quantiles.
`amcl_*_observation` exercises native/CUDA agreement with beam skipping, fallback,
nonuniform/zero priors, more than 4,000 included beams, beta 1 and 0.5, deskewed
base endpoints and raw maximum-range filtering. Existing timing and strict-device
checks remain enabled. Build the package with CUDA and run its CTest targets.

The native CPU/CUDA tests also load the real source scan
`scan_front_link:1733375860478977632` and its original TF support, with CDR hashes
in `tests/integration/fixtures/lilocbench_geometry_scan.json`. This 811-beam scan
contains four recorded invalid finite short ranges. First/middle/last reference
times preserve world endpoints and ray origins, and the same 128 physical pose
candidates are mapped into each reference before native likelihood scoring.
The component map is constructed for this invariance check; this is not an ATE
experiment or an independent timing calibration.
