# Native AMCL with CUDA observation scoring

This package vendors `nav2_amcl` from Nav2 Humble **1.1.20**, commit
`a097086719c88f781aa59788eca29ac6ca5e56db`. `UPSTREAM.json` retains hashes of every
original source file. The original LGPL notices are retained. Its native executable
is `ros2 run duojin02_amcl_cuda amcl`, with the same `amcl` node name and
`nav2_amcl::OmniMotionModel` / `nav2_amcl::DifferentialMotionModel` parameter values.
All library filenames and the motion plugin resource registry are private to this
package, so an installed stock AMCL does not override the new implementation.

## Execution contract

- `observation_backend`: `cpu`, `auto` (default), or `cuda`. Explicit `cuda` validates
  device availability during lifecycle configuration and raises on runtime CUDA
  errors. `auto` falls back to the native CPU observation on unavailable devices,
  a CPU-only build, or a CUDA failure; the failure is logged and the failed device
  is disabled until the laser model is reconstructed. There is no partial weight
  update before a successful CUDA result.
- `observation_cuda_min_particles`: default `0`; only `auto` uses this optional
  particle count threshold. Strict `cuda` never silently chooses a size threshold.
- `observation_log_every`: default `0` (per-laser destruction summary); set `1` to
  log every observation during an audit. All three parameters are read-only after
  node construction; restart or lifecycle recreation with new node parameters.
- `likelihood_field` and `likelihood_field_prob` execute actual particle × beam
  kernels. Probabilistic beam skipping, convergence gating, particle agreement,
  strict mask threshold, skipped-beam error fallback and log-product accumulation
  are implemented. The raycast `beam` model uses CPU in auto mode and is rejected
  by strict CUDA, including attempts to switch to it dynamically.

Host pose composition uses the upstream function. Selected beam indices, map
origin/rounding convention, unknown and off-map distances, Gaussian/random terms,
field cubic sum and probabilistic log product retain the upstream formulas.
CUDA uses double precision without FMA contraction or fast-math. One GPU thread
computes each particle/beam likelihood; a second pass accumulates each particle's
beams in source order. The final host weight multiplication and total sum retain
source order. `pf_update_sensor`, normalization, recovery averages, motion models,
KLD tree, adaptive resampling and publication remain native CPU code unchanged.

Device map distances are uploaded once per laser model, with persistent capacities
for particles, beams, likelihoods and reductions. Native map replacement and laser
parameter changes reconstruct the model and invalidate its cached map. Laser
ownership uses `unique_ptr` to release native and device memory when models are
cleared (upstream raw-pointer clear leaked those objects). Async operations drain
on both success and exceptions before host buffers expire.

GPU/CPU transcendental rounding can disagree at a cell boundary. A GPU grid
coordinate within `1e-10` of an integer triggers the original CPU observation for
that complete update; `boundary_fallbacks` reports it, even in strict CUDA mode.
This threshold is a tested numerical guard, **not a formal floating-point error
certificate for arbitrary coordinate magnitudes or resolutions**. Exact bitwise
CPU/GPU equivalence is not claimed. Invalid nonfinite range/bearing values are
ignored on both paths before indexing the map.

The probabilistic beamskip path has one explicit semantic repair: ignored and
padded beam slots are initialized to neutral likelihood `1` on every update. The
upstream source retains zero or prior-scan values in those slots, causing invalid
measurements to zero or bias weights during error fallback. Both CPU and GPU
references here apply the repair. The configured `do_beamskip=false` path keeps
its original behavior for valid input.

`AMCL_OBSERVATION_STATS` logs per-laser cumulative `cuda_updates`, `cpu_updates`,
`failures`, `map_uploads`, `boundary_fallbacks`, last particle/selected-beam counts,
`total_us` and `last_us`. Time measures the sensor likelihood callback including
host preparation/transfers/synchronization and guard fallback; it excludes native
PF normalization, resampling, other node callbacks, and map/model construction.
A model reconstruction starts a new counter lifetime. Destruction repeats the
last cumulative sample: do not sum repeated summaries.

## Build and validation

CMake 3.24+, ROS Humble dependencies, C++17. CUDA is optional. Use repository
`scripts/build_cuda_local.sh`, or configure this package with
`-DDUOJIN_ENABLE_CUDA=ON -DDUOJIN_CUDA_REQUIRED=ON` and an explicit
`-DCMAKE_CUDA_COMPILER=/opt/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120` for the
RTX 5070 validation machine. A CPU build uses `-DDUOJIN_ENABLE_CUDA=OFF` and never
checks for or links a CUDA toolkit. Architecture `native` is the CUDA default;
compiling for another architecture is not evidence of testing that device.

Enable `-DBUILD_TESTING=ON`; `ctest --test-dir <build> --output-on-failure` includes
CPU reference, unavailable-device auto, strict refusal and actual CUDA differential
cases (CUDA absence is skip code 77, not a device pass). The native PF tests cover
both likelihood models, full normalized weights, equal seeded KLD resampling,
converged/nonconverged beam skipping, skipped-beam error fallback, invalid/short
and padded scans, nonzero map origins, and guarded exact map boundaries. The
repository `tests/integration/cuda_amcl_replay_probe.py` exercises installed nodes
with a synthetic map/odometry/scan stream and saves poses, covariance, library
maps and logs. These fixtures are component and synthetic replay evidence;
physical localization accuracy and Jetson timing require their own trials.
