# Duojin MPPI CUDA optimizer

This is an isolated source fork of Navigation2 Humble **1.1.20**, commit
`a097086719c88f781aa59788eca29ac6ca5e56db`. Every original package blob was
checked against that commit; `UPSTREAM.json` records the original Git blob and
SHA-256 hashes. Namespaces, plugin registration and shared-library names are
renamed, so the system Nav2 MPPI library is neither overwritten nor interposed.
Original source copyright notices and the upstream license are retained.

`duojin02_navigation::PreciseMPPI` now inherits this controller. Its direct
tracking and docking branches can bypass MPPI; those branches' geometry counters
are separate from actual MPPI optimizer device counters.

## Execution contract

`FollowPath.optimizer_backend` is a read-only startup parameter:

- `cpu`: source-pinned CPU optimizer and critics.
- `auto`: select CUDA at configure when available; otherwise select CPU.
- `cuda`: require an available CUDA backend; configure fails otherwise.

A runtime CUDA failure does **not** switch to CPU. The optimizer latches the fault
until lifecycle reconfigure. PreciseMPPI returns a zero command and latches its
own stop. Device counters advance after successful synchronization/readback,
not merely after a kernel launch. `optimizerStatistics()` and the
`MPPI_CORE_SUMMARY` cleanup log expose real counts. The CPU core has independent
rollout/update counters. The launch-level `use_gpu=false` selects `cpu`; direct
controller callers can set `optimizer_backend` independently of geometry.

`FollowPath.optimizer_seed` is also read-only:

- `-1` (library default): preserve the upstream Gaussian generator on the CPU;
  CUDA consumes the same host-generated noise arrays.
- A nonnegative integer: use a scheduling-independent counter generator with
  Gaussian Box–Muller conversion. CPU and CUDA share the integer stream. This
  is a reproducible Gaussian sampling mode, not the original xtensor RNG stream.
  The shipped robot configuration uses seed `42`.

The upstream `regenerate_noises` parameter controls whether the deterministic
noise epoch advances every optimizer iteration. With `false`, each iteration
reuses the same fixed noise realization, as the upstream default does. Reset
returns the deterministic epoch to zero. Floating-point CPU/GPU results are
compared with explicit tolerances, not claimed bitwise identical.

## GPU coverage

CUDA executes Gaussian generation (nonnegative seed), sampling around the
nominal sequence, one-step velocity propagation, XY/yaw trajectory integration,
control regularization, normalized softmax, weighted control update, velocity
clipping and Ackermann curvature constraints. Five critics execute their batch
scoring on CUDA: Constraint, Goal, GoalAngle, Twirling and PathFollow. Their
original enable/proximity checks and PathFollow's map/path target selection stay
on the CPU.

CostCritic's footprint/map semantics and PathAlignCritic remain the source-pinned
CPU implementations. Other unmodified critics also remain CPU. Candidate state
and trajectory arrays are copied to host for these critics and visualization;
this is a hybrid optimizer, not a fully GPU-resident Nav2 controller. No claim of
end-to-end speedup follows from device kernel counters alone.

CPU and CUDA apply a speed limit after smoothing and preserve it across
periodic/fallback resets. Invalid limits stop the nominal controls. Fallback
retry counters are per optimizer instance, correcting the upstream function's
shared static counter. The native PathAlign nearest-index helper is bounded at
its final segment and preserves a supplied starting index; this fixes an upstream
`end()` dereference demonstrated by AddressSanitizer. Empty segment ranges are
skipped by PathAlign, and an empty direct helper query is rejected. This correction
is shared by the CPU and CUDA arms. Runtime CUDA/nonfinite softmax failures do not replace
valid nominal controls with invalid device output.

## Build and verification

The repository's `scripts/build_cuda_local.sh` builds this package with the rest
of the navigation stack. `DUOJIN_ENABLE_CUDA=OFF` builds the complete CPU backend;
`DUOJIN_CUDA_REQUIRED=ON` requires a CUDA compiler and makes GPU checks mandatory.
The CPU controller and critics retain the upstream compiler-supported x86 SIMD
flags (SSE4.2/AVX2/FMA), with fast math disabled and contraction off. Timings for
this CPU reference are not timings for the system-installed stock Nav2 binary.
The CUDA target honors `CMAKE_CUDA_ARCHITECTURES`; local RTX 5070 verification uses
`120`. CUDA fast math is disabled; FTZ is disabled and fused multiply-add is
disabled for the migrated kernels.

With `BUILD_TESTING=ON`, `mppi_core_checks` compares fixed-noise trajectories and
control updates against the source-pinned xtensor CPU optimizer for DiffDrive
and Omni across non-power-of-two batch sizes. It also checks device RNG agreement,
five critic formulas, limits/reset and rejection of nonfinite costs. Public
Nav2 controller API replay is a separate repository integration probe. Run tests
after installing the current build, or ensure its libraries precede older
overlays in `LD_LIBRARY_PATH`.
