# GPU workspace map export and graph diagnostics

The GPU workspace builds `nav2_map_server` 1.1.20 from commit
`a097086719c88f781aa59788eca29ac6ca5e56db`, alongside the pinned SLAM Toolbox
backend. Both upstream archives and the ordered patches have checked SHA256
values. `scripts/build_slam_backend.sh` installs these into the same backend
overlay. Source that overlay before starting SLAM: its `ros2 run
nav2_map_server map_saver_cli` resolves the patched executable and `libmap_io`.
The Docker source/builder stages include both dependencies. This does not
modify the CPU workspace or system ROS packages.

Map export uses `max_digits10` for each ROS interface type: resolution is
float32, origin/yaw and thresholds are double. The native writer checks finite
metadata, dimensions, emitter errors and metadata stream flush/close. Image
encoding and occupancy thresholds are unchanged. The snapshot publisher still
stages artifacts before publishing the YAML; it now preserves and hashes the
`<prefix>.scan_geometry.json` contract emitted by SLAM. New motion/time mapping
settings require this sidecar. Existing snapshots without it retain the legacy
D0 identity interpretation; the geometry consumer checks semantic compatibility.
Snapshot provenance records the map saver source build as well as the SLAM build.

Ceres adds two startup parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `ceres_loss_scale` | `0.7` | Positive finite, dimensionless radius in **whitened** coordinate residuals. Reproduces the previous hardcoded Huber/Cauchy scale. `ceres_loss_function: None` still uses ordinary squared loss. |
| `ceres_diagnostics_path` | empty | Append JSONL events to an existing writable parent directory; empty disables event collection. An enabled stream that cannot be opened or written fails visibly. |

Neither parameter can change after startup. No correspondence threshold,
optimizer tolerance, covariance weighting or robust kernel selection is tuned
by this change. The configured residual remains source-base XY plus wrapped yaw,
whitened by the upper LLT factor on the left.

Each added factor records a monotonic edge ID, source/target scan IDs and times,
`sequential`, `running_chain`, `near_chain`, `explicit_loop` or `cross_sensor`
provenance, source-chain membership, available coarse/fine response, matching
search center, measurement and covariance coordinates. Chain IDs expose reused
observations; they are not a cross-covariance estimate. The provenance is stored
in versioned Karto `LinkInfo` serialization; historical v0 factors load as
`unknown_legacy`. This records the path that actually created a factor. A later
matching attempt that finds the same existing edge is not a new factor event.
A negative response value means that response was unavailable on that path.
A `null` search center means the original matching center was unavailable
(sequential/running-chain paths and historical factors).

Each optimization ID has complete before/after node states, the fixed anchor,
each live edge's unrobustified whitened squared norm, robust influence
`rho_prime`, robust cost, solver termination, usability, cost, step counts and
runtime. Edge removal and graph reset events are included. The JSONL `session`
event begins a new solver instance, so IDs are interpreted within its session.
The input geometry sidecar carries physical time mapping provenance.

Search boundary flags are explicitly `null`: this patch does not instrument the
matcher's full response surface, alternate peaks or rejected candidates. Search
center/response fields do not establish search coverage. Loop-trigger attribution
is available through the newly added explicit-loop factor and subsequent solve
ID, rather than inferred from edge temporal separation. No GT, posterior edge
classification, new independence assumption or parameter recommendation enters
the optimizer.

## Verification

`tests/integration/map_graph` builds two probes against prepared production
source. Select the source and installed backend with normal CMake prefix paths;
`KARTO_LIBRARY` can select a backend library from an uninstalled build. For
example, after sourcing ROS and the newly built backend overlay:

```bash
cmake -S tests/integration/map_graph -B build/map-graph-probes \
  -DSLAM_SOURCE="$PWD/.cache/slam_backend/src/slam_toolbox" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build build/map-graph-probes -j2
mkdir -p output/map-graph-probe
build/map-graph-probes/graph_diagnostics "$PWD/output/map-graph-probe"
/usr/bin/python3 tests/integration/map_graph/validate_graph_diagnostics.py \
  output/map-graph-probe
build/map-graph-probes/metadata_roundtrip --synthetic-only output/map-graph-probe/maps
```

The graph probe exercises real Ceres with a nondiagonal covariance, an injected
outlier, default versus explicit scale, four invalid scales, removals/reset and
binary provenance roundtrip. The validator independently recomputes each
coordinate residual, covariance quadratic form, kernel influence, solver cost
and fixed-gauge identity from the production JSONL.

`metadata_roundtrip GRAPH_PREFIX OUTPUT_DIRECTORY LEGACY_PGM` reconstructs a
frozen graph with native Karto, calls native `saveMapToFile` and
`loadMapFromYaml`, checks four synthetic maps, typed numeric roundtrip, physical
yaw, cell coordinates, trinary classes, invalid inputs and PGM byte equality.
`--write-only` in place of `LEGACY_PGM` writes a reference image, useful when
running the same probe with the unmodified system `libmap_io` first. Compare
identical immutable graph input and raster parameters, not two live map queries.

The 2026-10-16 validation artifacts are in
`output/drift-remediation-implementation/map-graph/`. The native old saver, new
saver and historical frozen PGM all have SHA256
`0423a7e32a01a45bf5e0699a23359e0bb83e7671c1b602fa707042977834d395`.
Five native map roundtrips passed, including the real 3,001-node graph and four
synthetic coordinate/yaw/resolution cases. Four invalid-input/write-path checks
passed. The native graph probe and independent event checks passed. The 53
Python preparation/snapshot/SLAM workflow/raster checks passed in the ROS Humble
container. This establishes contracts and instrumentation; it does not establish
a new trajectory ATE or cross-sequence accuracy improvement.
