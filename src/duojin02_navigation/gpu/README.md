# Local CUDA geometry backend

The geometry interface and CUDA collision kernel are extracted from the
`duojin02-gpu` GeometryCompute implementation at commit
`38be60d5a1fa29f805b829653888dc0382b10194`. The same geometry implementation is
present in the adopted Orin v4 frozen source archive
`joint-v4-source.tar.gz`, SHA-256
`3863106963f2ce5e8898f7c5ab5e7d19e1f4e3ec873162d7fb64f1625b423b42`.

The local CUDA v2 implementation adds owned, content-versioned scenes and a
resident uniform-grid index shared by the CPU and CUDA implementations. It
evaluates collision and signed capped clearance in batches. Queries can carry
the original whole-sweep world bounds as well as individual margins, preserving
both broad-phase and local footprint gates. Floating-point contacts and large
coordinates use the original CPU predicates for rechecks. Clearance ranking
intervals that overlap cause the controller to recompute the small group on CPU.

The controller batches centering candidates and later braking scales, retaining
the measured-speed and first-scale CPU early exits. Every selected later scale
passes the original CPU braking oracle before admission. Sampling, native Nav2
trajectory critics, MPPI weight reduction, AMCL, SLAM and EDT remain outside this
backend. This is local desktop validation; no Jetson result is claimed.

`FollowPath.use_gpu` is the sole runtime choice. CUDA is optional at build time;
`DUOJIN_ENABLE_CUDA=OFF` forces a build with no CUDA dependency. A missing device
or incompatible kernel at startup falls back to the original CPU path. A CUDA
runtime fault stops the current command and latches the device executor until
lifecycle reconfiguration. Compute duration is diagnostic only. No soft deadline
parks the robot; loss of controller output is covered by the shared command
chain, for both CPU and CUDA.

The component can be built independently of ROS:

```bash
cmake -S src/duojin02_navigation/gpu -B /tmp/duojin02-geometry \
  -DBUILD_TESTING=ON -DDUOJIN_ENABLE_CUDA=OFF
cmake --build /tmp/duojin02-geometry
ctest --test-dir /tmp/duojin02-geometry --output-on-failure
```

For this local version, CUDA builds default to `native` with CMake 3.24 or later;
use `-DCMAKE_CUDA_ARCHITECTURES=120` explicitly for the RTX 5070 or a build without
device visibility. `DUOJIN_CUDA_REQUIRED=ON` requires a CUDA compiler and makes
unavailable-device test results fail. Optional builds still support skip code 77;
skips do not establish device validation. The reproducible local validation entry
is `python3 scripts/validate_cuda_local.py --architecture 120 --samples 40`.
