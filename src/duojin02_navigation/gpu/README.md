# Geometry backend provenance

The geometry interface and CUDA collision kernel are extracted from the
`duojin02-gpu` GeometryCompute implementation at commit
`38be60d5a1fa29f805b829653888dc0382b10194`. The same geometry implementation is
present in the adopted Orin v4 frozen source archive
`joint-v4-source.tar.gz`, SHA-256
`3863106963f2ce5e8898f7c5ab5e7d19e1f4e3ec873162d7fb64f1625b423b42`.

This extraction omits sampling, trajectory costs, weight reduction, EDT,
study optimizations and HIL instrumentation. It evaluates the existing CPU
filled-footprint collision predicate in batches. Floating-point contacts and
large-coordinate cases use the original CPU predicate as the final oracle.
Clearance scores and candidate ordering remain on the CPU.

`FollowPath.use_gpu` is the sole runtime choice. CUDA is optional at build time;
`DUOJIN_ENABLE_CUDA=OFF` forces a build with no CUDA dependency. A missing device
or incompatible kernel at startup falls back to the original CPU path. A CUDA
runtime fault stops the current command and latches the device executor until
lifecycle reconfiguration. The geometry deadline is a soft deadline: it cannot
interrupt a blocked driver operation and does not add a deadline to the native
CPU optimizer.

The component can be built independently of ROS:

```bash
cmake -S src/duojin02_navigation/gpu -B /tmp/duojin02-geometry \
  -DBUILD_TESTING=ON -DDUOJIN_ENABLE_CUDA=OFF
cmake --build /tmp/duojin02-geometry
ctest --test-dir /tmp/duojin02-geometry --output-on-failure
```

For CUDA builds select architectures supported by the local toolkit, such as
`-DCMAKE_CUDA_ARCHITECTURES=87` on Orin or `120` on an RTX 50-series host.
CUDA CTest cases return a skip when no compatible device is available; inspect
the result before claiming device validation.
