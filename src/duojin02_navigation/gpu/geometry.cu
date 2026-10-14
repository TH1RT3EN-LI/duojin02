// Geometry kernel extracted from duojin02-gpu; sampling kernels are omitted.
#include "geometry_internal.hpp"
#include <cuda_runtime.h>
#include <cfloat>
#include <limits>

namespace duojin02_navigation {
namespace {
void check(cudaError_t error, const char *operation) {
  if (error != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(error));
}
template<class T> struct Buffer {
  T *data{nullptr};
  size_t capacity{0};
  ~Buffer() { if (data) cudaFree(data); }
  void reserve(size_t n) {
    if (n <= capacity) return;
    if (n > std::numeric_limits<size_t>::max() / sizeof(T))
      throw std::runtime_error("CUDA geometry allocation size overflow");
    T *next = nullptr;
    check(cudaMalloc(reinterpret_cast<void **>(&next), n * sizeof(T)), "cudaMalloc");
    if (data) {
      const auto error = cudaFree(data);
      if (error != cudaSuccess) { cudaFree(next); check(error, "cudaFree"); }
    }
    data = next;
    capacity = n;
  }
};
struct Bounds { double min_x, min_y, max_x, max_y; };
__device__ double segmentDistance(double x, double y, const Point &a, const Point &b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double t = fmin(1., fmax(0., ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)));
  return hypot(x - a.x - t * dx, y - a.y - t * dy);
}
__global__ void geometryKernel(const Point *polygon, size_t vertices, Bounds bounds,
    const Point *points, size_t count, const PoseFrame *poses, size_t tiles,
    double margin, DeviceGeometryResult *results) {
  const size_t pose_id = blockIdx.x / tiles;
  const size_t point_id = (blockIdx.x % tiles) * blockDim.x + threadIdx.x;
  unsigned int hit = 0, uncertain = 0;
  if (point_id < count) {
    const auto &pose = poses[pose_id];
    const auto &p = points[point_id];
    const double dx = p.x - pose.x, dy = p.y - pose.y;
    const double x = pose.c * dx + pose.s * dy, y = -pose.s * dx + pose.c * dy;
    const double padding = margin + p.radius;
    // Preserve the host oracle at contacts and at scales where an absolute
    // tolerance alone cannot bound host/device rounding. World-frame cosine
    // and sine are computed on the host, exactly as in collision().
    const double scale = fmax(1., fmax(fmax(fabs(p.x), fabs(p.y)),
      fmax(fmax(fabs(pose.x), fabs(pose.y)), fmax(fabs(x), fmax(fabs(y), fabs(padding))))));
    const double tolerance = fmax(1e-10, 64 * DBL_EPSILON * scale);
    if (scale > 1e6 || !isfinite(x) || !isfinite(y) || !isfinite(padding)) uncertain = 1;
    if (x >= bounds.min_x - padding && x <= bounds.max_x + padding &&
        y >= bounds.min_y - padding && y <= bounds.max_y + padding) {
      bool inside = true;
      double distance = 1e100;
      for (size_t i = 0; i < vertices; ++i) {
        const auto &a = polygon[i], &b = polygon[(i + 1) % vertices];
        const double cross = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
        if (cross < -1e-10) inside = false;
        if (fabs(cross + 1e-10) <= fmax(1e-12, tolerance * scale)) uncertain = 1;
        distance = fmin(distance, segmentDistance(x, y, a, b));
      }
      hit = inside || distance <= padding;
      if (fabs(distance - padding) <= tolerance || !isfinite(distance)) uncertain = 1;
    }
    if (fabs(x - bounds.min_x + padding) <= tolerance ||
        fabs(x - bounds.max_x - padding) <= tolerance ||
        fabs(y - bounds.min_y + padding) <= tolerance ||
        fabs(y - bounds.max_y - padding) <= tolerance) uncertain = 1;
  }
  __shared__ unsigned int flags[128];
  flags[threadIdx.x] = hit | (uncertain << 1);
  __syncthreads();
  for (unsigned int stride = 64; stride; stride >>= 1) {
    if (threadIdx.x < stride) flags[threadIdx.x] |= flags[threadIdx.x + stride];
    __syncthreads();
  }
  if (threadIdx.x == 0) {
    if (flags[0] & 1) atomicOr(&results[pose_id].collision, 1);
    if (flags[0] & 2) atomicOr(&results[pose_id].uncertain, 1);
  }
}
class Executor final : public CudaExecutor {
  cudaStream_t stream_{nullptr};
  Buffer<Point> polygon_, points_;
  Buffer<PoseFrame> poses_;
  Buffer<DeviceGeometryResult> results_;
  uint64_t epoch_{0};
  std::string name_;
  bool failed_{false};
 public:
  Executor() {
    try {
      int device = 0;
      cudaDeviceProp properties{};
      check(cudaGetDevice(&device), "cudaGetDevice");
      check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
      name_ = properties.name;
      check(cudaFree(nullptr), "CUDA context initialization");
      // A driver/device may exist even when this build has no compatible
      // kernel image. Reject it during startup so the original CPU path runs.
      cudaFuncAttributes attributes{};
      check(cudaFuncGetAttributes(&attributes, geometryKernel), "CUDA geometry kernel compatibility");
      check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    } catch (...) {
      if (stream_) cudaStreamDestroy(stream_);
      throw;
    }
  }
  ~Executor() override {
    if (stream_) cudaStreamSynchronize(stream_);
    if (stream_) cudaStreamDestroy(stream_);
  }
  std::string name() const override { return name_; }
  std::vector<DeviceGeometryResult> geometry(const Polygon &polygon, const std::vector<Point> &points,
      uint64_t epoch, const std::vector<PoseFrame> &poses, double margin) override {
    if (failed_) throw std::runtime_error("CUDA geometry fault latched; reconfigure required");
    try {
      polygon_.reserve(polygon.size());
      points_.reserve(points.size());
      poses_.reserve(poses.size());
      results_.reserve(poses.size());
      if (epoch_ != epoch) {
        check(cudaMemcpyAsync(polygon_.data, polygon.data(), polygon.size() * sizeof(Point),
          cudaMemcpyHostToDevice, stream_), "CUDA polygon upload");
        check(cudaMemcpyAsync(points_.data, points.data(), points.size() * sizeof(Point),
          cudaMemcpyHostToDevice, stream_), "CUDA obstacle upload");
        epoch_ = epoch;
      }
      check(cudaMemcpyAsync(poses_.data, poses.data(), poses.size() * sizeof(PoseFrame),
        cudaMemcpyHostToDevice, stream_), "CUDA pose upload");
      check(cudaMemsetAsync(results_.data, 0, poses.size() * sizeof(DeviceGeometryResult), stream_), "CUDA result initialization");
      Bounds bounds{1e100, 1e100, -1e100, -1e100};
      for (const auto &p : polygon) {
        bounds.min_x = std::min(bounds.min_x, p.x); bounds.min_y = std::min(bounds.min_y, p.y);
        bounds.max_x = std::max(bounds.max_x, p.x); bounds.max_y = std::max(bounds.max_y, p.y);
      }
      const size_t tiles = (points.size() + 127) / 128;
      if (!tiles || poses.size() > 2147483647ULL / tiles)
        throw std::runtime_error("CUDA geometry batch exceeds device grid limit");
      geometryKernel<<<static_cast<unsigned int>(tiles * poses.size()), 128, 0, stream_>>>(
        polygon_.data, polygon.size(), bounds, points_.data, points.size(), poses_.data, tiles, margin, results_.data);
      check(cudaGetLastError(), "CUDA kernel launch");
      check(cudaStreamSynchronize(stream_), "CUDA geometry completion");
      std::vector<DeviceGeometryResult> result(poses.size());
      check(cudaMemcpy(result.data(), results_.data, result.size() * sizeof(DeviceGeometryResult),
        cudaMemcpyDeviceToHost), "CUDA geometry readback");
      return result;
    } catch (...) { failed_ = true; throw; }
  }
};
}
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason) {
  int count = 0;
  const auto error = cudaGetDeviceCount(&count);
  if (error != cudaSuccess || count == 0) {
    reason = error == cudaSuccess ? "no CUDA device" : cudaGetErrorString(error);
    return {};
  }
  try { return std::make_unique<Executor>(); }
  catch (const std::exception &e) { reason = e.what(); return {}; }
}
}
