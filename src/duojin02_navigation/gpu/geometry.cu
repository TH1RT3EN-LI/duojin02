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

template <class T> struct Buffer {
  T *data{nullptr};
  size_t capacity{0};
  ~Buffer() { release(); }
  void release() {
    if (data) cudaFree(data);
    data = nullptr;
    capacity = 0;
  }
  void reserve(size_t n) {
    if (n <= capacity) return;
    if (n > std::numeric_limits<size_t>::max() / sizeof(T))
      throw std::runtime_error("CUDA geometry allocation size overflow");
    T *next = nullptr;
    check(cudaMalloc(reinterpret_cast<void **>(&next), n * sizeof(T)), "cudaMalloc");
    if (data) {
      const auto error = cudaFree(data);
      if (error != cudaSuccess) {
        cudaFree(next);
        check(error, "cudaFree");
      }
    }
    data = next;
    capacity = n;
  }
};

struct DeviceGrid {
  double origin_x, origin_y, cell_size, max_radius, footprint_extent;
  uint32_t width, height;
};

__device__ double segmentDistance(double x, double y, const Point &a, const Point &b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double t = fmin(1., fmax(0., ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)));
  return hypot(x - a.x - t * dx, y - a.y - t * dy);
}

__device__ bool cellRange(const PoseFrame &pose, double padding, DeviceGrid grid,
                          int &x0, int &y0, int &x1, int &y1) {
  if (!grid.width || fabs(pose.x) > 1e6 || fabs(pose.y) > 1e6)
    return false;
  const double radius = grid.footprint_extent + 2 * (padding + grid.max_radius);
  if (!isfinite(radius) || radius > 1e6)
    return false;
  const double guard = 1e-7 * (1 + fabs(pose.x) + fabs(pose.y) + radius);
  const double low_x = floor((pose.x - radius - guard - grid.origin_x) / grid.cell_size);
  const double high_x = floor((pose.x + radius + guard - grid.origin_x) / grid.cell_size);
  const double low_y = floor((pose.y - radius - guard - grid.origin_y) / grid.cell_size);
  const double high_y = floor((pose.y + radius + guard - grid.origin_y) / grid.cell_size);
  x0 = static_cast<int>(fmin(static_cast<double>(grid.width), fmax(0., low_x)));
  x1 = static_cast<int>(fmin(static_cast<double>(grid.width - 1), fmax(-1., high_x)));
  y0 = static_cast<int>(fmin(static_cast<double>(grid.height), fmax(0., low_y)));
  y1 = static_cast<int>(fmin(static_cast<double>(grid.height - 1), fmax(-1., high_y)));
  return true;
}

__device__ bool inBounds(double x, double y, GeometryBounds bounds, double padding) {
  return x >= bounds.min_x - padding && x <= bounds.max_x + padding &&
         y >= bounds.min_y - padding && y <= bounds.max_y + padding;
}

__device__ bool nearBounds(double x, double y, GeometryBounds bounds, double padding,
                           double tolerance) {
  return fabs(x - bounds.min_x + padding) <= tolerance ||
         fabs(x - bounds.max_x - padding) <= tolerance ||
         fabs(y - bounds.min_y + padding) <= tolerance ||
         fabs(y - bounds.max_y - padding) <= tolerance;
}

__device__ void evaluatePoint(const Point &p, const Point *polygon, size_t vertices,
                              GeometryBounds bounds, const PoseFrame &pose, double cap,
                              unsigned int &hit, unsigned int &uncertain, double &clearance,
                              double &error) {
  if (pose.has_world_bounds) {
    const auto bounds = pose.world_bounds;
    const double padding = pose.margin + p.radius;
    const double scale = fmax(1., fmax(fmax(fabs(p.x), fabs(p.y)),
        fmax(fmax(fabs(bounds.min_x), fabs(bounds.min_y)),
             fmax(fabs(bounds.max_x), fabs(bounds.max_y)))));
    if (scale > 1e6 || !isfinite(padding) ||
        nearBounds(p.x, p.y, bounds, padding, fmax(1e-10, 64 * DBL_EPSILON * scale)))
      uncertain = 1;
    if (!inBounds(p.x, p.y, bounds, padding))
      return;
  }
  const double dx = p.x - pose.x, dy = p.y - pose.y;
  const double x = pose.c * dx + pose.s * dy, y = -pose.s * dx + pose.c * dy;
  const double padding = pose.margin + p.radius;
  const double clearance_padding = cap + p.radius;
  const double polygon_scale = fmax(fmax(fabs(bounds.min_x), fabs(bounds.min_y)),
                                     fmax(fabs(bounds.max_x), fabs(bounds.max_y)));
  const double scale = fmax(1., fmax(polygon_scale,
      fmax(fmax(fabs(p.x), fabs(p.y)), fmax(fmax(fabs(pose.x), fabs(pose.y)),
      fmax(fabs(x), fmax(fabs(y), fmax(fabs(padding), fabs(clearance_padding))))))));
  const double tolerance = fmax(1e-10, 64 * DBL_EPSILON * scale);
  if (scale > 1e6 || !isfinite(x) || !isfinite(y) || !isfinite(padding) ||
      (cap >= 0 && !isfinite(clearance_padding)))
    uncertain = 1;
  const bool collision_gate = inBounds(x, y, bounds, padding);
  const bool clearance_gate = cap >= 0 && inBounds(x, y, bounds, clearance_padding);
  if (collision_gate || clearance_gate) {
    bool inside = true;
    double distance = 1e100;
    for (size_t i = 0; i < vertices; ++i) {
      const auto &a = polygon[i], &b = polygon[(i + 1) % vertices];
      const double cross = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
      if (cross < -1e-10)
        inside = false;
      if (!isfinite(cross) || fabs(cross + 1e-10) <= fmax(1e-12, tolerance * scale))
        uncertain = 1;
      distance = fmin(distance, segmentDistance(x, y, a, b));
    }
    // touches() initializes its distance at 1e10; footprintClearance() uses
    // 1e100. Preserve that distinction even for unusually large margins.
    if (collision_gate) {
      hit |= inside || fmin(distance, 1e10) <= padding;
      if (fabs(fmin(distance, 1e10) - padding) <= tolerance || !isfinite(distance))
        uncertain = 1;
    }
    if (clearance_gate) {
      const double value = (inside ? -distance : distance) - p.radius;
      clearance = fmin(clearance, value);
      error = fmax(error, fmax(1e-12, 1024 * DBL_EPSILON * scale * fmax(1., polygon_scale)));
      if (!isfinite(value) || fabs(value - cap) <= tolerance)
        uncertain = 1;
    }
  }
  if (nearBounds(x, y, bounds, padding, tolerance) ||
      (cap >= 0 && nearBounds(x, y, bounds, clearance_padding, tolerance)))
    uncertain = 1;
}

// A block owns one pose. Its threads visit only neighbouring resident cells;
// no per-query obstacle list or query-by-obstacle-sized launch is transferred.
__global__ void geometryKernel(const Point *polygon, size_t vertices, GeometryBounds bounds,
                               const Point *points, size_t count, const uint64_t *offsets,
                               DeviceGrid grid, const PoseFrame *poses, double cap,
                               DeviceGeometryResult *results) {
  const auto pose = poses[blockIdx.x];
  unsigned int hit = 0, uncertain = 0;
  double clearance = cap, error = 0;
  int x0, y0, x1, y1;
  if (cellRange(pose, fmax(pose.margin, cap), grid, x0, y0, x1, y1)) {
    if (x0 <= x1)
      for (int y = y0; y <= y1; ++y) {
        const size_t cell = static_cast<size_t>(y) * grid.width;
        const size_t begin = offsets[cell + x0], end = offsets[cell + x1 + 1];
        for (size_t i = begin + threadIdx.x; i < end; i += blockDim.x)
          evaluatePoint(points[i], polygon, vertices, bounds, pose, cap,
                         hit, uncertain, clearance, error);
      }
  } else {
    for (size_t i = threadIdx.x; i < count; i += blockDim.x)
      evaluatePoint(points[i], polygon, vertices, bounds, pose, cap,
                     hit, uncertain, clearance, error);
  }
  __shared__ unsigned int flags[128];
  __shared__ double clearances[128], errors[128];
  flags[threadIdx.x] = hit | (uncertain << 1);
  clearances[threadIdx.x] = clearance;
  errors[threadIdx.x] = error;
  __syncthreads();
  for (unsigned int stride = 64; stride; stride >>= 1) {
    if (threadIdx.x < stride) {
      flags[threadIdx.x] |= flags[threadIdx.x + stride];
      clearances[threadIdx.x] = fmin(clearances[threadIdx.x], clearances[threadIdx.x + stride]);
      errors[threadIdx.x] = fmax(errors[threadIdx.x], errors[threadIdx.x + stride]);
    }
    __syncthreads();
  }
  if (threadIdx.x == 0)
    results[blockIdx.x] = {flags[0] & 1, (flags[0] >> 1) & 1, clearances[0], errors[0]};
}

class Executor final : public CudaExecutor {
  cudaStream_t stream_{nullptr};
  Buffer<Point> polygon_, points_;
  Buffer<uint64_t> offsets_;
  Buffer<PoseFrame> poses_;
  Buffer<DeviceGeometryResult> results_;
  uint64_t epoch_{0};
  std::string name_;
  bool failed_{false};
  int device_{0};

public:
  Executor() {
    try {
      cudaDeviceProp properties{};
      check(cudaGetDevice(&device_), "cudaGetDevice");
      check(cudaGetDeviceProperties(&properties, device_), "cudaGetDeviceProperties");
      name_ = properties.name;
      check(cudaFree(nullptr), "CUDA context initialization");
      cudaFuncAttributes attributes{};
      check(cudaFuncGetAttributes(&attributes, geometryKernel), "CUDA geometry kernel compatibility");
      check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    } catch (...) {
      if (stream_) cudaStreamDestroy(stream_);
      throw;
    }
  }
  ~Executor() override {
    // Each executor frees resources on the device that owns its stream/buffers.
    int previous = device_;
    cudaGetDevice(&previous);
    cudaSetDevice(device_);
    if (stream_) cudaStreamSynchronize(stream_);
    if (stream_) cudaStreamDestroy(stream_);
    polygon_.release();
    points_.release();
    offsets_.release();
    poses_.release();
    results_.release();
    cudaSetDevice(previous);
  }
  std::string name() const override { return name_; }

  DeviceGeometryBatch geometry(const Polygon &polygon, GeometryBounds bounds,
                               const std::vector<Point> &points, const GeometryGrid &grid,
                               uint64_t epoch, const std::vector<PoseFrame> &poses,
                               double cap) override {
    if (failed_)
      throw std::runtime_error("CUDA geometry fault latched; reconfigure required");
    try {
      check(cudaSetDevice(device_), "CUDA geometry owning device");
      polygon_.reserve(polygon.size());
      points_.reserve(points.size());
      offsets_.reserve(grid.offsets.size());
      poses_.reserve(poses.size());
      results_.reserve(poses.size());
      DeviceGeometryBatch batch;
      if (epoch_ != epoch) {
        check(cudaMemcpyAsync(polygon_.data, polygon.data(), polygon.size() * sizeof(Point),
                              cudaMemcpyHostToDevice, stream_), "CUDA polygon upload");
        check(cudaMemcpyAsync(points_.data, points.data(), points.size() * sizeof(Point),
                              cudaMemcpyHostToDevice, stream_), "CUDA obstacle upload");
        if (!grid.offsets.empty())
          check(cudaMemcpyAsync(offsets_.data, grid.offsets.data(), grid.offsets.size() * sizeof(uint64_t),
                                cudaMemcpyHostToDevice, stream_), "CUDA spatial index upload");
        batch.scene_uploaded = true;
        batch.uploaded_bytes = (polygon.size() + points.size()) * sizeof(Point) +
                               grid.offsets.size() * sizeof(uint64_t);
      }
      check(cudaMemcpyAsync(poses_.data, poses.data(), poses.size() * sizeof(PoseFrame),
                            cudaMemcpyHostToDevice, stream_), "CUDA pose upload");
      if (poses.size() > 2147483647ULL)
        throw std::runtime_error("CUDA geometry batch exceeds device grid limit");
      const DeviceGrid device_grid{grid.origin_x, grid.origin_y, grid.cell_size, grid.max_radius,
                                    grid.footprint_extent, grid.width, grid.height};
      geometryKernel<<<static_cast<unsigned int>(poses.size()), 128, 0, stream_>>>(
          polygon_.data, polygon.size(), bounds, points_.data, points.size(), offsets_.data,
          device_grid, poses_.data, cap, results_.data);
      check(cudaGetLastError(), "CUDA kernel launch");
      batch.results.resize(poses.size());
      check(cudaMemcpyAsync(batch.results.data(), results_.data,
                            batch.results.size() * sizeof(DeviceGeometryResult),
                            cudaMemcpyDeviceToHost, stream_), "CUDA geometry readback");
      check(cudaStreamSynchronize(stream_), "CUDA geometry completion");
      // Commit the resident epoch only after every operation has succeeded.
      epoch_ = epoch;
      return batch;
    } catch (...) {
      // Drain work while the caller's pose frames and the owned scene upload
      // sources remain alive. Cleanup must not replace the original failure.
      if (stream_)
        cudaStreamSynchronize(stream_);
      failed_ = true;
      throw;
    }
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
  try {
    return std::make_unique<Executor>();
  } catch (const std::exception &e) {
    reason = e.what();
    return {};
  }
}
}
