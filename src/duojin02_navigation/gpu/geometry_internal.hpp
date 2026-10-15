#pragma once

#include "duojin02_navigation/gpu_geometry.hpp"

namespace duojin02_navigation {
struct PoseFrame {
  double x, y, c, s, margin;
  GeometryWorldBounds world_bounds;
  unsigned int has_world_bounds;
};

struct DeviceGeometryResult {
  unsigned int collision, uncertain;
  double clearance, clearance_error;
};

using GeometryBounds = GeometryWorldBounds;

// Obstacle centres are grouped in row-major grid cells. Each obstacle remains
// represented by its original double coordinates and individual radius.
struct GeometryGrid {
  double origin_x{0}, origin_y{0}, cell_size{1}, max_radius{0}, footprint_extent{0};
  uint32_t width{0}, height{0};
  std::vector<uint64_t> offsets;
};

struct DeviceGeometryBatch {
  std::vector<DeviceGeometryResult> results;
  uint64_t uploaded_bytes{0};
  bool scene_uploaded{false};
};

class CudaExecutor {
public:
  virtual ~CudaExecutor() = default;
  virtual std::string name() const = 0;
  virtual DeviceGeometryBatch geometry(const Polygon &, GeometryBounds,
                                       const std::vector<Point> &, const GeometryGrid &,
                                       uint64_t epoch, const std::vector<PoseFrame> &,
                                       double clearance_cap) = 0;
};

std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason);
}
