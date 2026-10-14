#pragma once

#include "duojin02_navigation/gpu_geometry.hpp"

namespace duojin02_navigation {
struct PoseFrame {
  double x, y, c, s;
};

struct DeviceGeometryResult {
  unsigned int collision, uncertain;
};

class CudaExecutor {
public:
  virtual ~CudaExecutor() = default;
  virtual std::string name() const = 0;
  virtual std::vector<DeviceGeometryResult> geometry(const Polygon &, const std::vector<Point> &,
                                                     uint64_t epoch, const std::vector<PoseFrame> &,
                                                     double margin) = 0;
};

std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason);
}
