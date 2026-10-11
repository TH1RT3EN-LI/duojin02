#pragma once
#include "duojin01_navigation/gpu_backend.hpp"
#include "sampling_math.hpp"

namespace duojin01_navigation {
struct PoseFrame {double x,y,c,s;};
struct DeviceGeometryResult {
  double clearance;
  unsigned int collision,uncertain;
};
class CudaExecutor {
 public:
  virtual ~CudaExecutor()=default;
  virtual std::string name() const=0;
  virtual std::vector<DeviceGeometryResult> geometry(
      const Polygon &,const std::vector<Point> &,uint64_t epoch,
      const std::vector<PoseFrame> &,double margin,bool clearance,
      double cap,double deadline_ms)=0;
  virtual std::pair<std::vector<double>,std::vector<Velocity>> sampling(
    const SamplingView &,const std::vector<unsigned char> &,
    const std::vector<unsigned int> &,const std::vector<unsigned int> &,
    const Polygon &,const std::vector<Pose> &,
    const std::vector<Velocity> &,const std::vector<Velocity> &,const std::vector<Pose> &,double deadline_ms)=0;
};
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason);
}
