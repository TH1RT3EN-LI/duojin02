#include "compute_internal.hpp"
#include <chrono>
#include <limits>
#include <stdexcept>

namespace duojin01_navigation {
ComputeBackend parseComputeBackend(const std::string &name) {
  if(name=="cpu") return ComputeBackend::Cpu;
  if(name=="cuda") return ComputeBackend::Cuda;
  if(name=="auto") return ComputeBackend::Auto;
  throw std::invalid_argument("compute_backend must be cpu, cuda or auto");
}
#ifndef DUOJIN_HAS_CUDA
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason) {
  reason="this build has DUOJIN_ENABLE_CUDA=OFF";return {};
}
#endif
struct GeometryCompute::Impl {
  ComputeOptions options;
  ComputeStatistics stats;
  const Polygon *polygon{nullptr};const std::vector<Point> *points{nullptr};
  uint64_t epoch{0};
  std::unique_ptr<CudaExecutor> cuda;
  std::string reason;
  explicit Impl(ComputeOptions o):options(o) {
    if(!std::isfinite(o.deadline_ms) || o.deadline_ms<=0)
      throw std::invalid_argument("GPU deadline must be finite and positive");
    if(o.backend!=ComputeBackend::Cpu) cuda=makeCudaExecutor(reason);
    if(o.backend==ComputeBackend::Cuda && !cuda)
      throw std::runtime_error("CUDA backend unavailable: "+reason);
  }
};
GeometryCompute::GeometryCompute(ComputeOptions options):impl_(new Impl(options)) {}
GeometryCompute::~GeometryCompute()=default;
GeometryCompute::GeometryCompute(GeometryCompute &&) noexcept=default;
GeometryCompute &GeometryCompute::operator=(GeometryCompute &&) noexcept=default;
void GeometryCompute::setScene(const Polygon &polygon,const std::vector<Point> &points) {
  if(polygon.size()<3 || polygon.size()>64) throw std::invalid_argument("Invalid geometry footprint size");
  for(size_t i=0;i<polygon.size();++i) {
    const auto &a=polygon[i],&b=polygon[(i+1)%polygon.size()],&c=polygon[(i+2)%polygon.size()];
    if(!std::isfinite(a.x+a.y) || std::hypot(b.x-a.x,b.y-a.y)<1e-12 ||
       (b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x)<-1e-12)
      throw std::invalid_argument("GPU geometry requires a finite convex CCW footprint");
  }
  double area=0;for(size_t i=0;i<polygon.size();++i) {
    const auto &a=polygon[i],&b=polygon[(i+1)%polygon.size()];area+=a.x*b.y-a.y*b.x;
    for(const auto &p:polygon) if((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x)<-1e-12)
      throw std::invalid_argument("Footprint is not a convex CCW polygon");
  }
  if(area<=1e-12) throw std::invalid_argument("Degenerate or clockwise footprint");
  for(const auto &p:points) if(!std::isfinite(p.x+p.y+p.radius) || p.radius<0)
    throw std::invalid_argument("Invalid obstacle geometry");
  impl_->polygon=&polygon;impl_->points=&points;++impl_->epoch;
}
GeometryResult GeometryCompute::evaluate(const std::vector<Pose> &poses,double margin,bool clearance,double cap) {
  const auto begin=std::chrono::steady_clock::now();
  if(!impl_->polygon || !impl_->points) throw std::logic_error("Geometry scene not set");
  if(!std::isfinite(margin) || margin<0 || !std::isfinite(cap) || cap<=0)
    throw std::invalid_argument("Invalid geometry margin/cap");
  for(const auto &p:poses) if(!std::isfinite(p.x+p.y+p.yaw)) throw std::invalid_argument("Invalid query pose");
  GeometryResult result;result.collisions.resize(poses.size());
  if(clearance) result.clearances.resize(poses.size());
  const auto &polygon=*impl_->polygon;const auto &points=*impl_->points;
  const bool gpu=impl_->cuda && !poses.empty() && !points.empty() &&
    (impl_->options.backend==ComputeBackend::Cuda ||
     static_cast<long double>(poses.size())*points.size()>=impl_->options.minimum_work);
  if(gpu) {
    std::vector<PoseFrame> frames;frames.reserve(poses.size());
    for(const auto &p:poses)frames.push_back({p.x,p.y,std::cos(p.yaw),std::sin(p.yaw)});
    std::vector<DeviceGeometryResult> raw;
    try {raw=impl_->cuda->geometry(polygon,points,impl_->epoch,frames,margin,clearance,cap,impl_->options.deadline_ms);}
    catch(const std::exception &e) {throw ComputeFailure(e.what());}
    if(raw.size()!=poses.size())throw std::runtime_error("CUDA result cardinality mismatch");
    for(size_t i=0;i<poses.size();++i) {
      // Device hypot and host hypot may differ at a floating-point boundary.
      // Recheck ambiguous rows against the original complete CPU predicate.
      if(raw[i].uncertain) {
        result.collisions[i]=collision(poses[i],polygon,points,margin);
        if(clearance)result.clearances[i]=footprintClearance(poses[i],polygon,points,cap);
        ++impl_->stats.boundary_rechecks;
      } else {
        result.collisions[i]=raw[i].collision!=0;
        if(clearance)result.clearances[i]=raw[i].clearance;
      }
    }
    result.used_cuda=true;++impl_->stats.cuda_batches;
  } else {
    for(size_t i=0;i<poses.size();++i) {
      result.collisions[i]=collision(poses[i],polygon,points,margin);
      if(clearance)result.clearances[i]=footprintClearance(poses[i],polygon,points,cap);
    }
    ++impl_->stats.cpu_batches;
  }
  impl_->stats.last_wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  if(result.used_cuda && impl_->stats.last_wall_ms>impl_->options.deadline_ms)
    throw ComputeFailure("CUDA geometry exceeded whole-call deadline");
  return result;
}
bool GeometryCompute::available() const{return impl_->cuda!=nullptr;}
bool GeometryCompute::willUseCuda(size_t pose_count) const {
  return impl_->cuda && impl_->points && pose_count && !impl_->points->empty() &&
    (impl_->options.backend==ComputeBackend::Cuda ||
     static_cast<long double>(pose_count)*impl_->points->size()>=impl_->options.minimum_work);
}
std::string GeometryCompute::deviceName() const{return impl_->cuda ? impl_->cuda->name() : impl_->reason;}
const ComputeStatistics &GeometryCompute::statistics() const{return impl_->stats;}
}
