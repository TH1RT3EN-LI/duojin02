// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/sensors/laser/observation_backend.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace nav2_amcl {
namespace {
void check(cudaError_t code, const char * stage) {
  if (code != cudaSuccess) { throw std::runtime_error(std::string(stage)+": "+cudaGetErrorString(code)); }
}
struct StreamDrain {
  cudaStream_t stream;
  ~StreamDrain() { cudaStreamSynchronize(stream); }
};
template<typename T> class Buffer {
public:
  T * ptr{nullptr}; size_t capacity{0};
  ~Buffer() { if (ptr) { cudaFree(ptr); } }
  void reserve(size_t count) {
    if (count <= capacity) { return; }
    if (count > std::numeric_limits<size_t>::max()/sizeof(T)) { throw std::overflow_error("AMCL CUDA buffer overflow"); }
    T * next=nullptr; check(cudaMalloc(reinterpret_cast<void **>(&next), count*sizeof(T)), "AMCL cudaMalloc");
    if (ptr) { cudaFree(ptr); } ptr=next; capacity=count;
  }
};
__global__ void scoreBeams(ObservationSettings settings, ObservationMap map, const double * distances,
  const ObservationPose * poses, const ObservationBeam * beams, int particle_count, int beam_count,
  double range_max, double * probabilities, int * agree, int * boundary) {
  const size_t k=static_cast<size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  if (k >= static_cast<size_t>(particle_count)*beam_count) { return; }
  const int p=k/beam_count, b=k%beam_count;
  const auto beam=beams[b]; const auto pose=poses[p];
  // Neutral elements for ignored and padded beams. Invalid negative infinity and
  // bearing values cannot safely index a map; ask the unchanged CPU oracle.
  if (beam.range >= range_max || !isfinite(beam.range) || !isfinite(beam.bearing)) {
    probabilities[k]=settings.probability_model ? 1.0 : 0.0; return;
  }
  const double x=pose.x+beam.range*cos(pose.yaw+beam.bearing);
  const double y=pose.y+beam.range*sin(pose.yaw+beam.bearing);
  const double gx=(x-map.origin_x)/map.scale+0.5;
  const double gy=(y-map.origin_y)/map.scale+0.5;
  if (!isfinite(gx) || !isfinite(gy)) {
    atomicExch(boundary, 1); probabilities[k]=settings.probability_model ? 1.0 : 0.0; return;
  }
  // Transcendentals may round differently on CPU/GPU. Near an integer grid
  // boundary do not silently pick a different cell: rerun the entire native model.
  if (fabs(gx-nearbyint(gx)) < 1e-10 || fabs(gy-nearbyint(gy)) < 1e-10) {
    atomicExch(boundary, 1);
  }
  const double ix=floor(gx)+map.width/2, iy=floor(gy)+map.height/2;
  const bool valid=ix>=0 && iy>=0 && ix<map.width && iy<map.height;
  double distance=map.max_distance;
  if (valid) {
    distance=distances[static_cast<int>(ix)+static_cast<int>(iy)*map.width];
    if (settings.probability_model && distance<settings.beam_skip_distance) { atomicAdd(agree+b,1); }
  }
  const double denom=2*settings.sigma_hit*settings.sigma_hit;
  probabilities[k]=settings.z_hit*exp(-(distance*distance)/denom)+settings.z_rand*(1.0/range_max);
}
__global__ void computeMask(ObservationSettings settings, const int * agree, int count,
  int particles, int * mask, int * skip_error) {
  // One small serial mask preserves native threshold/padding/error semantics.
  if (threadIdx.x || blockIdx.x) { return; }
  int skipped=0;
  for (int b=0;b<count;++b) {
    mask[b]=(agree[b]/static_cast<double>(particles))>settings.beam_skip_threshold;
    if (!mask[b]) { ++skipped; }
  }
  *skip_error=skipped>=(count*settings.beam_skip_error_threshold);
}
__global__ void combineBeams(ObservationSettings settings, const double * probabilities,
  int particles, int beams, bool skipping, const int * mask, const int * skip_error, double * factors) {
  const int p=blockIdx.x*blockDim.x+threadIdx.x;
  if (p>=particles) { return; }
  double sum=settings.probability_model ? 0.0 : 1.0;
  for (int b=0;b<beams;++b) {
    const double probability=probabilities[static_cast<size_t>(p)*beams+b];
    if (settings.probability_model) {
      if (!skipping || *skip_error || mask[b]) { sum+=log(probability); }
    } else { sum+=probability*probability*probability; }
  }
  factors[p]=settings.probability_model ? exp(sum) : sum;
}
class CudaObservation final : public DeviceObservation {
  cudaStream_t stream_{};
  ObservationMap map_{};
  Buffer<double> distances_, probabilities_, factors_;
  Buffer<ObservationPose> poses_;
  Buffer<ObservationBeam> beams_;
  Buffer<int> agree_, mask_, flags_;
public:
  CudaObservation() {
    int count=0; check(cudaGetDeviceCount(&count), "AMCL cudaGetDeviceCount");
    if (!count) { throw std::runtime_error("AMCL requires an available CUDA device"); }
    check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "AMCL cudaStreamCreate");
  }
  ~CudaObservation() override { if(stream_) { cudaStreamSynchronize(stream_); cudaStreamDestroy(stream_); } }
  void map(const ObservationMap & metadata, const std::vector<double> & distances) override {
    StreamDrain drain{stream_};
    map_=metadata; distances_.reserve(distances.size());
    check(cudaMemcpyAsync(distances_.ptr,distances.data(),distances.size()*sizeof(double),cudaMemcpyHostToDevice,stream_),"AMCL upload map");
    check(cudaStreamSynchronize(stream_),"AMCL finish map upload");
  }
  bool score(const ObservationSettings & settings, const ObservationPose * poses, int particles,
    const ObservationBeam * beams, int beam_count, double range_max, bool converged, double * factors) override {
    if (particles<=0) { return true; }
    if (beam_count<=0) {
      for (int i=0; i<particles; ++i) { factors[i]=1.0; }
      return true;
    }
    int boundary=0;
    StreamDrain drain{stream_};
    const size_t pairs=static_cast<size_t>(particles)*beam_count;
    poses_.reserve(particles); beams_.reserve(beam_count); probabilities_.reserve(pairs);
    factors_.reserve(particles); agree_.reserve(beam_count); mask_.reserve(beam_count); flags_.reserve(2);
    check(cudaMemcpyAsync(poses_.ptr,poses,particles*sizeof(ObservationPose),cudaMemcpyHostToDevice,stream_),"AMCL upload poses");
    check(cudaMemcpyAsync(beams_.ptr,beams,beam_count*sizeof(ObservationBeam),cudaMemcpyHostToDevice,stream_),"AMCL upload scan");
    check(cudaMemsetAsync(agree_.ptr,0,beam_count*sizeof(int),stream_),"AMCL clear agreement");
    check(cudaMemsetAsync(flags_.ptr,0,2*sizeof(int),stream_),"AMCL clear flags");
    scoreBeams<<<(pairs+255)/256,256,0,stream_>>>(settings,map_,distances_.ptr,poses_.ptr,beams_.ptr,
      particles,beam_count,range_max,probabilities_.ptr,agree_.ptr,flags_.ptr);
    check(cudaGetLastError(),"AMCL scoreBeams kernel");
    const bool skipping=settings.probability_model && settings.beam_skip && converged;
    if (skipping) {
      computeMask<<<1,1,0,stream_>>>(settings,agree_.ptr,beam_count,particles,mask_.ptr,flags_.ptr+1);
      check(cudaGetLastError(),"AMCL computeMask kernel");
    }
    combineBeams<<<(particles+127)/128,128,0,stream_>>>(settings,probabilities_.ptr,particles,
      beam_count,skipping,mask_.ptr,flags_.ptr+1,factors_.ptr);
    check(cudaGetLastError(),"AMCL combineBeams kernel");
    check(cudaMemcpyAsync(factors,factors_.ptr,particles*sizeof(double),cudaMemcpyDeviceToHost,stream_),"AMCL download factors");
    check(cudaMemcpyAsync(&boundary,flags_.ptr,sizeof(int),cudaMemcpyDeviceToHost,stream_),"AMCL download guard");
    check(cudaStreamSynchronize(stream_),"AMCL complete observation");
    return boundary==0;
  }
};
}  // namespace
std::unique_ptr<DeviceObservation> makeDeviceObservation() { return std::make_unique<CudaObservation>(); }
}  // namespace nav2_amcl
