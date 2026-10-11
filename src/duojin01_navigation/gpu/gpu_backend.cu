#include "compute_internal.hpp"
#include <cuda_runtime.h>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace duojin01_navigation {
namespace {
void check(cudaError_t error,const char *operation) {
  if(error!=cudaSuccess)throw std::runtime_error(std::string(operation)+": "+cudaGetErrorString(error));
}
template<class T> struct Buffer {
  T *data{nullptr};size_t capacity{0};
  ~Buffer(){if(data)cudaFree(data);}
  void reserve(size_t n) {
    if(n<=capacity)return;
    T *next=nullptr;check(cudaMalloc(reinterpret_cast<void **>(&next),n*sizeof(T)),"cudaMalloc");
    if(data)check(cudaFree(data),"cudaFree");data=next;capacity=n;
  }
};
struct Bounds {double min_x,min_y,max_x,max_y;};
__device__ double segmentDistance(double x,double y,const Point &a,const Point &b) {
  const double dx=b.x-a.x,dy=b.y-a.y;
  const double t=fmin(1.,fmax(0.,((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy)));
  return hypot(x-a.x-t*dx,y-a.y-t*dy);
}
__device__ void atomicMinimum(double *address,double value) {
  auto bits=reinterpret_cast<unsigned long long *>(address);auto old=*bits;
  while(value<__longlong_as_double(old)) {
    const auto expected=old;old=atomicCAS(bits,expected,__double_as_longlong(value));
    if(old==expected)break;
  }
}
__global__ void initialize(DeviceGeometryResult *results,size_t n,double cap) {
  const auto i=static_cast<size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  if(i<n)results[i]={cap,0,0};
}
__global__ void geometryKernel(const Point *polygon,size_t vertices,Bounds bounds,
    const Point *points,size_t count,const PoseFrame *poses,size_t tiles,
    double margin,bool clearance,double cap,DeviceGeometryResult *results) {
  const size_t pose_id=blockIdx.x/tiles;
  const size_t point_id=(blockIdx.x%tiles)*blockDim.x+threadIdx.x;
  double best=cap;unsigned int hit=0,uncertain=0;
  if(point_id<count) {
    const auto &pose=poses[pose_id];const auto &p=points[point_id];
    const double dx=p.x-pose.x,dy=p.y-pose.y;
    const double x=pose.c*dx+pose.s*dy,y=-pose.s*dx+pose.c*dy;
    const double padding=(clearance ? fmax(cap,margin) : margin)+p.radius;
    if(x>=bounds.min_x-padding && x<=bounds.max_x+padding &&
       y>=bounds.min_y-padding && y<=bounds.max_y+padding) {
      bool inside=true;double distance=1e100;
      for(size_t i=0;i<vertices;++i) {
        const auto &a=polygon[i],&b=polygon[(i+1)%vertices];
        const double cross=(b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x);
        if(cross < -1e-10)inside=false;
        if(fabs(cross+1e-10)<1e-12)uncertain=1;
        distance=fmin(distance,segmentDistance(x,y,a,b));
      }
      hit=inside || distance<=margin+p.radius;
      if(fabs(distance-margin-p.radius)<1e-10)uncertain=1;
      if(clearance)best=fmin(cap,(inside ? -distance : distance)-p.radius);
    }
    // AABB edge decisions also need the host oracle on close boundaries.
    if(fabs(x-bounds.min_x+padding)<1e-10 || fabs(x-bounds.max_x-padding)<1e-10 ||
       fabs(y-bounds.min_y+padding)<1e-10 || fabs(y-bounds.max_y-padding)<1e-10)uncertain=1;
  }
  __shared__ double minimum[128];__shared__ unsigned int flags[128];
  minimum[threadIdx.x]=best;flags[threadIdx.x]=hit | (uncertain<<1);__syncthreads();
  for(unsigned int stride=64;stride;stride>>=1) {
    if(threadIdx.x<stride) {minimum[threadIdx.x]=fmin(minimum[threadIdx.x],minimum[threadIdx.x+stride]);flags[threadIdx.x]|=flags[threadIdx.x+stride];}
    __syncthreads();
  }
  if(threadIdx.x==0) {
    if(flags[0]&1)atomicOr(&results[pose_id].collision,1);
    if(flags[0]&2)atomicOr(&results[pose_id].uncertain,1);
    if(clearance)atomicMinimum(&results[pose_id].clearance,minimum[0]);
  }
}
__global__ void rolloutKernel(SamplingView view,const Velocity *nominal,const Velocity *noise,
    Pose *states,Velocity *controls) {
  const size_t k=static_cast<size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  if(k>=view.settings.batch_size)return;
  Pose pose=view.start;Velocity u=view.measured;size_t waypoint=0;
  for(size_t t=0;t<view.settings.time_steps;++t) {
    u=candidateControl(view,nominal,noise,k,t,pose,u,waypoint);pose=sampleAdvance(pose,u,view.settings.dt);
    const size_t index=k*view.settings.time_steps+t;states[index]=pose;controls[index]=u;
  }
}
__global__ void sampleCostKernel(SamplingView view,const Pose *states,
    const Velocity *controls,const Velocity *nominal,double *costs) {
  const size_t i=static_cast<size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  if(i>=view.settings.batch_size*view.settings.time_steps)return;
  const size_t t=i%view.settings.time_steps;
  costs[i]=sampleCost(view,t ? states[i-1]:view.start,states[i],controls[i],nominal[t],t);
}
__global__ void totalCostKernel(const double *costs,size_t batch,size_t steps,double *totals) {
  const size_t k=static_cast<size_t>(blockIdx.x)*blockDim.x+threadIdx.x;
  if(k>=batch)return;double total=0;
  for(size_t t=0;t<steps;++t)total+=costs[k*steps+t];totals[k]=total;
}
__global__ void weightsKernel(const double *totals,double *weights,size_t batch,double temperature,double *summary) {
  __shared__ double values[256];double minimum=DBL_MAX;
  for(size_t k=threadIdx.x;k<batch;k+=blockDim.x)minimum=fmin(minimum,totals[k]);
  values[threadIdx.x]=minimum;__syncthreads();
  for(unsigned int stride=128;stride;stride>>=1){if(threadIdx.x<stride)values[threadIdx.x]=fmin(values[threadIdx.x],values[threadIdx.x+stride]);__syncthreads();}
  minimum=values[0];double sum=0;__syncthreads();
  for(size_t k=threadIdx.x;k<batch;k+=blockDim.x){const double w=totals[k]<1e9 ? exp(-(totals[k]-minimum)/temperature):0.;weights[k]=w;sum+=w;}
  values[threadIdx.x]=sum;__syncthreads();
  for(unsigned int stride=128;stride;stride>>=1){if(threadIdx.x<stride)values[threadIdx.x]+=values[threadIdx.x+stride];__syncthreads();}
  if(threadIdx.x==0)summary[0]=values[0];
}
__global__ void sequenceKernel(const Velocity *controls,const double *weights,
    size_t batch,size_t steps,const double *summary,Velocity *sequence) {
  const double weight_sum=summary[0];
  const size_t t=blockIdx.x;Velocity total{};
  if(weight_sum>0)for(size_t k=threadIdx.x;k<batch;k+=blockDim.x) {
    const auto u=controls[k*steps+t];const double w=weights[k]/weight_sum;
    total.x+=w*u.x;total.y+=w*u.y;total.yaw+=w*u.yaw;
  }
  __shared__ Velocity values[128];values[threadIdx.x]=total;__syncthreads();
  for(unsigned int stride=64;stride;stride>>=1) {
    if(threadIdx.x<stride){values[threadIdx.x].x+=values[threadIdx.x+stride].x;values[threadIdx.x].y+=values[threadIdx.x+stride].y;values[threadIdx.x].yaw+=values[threadIdx.x+stride].yaw;}__syncthreads();
  }
  if(threadIdx.x==0)sequence[t]=values[0];
}
class Executor final:public CudaExecutor {
  cudaStream_t stream_{nullptr};cudaEvent_t done_{nullptr};
  Buffer<Point> polygon_,points_;Buffer<PoseFrame> poses_;
  Buffer<DeviceGeometryResult> results_;
  Buffer<unsigned char> cells_;Buffer<unsigned int> rows_,columns_;
  Buffer<Point> sample_polygon_;Buffer<Pose> path_,guidance_,states_;
  Buffer<Velocity> nominal_,noise_,controls_,sequence_;
  Buffer<double> costs_,totals_,weights_,summary_;
  uint64_t epoch_{0};std::string name_;bool timed_out_{false};
  void waitUntil(std::chrono::steady_clock::time_point end) {
    check(cudaGetLastError(),"CUDA kernel launch");check(cudaEventRecord(done_,stream_),"completion event");
    while(true) {
      const auto error=cudaEventQuery(done_);if(error==cudaSuccess)break;
      if(error!=cudaErrorNotReady)check(error,"CUDA completion");
      if(std::chrono::steady_clock::now()>end){timed_out_=true;throw std::runtime_error("CUDA completion deadline exceeded");}
      std::this_thread::sleep_for(std::chrono::microseconds(25));
    }
  }
  template<class T> void upload(Buffer<T> &buffer,const std::vector<T> &values) {
    buffer.reserve(values.size());
    if(!values.empty())check(cudaMemcpyAsync(buffer.data,values.data(),values.size()*sizeof(T),cudaMemcpyHostToDevice,stream_),"CUDA sampling upload");
  }
 public:
  Executor() {
    cudaDeviceProp properties{};int device=0;check(cudaGetDevice(&device),"cudaGetDevice");
    check(cudaGetDeviceProperties(&properties,device),"cudaGetDeviceProperties");name_=properties.name;
    check(cudaStreamCreateWithFlags(&stream_,cudaStreamNonBlocking),"cudaStreamCreate");
    check(cudaEventCreateWithFlags(&done_,cudaEventDisableTiming),"cudaEventCreate");
    // Prime the context before active commands. Storage grows only as needed.
    check(cudaFree(nullptr),"CUDA context initialization");
  }
  ~Executor() override {
    if(stream_)cudaStreamSynchronize(stream_);
    if(done_)cudaEventDestroy(done_);if(stream_)cudaStreamDestroy(stream_);
  }
  std::string name() const override{return name_;}
  std::vector<DeviceGeometryResult> geometry(const Polygon &polygon,const std::vector<Point> &points,
      uint64_t epoch,const std::vector<PoseFrame> &poses,double margin,bool clearance,double cap,double deadline_ms) override {
    if(timed_out_)throw std::runtime_error("CUDA backend latched after deadline miss; reconfigure required");
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double,std::milli>(deadline_ms));
    polygon_.reserve(polygon.size());points_.reserve(points.size());poses_.reserve(poses.size());results_.reserve(poses.size());
    if(epoch_!=epoch) {
      check(cudaMemcpyAsync(polygon_.data,polygon.data(),polygon.size()*sizeof(Point),cudaMemcpyHostToDevice,stream_),"polygon upload");
      check(cudaMemcpyAsync(points_.data,points.data(),points.size()*sizeof(Point),cudaMemcpyHostToDevice,stream_),"point upload");epoch_=epoch;
    }
    check(cudaMemcpyAsync(poses_.data,poses.data(),poses.size()*sizeof(PoseFrame),cudaMemcpyHostToDevice,stream_),"pose upload");
    Bounds bounds{1e100,1e100,-1e100,-1e100};
    for(const auto &p:polygon) {bounds.min_x=std::min(bounds.min_x,p.x);bounds.min_y=std::min(bounds.min_y,p.y);bounds.max_x=std::max(bounds.max_x,p.x);bounds.max_y=std::max(bounds.max_y,p.y);}
    const size_t tiles=(points.size()+127)/128,blocks=tiles*poses.size();
    if(blocks>2147483647ULL)throw std::invalid_argument("CUDA geometry batch too large");
    initialize<<<static_cast<unsigned int>((poses.size()+127)/128),128,0,stream_>>>(results_.data,poses.size(),cap);
    geometryKernel<<<static_cast<unsigned int>(blocks),128,0,stream_>>>(polygon_.data,polygon.size(),bounds,points_.data,points.size(),poses_.data,tiles,margin,clearance,cap,results_.data);
    waitUntil(end);
    std::vector<DeviceGeometryResult> result(poses.size());
    check(cudaMemcpy(result.data(),results_.data,result.size()*sizeof(DeviceGeometryResult),cudaMemcpyDeviceToHost),"geometry readback");return result;
  }
  std::pair<std::vector<double>,std::vector<Velocity>> sampling(const SamplingView &input,
      const std::vector<unsigned char> &cells,const std::vector<unsigned int> &rows,const std::vector<unsigned int> &columns,
      const Polygon &polygon,const std::vector<Pose> &path,const std::vector<Velocity> &nominal,const std::vector<Velocity> &noise,const std::vector<Pose> &guidance,double deadline_ms) override {
    if(timed_out_)throw std::runtime_error("CUDA backend latched after deadline miss; reconfigure required");
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double,std::milli>(deadline_ms));
    upload(cells_,cells);upload(rows_,rows);upload(columns_,columns);upload(sample_polygon_,polygon);upload(path_,path);upload(nominal_,nominal);upload(noise_,noise);
    upload(guidance_,guidance);
    SamplingView view=input;view.cells=cells_.data;view.rows=rows_.data;view.columns=columns_.data;
    view.polygon=sample_polygon_.data;view.path=path_.data;
    view.guidance=guidance_.data;
    const size_t batch=view.settings.batch_size,steps=view.settings.time_steps,n=batch*steps;
    states_.reserve(n);controls_.reserve(n);costs_.reserve(n);totals_.reserve(batch);weights_.reserve(batch);sequence_.reserve(steps);summary_.reserve(1);
    rolloutKernel<<<static_cast<unsigned int>((batch+127)/128),128,0,stream_>>>(view,nominal_.data,noise_.data,states_.data,controls_.data);
    sampleCostKernel<<<static_cast<unsigned int>((n+127)/128),128,0,stream_>>>(view,states_.data,controls_.data,nominal_.data,costs_.data);
    totalCostKernel<<<static_cast<unsigned int>((batch+127)/128),128,0,stream_>>>(costs_.data,batch,steps,totals_.data);
    weightsKernel<<<1,256,0,stream_>>>(totals_.data,weights_.data,batch,view.settings.temperature,summary_.data);
    sequenceKernel<<<static_cast<unsigned int>(steps),128,0,stream_>>>(controls_.data,weights_.data,batch,steps,summary_.data,sequence_.data);
    waitUntil(end);
    std::vector<double> totals(batch);check(cudaMemcpy(totals.data(),totals_.data,batch*sizeof(double),cudaMemcpyDeviceToHost),"sampling cost readback");
    std::vector<Velocity> sequence(steps);check(cudaMemcpy(sequence.data(),sequence_.data,steps*sizeof(Velocity),cudaMemcpyDeviceToHost),"sampling sequence readback");
    return {std::move(totals),std::move(sequence)};
  }
};
}
std::unique_ptr<CudaExecutor> makeCudaExecutor(std::string &reason) {
  int count=0;const auto error=cudaGetDeviceCount(&count);
  if(error!=cudaSuccess || count==0) {reason=error==cudaSuccess ? "no CUDA device" : cudaGetErrorString(error);return {};}
  try {return std::make_unique<Executor>();}catch(const std::exception &e){reason=e.what();return {};}
}
}
