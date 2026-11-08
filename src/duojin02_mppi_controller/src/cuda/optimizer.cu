// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: Apache-2.0
#include "duojin02_mppi_controller/cuda_optimizer.hpp"
#include "normal.hpp"
#include <cuda_runtime.h>
#include <algorithm>
#include <limits>
#include <vector>
namespace duojin_mppi {
namespace {
void check(cudaError_t e, const char *operation) {
  if (e != cudaSuccess) throw OptimizerComputeError(std::string(operation)+": "+cudaGetErrorString(e));
}
__global__ void sample(CoreSettings s, CoreBuffers b, float *nx, float *ny, float *nw,
                       uint64_t seed, uint64_t epoch, bool supplied, int *invalid) {
  const size_t i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=s.batch*s.steps) return;
  const size_t t=i%s.steps;
  b.cvx[i]=b.ux[t]+(supplied?nx[i]:s.std_vx*sharedNormal(seed,epoch,i,0));
  b.cwz[i]=b.uw[t]+(supplied?nw[i]:s.std_wz*sharedNormal(seed,epoch,i,2));
  b.cvy[i]=s.holonomic?(b.uy[t]+(supplied?ny[i]:s.std_vy*sharedNormal(seed,epoch,i,1))):0.f;
  if(!isfinite(b.cvx[i]) || !isfinite(b.cvy[i]) || !isfinite(b.cwz[i]))atomicExch(invalid,1);
}
__global__ void integrate(CoreSettings s, CoreBuffers b, CorePose p, int *invalid) {
  const size_t k=blockIdx.x*blockDim.x+threadIdx.x;
  if(k>=s.batch) return;
  float dx=0,dy=0,da=0;
  for(size_t t=0;t<s.steps;++t) {
    size_t i=k*s.steps+t;
    b.vx[i]=t?b.cvx[i-1]:p.vx;
    b.vy[i]=s.holonomic?(t?b.cvy[i-1]:p.vy):0;
    b.wz[i]=t?b.cwz[i-1]:p.wz;
    const float a=p.yaw+da, c=cosf(a), sn=sinf(a);
    dx+=(b.vx[i]*c-(s.holonomic?b.vy[i]*sn:0.f))*s.dt;
    dy+=(b.vx[i]*sn+(s.holonomic?b.vy[i]*c:0.f))*s.dt;
    da+=b.wz[i]*s.dt;
    b.x[i]=p.x+dx; b.y[i]=p.y+dy; b.yaw[i]=p.yaw+da;
    if(!isfinite(b.x[i]) || !isfinite(b.y[i]) || !isfinite(b.yaw[i]))atomicExch(invalid,1);
  }
}
__global__ void controlCost(CoreSettings s, CoreBuffers b) {
  size_t k=blockIdx.x*blockDim.x+threadIdx.x;
  if(k>=s.batch) return;
  float x=0,y=0,w=0;
  for(size_t t=0;t<s.steps;++t) {
    size_t i=k*s.steps+t;
    x+=b.ux[t]*(b.cvx[i]-b.ux[t]);
    w+=b.uw[t]*(b.cwz[i]-b.uw[t]);
    if(s.holonomic) y+=b.uy[t]*(b.cvy[i]-b.uy[t]);
  }
  b.cost[k]+=s.gamma/(s.std_vx*s.std_vx)*x;
  b.cost[k]+=s.gamma/(s.std_wz*s.std_wz)*w;
  if(s.holonomic) b.cost[k]+=s.gamma/(s.std_vy*s.std_vy)*y;
}
__global__ void weights(CoreSettings s, CoreBuffers b, float *weight, int *invalid) {
  __shared__ float values[256];
  float m=INFINITY;
  for(size_t k=threadIdx.x;k<s.batch;k+=blockDim.x) {
    if(!isfinite(b.cost[k])) atomicExch(invalid,1);
    m=fminf(m,b.cost[k]);
  }
  values[threadIdx.x]=m; __syncthreads();
  for(int stride=128;stride;stride/=2) {
    if(threadIdx.x<stride) values[threadIdx.x]=fminf(values[threadIdx.x],values[threadIdx.x+stride]);
    __syncthreads();
  }
  m=values[0]; float sum=0;
  for(size_t k=threadIdx.x;k<s.batch;k+=blockDim.x) {
    weight[k]=expf((-1.f/s.temperature)*(b.cost[k]-m)); sum+=weight[k];
  }
  values[threadIdx.x]=sum; __syncthreads();
  for(int stride=128;stride;stride/=2) {
    if(threadIdx.x<stride) values[threadIdx.x]+=values[threadIdx.x+stride];
    __syncthreads();
  }
  const float total=values[0];
  if(threadIdx.x==0 && (!(total>0) || !isfinite(total))) atomicExch(invalid,1);
  for(size_t k=threadIdx.x;k<s.batch;k+=blockDim.x) weight[k]/=total;
}
__global__ void criticKernel(CoreBuffers b, size_t batch, size_t steps, unsigned kind,
  float weight, unsigned power, float x, float y, float minv, float maxv, float radius, float dt) {
  const size_t k=blockIdx.x*blockDim.x+threadIdx.x;
  if(k>=batch)return;
  float sum=0;
  for(size_t t=0;t<steps;++t) {
    const size_t i=k*steps+t;
    if(kind==0) {const float dx=b.x[i]-x,dy=b.y[i]-y;sum+=sqrtf(dx*dx+dy*dy);}
    else if(kind==1) {
      const float delta=x-b.yaw[i];
      const float wrapped=delta-6.2831853071795864769f*floorf((delta+3.14159265358979323846f)/6.2831853071795864769f);
      sum+=fabsf(wrapped);
    } else if(kind==2)sum+=fabsf(b.wz[i]);
    else if(kind==3) {
      const float v=(b.vx[i]>0?1.f:-1.f)*sqrtf(b.vx[i]*b.vx[i]+b.vy[i]*b.vy[i]);
      float penalty=fmaxf(v-maxv,0.f)+fmaxf(minv-v,0.f);
      if(radius>0)penalty+=fmaxf(radius-fabsf(b.vx[i])/fabsf(b.wz[i]),0.f);
      sum+=penalty*dt;
    } else if(kind==4 && t+1==steps) {const float dx=b.x[i]-x,dy=b.y[i]-y;sum=sqrtf(dx*dx+dy*dy);}
  }
  if(kind<=2)sum/=steps;
  b.cost[k]+=powf(sum*weight,static_cast<float>(power));
}
__global__ void weightedControls(CoreSettings s, CoreBuffers b, const float *weight) {
  __shared__ float xs[256],ys[256],ws[256];
  const size_t t=blockIdx.x; float x=0,y=0,w=0;
  for(size_t k=threadIdx.x;k<s.batch;k+=blockDim.x) {
    const size_t i=k*s.steps+t;
    x+=b.cvx[i]*weight[k]; w+=b.cwz[i]*weight[k];
    if(s.holonomic)y+=b.cvy[i]*weight[k];
  }
  xs[threadIdx.x]=x;ys[threadIdx.x]=y;ws[threadIdx.x]=w;__syncthreads();
  for(int stride=128;stride;stride/=2) {
    if(threadIdx.x<stride) {
      xs[threadIdx.x]+=xs[threadIdx.x+stride];
      ys[threadIdx.x]+=ys[threadIdx.x+stride];
      ws[threadIdx.x]+=ws[threadIdx.x+stride];
    }
    __syncthreads();
  }
  if(threadIdx.x==0) {
    x=fminf(s.vx_max,fmaxf(s.vx_min,xs[0]));
    w=fminf(s.wz_max,fmaxf(-s.wz_max,ws[0]));
    if(s.min_turning_radius>0 && fabsf(x)/fabsf(w)<s.min_turning_radius)
      w=copysignf(fabsf(x)/s.min_turning_radius,w);
    b.ux[t]=x;b.uw[t]=w;
    if(s.holonomic)b.uy[t]=fminf(s.vy_max,fmaxf(-s.vy_max,ys[0]));
  }
}
}
struct CudaOptimizerCore::Impl {
  cudaStream_t stream{};
  bool available{}; std::string device; OptimizerStatistics stats;
  float *allocation{}; int *invalid{}; size_t capacity{},steps{},batch{};
  CoreBuffers b; float *nx{},*ny{},*nw{},*weight{};
  Impl() {
    int count=0; auto status=cudaGetDeviceCount(&count);
    if(status!=cudaSuccess || !count) {device=status==cudaSuccess?"no CUDA device":cudaGetErrorString(status);return;}
    int current=0; check(cudaGetDevice(&current),"get CUDA device");
    cudaDeviceProp prop{};check(cudaGetDeviceProperties(&prop,current),"get CUDA device properties");
    device=prop.name;
    check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),"create optimizer stream");
    available=true;
  }
  ~Impl() {if(stream)cudaStreamSynchronize(stream);if(allocation)cudaFree(allocation);if(invalid)cudaFree(invalid);if(stream)cudaStreamDestroy(stream);}
  void ensure(CoreSettings s) {
    if(!available) throw OptimizerComputeError("CUDA optimizer unavailable: "+device);
    if(!s.batch || s.steps<2 || s.batch>1000000 || s.steps>10000 ||
      !(s.dt>0) || !(s.temperature>0) || !(s.std_vx>0) || !(s.std_wz>0) ||
      (s.holonomic && !(s.std_vy>0)) || !std::isfinite(s.dt) || !std::isfinite(s.temperature) ||
      !std::isfinite(s.gamma) || !std::isfinite(s.std_vx) || !std::isfinite(s.std_vy) || !std::isfinite(s.std_wz) ||
      !std::isfinite(s.vx_min) || !std::isfinite(s.vx_max) || !std::isfinite(s.vy_max) || !std::isfinite(s.wz_max) ||
      s.vx_min>s.vx_max || s.vy_max<0 || s.wz_max<0) throw OptimizerComputeError("Invalid CUDA optimizer settings");
    const size_t n=s.batch*s.steps, required=12*n+3*s.steps+2*s.batch;
    if(required>capacity) {
      check(cudaStreamSynchronize(stream),"resize optimizer stream");
      if(allocation) {check(cudaFree(allocation),"free optimizer buffers"); allocation=nullptr;capacity=0;}
      check(cudaMalloc(&allocation,required*sizeof(float)),"allocate optimizer buffers");capacity=required;
    }
    if(!invalid)check(cudaMalloc(&invalid,sizeof(int)),"allocate optimizer status");
    float *p=allocation;
    b.cvx=p;p+=n;b.cvy=p;p+=n;b.cwz=p;p+=n;
    b.vx=p;p+=n;b.vy=p;p+=n;b.wz=p;p+=n;
    b.x=p;p+=n;b.y=p;p+=n;b.yaw=p;p+=n;
    nx=p;p+=n;ny=p;p+=n;nw=p;p+=n;
    b.ux=p;p+=s.steps;b.uy=p;p+=s.steps;b.uw=p;p+=s.steps;
    b.cost=p;p+=s.batch;weight=p;steps=s.steps;batch=s.batch;
  }
  void upload(float *d,const float *h,size_t n) {check(cudaMemcpyAsync(d,h,n*sizeof(float),cudaMemcpyHostToDevice,stream),"optimizer upload");}
  void read(float *h,const float *d,size_t n) {check(cudaMemcpyAsync(h,d,n*sizeof(float),cudaMemcpyDeviceToHost,stream),"optimizer readback");}
};
CudaOptimizerCore::CudaOptimizerCore():impl_(new Impl){}
CudaOptimizerCore::~CudaOptimizerCore()=default;
bool CudaOptimizerCore::available()const{return impl_->available;}
const std::string &CudaOptimizerCore::deviceName()const{return impl_->device;}
const OptimizerStatistics &CudaOptimizerCore::statistics()const{return impl_->stats;}
void CudaOptimizerCore::rollout(const CoreSettings &s,CoreBuffers h,CorePose p,uint64_t seed,
  uint64_t epoch,const float *nx,const float *ny,const float *nw) {
  auto &d=*impl_;int invalid=0;try {
    d.ensure(s);const size_t n=s.batch*s.steps;
    d.upload(d.b.ux,h.ux,s.steps);d.upload(d.b.uy,h.uy,s.steps);d.upload(d.b.uw,h.uw,s.steps);
    if(nx) {if(!ny || !nw)throw OptimizerComputeError("incomplete supplied noises");d.upload(d.nx,nx,n);d.upload(d.ny,ny,n);d.upload(d.nw,nw,n);}
    check(cudaMemsetAsync(d.invalid,0,sizeof(int),d.stream),"clear rollout status");
    sample<<<(n+255)/256,256,0,d.stream>>>(s,d.b,d.nx,d.ny,d.nw,seed,epoch,nx!=nullptr,d.invalid);
    check(cudaGetLastError(),"launch MPPI sampling");
    integrate<<<(s.batch+127)/128,128,0,d.stream>>>(s,d.b,p,d.invalid);
    check(cudaGetLastError(),"launch MPPI rollout");
    d.read(h.cvx,d.b.cvx,n);d.read(h.cvy,d.b.cvy,n);d.read(h.cwz,d.b.cwz,n);
    d.read(h.vx,d.b.vx,n);d.read(h.vy,d.b.vy,n);d.read(h.wz,d.b.wz,n);
    d.read(h.x,d.b.x,n);d.read(h.y,d.b.y,n);d.read(h.yaw,d.b.yaw,n);
    check(cudaMemcpyAsync(&invalid,d.invalid,sizeof(int),cudaMemcpyDeviceToHost,d.stream),"read rollout status");
    check(cudaStreamSynchronize(d.stream),"synchronize MPPI rollout");
    if(invalid)throw OptimizerComputeError("nonfinite MPPI sampled controls or trajectories");
    ++d.stats.cuda_rollout_batches;if(!nx)++d.stats.cuda_noise_batches;
  } catch(...) {if(d.stream)cudaStreamSynchronize(d.stream);++d.stats.failures;throw;}
}
void CudaOptimizerCore::update(const CoreSettings &s,CoreBuffers h) {
  auto &d=*impl_;int invalid=0;std::vector<float> out(3*s.steps+s.batch);try {
    d.ensure(s);const size_t n=s.batch*s.steps;
    // Upload explicitly: this also permits deterministic fixed-noise independent tests.
    d.upload(d.b.cvx,h.cvx,n);d.upload(d.b.cvy,h.cvy,n);d.upload(d.b.cwz,h.cwz,n);
    d.upload(d.b.ux,h.ux,s.steps);d.upload(d.b.uy,h.uy,s.steps);d.upload(d.b.uw,h.uw,s.steps);
    d.upload(d.b.cost,h.cost,s.batch);check(cudaMemsetAsync(d.invalid,0,sizeof(int),d.stream),"clear MPPI status");
    controlCost<<<(s.batch+127)/128,128,0,d.stream>>>(s,d.b);check(cudaGetLastError(),"launch MPPI control cost");
    weights<<<1,256,0,d.stream>>>(s,d.b,d.weight,d.invalid);check(cudaGetLastError(),"launch MPPI softmax");
    weightedControls<<<s.steps,256,0,d.stream>>>(s,d.b,d.weight);check(cudaGetLastError(),"launch MPPI update");
    check(cudaMemcpyAsync(&invalid,d.invalid,sizeof(int),cudaMemcpyDeviceToHost,d.stream),"read MPPI status");
    // Stage results so invalid device data cannot enter the optimizer's control sequence.
    d.read(out.data(),d.b.ux,s.steps);d.read(out.data()+s.steps,d.b.uy,s.steps);
    d.read(out.data()+2*s.steps,d.b.uw,s.steps);d.read(out.data()+3*s.steps,d.b.cost,s.batch);
    check(cudaStreamSynchronize(d.stream),"synchronize MPPI update");
    if(invalid)throw OptimizerComputeError("nonfinite MPPI costs or invalid softmax");
    std::copy_n(out.data(),s.steps,h.ux);std::copy_n(out.data()+s.steps,s.steps,h.uy);
    std::copy_n(out.data()+2*s.steps,s.steps,h.uw);std::copy_n(out.data()+3*s.steps,s.batch,h.cost);
    ++d.stats.cuda_update_batches;
  } catch(...) {if(d.stream)cudaStreamSynchronize(d.stream);++d.stats.failures;throw;}
}
void CudaOptimizerCore::scoreCritic(unsigned kind,float weight,unsigned power,float x,float y,
  float minv,float maxv,float radius,float dt,float *costs) {
  auto &d=*impl_;std::vector<float> out(d.batch);
  try {
    if(!d.available || !d.batch || kind>4)throw OptimizerComputeError("invalid CUDA critic state");
    d.upload(d.b.cost,costs,d.batch);
    criticKernel<<<(d.batch+127)/128,128,0,d.stream>>>(d.b,d.batch,d.steps,kind,weight,power,x,y,minv,maxv,radius,dt);
    check(cudaGetLastError(),"launch MPPI critic");d.read(out.data(),d.b.cost,d.batch);
    check(cudaStreamSynchronize(d.stream),"synchronize MPPI critic");
    std::copy(out.begin(),out.end(),costs);++d.stats.cuda_critic_batches;
  }catch(...){if(d.stream)cudaStreamSynchronize(d.stream);++d.stats.failures;throw;}
}
}
