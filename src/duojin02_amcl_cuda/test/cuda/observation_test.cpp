// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#include "nav2_amcl/sensors/laser/laser.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
using namespace nav2_amcl;
void require(bool condition, const std::string & message) { if (!condition) { throw std::runtime_error(message); } }
struct MapDeleter { void operator()(map_t * p) const { map_free(p); } };
struct PfDeleter { void operator()(pf_t * p) const { pf_free(p); } };
using Map=std::unique_ptr<map_t,MapDeleter>;
using Pf=std::unique_ptr<pf_t,PfDeleter>;
Map makeMap(int variant=0) {
  Map map(map_alloc()); map->size_x=151; map->size_y=129; map->scale=.05;
  map->origin_x=variant ? 1234.013 : .013; map->origin_y=variant ? -987.017 : -.017;
  map->cells=static_cast<map_cell_t *>(calloc(map->size_x*map->size_y,sizeof(map_cell_t)));
  for(int y=0;y<map->size_y;++y) for(int x=0;x<map->size_x;++x) {
    const bool wall=(x==12 || x==140 || y==7 || y==120 || (x==93+variant && y>35 && y<95));
    map->cells[MAP_INDEX(map.get(),x,y)].occ_state=wall ? 1 : ((x<3) ? 0 : -1);
  }
  return map;
}
pf_vector_t sample(void * value) {
  auto & index=*static_cast<int *>(value); const int i=index++;
  return {{ .3*std::sin(i*1.17), .22*std::cos(i*.37), .1*std::sin(i*.23)}};
}
Pf makePf(int n, double ox=0, double oy=0) {
  Pf pf(pf_alloc(70,n,0,0,sample)); int index=0; pf_init_model(pf.get(),sample,&index);
  auto & set=pf->sets[pf->current_set];
  for(int i=0;i<set.sample_count;++i) { set.samples[i].pose.v[0]+=ox; set.samples[i].pose.v[1]+=oy; }
  pf->pop_err=.05; pf->pop_z=.99;
  return pf;
}
std::unique_ptr<Laser> model(map_t * map, bool prob, bool skip, const std::string & backend,
  double error_threshold=.9) {
  std::unique_ptr<Laser> laser;
  if(prob) { laser=std::make_unique<LikelihoodFieldModelProb>(.94,.06,.15,std::min(1.5,30*map->scale),skip,.4,.3,error_threshold,120,map); }
  else { laser=std::make_unique<LikelihoodFieldModel>(.94,.06,.15,std::min(1.5,30*map->scale),120,map); }
  pf_vector_t offset{{.017,.008,.003}}; laser->SetLaserPose(offset);
  laser->configureObservationBackend(backend,0,0); return laser;
}
void scan(LaserData & data, int count, int variant) {
  data.range_count=count; data.range_max=8; data.ranges=new double[count][2];
  for(int i=0;i<count;++i) {
    data.ranges[i][0]=1.4+.2*std::sin(i*.19+variant*.2);
    data.ranges[i][1]=-2.2+4.4*i/std::max(1,count-1);
    if(i%71==0) data.ranges[i][0]=std::numeric_limits<double>::quiet_NaN();
    if(i%97==0) data.ranges[i][0]=8;
    if(i%113==0) data.ranges[i][0]=std::numeric_limits<double>::infinity();
  }
}
double compare(pf_t * a,pf_t * b) {
  auto & x=a->sets[a->current_set]; auto & y=b->sets[b->current_set];
  require(x.sample_count==y.sample_count,"KLD particle count mismatch");
  double error=0;
  for(int i=0;i<x.sample_count;++i) {
    error=std::max(error,std::abs(x.samples[i].weight-y.samples[i].weight));
    for(int j=0;j<3;++j) require(x.samples[i].pose.v[j]==y.samples[i].pose.v[j],"resampled pose mismatch");
  }
  require(error<2e-11,"normalized particle weight mismatch: "+std::to_string(error));
  return error;
}
void benchmarkCuda() {
  for(int count: {700,3000}) {
    auto map=makeMap(); auto cpu=model(map.get(),true,false,"cpu");
    auto gpu=model(map.get(),true,false,"cuda");
    auto a=makePf(count,map->origin_x,map->origin_y), b=makePf(count,map->origin_x,map->origin_y);
    LaserData data; scan(data,720,7);
    std::vector<double> cpu_times, gpu_times;
    double cold_cpu=0, cold_gpu=0, max_error=0;
    for(int frame=0;frame<35;++frame) {
      auto invoke=[&](Laser * laser,pf_t * pf) {
        auto & set=pf->sets[pf->current_set];
        for(int i=0;i<set.sample_count;++i) set.samples[i].weight=1.0/set.sample_count;
        data.laser=laser;
        auto start=std::chrono::steady_clock::now(); laser->sensorUpdate(pf,&data);
        return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
      };
      double tc,tg;
      if(frame%2) { tg=invoke(gpu.get(),b.get()); tc=invoke(cpu.get(),a.get()); }
      else { tc=invoke(cpu.get(),a.get()); tg=invoke(gpu.get(),b.get()); }
      if(frame==0) { cold_cpu=tc; cold_gpu=tg; }
      if(frame>=5) { cpu_times.push_back(tc); gpu_times.push_back(tg); }
      max_error=std::max(max_error,compare(a.get(),b.get()));
    }
    std::sort(cpu_times.begin(),cpu_times.end()); std::sort(gpu_times.begin(),gpu_times.end());
    require(gpu->observationStats()->cuda_updates==35,"benchmark did not use device for all calls");
    std::cout<<"AMCL_TIMING_RESULT particles="<<count<<" selected_beams=120 samples=30 warmup=5"
      <<" cpu_median_us="<<(cpu_times[14]+cpu_times[15])/2
      <<" gpu_median_us="<<(gpu_times[14]+gpu_times[15])/2
      <<" cpu_p95_us="<<cpu_times[28]<<" gpu_p95_us="<<gpu_times[28]
      <<" cold_cpu_us="<<cold_cpu<<" cold_gpu_us="<<cold_gpu
      <<" max_weight_error="<<max_error<<" cuda_updates=35 scope=sensorUpdate_without_resampling\n";
  }
}
int main(int argc,char ** argv) {
  try {
    const std::string backend=argc>1 ? argv[1] : "cpu";
    if(backend=="unavailable") {
      bool refused=false; try { ObservationBackend::validate("cuda"); } catch(const std::exception &) { refused=true; }
      require(refused,"strict CUDA accepted unavailable device"); return 0;
    }
    if(backend=="cuda" || backend=="benchmark") { try { ObservationBackend::validate("cuda"); } catch(const std::exception & e) {
      std::cerr<<"SKIP no CUDA device: "<<e.what()<<'\n'; return 77;
    } }
    if(backend=="benchmark") { benchmarkCuda(); return 0; }
    double max_error=0; uint64_t cuda_updates=0;
    for(bool prob: {false,true}) for(bool skip: {false,true}) for(int variant=0;variant<3;++variant) {
      if(!prob && skip) continue;
      auto map=makeMap(variant==2); auto cpu=model(map.get(),prob,skip,"cpu",variant==1 ?.01 :.9);
      auto accelerated=model(map.get(),prob,skip,backend,variant==1 ?.01 :.9);
      auto a=makePf(1536,map->origin_x,map->origin_y), b=makePf(1536,map->origin_x,map->origin_y);
      for(int frame=0;frame<5;++frame) {
        a->sets[a->current_set].converged=b->sets[b->current_set].converged=(frame!=0);
        LaserData data; scan(data,variant==1 ? 37 : 801,frame);
        if(frame==4) for(int i=0;i<data.range_count;++i) data.ranges[i][0]=data.range_max;
        data.laser=cpu.get(); require(cpu->sensorUpdate(a.get(),&data),"CPU update rejected");
        data.laser=accelerated.get(); require(accelerated->sensorUpdate(b.get(),&data),"GPU update rejected");
        max_error=std::max(max_error,compare(a.get(),b.get()));
        int index=0; srand48(4242+frame); pf_update_resample(a.get(),&index);
        index=0; srand48(4242+frame); pf_update_resample(b.get(),&index);
        compare(a.get(),b.get());
      }
      cuda_updates+=accelerated->observationStats()->cuda_updates;
    }
    // Exact cell-boundary case must reach the native CPU oracle even in strict mode.
    {
      auto map=makeMap(); auto a=makePf(128), b=makePf(128);
      auto cpu=model(map.get(),true,false,"cpu"), gpu=model(map.get(),true,false,backend);
      pf_vector_t zero{{0,0,0}}; cpu->SetLaserPose(zero); gpu->SetLaserPose(zero);
      a->sets[0].samples[0].pose=zero; b->sets[0].samples[0].pose=zero;
      LaserData data; scan(data,1,0); data.ranges[0][0]=map->origin_x+.025; data.ranges[0][1]=0;
      data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data); data.laser=gpu.get(); gpu->sensorUpdate(b.get(),&data); compare(a.get(),b.get());
      if(backend=="cuda") require(gpu->observationStats()->boundary_fallbacks==1,"boundary guard did not execute");
    }
    // Empty scans preserve the prior; extreme map offset/resolution boundaries
    // are checked against the native oracle without claiming a roundoff proof.
    for (bool prob: {false,true}) {
      auto map=makeMap(); auto a=makePf(128), b=makePf(128);
      auto cpu=model(map.get(),prob,false,"cpu"), gpu=model(map.get(),prob,false,backend);
      LaserData data; scan(data,0,0);
      data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data);
      data.laser=gpu.get(); gpu->sensorUpdate(b.get(),&data); compare(a.get(),b.get());
      require(std::abs(b->sets[0].samples[0].weight-1.0/128)<1e-15,"empty scan changed prior");
    }
    {
      auto map=makeMap(1); map->scale=1e-5;
      auto a=makePf(128,map->origin_x,map->origin_y), b=makePf(128,map->origin_x,map->origin_y);
      auto cpu=model(map.get(),true,false,"cpu"), gpu=model(map.get(),true,false,backend);
      pf_vector_t zero{{0,0,0}}; cpu->SetLaserPose(zero); gpu->SetLaserPose(zero);
      auto pose=pf_vector_t{{map->origin_x,map->origin_y,0}};
      a->sets[0].samples[0].pose=pose; b->sets[0].samples[0].pose=pose;
      LaserData data; scan(data,3,0);
      for(int i=0;i<3;++i) { data.ranges[i][0]=map->scale*(.5+(i-1)*1e-7); data.ranges[i][1]=0; }
      data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data);
      data.laser=gpu.get(); gpu->sensorUpdate(b.get(),&data); compare(a.get(),b.get());
    }
    require(backend!="cuda" || cuda_updates>0,"CUDA path did not execute");
    std::cout<<"AMCL_DIFFERENTIAL_RESULT backend="<<backend<<" cases=13 frames=49 max_weight_error="<<max_error<<" cuda_updates="<<cuda_updates<<" KLD=equal\n";
    return 0;
  } catch(const std::exception & error) { std::cerr<<error.what()<<'\n'; return 1; }
}
