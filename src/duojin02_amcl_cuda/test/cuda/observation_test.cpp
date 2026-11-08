// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
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
#include "scan_geometry_fixture.hpp"
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
  double error_threshold=.9, double beta=1.0, int max_beams=120) {
  std::unique_ptr<Laser> laser;
  if(prob) { laser=std::make_unique<LikelihoodFieldModelProb>(.94,.06,.15,std::min(1.5,30*map->scale),skip,.4,.3,error_threshold,max_beams,map); }
  else { laser=std::make_unique<LikelihoodFieldModel>(.94,.06,.15,std::min(1.5,30*map->scale),max_beams,map); }
  pf_vector_t offset{{.017,.008,.003}}; laser->SetLaserPose(offset);
  laser->setLikelihoodTemperatureBeta(beta);
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
void remediationCases(const std::string & backend) {
  double max_error = 0.0;
  // Actual observation products below exp(-10000) remain discriminative, and
  // beta=1/2 tempers only log likelihood, including nonuniform/zero priors.
  for (double beta : {1.0, .5}) {
    auto map=makeMap(); auto a=makePf(128), b=makePf(128);
    auto cpu=model(map.get(),true,false,"cpu",.9,beta,4096);
    auto accelerated=model(map.get(),true,false,backend,.9,beta,4096);
    auto & x=a->sets[0]; auto & y=b->sets[0];
    double mass=0;
    for (int j=0;j<x.sample_count;++j) {mass+=j;}
    for (int j=0;j<x.sample_count;++j) {x.samples[j].weight=y.samples[j].weight=j/mass;}
    LaserData data; scan(data,8192,2);
    data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data);
    data.laser=accelerated.get(); accelerated->sensorUpdate(b.get(),&data);
    max_error = std::max(max_error, compare(a.get(),b.get()));
    require(a->diagnostics.status==PF_SENSOR_OK,"extreme real likelihood was rejected");
    require(a->diagnostics.log_evidence < -1000,"extreme fixture did not reach exponential underflow");
    require(x.samples[0].weight==0 && y.samples[0].weight==0,"zero prior gained observation mass");
    const auto & stats=*accelerated->observationStats();
    require(backend!="cuda" || stats.cuda_updates==1,"extreme fixture did not execute the CUDA kernel");
    require(stats.valid_beams==stats.effective_beams && stats.effective_beams>3500,"actual beam diagnostics");
    require(stats.distance_p50>=0 && stats.distance_p99>=stats.distance_p50,"distance diagnostic quantiles");
    require(stats.random_floor_fraction>=0 && stats.random_floor_fraction<=1,"random floor diagnostic");
    require(stats.likelihood_temperature_beta==beta,"beta diagnostic changed");
  }
  for (bool prob : {false,true}) {
    auto map=makeMap(); auto a=makePf(128), b=makePf(128);
    auto cpu=model(map.get(),prob,false,"cpu"), accelerated=model(map.get(),prob,false,backend);
    LaserData data; scan(data,801,3); data.endpoints=new double[data.range_count][2];
    // These are already base_ref endpoints, including the full laser extrinsic.
    for(int i=0;i<data.range_count;++i) {
      const double bearing=data.ranges[i][1]+.003, range=data.ranges[i][0];
      const bool hit=std::isfinite(range) && range<data.range_max;
      const bool no_return=range>=data.range_max;
      data.measurement_states.push_back(hit ? 1 : (no_return ? 2 : 0));
      data.selection_reasons.push_back(hit ? LaserData::SelectionReason::Included :
        (no_return ? LaserData::SelectionReason::NoReturn : LaserData::SelectionReason::SensorInvalid));
      data.endpoints[i][0]=hit ? .017+range*std::cos(bearing) : 0;
      data.endpoints[i][1]=hit ? .008+range*std::sin(bearing) : 0;
    }
    data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data);
    data.laser=accelerated.get(); accelerated->sensorUpdate(b.get(),&data);
    max_error = std::max(max_error, compare(a.get(),b.get()));
    // A corrected endpoint length below max range must not turn a raw max-range
    // return into an accepted observation.
    for(int i=0;i<data.range_count;++i) {
      data.ranges[i][0]=data.range_max; data.endpoints[i][0]=.2; data.endpoints[i][1]=.3;
      data.measurement_states[i]=2;
      data.selection_reasons[i]=LaserData::SelectionReason::NoReturn;
    }
    std::vector<double> prior;
    for(int i=0;i<a->sets[0].sample_count;++i) {prior.push_back(a->sets[0].samples[i].weight);}
    data.laser=cpu.get(); cpu->sensorUpdate(a.get(),&data);
    data.laser=accelerated.get(); accelerated->sensorUpdate(b.get(),&data);
    max_error = std::max(max_error, compare(a.get(),b.get()));
    for(int i=0;i<a->sets[0].sample_count;++i) {
      require(std::abs(a->sets[0].samples[i].weight-prior[i])<1e-14,"deskew changed raw range cutoff");
    }
    if(prob) require(a->diagnostics.status==PF_SENSOR_NO_INFORMATION,"raw max-range scan was accepted");
  }
  auto map=makeMap();
  // A future manually-created view must not silently reconstruct rigid raw
  // geometry just because it forgot the prepared measurement validity fields.
  for (bool probability : {false, true}) {
    auto pf=makePf(128);
    auto laser=model(map.get(),probability,false,backend);
    LaserData data; scan(data,1,0); data.laser=laser.get();
    data.ranges[0][0]=1.; data.endpoints=new double[1][2]{{.2,.3}};
    auto rejects_view=[&] {
      bool rejected=false;
      try {laser->sensorUpdate(pf.get(),&data);} catch(const std::invalid_argument &) {rejected=true;}
      require(rejected && pf->diagnostics.update_count==0,"incomplete prepared view consumed raw fallback");
    };
    rejects_view();
    data.measurement_states={0}; data.selection_reasons={LaserData::SelectionReason::Included};
    rejects_view();
    data.measurement_states={1}; data.endpoints[0][0]=NAN;
    rejects_view();
  }
  for(double beta : {0.0,-1.0,1.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;
    try {auto laser=model(map.get(),true,false,"cpu",.9,beta);} catch(const std::invalid_argument &) {rejected=true;}
    require(rejected,"invalid beta accepted");
  }
  std::cout<<"AMCL_REMEDIATION_RESULT backend="<<backend<<" beta_cases=2 deskew_cases=2 paired_frames=6 max_weight_error="<<max_error<<" raw_cutoff=preserved\n";
}
void realReferenceInvariance(const std::string & backend) {
  namespace geometry = duojin02_scan_geometry;
  RecordedScanFixture fixture(DUOJIN02_RECORDED_SCAN_FIXTURE);
  const auto first=fixture.prepare(0.0);
  const geometry::Pose map_from_odom=geometry::compose({.013,.017,.21},
    geometry::inverse(first.odom_from_base_reference));
  Map map(map_alloc()); map->size_x=601; map->size_y=601; map->scale=.04;
  map->origin_x=.0063; map->origin_y=-.0087;
  map->cells=static_cast<map_cell_t *>(calloc(map->size_x*map->size_y,sizeof(map_cell_t)));
  for(int i=0;i<map->size_x*map->size_y;++i) {map->cells[i].occ_state=-1;}
  std::vector<geometry::Point> world_endpoints(first.beams.size()), world_origins(first.beams.size());
  int invalid_count=0;
  for(const auto & beam : first.beams) {
    if (!beam.valid()) {++invalid_count; continue;}
    const auto world=geometry::compose(map_from_odom,first.odom_from_base_reference);
    const auto endpoint=geometry::apply(world,beam.endpointPoint());
    world_endpoints[beam.original_index]=endpoint;
    world_origins[beam.original_index]=geometry::apply(world,beam.originPoint());
    if (beam.hasReturn() && beam.raw_range>.3 && beam.raw_range<8) {
      const double gx=MAP_GXWX(map,endpoint.x), gy=MAP_GYWY(map,endpoint.y);
      if(gx>=0 && gy>=0 && gx<map->size_x && gy<map->size_y) {
        map->cells[MAP_INDEX(map,static_cast<int>(gx),static_cast<int>(gy))].occ_state=1;
      }
    }
  }
  require(invalid_count>=4,"real fixture lost recorded finite short beams");
  double world_error=0, weight_error=0, likelihood_error=0;
  uint64_t cuda_updates=0;
  for(bool probability : {false,true}) {
    std::vector<double> reference_weights, reference_logs;
    for(double fraction : {0.,.5,1.}) {
      auto prepared=fixture.prepare(fraction);
      std::unique_ptr<Laser> laser;
      if(probability) {laser=std::make_unique<LikelihoodFieldModelProb>(.85,.15,.025,.6,false,.5,.3,.9,120,map.get());}
      else {laser=std::make_unique<LikelihoodFieldModel>(.85,.15,.025,.6,120,map.get());}
      laser->configureObservationBackend(backend,0,0);
      // A deliberately nonzero extrinsic must be ignored for base_ref endpoints.
      pf_vector_t deliberate_extrinsic{{.305,.021,.18}};
      laser->SetLaserPose(deliberate_extrinsic);
      auto pf=makePf(128); auto & set=pf->sets[pf->current_set];
      const auto reference_pose=geometry::compose(map_from_odom,prepared.odom_from_base_reference);
      for(int j=0;j<set.sample_count;++j) {
        const geometry::Pose physical_error{.04*std::sin(j*.17),.04*std::cos(j*.29),.01*std::sin(j*.23)};
        const auto candidate=geometry::compose(physical_error,reference_pose);
        set.samples[j].pose={{candidate.x,candidate.y,candidate.yaw}};
        set.samples[j].weight=1./set.sample_count;
      }
      LaserData data; data.laser=laser.get(); data.range_count=prepared.beams.size(); data.range_max=8;
      data.ranges=new double[data.range_count][2]; data.endpoints=new double[data.range_count][2];
      for(const auto & beam : prepared.beams) {
        const auto i=beam.original_index;
        data.ranges[i][0]=beam.raw_range;
        data.ranges[i][1]=fixture.angle_min+i*fixture.angle_increment;
        data.measurement_states.push_back(static_cast<uint8_t>(beam.state));
        using Selection=LaserData::SelectionReason;
        const auto reason=!beam.valid() ? Selection::SensorInvalid :
          (!beam.hasReturn() ? Selection::NoReturn :
          (beam.raw_range<=.3 ? Selection::BelowAlgorithmMinimum :
          (beam.raw_range>=8 ? Selection::AtOrAboveAlgorithmMaximum : Selection::Included)));
        data.selection_reasons.push_back(reason);
        const auto endpoint=beam.valid() ? beam.endpointPoint() : geometry::Point{};
        data.endpoints[i][0]=endpoint.x; data.endpoints[i][1]=endpoint.y;
        if (beam.valid()) {
          const auto endpoint_world=geometry::apply(reference_pose,beam.endpointPoint());
          const auto origin_world=geometry::apply(reference_pose,beam.originPoint());
          world_error=std::max(world_error,std::hypot(endpoint_world.x-world_endpoints[i].x,
            endpoint_world.y-world_endpoints[i].y));
          world_error=std::max(world_error,std::hypot(origin_world.x-world_origins[i].x,
            origin_world.y-world_origins[i].y));
        }
      }
      laser->sensorUpdate(pf.get(),&data);
      require(pf->diagnostics.status==PF_SENSOR_OK,"reference invariance observation rejected");
      for(int j=0;j<set.sample_count;++j) {
        if(fraction==0) {
          reference_weights.push_back(set.samples[j].weight);
          reference_logs.push_back(set.log_likelihoods[j]);
        } else {
          weight_error=std::max(weight_error,std::abs(reference_weights[j]-set.samples[j].weight));
          likelihood_error=std::max(likelihood_error,std::abs(reference_logs[j]-set.log_likelihoods[j]));
        }
      }
      cuda_updates+=laser->observationStats()->cuda_updates;
    }
  }
  require(world_error<1e-11,"physical world rays changed with the reference time");
  require(weight_error<1e-11 && likelihood_error<1e-9,"native likelihood changed for mapped physical candidates");
  require(backend!="cuda" || cuda_updates==6,"real reference invariance did not execute CUDA in all six cases");
  std::cout<<"AMCL_REAL_REFERENCE_INVARIANCE source_scan_id="<<first.source_scan_id
    <<" invalid_recorded="<<invalid_count<<" reference_cases=6 candidate_count=128 world_ray_max_error_m="
    <<world_error<<" weight_max_error="<<weight_error<<" log_factor_max_error="<<likelihood_error
    <<" cuda_updates="<<cuda_updates<<" clock_evidence=uncalibrated\n";
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
    remediationCases(backend);
    realReferenceInvariance(backend);
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
