// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#define main legacyProbeMain
#include "legacy_probe.cpp"
#undef main
#include "nav2_amcl/pf/fine_modes.hpp"
#include "nav2_amcl/occupancy_identity.hpp"
#include <chrono>
void check(bool v,const char*s){if(!v)throw std::runtime_error(s);}
int main(int argc,char**argv) {
 const std::string backend=argc>1?argv[1]:"cpu";
 using namespace nav2_amcl;
 check(occupancyIdentity("map",2,1,.05,{0,-.12345678912345678,0,0,0,0,1},{0,100})=="occupancy_fnv1a64:526bf651ce37d530","map identity mismatch with Python");
 auto*pf=pf_alloc(2,1000,0,0,zero);pf_init_model(pf,zero,nullptr);
 auto&s=pf->sets[pf->current_set];s.sample_count=1000;
 for(double gap:{.04,.10,.20}) {
  for(int i=0;i<1000;++i){s.samples[i].pose={{i<500?-gap*.5:gap*.5,0,0}};s.samples[i].weight=.001;}
  pf_recompute_stats(pf);const int bins=s.kdtree->leaf_count;
  srand48(77);const auto fine=resolveFineModes(s,.01,M_PI/180);const double next=drand48();srand48(77);
  check(next==drand48(),"diagnostic consumed RNG");
  check(s.kdtree->leaf_count==bins,"fine modes changed KLD bins");
  check(s.cluster_count==1 && fine.modes.size()==2,"failed coarse merged/fine separated counterexample");
  check(std::abs(fine.modes[0].weight-.5)<1e-12 && std::abs(fine.separation_m-gap)<1e-12,"fine mode mass/separation wrong");
 }
 for(int i=0;i<1000;++i){s.samples[i].pose={{.001+1e-4*(i%3),0,i<500?M_PI-.004:-M_PI+.004}};}
 auto circle=resolveFineModes(s,.01,M_PI/180);check(circle.modes.size()==1,"wrap split one yaw mode");
 check(std::abs(std::abs(circle.modes[0].mean.v[2])-M_PI)<1e-10,"wrap yaw mean wrong");
 check(circle.modes[0].angular_chart_valid,"wrap chart incorrectly invalid");
 map_t *m=wallmap(true);
 double maximum_error=0,gap_score=0;unsigned long long updates=0;
 for(bool beam_skip:{false,true}) for(int beams:{1,10,120}) {
  LikelihoodFieldModelProb model(.85,.15,.025,2,beam_skip,.3,.3,.9,120,m);model.configureObservationBackend(backend,0,0);auto pose=zero(nullptr);model.SetLaserPose(pose);
  LaserData d;d.laser=&model;d.range_count=beams;d.range_max=8;d.ranges=new double[beams][2];d.endpoints=new double[beams][2];
  for(int b=0;b<beams;++b){d.ranges[b][0]=(beam_skip && b%4==3)?1.501:1;d.ranges[b][1]=0;d.endpoints[b][0]=d.ranges[b][0];d.endpoints[b][1]=.001;d.measurement_states.push_back(1);d.selection_reasons.push_back(LaserData::SelectionReason::Included);}
  s.sample_count=3;for(int i=0;i<3;++i){s.samples[i].pose={{(i-1)*.04,0,0}};s.samples[i].weight=1./3;}
  s.converged=1;
  check(model.sensorUpdate(pf,&d) && pf->diagnostics.status==PF_SENSOR_OK,"sensor update failed");
  const auto before_updates=pf->diagnostics.update_count;const auto count=model.observationStats()->cuda_updates;
  for(int i=0;i<3;++i){const double value=model.scorePoseOnLastSupport(d,s.samples[i].pose);maximum_error=std::max(maximum_error,std::abs(value-(s.log_likelihoods[i]-std::log(1./3))));}
  check(pf->diagnostics.update_count==before_updates && model.observationStats()->cuda_updates==count,"rescoring consumed observation again");
  gap_score=model.scorePoseOnLastSupport(d,{{-.04,0,0}})-model.scorePoseOnLastSupport(d,{{0,0,0}});updates+=count;
 }
 check(maximum_error<1e-10,"same support re-score mismatch");check(gap_score>100,"diagnostic failed low-score mean counterexample");
 pf_free(pf);map_free(m);
 std::cout<<std::setprecision(17)<<"{\"passed\":true,\"backend\":\""<<backend<<"\",\"double_peak_gaps_m\":[0.04,0.1,0.2],\"single_mode_wrap\":true,\"kld_bins_unchanged\":true,\"map_identity_python_fixture\":true,\"max_same_support_score_error\":"<<maximum_error<<",\"peak_minus_mean_score\":"<<gap_score<<",\"cuda_updates\":"<<updates<<",\"physical_precision\":\"unvalidated\"}\n";
}
