#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "nav2_amcl/sensors/laser/laser.hpp"
using namespace nav2_amcl;
pf_vector_t zero(void *) { return {{0,0,0}}; }

map_t * wallmap(bool two) {
  map_t * m=map_alloc(); m->size_x=401; m->size_y=101; m->scale=.01;
  m->origin_x=0; m->origin_y=0;
  m->cells=static_cast<map_cell_t *>(calloc(m->size_x*m->size_y,sizeof(map_cell_t)));
  for(int i=0;i<m->size_x*m->size_y;++i) m->cells[i].occ_state=-1;
  std::vector<double> walls=two?std::vector<double>{.96,1.04}:std::vector<double>{1.0};
  for(double x:walls) for(int y=0;y<m->size_y;++y)
    m->cells[MAP_INDEX(m,static_cast<int>(MAP_GXWX(m,x)),y)].occ_state=1;
  return m;
}
struct Scores {std::vector<double> logf, weights; unsigned long long gpu;};
Scores score(map_t *m, const std::string &backend, int beams, const std::vector<double>&x) {
  LikelihoodFieldModelProb model(.85,.15,.025,2.,false,.3,.3,.9,120,m);
  model.configureObservationBackend(backend,0,0); auto pose=zero(nullptr); model.SetLaserPose(pose);
  LaserData d; d.laser=&model; d.range_count=beams; d.range_max=8.;
  d.ranges=new double[beams][2]; d.endpoints=new double[beams][2];
  for(int b=0;b<beams;++b) {d.ranges[b][0]=1.; d.ranges[b][1]=0.;d.endpoints[b][0]=1.;d.endpoints[b][1]=.001;
    d.measurement_states.push_back(1);d.selection_reasons.push_back(LaserData::SelectionReason::Included);}
  pf_t *pf=pf_alloc(1,x.size(),0,0,zero); pf_init_model(pf,zero,nullptr);
  auto &s=pf->sets[pf->current_set];
  for(size_t i=0;i<x.size();++i){s.samples[i].pose.v[0]=x[i];s.samples[i].weight=1./x.size();}
  if(!model.sensorUpdate(pf,&d)||pf->diagnostics.status!=PF_SENSOR_OK)throw std::runtime_error("score rejected");
  Scores out;
  for(size_t i=0;i<x.size();++i){out.logf.push_back(s.log_likelihoods[i]-std::log(1./x.size()));out.weights.push_back(s.samples[i].weight);}
  out.gpu=model.observationStats()->cuda_updates;pf_free(pf);return out;
}
void arr(const std::vector<double>&v){std::cout<<"[";for(size_t i=0;i<v.size();++i){if(i)std::cout<<",";std::cout<<v[i];}std::cout<<"]";}
int main(int argc,char**argv){
  const std::string backend=argc>1?argv[1]:"cpu";std::cout<<std::setprecision(17);
  map_t *m=wallmap(false);
  std::cout<<"{\"backend\":\""<<backend<<"\",\"duplicate_beams\":[";
  bool first=true;for(int n:{1,10,120}){auto s=score(m,backend,n,{.001,.021});if(!first)std::cout<<",";first=false;
    std::cout<<"{\"beams\":"<<n<<",\"log_odds\":"<<s.logf[0]-s.logf[1]<<",\"posterior_best\":"<<s.weights[0]<<",\"cuda_updates\":"<<s.gpu<<"}";}
  auto flat=score(m,backend,120,{-.004,-.001,.001,.004,.006});
  std::cout<<"],\"grid_plateau\":{\"x_m\":[-0.004,-0.001,0.001,0.004,0.006],\"log_likelihood\":";arr(flat.logf);std::cout<<",\"cuda_updates\":"<<flat.gpu<<"}";map_free(m);
  m=wallmap(true);auto modes=score(m,backend,120,{-.04,0.,.04});
  pf_t *pf=pf_alloc(700,3000,0,0,zero);pf->pop_z=pf_kld_probability_to_z(.99);pf_init_model(pf,zero,nullptr);
  auto set_modes=[&](){auto&s=pf->sets[pf->current_set];s.sample_count=3000;for(int i=0;i<3000;++i){s.samples[i].pose={{i<1500?-.04:.04,0.,0.}};s.samples[i].weight=1./3000;s.samples[i].lineage=i+1;}pf_recompute_stats(pf);};
  set_modes(); auto&s=pf->sets[pf->current_set];double before=s.clusters[0].mean.v[0];
  std::cout<<",\"merged_modes\":{\"cluster_count\":"<<s.cluster_count<<",\"mean_x_m\":"<<before<<",\"variance_x_m2\":"<<s.clusters[0].cov.m[0][0]<<",\"log_likelihood_peaks_and_mean\":";arr(modes.logf);
  std::cout<<",\"cuda_updates\":"<<modes.gpu<<"},\"resampling\":{\"trials\":1000,\"mean_before_m\":"<<before<<",\"means_after_m\":[";
  for(int trial=0;trial<1000;++trial){set_modes();srand48(10000+trial);pf_update_resample(pf,nullptr);auto&after=pf->sets[pf->current_set];if(trial)std::cout<<",";std::cout<<after.mean.v[0];}
  std::cout<<"],\"last_output_n\":"<<pf->sets[pf->current_set].sample_count<<"}}\n";
  pf_free(pf);map_free(m);return 0;
}
