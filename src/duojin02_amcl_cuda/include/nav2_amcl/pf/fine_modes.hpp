// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__PF__FINE_MODES_HPP_
#define NAV2_AMCL__PF__FINE_MODES_HPP_
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>
#include "nav2_amcl/pf/posterior_snapshot.hpp"
namespace nav2_amcl {
struct FineModeSet {
  std::vector<PosteriorCluster> modes;
  double position_bandwidth{0.01}, yaw_bandwidth{0.017453292519943295};
  double separation_m{0.0};
  int selected{-1};
};
inline double modeWrap(double angle) {return std::remainder(angle, 2.0 * M_PI);}
// Experimental binned Gaussian KDE watershed. Its scales describe diagnostic
// resolution, never sensor noise or physical accuracy. KLD is untouched.
inline FineModeSet resolveFineModes(const pf_sample_set_t & set, double hp, double ha) {
  if (!std::isfinite(hp) || !std::isfinite(ha) || hp <= 0 || ha <= 0 || ha > M_PI) {
    throw std::invalid_argument("fine mode bandwidths must be positive finite; yaw <= pi");
  }
  struct Cell {double mass{0}, x{0}, y{0}, cosine{0}, sine{0}; pf_vector_t pose{};};
  std::map<std::tuple<double, double, double>, Cell> bins;
  for (int i=0; i<set.sample_count; ++i) {
    const auto & p=set.samples[i];
    if (!std::isfinite(p.weight) || p.weight < 0) {throw std::invalid_argument("invalid diagnostic particle weight");}
    if (p.weight == 0) {continue;}
    if (!std::isfinite(p.pose.v[0]) || !std::isfinite(p.pose.v[1]) || !std::isfinite(p.pose.v[2])) {
      throw std::invalid_argument("invalid diagnostic particle pose");
    }
    auto & b=bins[{std::floor(p.pose.v[0]/(hp*.5)),std::floor(p.pose.v[1]/(hp*.5)),
      std::floor(modeWrap(p.pose.v[2])/(ha*.5))}];
    b.mass+=p.weight;b.x+=p.weight*p.pose.v[0];b.y+=p.weight*p.pose.v[1];
    b.cosine+=p.weight*std::cos(p.pose.v[2]);b.sine+=p.weight*std::sin(p.pose.v[2]);
  }
  std::vector<Cell> cells;
  for (auto & item:bins) {auto b=item.second;b.pose={{b.x/b.mass,b.y/b.mass,std::atan2(b.sine,b.cosine)}};cells.push_back(b);}
  auto distance=[&](const pf_vector_t & a,const pf_vector_t & b) {
    const double x=(a.v[0]-b.v[0])/hp,y=(a.v[1]-b.v[1])/hp,t=modeWrap(a.v[2]-b.v[2])/ha;
    return x*x+y*y+t*t;
  };
  const size_t n=cells.size();std::vector<double> density(n,0);std::vector<int> parent(n);
  for(size_t i=0;i<n;++i) for(size_t j=0;j<n;++j) density[i]+=cells[j].mass*std::exp(-.5*distance(cells[i].pose,cells[j].pose));
  // Follow the steepest higher KDE neighbor within three bandwidths. Equal
  // density plateaus have a deterministic lexicographic tie rule, no RNG draw.
  for(size_t i=0;i<n;++i) {parent[i]=static_cast<int>(i);double best=density[i];
    for(size_t j=0;j<n;++j) if(distance(cells[i].pose,cells[j].pose)<=9.0 &&
      (density[j]>best || (density[j]==best && j<static_cast<size_t>(parent[i])))) {parent[i]=j;best=density[j];}}
  auto root=[&](int i) {while(parent[i]!=i) {i=parent[i];}return i;};
  std::map<int,int> ids;for(size_t i=0;i<n;++i) {const int r=root(i);if(!ids.count(r)) ids[r]=ids.size();}
  FineModeSet out;out.position_bandwidth=hp;out.yaw_bandwidth=ha;
  out.modes.resize(ids.size(),{0.0,pf_vector_zero(),pf_matrix_zero(),true});
  std::vector<double> cosines(ids.size(),0),sines(ids.size(),0),minimum(ids.size(),INFINITY),maximum(ids.size(),-INFINITY);
  std::map<std::tuple<double,double,double>,int> labels;
  size_t cell_index=0;for(const auto & item:bins) labels[item.first]=ids[root(cell_index++)];
  auto label=[&](const pf_vector_t & p) {return labels.at({std::floor(p.v[0]/(hp*.5)),std::floor(p.v[1]/(hp*.5)),std::floor(modeWrap(p.v[2])/(ha*.5))});};
  double total=0;
  for(int i=0;i<set.sample_count;++i) {const auto & p=set.samples[i];if(!(p.weight>0))continue;const int k=label(p.pose);auto & m=out.modes[k];m.weight+=p.weight;total+=p.weight;m.mean.v[0]+=p.weight*p.pose.v[0];m.mean.v[1]+=p.weight*p.pose.v[1];cosines[k]+=p.weight*std::cos(p.pose.v[2]);sines[k]+=p.weight*std::sin(p.pose.v[2]);}
  for(size_t k=0;k<out.modes.size();++k) {auto &m=out.modes[k];m.mean.v[0]/=m.weight;m.mean.v[1]/=m.weight;m.mean.v[2]=std::atan2(sines[k],cosines[k]);}
  std::vector<double> yaw_offsets(ids.size(),0);
  for(int i=0;i<set.sample_count;++i) {const auto&p=set.samples[i];if(!(p.weight>0))continue;const int k=label(p.pose);const double d=modeWrap(p.pose.v[2]-out.modes[k].mean.v[2]);yaw_offsets[k]+=p.weight*d/out.modes[k].weight;minimum[k]=std::min(minimum[k],d);maximum[k]=std::max(maximum[k],d);}
  for(size_t k=0;k<out.modes.size();++k) {out.modes[k].mean.v[2]=modeWrap(out.modes[k].mean.v[2]+yaw_offsets[k]);out.modes[k].angular_chart_valid=maximum[k]-minimum[k]<M_PI;}
  for(int i=0;i<set.sample_count;++i) {const auto&p=set.samples[i];if(!(p.weight>0))continue;auto&m=out.modes[label(p.pose)];double d[3]={p.pose.v[0]-m.mean.v[0],p.pose.v[1]-m.mean.v[1],modeWrap(p.pose.v[2]-m.mean.v[2])};for(int j=0;j<3;++j)for(int k=0;k<3;++k)m.covariance.m[j][k]+=p.weight/m.weight*d[j]*d[k];}
  for(size_t k=0;k<out.modes.size();++k) {out.modes[k].weight/=total;if(out.selected<0 || out.modes[k].weight>out.modes[out.selected].weight)out.selected=k;}
  if(out.selected>=0)for(size_t k=0;k<out.modes.size();++k)if(static_cast<int>(k)!=out.selected)out.separation_m=std::max(out.separation_m,std::hypot(out.modes[k].mean.v[0]-out.modes[out.selected].mean.v[0],out.modes[k].mean.v[1]-out.modes[out.selected].mean.v[1]));
  return out;
}
}  // namespace nav2_amcl
#endif
