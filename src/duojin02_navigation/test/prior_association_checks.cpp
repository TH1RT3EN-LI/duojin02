#include "duojin02_navigation/guard_query.hpp"
#include "duojin02_navigation/prior_association_cache.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace duojin02_navigation;
int main() {
  PriorAssociationCache cache;
  PriorAssociationContext context{1,1,{},.025,.008};
  cache.begin(3,context);
  assert(cache.match(0,[]{return true;}));
  assert(!cache.match(1,[]{return false;}));
  cache.begin(3,context);
  assert(cache.match(0,[]{assert(false);return false;}));
  assert(!cache.match(1,[]{assert(false);return true;}));
  assert(cache.hits()==2 && cache.misses()==0);
  auto invalidate=[&]() {
    cache.begin(3,context);
    assert(!cache.match(0,[]{return false;}));
    assert(cache.hits()==0 && cache.misses()==1);
  };
  ++context.map_epoch;invalidate();
  ++context.observation_epoch;invalidate();
  context.map_pose.x=std::nextafter(0.,1.);invalidate();
  context.map_pose.y=-.01;invalidate();
  context.map_pose.yaw=.73;invalidate();
  context.localization_margin=std::nextafter(context.localization_margin,1.);invalidate();
  context.range_error=std::nextafter(context.range_error,1.);invalidate();
  context.map_pose.x=-0.;invalidate();
  context.map_pose.x=0.;invalidate();
  cache.begin(1,context);assert(cache.match(0,[]{return true;}));
  cache.begin(4,context);assert(!cache.match(0,[]{return false;}));
  cache.begin(0,context);cache.begin(4,context);
  assert(!cache.match(0,[]{return false;}));

  // Independent full-vector association is the reference. Compare every
  // result, ordered output point and complete measured/requested sweep after
  // observation, transform, uncertainty and same-size map replacement.
  std::mt19937 rng(20261009);
  std::uniform_real_distribution<double> coordinate(-.8,.8);
  Polygon polygon{{.21,.20},{-.21,.20},{-.21,-.20},{.21,-.20}};
  std::vector<Point> prior(120),observed(40);
  for(auto &p:prior) p={coordinate(rng),coordinate(rng),.007071067812};
  for(auto &p:observed) p={coordinate(rng),coordinate(rng),.02};
  size_t comparisons=0,sweeps=0,reuse=0;
  context={10,10,{},.025,.008};
  SpatialPointIndex observed_index(.05);observed_index.rebuild(observed);
  for(int trial=0;trial<72;++trial) {
    if(trial%4==0) {++context.observation_epoch;observed[trial%observed.size()].x=coordinate(rng);observed_index.rebuild(observed);}
    if(trial%12==0) {++context.map_epoch;for(auto &p:prior) p.x=coordinate(rng);}
    if(trial%8==0) context.map_pose={-.1,.07,coordinate(rng)};
    if(trial%10==0) context.localization_margin=trial%20==0 ? .16 : .025;
    cache.begin(prior.size(),context);
    std::vector<Point> reference=observed,candidate=observed;
    for(size_t i=0;i<prior.size();++i) {
      auto p=transform(prior[i],context.map_pose);
      const double radius=p.radius+std::min(.10,context.localization_margin)+context.range_error+.015;
      bool full=false;for(const auto &q:observed) if(std::hypot(p.x-q.x,p.y-q.y)<=radius) full=true;
      full=full && context.localization_margin<.15;
      const bool matched=context.localization_margin<.15 && cache.match(i,[&](){return observed_index.hasWithin(observed,p,radius);});
      assert(matched==full);++comparisons;
      p.radius+=context.localization_margin;
      if(!full) reference.push_back(p);
      if(!matched) candidate.push_back(p);
    }
    assert(reference.size()==candidate.size());
    for(size_t i=0;i<reference.size();++i) {
      assert(reference[i].x==candidate[i].x && reference[i].y==candidate[i].y && reference[i].radius==candidate[i].radius);
    }
    for(Velocity v:std::vector<Velocity>{{.35,.2,.4},{-.2,.1,-.5},{0,0,1.2},{0,0,0}}) {
      assert(brakingSweep({},v,polygon,reference,.008,.2,.4,.8)==brakingSweep({},v,polygon,candidate,.008,.2,.4,.8));++sweeps;
    }
    reuse+=cache.hits();
  }
  assert(reuse>0 && comparisons==8640 && sweeps==288);
  std::cout<<comparisons<<" association decisions and "<<sweeps<<" complete sweeps matched; epoch/bitwise context invalidation passed\n";
}
