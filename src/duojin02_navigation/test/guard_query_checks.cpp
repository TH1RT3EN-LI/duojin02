#include "duojin02_navigation/guard_query.hpp"
#include "duojin02_navigation/local_grid_snapshot.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace duojin02_navigation;
int main() {
  Polygon polygon{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},
    {-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  std::mt19937 rng(20261009);std::uniform_real_distribution<double> coordinate(-2.4,2.4),velocity(-.65,.65);
  size_t clear=0,blocked=0,compared=0;
  for(int trial=0;trial<64;++trial) {
    Pose pose{coordinate(rng),coordinate(rng),coordinate(rng)};
    Pose map_tf{-1.2,.8,.73};
    const double c=std::cos(map_tf.yaw),s=std::sin(map_tf.yaw);
    const double mx=c*(pose.x-map_tf.x)+s*(pose.y-map_tf.y);
    const double my=-s*(pose.x-map_tf.x)+c*(pose.y-map_tf.y);
    Velocity measured{velocity(rng),velocity(rng),velocity(rng)*2};
    Velocity requested{velocity(rng),velocity(rng),velocity(rng)*2};
    // Complete occupied cell extents are part of the query bound.
    const double cell_radius=trial%3==0 ? .06 : .007071067812;
    std::vector<Point> map_points,full,pruned;
    for(int i=0;i<80;++i) {
      Point point{pose.x+coordinate(rng),pose.y+coordinate(rng),cell_radius};
      if(std::hypot(point.x-pose.x,point.y-pose.y)>2.5) continue;
      const double dx=point.x-map_tf.x,dy=point.y-map_tf.y;
      map_points.push_back({c*dx+s*dy,-s*dx+c*dy,cell_radius});
      full.push_back(point);
    }
    SpatialPointIndex index(.10);index.rebuild(map_points);
    const double radius=guardQueryRadius(polygon,measured,requested,.008,.20,.4,
      index.maxRadius(),2.5);
    index.forEachNear(map_points,mx,my,radius,[&](const Point &q){pruned.push_back(transform(q,map_tf));});
    auto compare=[&](Velocity v) {
      const bool original=brakingSweep(pose,v,polygon,full,.008,.20,.4,.8);
      const bool optimized=brakingSweep(pose,v,polygon,pruned,.008,.20,.4,.8);
      assert(original==optimized);original ? ++blocked : ++clear;++compared;
    };
    compare(measured);
    for(double scale:{1.,.8,.6,.4,.25,.15,.08,.04}) {
      Velocity v{requested.x*scale,requested.y*scale,requested.yaw*scale};
      compare(v);
      for(double f:{.25,.5,.75})
        compare({measured.x+f*(v.x-measured.x),measured.y+f*(v.y-measured.y),measured.yaw+f*(v.yaw-measured.yaw)});
    }
  }
  assert(clear>0 && blocked>0 && compared==2112);
  // Original scan association predicate, including negative tile boundaries.
  std::vector<Point> scan{{-.1,-.05,0},{.2,.1,.08},{-.05,0,.03}};
  SpatialPointIndex observed(.05);observed.rebuild(scan);
  for(int i=0;i<1000;++i) {
    Point q{coordinate(rng)/4,coordinate(rng)/4,0};double radius=.01+(i%15)*.01;
    bool expected=false;for(const auto &p:scan) if(std::hypot(p.x-q.x,p.y-q.y)<=radius) expected=true;
    assert(observed.hasWithin(scan,q,radius)==expected);
  }
  // A newly rebuilt map must not retain occupied cells from the previous map.
  observed.rebuild({});assert(!observed.hasWithin({},scan[0],.1));
  assert(guardQueryRadius(polygon,{4,3,2},{0,0,0},.008,.2,.4,.01,2.5)==2.5);
  LocalGridSnapshot grid;grid.width=17;grid.height=11;grid.resolution=.01;
  grid.origin_x=-.17;grid.origin_y=-.11;grid.data.resize(grid.width*grid.height);
  for(size_t i=0;i<grid.data.size();++i) grid.data[i]=i%5==0 ? 255 : i%3==0 ? 120 : 0;
  for(unsigned int y=0;y<grid.height;++y) for(unsigned int x=0;x<grid.width;++x)
    assert(grid.nearObstacle(grid.origin_x+(x+.5)*grid.resolution,grid.origin_y+(y+.5)*grid.resolution)==(grid.data[y*grid.width+x]>=120));
  grid.crop_x=5;grid.crop_y=3;
  for(int i=0;i<10000;++i) {
    const double x=grid.origin_x+grid.resolution*(5+(i%17)+(i%2 ? 1e-12 : 0));
    const double y=grid.origin_y+grid.resolution*(3+(i%11)+(i%2 ? 1e-12 : 0));
    const auto mx=static_cast<unsigned int>((x-grid.origin_x)/grid.resolution);
    const auto my=static_cast<unsigned int>((y-grid.origin_y)/grid.resolution);
    const bool outside=mx<grid.crop_x || my<grid.crop_y || mx>=grid.crop_x+grid.width || my>=grid.crop_y+grid.height;
    const bool expected=outside || grid.data[(my-grid.crop_y)*grid.width+mx-grid.crop_x]>=120;
    assert(grid.nearObstacle(x,y)==expected);
  }
  assert(grid.nearObstacle(-1,0));assert(grid.nearObstacle(NAN,0));
  std::cout<<"Compared "<<compared<<" complete braking/transition sweeps ("<<clear<<" clear, "<<blocked<<" blocked); association, map replacement and snapshot bytes passed\n";
}
