#include "duojin01_navigation/guard_input_validity.hpp"
#include <cassert>
#include <iostream>

using namespace duojin01_navigation;
struct Grid {
  struct Info {
    uint32_t width{8},height{6};
    double resolution{.02};
    struct Origin {
      struct Position {double x{0},y{0},z{0};} position;
      struct Orientation {double x{0},y{0},z{0},w{1};} orientation;
    } origin;
  } info;
  std::vector<int8_t> data=std::vector<int8_t>(48,0);
};
int main() {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  Grid good;
  assert(guardGridValid(good));
  assert(guardGridExtentValid(good,{}, {},.10));
  for(double value:{nan,inf,-inf}) {
    auto bad=good;bad.info.resolution=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.position.x=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.position.y=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.position.z=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.orientation.x=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.orientation.y=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.orientation.z=value;assert(!guardGridValid(bad));
    bad=good;bad.info.origin.orientation.w=value;assert(!guardGridValid(bad));
    assert(!guardGridExtentValid(good,{value,0,0},{},.10));
    assert(!guardGridExtentValid(good,{0,value,0},{},.10));
    assert(!guardGridExtentValid(good,{0,0,value},{},.10));
    assert(!guardGridExtentValid(good,{}, {value,0,0},.10));
    assert(!guardIndexQueryValid(value,0,1,.10,true));
    assert(!guardIndexQueryValid(0,value,1,.05,false));
    assert(!guardIndexQueryValid(0,0,value,.10,true));
  }
  for(double resolution:{0.,-.02}) {
    auto bad=good;bad.info.resolution=resolution;assert(!guardGridValid(bad));
  }
  auto bad=good;bad.info.width=0;assert(!guardGridValid(bad));
  bad=good;bad.info.height=0;assert(!guardGridValid(bad));
  bad=good;bad.data.pop_back();assert(!guardGridValid(bad));
  bad=good;bad.info.width=bad.info.height=std::numeric_limits<uint32_t>::max();assert(!guardGridValid(bad));
  bad=good;bad.info.origin.orientation.w=0;assert(!guardGridValid(bad));
  bad=good;bad.info.origin.orientation.x=.1;assert(!guardGridValid(bad));
  bad=good;bad.info.origin.orientation.y=.1;assert(!guardGridValid(bad));
  bad=good;bad.info.origin.orientation.w=1.01;assert(!guardGridValid(bad));
  good.info.origin.orientation.z=std::sin(.35);
  good.info.origin.orientation.w=std::cos(.35);
  assert(guardGridValid(good));
  // Rejection does not depend on occupancy: empty bad extents are invalid too.
  assert(!guardGridExtentValid(good,{1e30,0,.7},{},.10));
  assert(!guardGridExtentValid(good,{0,1e30,.7},{},.10));
  assert(!guardGridExtentValid(good,{}, {1e30,0,.7},.10));
  bad=good;bad.info.resolution=1e308;assert(!guardGridValid(bad));
  for(double bin:{.10,.05,.005}) {
    assert(guardIndexCoordinateValid(1e6,bin));
    assert(guardIndexCoordinateValid(-1e6,bin));
    assert(!guardIndexCoordinateValid(1e30,bin));
    assert(!guardIndexCoordinateValid(-1e30,bin));
    assert(!guardIndexCoordinateValid(std::nextafter(std::numeric_limits<int>::max()*bin,inf),bin));
    assert(guardIndexCoordinateValid((std::numeric_limits<int>::max()-2.)*bin,bin));
    assert(guardIndexCoordinateValid((std::numeric_limits<int>::min()+2.)*bin,bin));
    assert(!guardIndexQueryValid((std::numeric_limits<int>::max()-2.)*bin,0,4*bin,bin,true));
    assert(!guardIndexQueryValid((std::numeric_limits<int>::min()+2.)*bin,0,4*bin,bin,false));
    assert(guardIndexQueryValid(12,-13,.2,bin,true));
    assert(guardIndexQueryValid(12,-13,.2,bin,false));
  }
  assert(!guardIndexCoordinateValid(0,0));
  assert(!guardPointValid({0,0,-.1},.10));
  assert(!guardPointValid({0,0,nan},.10));
  assert(!guardIndexQueryValid(0,0,-.1,.10,true));
  // Ordinary rotated grids and TF translations accept every original point.
  const Pose origin{-3.2,2.1,.7},frame{1.5,-2.8,-.3};
  assert(guardGridExtentValid(good,origin,frame,.10));
  for(size_t i=0;i<good.data.size();++i) {
    const Point p{(i%good.info.width+.5)*good.info.resolution,
      (i/good.info.width+.5)*good.info.resolution,std::sqrt(2.0)*good.info.resolution/2};
    assert(guardPointValid(transform(transform(p,origin),frame),.10));
  }
  std::cout<<"Grid metadata, finite planar quaternion, transformed extent, int index/query bounds and ordinary rotated grids passed\n";
}
