#pragma once
// Frozen round-before class; source header SHA256 f179fcd6b6d8e3f5b7420c4f0bc18f12427e4e340e41bc4e0eed45eed94d964f
#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <unordered_map>
namespace duojin02_navigation {
class ReferenceSpatialPointIndex {
 public:
  explicit ReferenceSpatialPointIndex(double bin):bin_(bin) {}
  void rebuild(const std::vector<Point> &points) {
    bins_.clear();max_radius_=0;
    for(size_t i=0;i<points.size();++i) {
      bins_[key(coordinate(points[i].x),coordinate(points[i].y))].push_back(i);
      max_radius_=std::max(max_radius_,points[i].radius);
    }
  }
  double maxRadius() const {return max_radius_;}
  template<class F> size_t forEachNear(const std::vector<Point> &points,double x,double y,
      double radius,F visit) const {
    return forEachNearIndexed(points,x,y,radius,[&](const Point &p,size_t){visit(p);});
  }
  template<class F> size_t forEachNearIndexed(const std::vector<Point> &points,double x,double y,
      double radius,F visit) const {
    // Roundoff allowance for inverse rigid transforms and tile boundaries.
    const double epsilon=1e-8+1e-12*(std::abs(x)+std::abs(y));
    radius+=epsilon;
    const int lx=coordinate(x-radius),ux=coordinate(x+radius);
    const int ly=coordinate(y-radius),uy=coordinate(y+radius);
    size_t examined=0;
    for(int bx=lx;bx<=ux;++bx) for(int by=ly;by<=uy;++by) {
      auto found=bins_.find(key(bx,by));if(found==bins_.end()) continue;
      for(const auto index:found->second) {
        ++examined;const auto &p=points[index];
        if(std::hypot(p.x-x,p.y-y)<=radius) visit(p,index);
      }
    }
    return examined;
  }
  bool hasWithin(const std::vector<Point> &points,const Point &query,double radius) const {
    const int lx=coordinate(query.x-radius),ux=coordinate(query.x+radius);
    const int ly=coordinate(query.y-radius),uy=coordinate(query.y+radius);
    for(int bx=lx;bx<=ux;++bx) for(int by=ly;by<=uy;++by) {
      auto found=bins_.find(key(bx,by));if(found==bins_.end()) continue;
      for(auto index:found->second)
        if(std::hypot(query.x-points[index].x,query.y-points[index].y)<=radius) return true;
    }
    return false;
  }
 private:
  int coordinate(double x) const {return static_cast<int>(std::floor(x/bin_));}
  static uint64_t key(int x,int y) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x))<<32)|static_cast<uint32_t>(y);
  }
  double bin_,max_radius_{0};std::unordered_map<uint64_t,std::vector<size_t>> bins_;
};
}
