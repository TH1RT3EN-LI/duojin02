#pragma once
#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <unordered_map>

namespace duojin02_navigation {
// A speed norm bounds every scaled request and convex measured/requested
// transition. The discrete sweep advances before braking: allow two 20ms
// steps beyond reaction, plus its <=2mm interpolation and 5mm reserve.
inline double guardQueryRadius(const Polygon &polygon,const Velocity &measured,
    const Velocity &requested,double margin,double reaction,double deceleration,
    double cell_radius,double localization_margin,double memory_radius) {
  double footprint_radius=0;
  for(const auto &p:polygon) footprint_radius=std::max(footprint_radius,std::hypot(p.x,p.y));
  const double speed=std::max(std::hypot(measured.x,measured.y),std::hypot(requested.x,requested.y));
  const double radius=footprint_radius+speed*(reaction+.04)+speed*speed/(2*deceleration)
    +margin+.005+.002+cell_radius+localization_margin;
  return std::isfinite(radius) ? std::min(memory_radius,radius) : memory_radius;
}

// Rebuild only when the owning point vector changes. Querying retains the
// original point/radius; it never replaces occupied cells by wall outlines.
class SpatialPointIndex {
 public:
  explicit SpatialPointIndex(double bin):bin_(bin) {}
  void rebuild(const std::vector<Point> &points) {
    bins_.clear();offsets_.clear();indices_.clear();dense_=false;max_radius_=0;
    if(points.empty()) return;
    int lx=coordinate(points.front().x),ux=lx,ly=coordinate(points.front().y),uy=ly;
    // Bound both total storage and empty tiles per point. Reject a growing
    // sparse envelope immediately, so fallback does not scan its full extent.
    const size_t limit=points.size()>=4096 ? 65536 : 16*std::max(size_t{64},points.size());
    const auto extend=[&](const Point &p) {
      const int x=coordinate(p.x),y=coordinate(p.y);
      lx=std::min(lx,x);ux=std::max(ux,x);ly=std::min(ly,y);uy=std::max(uy,y);
      max_radius_=std::max(max_radius_,p.radius);
      const uint64_t width=static_cast<int64_t>(ux)-lx+1,height=static_cast<int64_t>(uy)-ly+1;
      return width>limit || height>limit || width*height>limit;
    };
    const auto sparse=[&]() {
      for(size_t i=0;i<points.size();++i) {
        bins_[key(coordinate(points[i].x),coordinate(points[i].y))].push_back(i);
        max_radius_=std::max(max_radius_,points[i].radius);
      }
    };
    // A sampled envelope is only a lower bound: it can prove fallback early,
    // but can never approve dense storage without checking every point.
    if(points.size()>1) for(size_t i:{points.size()/2,points.size()-1}) {
      if(extend(points[i])) {sparse();return;}
    }
    for(const auto &p:points) if(extend(p)) {sparse();return;}
    dense_lx_=lx;dense_ux_=ux;dense_ly_=ly;dense_uy_=uy;
    dense_height_=static_cast<size_t>(static_cast<int64_t>(uy)-ly+1);
    const size_t cells=static_cast<size_t>(static_cast<int64_t>(ux)-lx+1)*dense_height_;
    offsets_.assign(cells+1,0);indices_.resize(points.size());
    for(const auto &p:points) ++offsets_[cell(coordinate(p.x),coordinate(p.y))+1];
    for(size_t i=1;i<offsets_.size();++i) offsets_[i]+=offsets_[i-1];
    // Filling backwards from each tile's end preserves original point-index
    // order. Shift the resulting starts back into a standard CSR offset table.
    for(size_t i=points.size();i>0;--i) {
      const auto &p=points[i-1];const auto tile=cell(coordinate(p.x),coordinate(p.y));
      indices_[--offsets_[tile+1]]=i-1;
    }
    for(size_t i=0;i<cells;++i) offsets_[i]=offsets_[i+1];
    offsets_[cells]=points.size();dense_=true;
  }
  double maxRadius() const {return max_radius_;}
  size_t denseStorageCells() const {return dense_ ? offsets_.size()-1 : 0;}
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
    forEachBucket(lx,ux,ly,uy,[&](size_t index) {
      ++examined;const auto &p=points[index];
      if(std::hypot(p.x-x,p.y-y)<=radius) visit(p,index);
      return false;
    });
    return examined;
  }
  bool hasWithin(const std::vector<Point> &points,const Point &query,double radius) const {
    const int lx=coordinate(query.x-radius),ux=coordinate(query.x+radius);
    const int ly=coordinate(query.y-radius),uy=coordinate(query.y+radius);
    return forEachBucket(lx,ux,ly,uy,[&](size_t index) {
      return std::hypot(query.x-points[index].x,query.y-points[index].y)<=radius;
    });
  }
 private:
  template<class F> bool forEachBucket(int lx,int ux,int ly,int uy,F visit) const {
    if(dense_) {
      lx=std::max(lx,dense_lx_);ux=std::min(ux,dense_ux_);
      ly=std::max(ly,dense_ly_);uy=std::min(uy,dense_uy_);
      for(int bx=lx;bx<=ux;++bx) for(int by=ly;by<=uy;++by) {
        const auto tile=cell(bx,by);
        for(size_t i=offsets_[tile];i<offsets_[tile+1];++i) if(visit(indices_[i])) return true;
      }
    } else {
      for(int bx=lx;bx<=ux;++bx) for(int by=ly;by<=uy;++by) {
        auto found=bins_.find(key(bx,by));if(found==bins_.end()) continue;
        for(auto index:found->second) if(visit(index)) return true;
      }
    }
    return false;
  }
  size_t cell(int x,int y) const {
    return static_cast<size_t>(static_cast<int64_t>(x)-dense_lx_)*dense_height_
      +static_cast<size_t>(static_cast<int64_t>(y)-dense_ly_);
  }
  int coordinate(double x) const {return static_cast<int>(std::floor(x/bin_));}
  static uint64_t key(int x,int y) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x))<<32)|static_cast<uint32_t>(y);
  }
  double bin_,max_radius_{0};std::unordered_map<uint64_t,std::vector<size_t>> bins_;
  bool dense_{false};int dense_lx_{0},dense_ux_{0},dense_ly_{0},dense_uy_{0};size_t dense_height_{0};
  std::vector<size_t> offsets_,indices_;
};
}
