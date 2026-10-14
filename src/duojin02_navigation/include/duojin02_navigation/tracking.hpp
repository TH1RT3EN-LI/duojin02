#pragma once
#include "duojin02_navigation/sweep.hpp"

namespace duojin02_navigation {
// Cover simultaneous native MPPI vx/vy limits, measured speed and every
// interpolated transition. Crop before checking may only remove unreachable
// cells; a forward-only cap does not bound a holonomic braking trajectory.
inline double localSnapshotExtent(const Polygon &polygon,double forward,double reverse,
                                  double lateral,const Velocity &measured,double resolution) {
  double radius=0;for(const auto &p:polygon) radius=std::max(radius,std::hypot(p.x,p.y));
  const double configured=std::hypot(std::max(forward,reverse),lateral);
  const double v=std::max(configured,std::hypot(measured.x,measured.y));
  return radius+std::max(.45,.2*v+v*v/(2*.4))+.014+resolution*std::sqrt(2.0)/2;
}

// Signed clearance from the filled footprint to occupied cell extents. Values
// above cap are equivalent for local centering; obstacle radii are never lost.
inline double footprintClearance(const Pose &pose,const Polygon &polygon,
                                 const std::vector<Point> &points,double cap=.080) {
  double min_x=1e100,min_y=1e100,max_x=-1e100,max_y=-1e100;
  for(const auto &v:polygon) {
    min_x=std::min(min_x,v.x);min_y=std::min(min_y,v.y);
    max_x=std::max(max_x,v.x);max_y=std::max(max_y,v.y);
  }
  const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
  double result=cap;
  for(const auto &p:points) {
    const double dx=p.x-pose.x,dy=p.y-pose.y;
    Point q{c*dx+s*dy,-s*dx+c*dy,p.radius};
    const double padding=cap+p.radius;
    if(q.x<min_x-padding || q.x>max_x+padding || q.y<min_y-padding || q.y>max_y+padding) continue;
    bool inside=true;double distance=1e100;
    for(size_t i=0;i<polygon.size();++i) {
      const auto &a=polygon[i],&b=polygon[(i+1)%polygon.size()];
      if((b.x-a.x)*(q.y-a.y)-(b.y-a.y)*(q.x-a.x)<-1e-10) inside=false;
      distance=std::min(distance,distanceToSegment(q,a,b));
    }
    result=std::min(result,(inside ? -distance : distance)-p.radius);
  }
  return result;
}
}
