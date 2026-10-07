#pragma once
#include "duojin01_navigation/sweep.hpp"

namespace duojin01_navigation {
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
