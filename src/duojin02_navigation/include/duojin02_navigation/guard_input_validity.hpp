#pragma once
#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <limits>

namespace duojin02_navigation {
// Validate at the message/TF boundary before calling the unchanged geometry
// and index helpers. Reserve the final int tile for the bucket-loop increment.
inline bool guardIndexCoordinateValid(double value,double bin) {
  if(!std::isfinite(value) || !std::isfinite(bin) || bin<=0) return false;
  const double tile=std::floor(value/bin);
  return std::isfinite(tile) && tile>=std::numeric_limits<int>::min() &&
    tile<std::numeric_limits<int>::max();
}
inline bool guardPointValid(const Point &point,double bin) {
  return std::isfinite(point.radius) && point.radius>=0 &&
    guardIndexCoordinateValid(point.x,bin) && guardIndexCoordinateValid(point.y,bin);
}
inline bool guardPoseValid(const Pose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}
inline bool guardIndexQueryValid(double x,double y,double radius,double bin,bool roundoff) {
  if(!std::isfinite(radius) || radius<0 || !std::isfinite(x) || !std::isfinite(y)) return false;
  if(roundoff) radius+=1e-8+1e-12*(std::abs(x)+std::abs(y));
  return guardIndexCoordinateValid(x-radius,bin) && guardIndexCoordinateValid(x+radius,bin) &&
    guardIndexCoordinateValid(y-radius,bin) && guardIndexCoordinateValid(y+radius,bin);
}
template<class Grid> bool guardGridValid(const Grid &grid) {
  const auto &info=grid.info;
  const auto &p=info.origin.position;
  const auto &q=info.origin.orientation;
  if(!info.width || !info.height || !std::isfinite(info.resolution) || info.resolution<=0 ||
      static_cast<uint64_t>(info.width)*info.height!=grid.data.size() ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
  // The occupied grid is planar. Keep the same quaternion tolerances as the
  // localization map boundary; never normalize or alter accepted coordinates.
  if(std::abs(q.x)>1e-6 || std::abs(q.y)>1e-6 ||
      std::abs(q.w*q.w+q.z*q.z-1.0)>1e-3) return false;
  return std::isfinite(static_cast<double>(info.width)*info.resolution) &&
    std::isfinite(static_cast<double>(info.height)*info.resolution);
}
template<class Grid> bool guardGridExtentValid(const Grid &grid,const Pose &origin,
    const Pose &frame,double bin) {
  if(!guardPoseValid(origin) || !guardPoseValid(frame)) return false;
  // A rigid planar transform maps the grid rectangle to a convex rectangle;
  // checking its four cell-centre corners bounds every transformed cell.
  for(double x:{.5,static_cast<double>(grid.info.width)-.5})
    for(double y:{.5,static_cast<double>(grid.info.height)-.5}) {
      const Point cell{x*grid.info.resolution,y*grid.info.resolution,
        std::sqrt(2.0)*grid.info.resolution/2};
      if(!guardPointValid(transform(transform(cell,origin),frame),bin)) return false;
    }
  return true;
}
}
