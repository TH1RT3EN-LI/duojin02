#pragma once
#include <cmath>
#include <vector>

namespace duojin01_navigation {
// Bytes and geometry are captured under one costmap lock. All subsequent
// proximity decisions in this command use that same epoch; no stale cache.
struct LocalGridSnapshot {
  unsigned int width{0},height{0},crop_x{0},crop_y{0};double resolution{0},origin_x{0},origin_y{0};
  std::vector<unsigned char> data;
  bool nearObstacle(double x,double y) const {
    if(!std::isfinite(x+y) || resolution<=0 || x<origin_x || y<origin_y) return true;
    const double mx=(x-origin_x)/resolution,my=(y-origin_y)/resolution;
    if(mx<crop_x || my<crop_y || mx>=crop_x+width || my>=crop_y+height) return true;
    // Use the original raster origin before casting, just like worldToMap:
    // subtracting a cropped origin can change the rounding at cell boundaries.
    const size_t index=(static_cast<size_t>(my)-crop_y)*width+static_cast<size_t>(mx)-crop_x;
    return index>=data.size() || data[index]>=120;
  }
};
}
