#pragma once

#include <cmath>
#include <vector>

namespace duojin02_navigation {

struct LocalGridSnapshot {
  unsigned int width{0}, height{0}, crop_x{0}, crop_y{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  std::vector<unsigned char> data;

  bool nearObstacle(double x, double y) const {
    if (!std::isfinite(x + y) || resolution <= 0 || x < origin_x || y < origin_y)
      return true;
    const double mx = (x - origin_x) / resolution, my = (y - origin_y) / resolution;
    if (mx < crop_x || my < crop_y || mx >= crop_x + width || my >= crop_y + height)
      return true;

    const size_t index =
        (static_cast<size_t>(my) - crop_y) * width + static_cast<size_t>(mx) - crop_x;
    return index >= data.size() || data[index] >= 120;
  }
};
}
