/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "nav2_amcl/map/map.hpp"

namespace
{
// C++ integer division truncates toward zero; envelope breakpoints require floor.
int64_t floor_div(int64_t numerator, int64_t denominator)
{
  const int64_t quotient = numerator / denominator;
  return quotient - (numerator % denominator < 0 ? 1 : 0);
}
}  // namespace

// Exact, separable Euclidean distance to occupied cell CENTRES, capped in metres.
// Unknown cells are query locations, not obstacles. First find the nearest
// occupied cell in each column, then take the lower envelope of the horizontal
// squared-distance parabolas. Selecting the envelope uses integer arithmetic;
// only the final square root and conversion to metres are floating point.
void map_update_cspace(map_t * map, double max_occ_dist)
{
  if (!map || !map->cells || map->size_x <= 0 || map->size_y <= 0 ||
    !std::isfinite(map->scale) || map->scale <= 0.0 ||
    !std::isfinite(max_occ_dist) || max_occ_dist < 0.0)
  {
    throw std::invalid_argument("map_update_cspace: invalid map or distance limit");
  }
  const size_t width = static_cast<size_t>(map->size_x);
  const size_t height = static_cast<size_t>(map->size_y);
  if (height > std::numeric_limits<size_t>::max() / width) {
    throw std::length_error("map_update_cspace: map dimensions overflow");
  }
  const size_t count = width * height;
  constexpr int32_t absent = std::numeric_limits<int32_t>::max();
  // Allocate all scratch before changing the map. Scratch is 4*N + O(width)
  // bytes, without a per-cell heap or a process-global mutable distance cache.
  std::vector<int32_t> vertical(count, absent);
  std::vector<int32_t> nearest(width, absent);
  std::vector<int32_t> sites(width);
  std::vector<int64_t> starts(width);

  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      const size_t index = y * width + x;
      if (map->cells[index].occ_state == +1) {
        nearest[x] = static_cast<int32_t>(y);
      }
      if (nearest[x] != absent) {
        vertical[index] = static_cast<int32_t>(y) - nearest[x];
      }
    }
  }
  std::fill(nearest.begin(), nearest.end(), absent);
  for (size_t y = height; y-- > 0;) {
    for (size_t x = 0; x < width; ++x) {
      const size_t index = y * width + x;
      if (map->cells[index].occ_state == +1) {
        nearest[x] = static_cast<int32_t>(y);
      }
      if (nearest[x] != absent) {
        vertical[index] = std::min(vertical[index], nearest[x] - static_cast<int32_t>(y));
      }
    }
  }

  for (size_t y = 0; y < height; ++y) {
    const size_t row = y * width;
    size_t size = 0;
    for (int32_t q = 0; q < map->size_x; ++q) {
      if (vertical[row + q] == absent) {
        continue;
      }
      const int64_t dy = vertical[row + q];
      const int64_t fq = dy * dy;
      int64_t start = 0;
      while (size > 0) {
        const int64_t v = sites[size - 1];
        const int64_t dv = vertical[row + static_cast<size_t>(v)];
        // First integer x for which q is STRICTLY better than v:
        // 2*(q-v)*x > fq + q*q - fv - v*v.
        // int32 map dimensions guarantee both squared sums fit in int64.
        start = floor_div(
          fq + int64_t{q} * q - (dv * dv + v * v), 2 * (int64_t{q} - v)) + 1;
        if (start > starts[size - 1]) {
          break;
        }
        --size;
      }
      if (size == 0) {
        start = 0;
      }
      if (start < map->size_x) {
        sites[size] = q;
        starts[size] = start;
        ++size;
      }
    }
    size_t active = 0;
    for (size_t x = 0; x < width; ++x) {
      double distance = max_occ_dist;
      if (size > 0) {
        while (active + 1 < size && starts[active + 1] <= static_cast<int64_t>(x)) {
          ++active;
        }
        const int64_t dx = static_cast<int64_t>(x) - sites[active];
        const int64_t dy = vertical[row + sites[active]];
        distance = std::min(max_occ_dist, map->scale * std::sqrt(
          static_cast<double>(dx * dx + dy * dy)));
      }
      map->cells[row + x].occ_dist = distance;
    }
  }
  map->max_occ_dist = max_occ_dist;
}
