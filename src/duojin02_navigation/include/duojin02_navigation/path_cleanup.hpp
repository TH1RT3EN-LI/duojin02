#pragma once
#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <map>
#include <tuple>

namespace duojin02_navigation {
// Erase repeated SE2 states from overlapping lattice primitives. Different
// headings at the same position are retained; all resulting chords still
// receive filled-footprint validation in the planner.
inline std::vector<size_t> loopFreeIndices(const std::vector<Pose> &poses) {
  using Key=std::tuple<int64_t,int64_t,int64_t>;
  std::map<Key,size_t> visited;
  std::vector<Key> keys;
  std::vector<size_t> result;
  for(size_t i=0;i<poses.size();++i) {
    const auto &p=poses[i];
    Key key{std::llround(p.x*1e6),std::llround(p.y*1e6),std::llround(std::remainder(p.yaw,2*M_PI)*1e6)};
    auto found=visited.find(key);
    if(found!=visited.end()) {
      const auto keep=found->second+1;
      for(size_t j=keep;j<keys.size();++j) visited.erase(keys[j]);
      keys.resize(keep);result.resize(keep);result.back()=i;
    } else {
      visited[key]=result.size();keys.push_back(key);result.push_back(i);
    }
  }
  return result;
}
}
