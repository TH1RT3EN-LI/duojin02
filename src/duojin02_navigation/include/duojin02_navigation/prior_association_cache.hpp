#pragma once
#include "duojin02_navigation/sweep.hpp"
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace duojin02_navigation {
// Only the scan/prior association predicate is memoized, never a braking or
// freshness decision. The owner must advance epochs whenever points change.
struct PriorAssociationContext {
  uint64_t map_epoch{0}, observation_epoch{0};
  Pose map_pose;
  double range_error{0};
};
class PriorAssociationCache {
 public:
  void begin(size_t point_count, const PriorAssociationContext &context) {
    hits_=misses_=0;
    if(have_context_ && epochs_.size()==point_count && same(context_,context)) return;
    if(epochs_.size()!=point_count) {
      epochs_.assign(point_count,0);
      matched_.resize(point_count);
    }
    if(generation_==std::numeric_limits<uint32_t>::max()) {
      std::fill(epochs_.begin(),epochs_.end(),0);
      generation_=1;
    } else ++generation_;
    context_=context;have_context_=true;
  }
  template<class F> bool match(size_t index, F calculate) {
    // Indexed visits refer to the point vector whose epoch/count begin() took.
    auto &epoch=epochs_.at(index);
    if(epoch==generation_) {++hits_;return matched_[index]!=0;}
    const bool result=calculate();
    matched_[index]=result;epoch=generation_;++misses_;return result;
  }
  size_t hits() const {return hits_;}
  size_t misses() const {return misses_;}
 private:
  static bool sameBits(double a,double b) {return std::memcmp(&a,&b,sizeof(double))==0;}
  static bool same(const PriorAssociationContext &a,const PriorAssociationContext &b) {
    return a.map_epoch==b.map_epoch && a.observation_epoch==b.observation_epoch &&
      sameBits(a.map_pose.x,b.map_pose.x) && sameBits(a.map_pose.y,b.map_pose.y) &&
      sameBits(a.map_pose.yaw,b.map_pose.yaw) &&
      sameBits(a.range_error,b.range_error);
  }
  PriorAssociationContext context_;
  bool have_context_{false};uint32_t generation_{0};
  std::vector<uint32_t> epochs_;
  std::vector<uint8_t> matched_;
  size_t hits_{0},misses_{0};
};
}
