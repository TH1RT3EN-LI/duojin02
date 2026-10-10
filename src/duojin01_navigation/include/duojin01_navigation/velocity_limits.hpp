#pragma once
#include "duojin01_navigation/sweep.hpp"
#include <limits>
#include <stdexcept>
#include <string>

namespace duojin01_navigation {
struct OmniVelocityBounds {
  double vx_max{.5},vx_min{-.2},vy_max{.3},wz_max{.65};
};

// Humble scales every axis from the configured forward maximum for an
// absolute speed limit. Zero is NO_SPEED_LIMIT, rather than a stop request.
class OmniVelocityLimits {
 public:
  void configure(const OmniVelocityBounds &bounds) {
    validate(bounds.vx_max,"vx_max",bounds.vx_max>0);
    validate(bounds.vx_min,"vx_min",bounds.vx_min<=0);
    validate(bounds.vy_max,"vy_max",bounds.vy_max>=0);
    validate(bounds.wz_max,"wz_max",bounds.wz_max>=0);
    configured_=bounds;
    update(0,false);
  }
  bool update(double limit,bool percentage) {
    const bool valid=std::isfinite(limit) && limit>=0;
    ratio_=!valid ? 0 : limit==0 ? 1 :
      std::min(1.0,percentage ? limit/100.0 : limit/configured_.vx_max);
    effective_={configured_.vx_max*ratio_,configured_.vx_min*ratio_,
                configured_.vy_max*ratio_,configured_.wz_max*ratio_};
    return valid;
  }
  const OmniVelocityBounds &configured() const {return configured_;}
  const OmniVelocityBounds &effective() const {return effective_;}
  bool stopped() const {return ratio_==0;}
  // A positive underflowing percentage represents an all-axis stop to the
  // native float constraints without accidentally invoking its zero reset.
  double nativePercentage() const {
    return ratio_==0 ? std::numeric_limits<double>::min() : ratio_*100.0;
  }
  Velocity clamp(const Velocity &velocity) const {
    if(!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
       !std::isfinite(velocity.yaw)) return {};
    return {std::clamp(velocity.x,effective_.vx_min,effective_.vx_max),
            std::clamp(velocity.y,-effective_.vy_max,effective_.vy_max),
            std::clamp(velocity.yaw,-effective_.wz_max,effective_.wz_max)};
  }
 private:
  static void validate(double value,const char *name,bool correct_sign) {
    if(!std::isfinite(value) || std::abs(value)>std::numeric_limits<float>::max() ||
       (value!=0 && std::abs(value)<std::numeric_limits<float>::denorm_min()) || !correct_sign)
      throw std::invalid_argument(std::string("PreciseMPPI ")+name+
        " must be finite, representable by Humble's float constraints, and have a valid sign");
  }
  OmniVelocityBounds configured_,effective_;
  double ratio_{1};
};
}
