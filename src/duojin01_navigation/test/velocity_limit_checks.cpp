#include "duojin01_navigation/velocity_limits.hpp"
#include <cassert>
#include <iostream>
#include <random>

using namespace duojin01_navigation;
bool close(double a,double b) {return std::abs(a-b)<1e-12;}
void expected(const OmniVelocityLimits &limits,double forward,double reverse,double lateral,double yaw) {
  const auto &b=limits.effective();
  assert(close(b.vx_max,forward));assert(close(b.vx_min,reverse));
  assert(close(b.vy_max,lateral));assert(close(b.wz_max,yaw));
}
int main() {
  OmniVelocityLimits limits;limits.configure({.5,-.2,.3,.65});
  assert(limits.update(20,true));expected(limits,.1,-.04,.06,.13);
  assert(limits.update(20,true));expected(limits,.1,-.04,.06,.13); // No compounding.
  assert(limits.update(60,true));expected(limits,.3,-.12,.18,.39);
  assert(limits.update(.1,false));expected(limits,.1,-.04,.06,.13);
  assert(limits.update(0,false));expected(limits,.5,-.2,.3,.65);assert(!limits.stopped());
  assert(limits.update(0,true));expected(limits,.5,-.2,.3,.65);
  assert(limits.update(200,true));expected(limits,.5,-.2,.3,.65);
  assert(limits.update(2,false));expected(limits,.5,-.2,.3,.65);
  for(double invalid:{-1.,std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
    for(bool percentage:{false,true}) {
      assert(!limits.update(invalid,percentage));expected(limits,0,0,0,0);assert(limits.stopped());
      assert(limits.nativePercentage()>0); // A stopped request must not become NO_SPEED_LIMIT.
      const auto v=limits.clamp({1,-1,1});assert(v.x==0 && v.y==0 && v.yaw==0);
      limits.update(0,true);assert(!limits.stopped());
    }
  }
  const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
  for(const auto &bad:std::vector<std::pair<OmniVelocityBounds,std::string>>{
      {{0,-.2,.3,.65},"vx_max"},{{-.5,-.2,.3,.65},"vx_max"},{{nan,-.2,.3,.65},"vx_max"},
      {{inf,-.2,.3,.65},"vx_max"},{{std::numeric_limits<double>::max(),-.2,.3,.65},"vx_max"},
      {{std::numeric_limits<double>::min(),-.2,.3,.65},"vx_max"},
      {{.5,.2,.3,.65},"vx_min"},{{.5,-inf,.3,.65},"vx_min"},
      {{.5,-.2,-.3,.65},"vy_max"},{{.5,-.2,nan,.65},"vy_max"},
      {{.5,-.2,.3,-.65},"wz_max"},{{.5,-.2,.3,inf},"wz_max"}}) {
    bool rejected=false;
    try {limits.configure(bad.first);}catch(const std::invalid_argument &error) {
      rejected=std::string(error.what()).find(bad.second)!=std::string::npos;
    }
    assert(rejected);
  }
  limits.configure({.5,0,0,0});const auto disabled=limits.clamp({-1,1,1});
  assert(disabled.x==0 && disabled.y==0 && disabled.yaw==0);
  limits.configure({.5,-.2,.3,.65});
  std::mt19937 random(0xD021);std::uniform_real_distribution<double> requested(.001,200),command(-2,2);
  for(int i=0;i<10000;++i) {
    limits.update(requested(random),i%2);const auto &b=limits.effective();
    const auto v=limits.clamp({command(random),command(random),command(random)});
    assert(v.x>=b.vx_min && v.x<=b.vx_max && std::abs(v.y)<=b.vy_max && std::abs(v.yaw)<=b.wz_max);
    assert(b.vx_max<=.5 && b.vx_min>=-.2 && b.vy_max<=.3 && b.wz_max<=.65);
  }
  const auto finite=limits.clamp({nan,0,0});assert(finite.x==0 && finite.y==0 && finite.yaw==0);
  std::cout<<"Omni percentage/absolute/reset/saturation, invalid limits/configuration, and 10000 bounded commands passed\n";
}
