#include "duojin01_navigation/tracking.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace duojin01_navigation;
int main() {
  Polygon p{{.221,.199465},{.209465,.211},{-.210879,.211},{-.222,.199879},
            {-.222,-.198465},{-.209465,-.211},{.210879,-.211},{.221,-.200879}};
  std::mt19937 rng(0xD021);std::uniform_real_distribution<double> location(-.6,.6),rotation(-3.14,3.14),radius(0,.10);
  for(int i=0;i<10000;++i) {
    const Pose pose{location(rng),location(rng),rotation(rng)};
    const Point q{location(rng),location(rng),radius(rng)};
    const Point world=transform(q,pose);
    assert(collision(pose,p,{world},.010)==touches(q,p,.010));
  }
  std::vector<Point> walls;
  for(int i=-50;i<=50;++i) {walls.push_back({i*.01,.25,.007071});walls.push_back({i*.01,-.242,.007071});}
  const double biased=footprintClearance({},p,walls),centered=footprintClearance({0,.004,0},p,walls);
  assert(centered>biased+.0035);assert(!collision({0,.004,0},p,walls,.010));
  assert(footprintClearance({},p,{{.1,0,.007}})<0); // Interior thin pole.
  for(int i=0;i<=25;++i) assert(!collision({i*.004,0,0},p,{{.36,0,.007}},.010));
  assert(brakingSweep({}, {.5,0,0},p,{{.36,0,.007}},.010,.2,.4,.8));
  assert(!brakingSweep({}, {.04,0,0},p,{{.36,0,.007}},.010,.2,.4,.8));
  for(int i=0;i<=25;++i) assert(!collision({i*.004,0,i*.03/25},p,{{-.18,-.24,.007}},.010));
  assert(brakingSweep({}, {0,0,.45},p,{{-.18,-.24,.007}},.010,.2,.4,.8));
  // Test the cropped snapshot against actual combined translation/rotation
  // rollouts, including commands faster than the configured cap.
  double footprint_radius=0;for(const auto &q:p) footprint_radius=std::max(footprint_radius,std::hypot(q.x,q.y));
  const double old_forward_only=footprint_radius+.45+.014+.01*std::sqrt(2.0)/2;
  bool counterexample=false;
  for(double vx:{-.2,.0,.5,.75}) for(double vy:{-.3,.0,.3,.5}) for(double omega:{-.65,.0,.65,2.}) {
    Velocity velocity{vx,vy,omega};Pose pose;
    const double extent=localSnapshotExtent(p,.5,.2,.3,velocity,.01);
    const double stop=.2+std::max(std::hypot(vx,vy)/.4,std::abs(omega)/.8)+.01;
    for(double t=0;t<=stop;t+=.001) {
      for(const auto &vertex:p) {
        const auto point=transform(vertex,pose);
        assert(std::abs(point.x)+.012+.01*std::sqrt(2.0)/2<=extent);
        assert(std::abs(point.y)+.012+.01*std::sqrt(2.0)/2<=extent);
        if(vx==.5 && std::abs(vy)==.3 && std::abs(omega)==.65 && std::abs(point.x)+.012+.01*std::sqrt(2.0)/2>old_forward_only) counterexample=true;
      }
      pose=advance(pose,velocity,.001);
      if(t>=.2) {
        double speed=std::hypot(velocity.x,velocity.y);
        double factor=speed>0 ? std::max(0.,1-.4*.001/speed):0;
        velocity.x*=factor;velocity.y*=factor;velocity.yaw=approachZero(velocity.yaw,.8*.001);
      }
    }
  }
  assert(counterexample);
  std::cout << "10000 unpruned broad-phase comparisons; noisy corridor balance; thin interior; safe chord versus unsafe speed and angular-rate sweeps passed\n";
}
