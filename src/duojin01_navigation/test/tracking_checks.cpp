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
  std::cout << "10000 unpruned broad-phase comparisons; noisy corridor balance; thin interior; safe chord versus unsafe speed and angular-rate sweeps passed\n";
}
