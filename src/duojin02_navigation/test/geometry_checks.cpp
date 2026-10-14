#include "duojin02_navigation/sweep.hpp"
#include <cassert>
#include <iostream>
using namespace duojin02_navigation;
int main() {
  Polygon p{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},
            {-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  assert(collision({},p,{{0.1,0.0}},0)); // Interior pole, no edge intersects it.
  assert(!collision({},p,{{0,.225}},.008));
  assert(collision({},p,{{0,.210}},.008));
  assert(brakingSweep({}, {.4,0,0},p,{{.34,0}},.008,.2,.4,1.0));
  assert(!brakingSweep({}, {.04,0,0},p,{{.34,0}},.008,.2,.4,1.0));
  assert(brakingSweep({}, {0,0,.8},p,{{.230,-.130}},.008,.2,.4,1.0));
  assert(brakingSweep({}, {0,.3,0},p,{{0,.31}},.008,.2,.4,1.0));
  assert(!brakingSweep({}, {.3,0,0},p,{{0,.23}},.008,.2,.4,1.0));
  assert(brakingSweep({}, {0,0,0},p,{{.235,0,.025}},.008,.2,.4,1.0));
  std::cout << "Filled interior, thin obstacle, braking, lateral, rotated corner and cell extent checks passed\n";
}
