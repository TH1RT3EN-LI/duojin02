#include "duojin02_navigation/task_goal_contract.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace duojin02_navigation;
int main() {
  const double x = std::nextafter(1.0, 2.0), y = -.12345678912345678;
  ContractFields fields{{"original_frame", "map"}, {"original_x", contractNumber(x)},
    {"original_y", contractNumber(y)}, {"original_qx", "0"}, {"original_qy", "0"},
    {"original_qz", "0"}, {"original_qw", "1"}};
  assert(sameRequestedEndpoint(fields, "map", x, y, 0, 0, 0, 1));
  assert(!sameRequestedEndpoint(fields, "map", 1.0, y, 0, 0, 0, 1));
  assert(!sameRequestedEndpoint(fields, "odom", x, y, 0, 0, 0, 1));
  fields["original_x"] = "nan";
  assert(!sameRequestedEndpoint(fields, "map", x, y, 0, 0, 0, 1));
  fields["original_x"] = "1 trailing";
  double value;
  assert(!contractDouble(fields, "original_x", value));
  assert(!contractDouble(fields, "missing", value));
}
