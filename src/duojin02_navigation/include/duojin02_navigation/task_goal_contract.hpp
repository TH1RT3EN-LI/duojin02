#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>

namespace duojin02_navigation {
// Metadata only. Unknown calibration never changes control admission or claims
// a probability of physical arrival. All coordinate values remain binary64.
using ContractFields = std::map<std::string, std::string>;
inline std::string contractNumber(double value) {
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}
inline bool contractDouble(const ContractFields &fields, const std::string &key, double &value) {
  const auto it = fields.find(key);
  if (it == fields.end()) return false;
  try {
    std::size_t used = 0;
    value = std::stod(it->second, &used);
    return used == it->second.size() && std::isfinite(value);
  } catch (...) { return false; }
}
// Association is intentionally exact: an unavailable/action-only request or a
// changed planner endpoint must not be attributed to an unrelated clicked goal.
inline bool sameRequestedEndpoint(const ContractFields &fields, const std::string &frame,
                                  double x, double y, double qx, double qy,
                                  double qz, double qw) {
  const auto f = fields.find("original_frame");
  if (f == fields.end() || f->second != frame) return false;
  const double expected[] = {x, y, qx, qy, qz, qw};
  const char *keys[] = {"original_x", "original_y", "original_qx", "original_qy",
                        "original_qz", "original_qw"};
  for (unsigned i = 0; i < 6; ++i) {
    double actual;
    if (!contractDouble(fields, keys[i], actual) || actual != expected[i]) return false;
  }
  return true;
}
struct GoalTransformTrace {
  std::string status{"not_queried"};
  std::string source_frame, destination_frame;
  int64_t query_stamp_ns{0}, used_stamp_ns{0};
};
} // namespace duojin02_navigation
