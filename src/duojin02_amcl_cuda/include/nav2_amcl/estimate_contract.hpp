// Copyright 2026 Duojin contributors. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__ESTIMATE_CONTRACT_HPP_
#define NAV2_AMCL__ESTIMATE_CONTRACT_HPP_

#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace nav2_amcl
{
inline std::string diagnosticNumber(double value)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

// Provenance only: this object neither changes PF state nor consumes randomness.
// ROS timestamps and host monotonic timestamps intentionally use separate fields.
struct EstimateContract
{
  uint64_t map_generation{0};
  std::string map_identity{"unknown"};
  std::string source_id{"none"};
  int64_t state_reference_stamp_ns{0};
  int64_t last_effective_observation_stamp_ns{0};
  int64_t received_ros_stamp_ns{0};
  int64_t received_steady_ns{0};
  int64_t first_available_steady_ns{0};
  int64_t tf_stamp_ns{0};
  bool duplicate_source_consumed{false};
  bool valid_observation{false};
  bool nonmonotonic_source{false};

  void resetObservation()
  {
    source_id = "none";
    state_reference_stamp_ns = last_effective_observation_stamp_ns = 0;
    received_ros_stamp_ns = received_steady_ns = first_available_steady_ns = 0;
    tf_stamp_ns = 0;
    valid_observation = duplicate_source_consumed = false;
    nonmonotonic_source = false;
  }

  void recordAccepted(const std::string & source, int64_t reference_ns,
    int64_t receipt_ros_ns, int64_t receipt_steady_ns, int64_t available_ns)
  {
    duplicate_source_consumed = valid_observation && source == source_id;
    nonmonotonic_source = valid_observation && reference_ns <= last_effective_observation_stamp_ns;
    // Constant-memory high-water mark. Equal/older observations may still be
    // consumed by the compatibility PF path, but cannot refresh causal age.
    // This diagnoses freshness, not full historical source deduplication.
    if (nonmonotonic_source) {return;}
    source_id = source;
    last_effective_observation_stamp_ns = reference_ns;
    received_ros_stamp_ns = receipt_ros_ns;
    received_steady_ns = receipt_steady_ns;
    first_available_steady_ns = available_ns;
    valid_observation = true;
  }
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__ESTIMATE_CONTRACT_HPP_
