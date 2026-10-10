#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace duojin01_navigation {
enum class ScanValidity { Valid, InvalidMetadata, InvalidRanges };

template<class Stamp>
inline bool validScanStamp(const Stamp &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000u;
}

template<class Stamp>
inline int64_t scanStampNanoseconds(const Stamp &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

// Replayed scans cannot renew the perception heartbeat or clear a fault.
// Retain the high-water mark across faults and clock rewinds; only newer sensor
// time can refresh the scan, unless the node is explicitly restarted.
class ScanRecoveryOrder {
 public:
  void fault(int64_t stamp_ns=-1) {
    recovery_after_ns_=std::max({recovery_after_ns_,accepted_ns_,stamp_ns});
    faulted_=true;
  }
  bool canAccept(int64_t stamp_ns) const {
    return stamp_ns>accepted_ns_ && (!faulted_ || stamp_ns>recovery_after_ns_);
  }
  void accepted(int64_t stamp_ns) {
    accepted_ns_=std::max(accepted_ns_,stamp_ns);
    faulted_=false;
  }
 private:
  int64_t accepted_ns_{-1},recovery_after_ns_{-1};
  bool faulted_{false};
};

// A positive infinite range is a legitimate no-return ray. NaNs may mark
// individual missing rays, but a message consisting only of invalid readings
// is not a usable sensor heartbeat. A negative infinite range can represent a
// too-close object, so discarding it as free space would weaken the guard.
template<class Scan>
inline ScanValidity scanValidity(const Scan &scan) {
  if (scan.header.frame_id.empty() || scan.ranges.empty() ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment == 0 ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0 || scan.range_max <= scan.range_min ||
      !std::isfinite(scan.time_increment) || scan.time_increment < 0 ||
      !std::isfinite(scan.scan_time) || scan.scan_time < 0)
    return ScanValidity::InvalidMetadata;

  const double last_angle = static_cast<double>(scan.angle_min) +
    static_cast<double>(scan.angle_increment) * (scan.ranges.size() - 1);
  const double tolerance = std::max(1e-5, std::abs(static_cast<double>(scan.angle_increment)) * 1e-3);
  if (!std::isfinite(last_angle) || std::abs(last_angle - scan.angle_max) > tolerance)
    return ScanValidity::InvalidMetadata;

  bool usable = false;
  for (const auto range : scan.ranges) {
    if (std::isinf(range)) {
      if (range < 0) return ScanValidity::InvalidRanges;
      usable = true;
    } else if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      usable = true;
    }
  }
  return usable ? ScanValidity::Valid : ScanValidity::InvalidRanges;
}
}
