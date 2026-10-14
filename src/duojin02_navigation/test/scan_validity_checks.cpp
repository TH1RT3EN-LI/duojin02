#include "duojin02_navigation/scan_validity.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace duojin02_navigation;
struct Stamp { int32_t sec{1}; uint32_t nanosec{0}; };
struct Scan {
  struct { std::string frame_id{"laser"}; } header;
  float angle_min{0},angle_max{0},angle_increment{.01f};
  float range_min{.3f},range_max{8},time_increment{0},scan_time{.1f};
  std::vector<float> ranges{1};
};
int main() {
  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  auto readings=[](std::vector<float> ranges) {
    Scan scan;scan.ranges=std::move(ranges);
    scan.angle_max=scan.angle_min+scan.angle_increment*(scan.ranges.empty()?0:scan.ranges.size()-1);
    return scan;
  };
  assert(scanValidity(readings({1,2,3}))==ScanValidity::Valid);
  assert(scanValidity(readings({inf,inf}))==ScanValidity::Valid);
  assert(scanValidity(readings({nan,1,inf,nan}))==ScanValidity::Valid);
  assert(scanValidity(readings({nan,nan}))==ScanValidity::InvalidRanges);
  assert(scanValidity(readings({-inf,-inf}))==ScanValidity::InvalidRanges);
  assert(scanValidity(readings({1,-inf,inf}))==ScanValidity::InvalidRanges);
  assert(scanValidity(readings({0,.2f,9}))==ScanValidity::InvalidRanges);
  assert(scanValidity(readings({}))==ScanValidity::InvalidMetadata);
  auto scan=readings({1,2,3});scan.angle_min=nan;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.angle_max=inf;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.angle_increment=0;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.angle_max+=.1f;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.angle_increment=-.01f;scan.angle_max=-.02f;
  assert(scanValidity(scan)==ScanValidity::Valid); // Clockwise scan arrays.
  scan=readings({1,2,3});scan.range_min=nan;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.range_max=inf;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.range_max=scan.range_min;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.time_increment=-.01f;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.scan_time=nan;
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  scan=readings({1,2,3});scan.header.frame_id.clear();
  assert(scanValidity(scan)==ScanValidity::InvalidMetadata);
  assert(validScanStamp(Stamp{}));
  assert(!validScanStamp(Stamp{-1,0}));
  assert(!validScanStamp(Stamp{1,1000000000u}));
  assert(scanStampNanoseconds(Stamp{2,17})==2000000017LL);
  ScanRecoveryOrder order;
  order.fault();
  assert(order.canAccept(0)); // Malformed first stamp does not lock initialization.
  order.accepted(100);
  assert(!order.canAccept(100)); // Normal-state duplicates cannot extend freshness.
  assert(!order.canAccept(50)); // Nor can a normal-state clock rewind.
  assert(order.canAccept(101));
  order.fault(200);
  assert(!order.canAccept(100)); // A replay is still fresh, but predates the fault.
  assert(!order.canAccept(200));
  assert(!order.canAccept(50)); // Clock rewind remains closed.
  assert(order.canAccept(201));
  order.accepted(201);
  order.fault(); // A malformed stamp falls back to the last accepted time.
  assert(!order.canAccept(201));
  assert(order.canAccept(202));
  order.accepted(202);
  order.fault(199); // An out-of-order fault cannot lower the recovery barrier.
  assert(!order.canAccept(201));
  assert(order.canAccept(203));
  std::cout<<"Scan fault, no-return, mixed rays, metadata, timestamp and recovery-order checks passed\n";
}
