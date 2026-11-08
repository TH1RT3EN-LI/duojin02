// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: LGPL-2.1-or-later
#ifndef NAV2_AMCL__OCCUPANCY_IDENTITY_HPP_
#define NAV2_AMCL__OCCUPANCY_IDENTITY_HPP_
#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
namespace nav2_amcl {
// Shared with task_goal_capture.py: UTF8 frame+NUL, little endian uint32
// width/height, binary64 resolution/origin xyz/quaternion xyzw, cell bytes.
inline std::string occupancyIdentity(const std::string & frame, uint32_t width, uint32_t height,
  double resolution, const std::array<double,7> & origin, const std::vector<int8_t> & cells)
{
  uint64_t hash=14695981039346656037ULL;
  auto byte=[&](uint8_t value) {hash=(hash^value)*1099511628211ULL;};
  auto integer=[&](uint64_t value,int bytes) {for(int i=0;i<bytes;++i)byte((value>>(8*i))&255);};
  for (unsigned char value : frame) {byte(value);}
  byte(0);
  integer(width, 4);
  integer(height, 4);
  auto real=[&](double value) {uint64_t bits;static_assert(sizeof(bits)==sizeof(value));std::memcpy(&bits,&value,sizeof(bits));integer(bits,8);};
  real(resolution);
  for (double value : origin) {real(value);}
  for (int8_t value : cells) {byte(static_cast<uint8_t>(value));}
  std::ostringstream out;out<<"occupancy_fnv1a64:"<<std::hex<<std::setfill('0')<<std::setw(16)<<hash;return out.str();
}
}  // namespace nav2_amcl
#endif
