// Copyright 2026 Duojin02 contributors. SPDX-License-Identifier: GPL-3.0-only
#ifndef DUOJIN02_SCAN_GEOMETRY__GEOMETRY_HPP_
#define DUOJIN02_SCAN_GEOMETRY__GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <sstream>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace duojin02_scan_geometry
{
constexpr double kPi = 3.14159265358979323846;
struct Point {double x = 0.0, y = 0.0;};
// mirrored permits a planar upside-down laser mount; robot motion remains in SE(2).
struct Pose {double x = 0.0, y = 0.0, yaw = 0.0; bool mirrored = false;};
enum class Failure {InvalidInput, MissingPast, MissingFuture, InterpolationGap, MissingFrame};
class Error : public std::runtime_error
{
public:
  Error(Failure why, const std::string & what) : std::runtime_error(what), reason(why) {}
  Failure reason;
};
inline double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
inline bool finite(const Pose & p)
{return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);}
inline Point apply(const Pose & p, Point q)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  if (p.mirrored) {q.y = -q.y;}
  return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y};
}
inline Pose compose(const Pose & a, const Pose & b)
{
  Point p = apply(a, {b.x, b.y});
  return {p.x, p.y, wrap(a.yaw + (a.mirrored ? -b.yaw : b.yaw)),
    a.mirrored != b.mirrored};
}
inline Pose inverse(const Pose & p)
{
  Pose result{0.0, 0.0, p.mirrored ? p.yaw : -p.yaw, p.mirrored};
  const Point point = apply(result, {-p.x, -p.y});
  result.x = point.x; result.y = point.y; return result;
}
// Principal-log SE(2) interpolation; exact for constant body twist.
inline Pose interpolate(const Pose & a, const Pose & b, double fraction)
{
  if (a.mirrored || b.mirrored) {
    throw Error(Failure::InvalidInput, "dynamic motion must be a proper SE(2) transform");
  }
  Pose d = compose(inverse(a), b);
  if (std::abs(std::abs(d.yaw) - kPi) < 1e-9) {
    throw Error(Failure::InvalidInput, "SE(2) interpolation crosses the pi branch");
  }
  auto coefficients = [](double w) {
      if (std::abs(w) < 1e-5) {
        const double w2 = w * w;
        return Point{1.0 - w2 / 6.0 + w2 * w2 / 120.0,
          w / 2.0 - w * w2 / 24.0 + w * w2 * w2 / 720.0};
      }
      return Point{std::sin(w) / w, (1.0 - std::cos(w)) / w};
    };
  Point ab = coefficients(d.yaw);
  double norm = ab.x * ab.x + ab.y * ab.y;
  double vx = (ab.x * d.x + ab.y * d.y) / norm;
  double vy = (-ab.y * d.x + ab.x * d.y) / norm;
  Point scaled = coefficients(fraction * d.yaw);
  return compose(a, {fraction * (scaled.x * vx - scaled.y * vy),
    fraction * (scaled.y * vx + scaled.x * vy), fraction * d.yaw});
}
struct Config
{
  bool enabled = false;
  double capture_time_offset_sec = 0.0;
  std::string mapping_id = "header-first-ray-assumed-v1";
  std::string mapping_source;
  double reference_fraction = 0.0;
  double max_interpolation_gap_sec = 0.1;
  double tf_history_sec = 120.0;
};
inline void validate(const Config & c)
{
  if (!std::isfinite(c.capture_time_offset_sec) ||
    !std::isfinite(c.reference_fraction) || c.reference_fraction < 0.0 ||
    c.reference_fraction > 1.0 || !std::isfinite(c.max_interpolation_gap_sec) ||
    c.max_interpolation_gap_sec <= 0.0 || !std::isfinite(c.tf_history_sec) ||
    c.tf_history_sec <= c.max_interpolation_gap_sec || c.mapping_id.empty())
  {
    throw Error(Failure::InvalidInput, "invalid scan geometry/time configuration");
  }
  if (c.capture_time_offset_sec != 0.0 &&
    (c.mapping_id == "header-first-ray-assumed-v1" || c.mapping_source.empty()))
  {
    throw Error(Failure::InvalidInput,
      "nonzero capture offset requires explicit mapping_id and mapping_source evidence");
  }
}
inline std::string jsonString(const std::string & value)
{
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 0x20) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c) << std::dec;}
    else {out << c;}
  }
  out << '"'; return out.str();
}
inline std::string motionModelJson(const Config & c)
{
  return std::string("{\"version\":\"conditional-trajectory.v1\",\"trajectory\":\"") +
    (c.enabled ? "se2_interpolated" : "rigid_at_reference") +
    "\",\"provider\":\"tf_pose_history\",\"temporal_covariance\":\"not_supplied\","
    "\"uncertainty\":\"conditional_not_marginalized\",\"evidence_independence\":\"unknown\","
    "\"shared_motion\":\"conditioned_not_marginalized\",\"beam_noise_model\":\"consumer_defined\"}";
}
inline std::string contractJson(const Config & c)
{
  validate(c);
  std::ostringstream out; out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema\":\"duojin02.scan_geometry.v2\","
      << "\"measurement_geometry_version\":\"typed-rays.v2\","
      << "\"sensor_no_return_policy\":\"max-or-positive-infinity\","
      << "\"capture_time_evidence_status\":\"uncalibrated\","
      << "\"beam_order_evidence_status\":\"original-index-increasing-time-assumed\","
      << "\"motion_model\":" << motionModelJson(c) << ","
      << "\"scan_motion_compensation\":"
      << (c.enabled ? "true" : "false")
      << ",\"capture_time_offset_sec\":" << c.capture_time_offset_sec
      << ",\"capture_time_mapping_id\":" << jsonString(c.mapping_id)
      << ",\"capture_time_mapping_source\":" << jsonString(c.mapping_source)
      << ",\"scan_reference_fraction\":" << c.reference_fraction
      << ",\"scan_max_tf_gap_sec\":" << c.max_interpolation_gap_sec << "}\n";
  return out.str();
}
inline bool legacyContract(const Config & c)
{return !c.enabled && c.capture_time_offset_sec == 0.0 && c.reference_fraction == 0.0 &&
    c.mapping_id == "header-first-ray-assumed-v1" && c.mapping_source.empty();}
inline int64_t secondsToNs(double seconds)
{
  const long double ns = static_cast<long double>(seconds) * 1000000000.0L;
  if (!std::isfinite(seconds) || ns > std::numeric_limits<int64_t>::max() ||
    ns < std::numeric_limits<int64_t>::min())
  {throw Error(Failure::InvalidInput, "non-finite or overflowing scan time");}
  return static_cast<int64_t>(std::llround(ns));
}
inline int64_t checkedAdd(int64_t a, int64_t b)
{
  if ((b > 0 && a > std::numeric_limits<int64_t>::max() - b) ||
    (b < 0 && a < std::numeric_limits<int64_t>::min() - b))
  {throw Error(Failure::InvalidInput, "scan timestamp overflow");}
  return a + b;
}
inline int64_t beamStampNs(int64_t raw, double increment, size_t index, const Config & c)
{return checkedAdd(checkedAdd(raw, secondsToNs(c.capture_time_offset_sec)),
    secondsToNs(static_cast<double>(index) * increment));}
inline int64_t referenceStampNs(
  int64_t raw, double increment, size_t count, const Config & c)
{
  validate(c);
  if (!std::isfinite(increment) || increment < 0.0 || count == 0) {
    throw Error(Failure::InvalidInput, "invalid scan beam timing/count");
  }
  return checkedAdd(checkedAdd(raw, secondsToNs(c.capture_time_offset_sec)),
    secondsToNs(c.reference_fraction * static_cast<double>(count - 1) * increment));
}
inline std::string frameName(std::string name)
{
  while (!name.empty() && name.front() == '/') {name.erase(name.begin());}
  if (name.empty()) {throw Error(Failure::InvalidInput, "empty TF frame");}
  return name;
}
// Records actual TF samples, rather than assuming tf2's interpolation implies dense support.
// Each dynamic edge must contain an exact sample or bracket no wider than max_gap_ns.
class History
{
  struct Edge {bool is_static = false; std::map<int64_t, Pose> samples;};
  std::map<std::pair<std::string, std::string>, Edge> edges_;
  mutable std::mutex mutex_;
public:
  void insert(std::string parent, std::string child, int64_t stamp, Pose pose,
    bool is_static, int64_t history_ns)
  {
    parent = frameName(parent); child = frameName(child);
    if (parent == child || !finite(pose) || (!is_static && pose.mirrored) || history_ns <= 0) {
      throw Error(Failure::InvalidInput, "invalid TF history sample");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto & edge = edges_[{parent, child}];
    if (!edge.samples.empty() && edge.is_static != is_static) {
      throw Error(Failure::InvalidInput, "TF edge changes static/dynamic kind");
    }
    edge.is_static = is_static;
    if (!is_static && !edge.samples.empty() && stamp < edge.samples.rbegin()->first) {
      throw Error(Failure::InvalidInput, "nonmonotonic TF edge sample");
    }
    const auto existing = edge.samples.find(stamp);
    if (existing != edge.samples.end() &&
      (std::abs(existing->second.x - pose.x) > 1e-12 ||
      std::abs(existing->second.y - pose.y) > 1e-12 ||
      std::abs(wrap(existing->second.yaw - pose.yaw)) > 1e-12 ||
      existing->second.mirrored != pose.mirrored))
    {throw Error(Failure::InvalidInput, "conflicting duplicate TF timestamp");}
    if (is_static) {edge.samples.clear();}
    edge.samples[stamp] = pose;
    if (!is_static) {
      const int64_t cutoff = stamp > std::numeric_limits<int64_t>::min() + history_ns ?
        stamp - history_ns : std::numeric_limits<int64_t>::min();
      auto first = edge.samples.lower_bound(cutoff);
      // Retain one predecessor to allow interpolation right at the retention boundary.
      if (first != edge.samples.begin()) {--first; edge.samples.erase(edge.samples.begin(), first);}
    }
  }
  Pose lookup(std::string target, std::string source, int64_t stamp, int64_t max_gap_ns) const
  {
    target = frameName(target); source = frameName(source);
    if (target == source) {return {};}
    if (max_gap_ns <= 0) {throw Error(Failure::InvalidInput, "invalid TF gap bound");}
    std::lock_guard<std::mutex> lock(mutex_);
    struct Step {std::string frame; Pose frame_from_source;};
    std::queue<Step> pending; pending.push({source, {}});
    std::map<std::string, bool> visited; visited[source] = true;
    // Find the frame path first. Temporal errors on unrelated TF branches are irrelevant.
    struct Back {std::string frame; const Edge * edge; bool inverse_edge;};
    std::map<std::string, Back> back;
    while (!pending.empty()) {
      auto current = pending.front(); pending.pop();
      if (current.frame == target) {break;}
      for (const auto & item : edges_) {
        const auto & key = item.first;
        const bool forward = key.second == current.frame;
        if (!forward && key.first != current.frame) {continue;}
        const std::string & next = forward ? key.first : key.second;
        if (visited[next]) {continue;}
        visited[next] = true;
        back[next] = {current.frame, &item.second, !forward};
        pending.push({next, {}});
      }
    }
    if (!visited[target]) {
      throw Error(Failure::MissingFrame, "TF history has no path " + target + " <- " + source);
    }
    Pose result;
    std::string cursor = target;
    // Walk backwards: target<-previous<-...<-source.
    while (cursor != source) {
      const Back & step = back.at(cursor);
      const auto & edge = *step.edge;
      Pose value;
      if (edge.is_static) {value = edge.samples.begin()->second;} else {
        auto right = edge.samples.lower_bound(stamp);
        if (right == edge.samples.end()) {
          throw Error(Failure::MissingFuture, "scan awaits future TF support for " + cursor);
        }
        if (right->first == stamp) {value = right->second;} else {
          if (right == edge.samples.begin()) {
            throw Error(Failure::MissingPast, "scan predates TF support for " + cursor);
          }
          auto left = std::prev(right);
          if (right->first - left->first > max_gap_ns) {
            throw Error(Failure::InterpolationGap, "internal TF sample gap exceeds limit for " + cursor);
          }
          const double fraction = static_cast<double>(stamp - left->first) /
            static_cast<double>(right->first - left->first);
          value = interpolate(left->second, right->second, fraction);
        }
      }
      if (step.inverse_edge) {value = inverse(value);}
      result = compose(result, value);
      cursor = step.frame;
    }
    return result;
  }
};
enum class BeamState : uint8_t {Invalid = 0, Hit = 1, NoReturn = 2};
enum class InvalidReason : uint8_t {None = 0, NotANumber, NegativeInfinity, BelowSensorMinimum,
  AboveSensorMaximum};
// A name alone does not make a point finite: only this checked constructor can
// create geometry that crosses the prepared-measurement boundary.
class FinitePoint
{
  Point point_;
public:
  explicit FinitePoint(Point point) : point_(point)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw Error(Failure::InvalidInput, "non-finite prepared ray geometry");
    }
  }
  Point point() const {return point_;}
};
struct InvalidBeam {InvalidReason reason;};
struct HitRay {FinitePoint origin, endpoint;};
struct NoReturnRay {FinitePoint origin, endpoint; double free_space_limit;};
class Beam
{
  std::variant<InvalidBeam, HitRay, NoReturnRay> measurement_;
  Beam(size_t index, int64_t stamp, double raw, BeamState kind,
    std::variant<InvalidBeam, HitRay, NoReturnRay> measurement)
  : measurement_(std::move(measurement)), original_index(index), stamp_ns(stamp),
    raw_range(raw), state(kind) {}
public:
  size_t original_index = 0;
  int64_t stamp_ns = 0;
  // Audit only. Consumers use state and the checked geometry, never reinterpret
  // this raw range as a rigid scan at the reference time.
  double raw_range = 0.0;
  BeamState state;
  static Beam invalid(size_t i, int64_t t, double raw, InvalidReason reason)
  {return Beam(i, t, raw, BeamState::Invalid, InvalidBeam{reason});}
  static Beam hit(size_t i, int64_t t, double raw, Point origin, Point endpoint)
  {return Beam(i, t, raw, BeamState::Hit, HitRay{FinitePoint(origin), FinitePoint(endpoint)});}
  static Beam noReturn(size_t i, int64_t t, double raw, Point origin, Point endpoint, double limit)
  {
    if (!std::isfinite(limit) || limit <= 0.0) {
      throw Error(Failure::InvalidInput, "invalid no-return free-space limit");
    }
    return Beam(i, t, raw, BeamState::NoReturn,
      NoReturnRay{FinitePoint(origin), FinitePoint(endpoint), limit});
  }
  bool valid() const {return state != BeamState::Invalid;}
  bool hasReturn() const {return state == BeamState::Hit;}
  InvalidReason invalidReason() const
  {return state == BeamState::Invalid ? std::get<InvalidBeam>(measurement_).reason : InvalidReason::None;}
  Point endpointPoint() const
  {
    if (state == BeamState::Hit) {return std::get<HitRay>(measurement_).endpoint.point();}
    if (state == BeamState::NoReturn) {return std::get<NoReturnRay>(measurement_).endpoint.point();}
    throw Error(Failure::InvalidInput, "invalid beam has no consumable endpoint");
  }
  Point originPoint() const
  {
    if (state == BeamState::Hit) {return std::get<HitRay>(measurement_).origin.point();}
    if (state == BeamState::NoReturn) {return std::get<NoReturnRay>(measurement_).origin.point();}
    throw Error(Failure::InvalidInput, "invalid beam has no consumable ray origin");
  }
};
struct RawScanAudit
{
  std::vector<float> ranges;
  double sensor_range_min = 0.0, sensor_range_max = 0.0;
};
struct TimedScan
{
  int64_t raw_stamp_ns = 0, reference_stamp_ns = 0;
  std::string source_scan_id, mapping_id, mapping_source;
  Pose odom_from_base_reference;
  RawScanAudit raw_audit;
  std::vector<Beam> beams;
};
inline TimedScan prepareReference(int64_t raw_stamp, const std::string & laser_frame,
  double time_increment, size_t count, const Config & config, const History & history,
  const std::string & odom_frame, const std::string & base_frame)
{
  TimedScan result;
  result.raw_stamp_ns = raw_stamp;
  result.reference_stamp_ns = referenceStampNs(raw_stamp, time_increment, count, config);
  if (result.reference_stamp_ns < 0) {
    throw Error(Failure::InvalidInput, "physical scan reference time is negative");
  }
  result.mapping_id = config.mapping_id; result.mapping_source = config.mapping_source;
  result.source_scan_id = frameName(laser_frame) + ":" + std::to_string(raw_stamp);
  result.odom_from_base_reference = history.lookup(odom_frame, base_frame,
    result.reference_stamp_ns, secondsToNs(config.max_interpolation_gap_sec));
  return result;
}
// All rays, including invalid ranges, must have valid temporal support. Beam indices are never compacted.
inline TimedScan prepare(
  int64_t raw_stamp, const std::string & laser_frame, double angle_min,
  double angle_increment, double time_increment, double range_min, double range_max,
  const std::vector<float> & ranges, const Config & config, const History & history,
  const std::string & odom_frame, const std::string & base_frame)
{
  validate(config);
  if (!config.enabled || !std::isfinite(angle_min) || !std::isfinite(angle_increment) ||
    !std::isfinite(range_min) || !std::isfinite(range_max) || range_min < 0.0 ||
    range_max <= range_min || (ranges.size() > 1 && time_increment <= 0.0))
  {throw Error(Failure::InvalidInput, "invalid deskew scan or disabled geometry request");}
  TimedScan result = prepareReference(raw_stamp, laser_frame, time_increment, ranges.size(),
    config, history, odom_frame, base_frame);
  const int64_t gap = secondsToNs(config.max_interpolation_gap_sec);
  const Pose base_from_odom = inverse(result.odom_from_base_reference);
  result.beams.reserve(ranges.size());
  result.raw_audit = {ranges, range_min, range_max};
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double raw = ranges[i];
    const int64_t stamp = beamStampNs(raw_stamp, time_increment, i, config);
    const Pose transform = compose(base_from_odom,
      history.lookup(odom_frame, laser_frame, stamp, gap));
    InvalidReason reason = InvalidReason::None;
    if (std::isnan(raw)) {reason = InvalidReason::NotANumber;}
    else if (raw == -std::numeric_limits<double>::infinity()) {reason = InvalidReason::NegativeInfinity;}
    else if (raw < range_min) {reason = InvalidReason::BelowSensorMinimum;}
    else if (std::isfinite(raw) && raw > range_max) {reason = InvalidReason::AboveSensorMaximum;}
    if (reason != InvalidReason::None) {
      result.beams.push_back(Beam::invalid(i, stamp, raw, reason));
    } else {
      const double range = std::min(raw, range_max);
      const double angle = angle_min + static_cast<double>(i) * angle_increment;
      const Point origin = apply(transform, {});
      const Point endpoint = apply(transform, {range * std::cos(angle), range * std::sin(angle)});
      result.beams.push_back(raw < range_max ? Beam::hit(i, stamp, raw, origin, endpoint) :
        Beam::noReturn(i, stamp, raw, origin, endpoint, range_max));
    }
  }
  return result;
}
}  // namespace duojin02_scan_geometry
#endif  // DUOJIN02_SCAN_GEOMETRY__GEOMETRY_HPP_
