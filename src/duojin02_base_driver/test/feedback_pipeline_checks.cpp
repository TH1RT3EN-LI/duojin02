// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: GPL-3.0-only
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "duojin02_base_driver/feedback_pipeline.hpp"
#include "duojin02_base_driver/quaternion_solution.hpp"

namespace {
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

duojin02::FeedbackParser::Frame frame(uint8_t tag) {
  duojin02::FeedbackParser::Frame result{};
  result.front() = 0x7b;
  result[19] = tag;
  result.back() = 0x7d;
  for (size_t i = 0; i < 22; ++i) result[22] ^= result[i];
  return result;
}
}

int main() {
  using duojin02::FeedbackParser;
  FeedbackParser parser;
  std::vector<FeedbackParser::Frame> accepted;
  auto feed = [&](const auto &bytes) {
    FeedbackParser::Frame output;
    for (uint8_t byte : bytes)
      if (parser.push(byte, output)) accepted.push_back(output);
  };
  feed(std::vector<uint8_t>{0, 1, 2, 0x7b, 0xff, 0xff, 0xff, 0xff});
  feed(frame(10));
  auto bad = frame(20);
  bad[22] ^= 1;
  feed(bad);
  feed(frame(0x7b));  // A header byte inside valid payload must be preserved.
  feed(frame(30));
  require(accepted == std::vector<FeedbackParser::Frame>{frame(10), frame(0x7b), frame(30)},
          "resynchronization lost, reordered or fabricated a frame");
  require(parser.frames_received == 3 && parser.checksum_errors == 1 && parser.tail_errors > 0,
          "parser diagnostics did not report damaged input");
  require(parser.bytes_received == parser.bytes_discarded + 24 * parser.frames_received,
          "parser byte accounting does not balance");
  feed(std::vector<uint8_t>{0x7b, 0, 0});
  parser.reset();
  feed(frame(40));
  require(accepted.back() == frame(40) && accepted.size() == 4,
          "partial data survived a connection reset");

  duojin02::FeedbackQueue<int> queue(8);
  for (int i = 0; i < 64; ++i) queue.push(i);
  require(queue.high_watermark == 8 && queue.overflow_dropped == 56,
          "queue is unbounded or overflow was silent");
  require(queue.take(3) == std::vector<int>{56, 57, 58}, "overflow did not retain recent FIFO order");
  queue.clear();
  queue.push(64);
  require(queue.take(16) == std::vector<int>{64}, "reconnect retained obsolete samples");
  require(queue.received == queue.consumed + queue.overflow_dropped + queue.discarded + queue.size(),
          "queue accounting does not balance");

  duojin02::QuaternionSolution attitude;
  for (int i = 0; i < 1000; ++i)
    require(attitude.update(0, 0, 0.2f, 0, 0, 9.80665f, 0.005f), "valid attitude update rejected");
  auto q = attitude.quaternion();
  require(std::abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.0f) < 1e-6,
          "attitude lost normalization");
  require(std::abs(2 * std::atan2(q.z, q.w) - 1.0f) < 1e-5, "constant-rate rotation changed");
  for (float dt : {0.0f, -0.1f, 0.6f, std::numeric_limits<float>::quiet_NaN()})
    require(!attitude.update(0, 0, 1, 0, 0, 9.80665f, dt), "invalid time reported a fresh attitude");
  require(!attitude.update(0, 0, std::numeric_limits<float>::infinity(), 0, 0, 9.80665f, 0.01f),
          "nonfinite measurement reported a fresh attitude");
  auto after = attitude.quaternion();
  require(q.w == after.w && q.x == after.x && q.y == after.y && q.z == after.z,
          "rejected samples changed the attitude");
  std::cout << "parser recovery, bounded FIFO, accounting and attitude validity passed\n";
}
