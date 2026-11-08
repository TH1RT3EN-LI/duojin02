// Copyright 2026 litianshun <litianshun.cn@gmail.com>. SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

#include "duojin02_base_driver/driver_types.hpp"

namespace duojin02 {

class FeedbackParser {
public:
  using Frame = std::array<uint8_t, RECEIVE_DATA_SIZE>;

  bool push(uint8_t byte, Frame &frame) {
    ++bytes_received;
    if (size_ == 0 && byte != FRAME_HEADER) {
      ++bytes_discarded;
      return false;
    }
    buffer_[size_++] = byte;
    if (size_ != buffer_.size())
      return false;
    uint8_t checksum = 0;
    for (size_t i = 0; i < buffer_.size() - 2; ++i)
      checksum ^= buffer_[i];
    if (buffer_.back() == FRAME_TAIL && checksum == buffer_[buffer_.size() - 2]) {
      frame = buffer_;
      size_ = 0;
      ++frames_received;
      return true;
    }
    if (buffer_.back() != FRAME_TAIL)
      ++tail_errors;
    else
      ++checksum_errors;
    const auto next = std::find(buffer_.begin() + 1, buffer_.end(), FRAME_HEADER);
    const size_t skipped = static_cast<size_t>(next - buffer_.begin());
    std::move(next, buffer_.end(), buffer_.begin());
    size_ -= skipped;
    bytes_discarded += skipped;
    return false;
  }

  void reset() {
    bytes_discarded += size_;
    size_ = 0;
  }

  uint64_t bytes_received{0}, bytes_discarded{0}, frames_received{0};
  uint64_t tail_errors{0}, checksum_errors{0};

private:
  Frame buffer_{};
  size_t size_{0};
};

// The caller serializes access; overflow keeps recent feedback and is accounted.
template<class Sample> class FeedbackQueue {
public:
  explicit FeedbackQueue(size_t capacity = 256) : capacity_(capacity) {
    if (!capacity)
      throw std::invalid_argument("feedback_queue_capacity must be positive");
  }

  void push(Sample sample) {
    if (samples_.size() == capacity_) {
      samples_.pop_front();
      ++overflow_dropped;
    }
    samples_.push_back(std::move(sample));
    ++received;
    high_watermark = std::max(high_watermark, samples_.size());
  }

  std::vector<Sample> take(size_t maximum) {
    std::vector<Sample> result;
    result.reserve(std::min(maximum, samples_.size()));
    while (!samples_.empty() && result.size() < maximum) {
      result.push_back(std::move(samples_.front()));
      samples_.pop_front();
      ++consumed;
    }
    return result;
  }

  void clear() {
    discarded += samples_.size();
    samples_.clear();
  }

  size_t size() const {return samples_.size();}
  uint64_t received{0}, consumed{0}, overflow_dropped{0}, discarded{0};
  size_t high_watermark{0};

private:
  size_t capacity_;
  std::deque<Sample> samples_;
};

}  // namespace duojin02
