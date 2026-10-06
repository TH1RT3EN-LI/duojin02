// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unistd.h>

namespace lslidar_driver {

using Packet = std::array<uint8_t, 108>;

class LSIOSR {
public:
    explicit LSIOSR(const std::string &port);
    ~LSIOSR();
    LSIOSR(const LSIOSR &) = delete;
    LSIOSR &operator=(const LSIOSR &) = delete;

    int getSerialData(Packet &packet);
    ssize_t send(const char *buffer, size_t length);

private:
    bool waitEvent(short events, int timeout_ms);
    ssize_t read(uint8_t *buffer, size_t length);
    std::string port_;
    int fd_;
};

} // namespace lslidar_driver
