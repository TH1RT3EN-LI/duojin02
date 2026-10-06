// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include "lslidar_driver/lsiosr.hpp"

namespace lslidar_driver {

// Transport boundary also used by the offline protocol probes.
class DataAcquisitionStrategy {
public:
    virtual ~DataAcquisitionStrategy() = default;
    virtual int getPacket(Packet &packet) = 0;
};

class SerialStrategy : public DataAcquisitionStrategy {
public:
    explicit SerialStrategy(std::shared_ptr<LSIOSR> serial) : serial_(std::move(serial)) {}
    int getPacket(Packet &packet) override {
        return serial_->getSerialData(packet);
    }
private:
    std::shared_ptr<LSIOSR> serial_;
};

} // namespace lslidar_driver
