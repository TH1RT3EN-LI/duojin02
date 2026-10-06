// Copyright 2022 LeiShen Intelligent Authors.
// SPDX-License-Identifier: Apache-2.0
#include "lslidar_driver/lsiosr.hpp"
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <system_error>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>

namespace lslidar_driver {

LSIOSR::LSIOSR(const std::string &port) : port_(port) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::system_error(errno, std::system_category(), "Unable to open " + port);
    try {
        termios options{};
        if (tcgetattr(fd_, &options) != 0) {
            throw std::system_error(errno, std::system_category(), "Cannot read serial settings");
        }
        cfmakeraw(&options);
        options.c_cflag = (options.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);
        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 0;
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            throw std::system_error(errno, std::system_category(), "Cannot configure serial port");
        }
        tcflush(fd_, TCIOFLUSH);
    } catch (...) {
        ::close(fd_);
        throw;
    }
}

LSIOSR::~LSIOSR() { ::close(fd_); }

bool LSIOSR::waitEvent(short events, int timeout_ms) {
    pollfd descriptor{fd_, events, 0};
    const int result = ::poll(&descriptor, 1, timeout_ms);
    if (result > 0) {
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            errno = EIO;
            return false;
        }
        return (descriptor.revents & events) != 0;
    }
    if (result == 0) errno = 0;
    return false;
}

    ssize_t LSIOSR::read(uint8_t* buffer, size_t length) {
        ssize_t totalBytesRead = 0;
        if (!waitEvent(POLLIN, 2000)) {
            return errno != 0 && errno != EINTR ? -1 : 0;
        }

        while (length > 0) {
            ssize_t rc = ::read(fd_, buffer, length);
            if (rc > 0) {
                length -= rc;
                buffer += rc;
                totalBytesRead += rc;
            } else {
                if (rc < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                    return -1;
                }
                // VMIN=0 allows a live tty to return zero between fragments.
                // poll() distinguishes that temporary gap from a disconnected port.
                if (!rclcpp::ok()) return -1;
                if (!waitEvent(POLLIN, 2000)) {
                    return errno != 0 && errno != EINTR ? -1 : totalBytesRead;
                }
            }
        }
        return totalBytesRead;
    }
    

int LSIOSR::getSerialData(Packet &packet) {
    const auto read_exactly = [this](uint8_t *destination, size_t length) {
        size_t received = 0;
        while (received < length && rclcpp::ok()) {
            const ssize_t count = read(destination + received, length - received);
            if (count < 0) return false;
            received += static_cast<size_t>(count);
        }
        return received == length;
    };
    if (!read_exactly(packet.data(), 1)) return -1;
    if (packet[0] != 0xA5) return 0;
    if (!read_exactly(packet.data() + 1, 1)) return -1;
    if (packet[1] != 0x5A) return 0;
    if (!read_exactly(packet.data() + 2, packet.size() - 2)) return -1;
    return packet.size();
}

ssize_t LSIOSR::send(const char *buffer, size_t length) {
    if (!waitEvent(POLLOUT, 2000)) return -1;
    size_t sent = 0;
    while (sent < length && rclcpp::ok()) {
        const ssize_t count = ::write(fd_, buffer + sent, length - sent);
        if (count > 0) {
            sent += static_cast<size_t>(count);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (!waitEvent(POLLOUT, 2000)) return -1;
        } else {
            return -1;
        }
    }
    return sent;
}

} // namespace lslidar_driver
