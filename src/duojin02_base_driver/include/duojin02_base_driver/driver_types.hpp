#pragma once

#include <cstdint>
#include <array>

namespace duojin02 {
constexpr uint8_t FRAME_HEADER = 0x7B;
constexpr uint8_t FRAME_TAIL = 0x7D;
constexpr std::size_t RECEIVE_DATA_SIZE = 24;

constexpr float PI = 3.1415926f;

constexpr float MAX_DELTA_T = 1.0f;

constexpr float GYROSCOPE_RATIO = 0.00026644f;
constexpr float ACCEl_RATIO = 1671.84f;

inline constexpr std::array<double, 36> odom_pose_covariance = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

inline constexpr std::array<double, 36> odom_pose_covariance2 = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};

inline constexpr std::array<double, 36> odom_twist_covariance = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

inline constexpr std::array<double, 36> odom_twist_covariance2 = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};

struct VelPosData {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct IMUData {
  int16_t accele_x_data{0};
  int16_t accele_y_data{0};
  int16_t accele_z_data{0};
  int16_t gyros_x_data{0};
  int16_t gyros_y_data{0};
  int16_t gyros_z_data{0};
};

}
