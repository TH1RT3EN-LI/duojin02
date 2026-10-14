#pragma once

#include <cstdint>

namespace duojin02 {
struct QuaternionWxyz {
  float w{1.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

class QuaternionSolution {
public:
  QuaternionSolution() = default;

  explicit QuaternionSolution(float kp, float ki) {
    set_gains(kp, ki);
  }

  void reset() noexcept;

  void set_gains(float kp, float ki) noexcept;

  float kp() const noexcept {
    return kp_;
  }

  float ki() const noexcept {
    return ki_;
  }

  void set_g_ref(float g_ref) noexcept;

  float g_ref() const noexcept {
    return g_ref_;
  }

  void set_gate_width_g(float gate_width_g) noexcept;

  float gate_width_g() const noexcept {
    return gate_width_g_;
  }

  void set_gate_min_weight(float w_min) noexcept;

  float gate_min_weight() const noexcept {
    return gate_min_weight_;
  }

  void set_accel_lpf_alpha(float alpha) noexcept;

  float accel_lpf_alpha() const noexcept {
    return accel_lpf_alpha_;
  }

  QuaternionWxyz quaternion() const noexcept {
    return q_;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) noexcept;

private:
  static inline bool finite(float v) noexcept;
  static inline float inv_sqrt(float v) noexcept;
  static inline float clamp(float v, float lo, float hi) noexcept;

private:
  QuaternionWxyz q_{};
  float kp_{2.0f};
  float ki_{0.0f};

  float ex_int_{0.0f};
  float ey_int_{0.0f};
  float ez_int_{0.0f};

  float g_ref_{9.80665f};
  float gate_width_g_{0.30f};
  float gate_min_weight_{0.0f};

  float accel_lpf_alpha_{1.0f};
  bool accel_lpf_init_{false};
  float ax_lpf_{0.0f};
  float ay_lpf_{0.0f};
  float az_lpf_{0.0f};
};

}
