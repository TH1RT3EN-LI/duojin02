#pragma once

#include <cstdint>

namespace duojin02
{
  struct QuaternionWxyz
  {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
  };

  /**
   * Mahony IMU (gyro+acc) attitude filter with:
   *  - acceleration magnitude gating
   *  - dynamic kp/ki scaling by accel reliability
   *  - optional accelerometer low-pass filter
   *
   * Units:
   *  - gyro: rad/s
   *  - accel: m/s^2 (recommended). If your accel is normalized to 1g, set g_ref=1.
   *  - dt: seconds
   */
  class QuaternionSolution
  {
  public:
    QuaternionSolution() = default;
    explicit QuaternionSolution(float kp, float ki) { set_gains(kp, ki); }

    void reset() noexcept;

    void set_gains(float kp, float ki) noexcept;
    float kp() const noexcept { return kp_; }
    float ki() const noexcept { return ki_; }

    // --- Gating config ---
    // g_ref: reference gravity magnitude (9.80665 for m/s^2, or 1.0 if accel normalized)
    void set_g_ref(float g_ref) noexcept;
    float g_ref() const noexcept { return g_ref_; }

    // gate_width_g: how far |a| can deviate from g_ref (in g units) before accel is fully rejected.
    // Example: 0.30 means when |a| differs by >=0.30*g_ref, accel correction weight -> 0.
    void set_gate_width_g(float gate_width_g) noexcept;
    float gate_width_g() const noexcept { return gate_width_g_; }

    // minimum weight floor (0..1). Usually 0.
    void set_gate_min_weight(float w_min) noexcept;
    float gate_min_weight() const noexcept { return gate_min_weight_; }

    // --- Accelerometer LPF ---
    // alpha in [0,1]. 0 = no update, 1 = no filtering (use raw).
    // Typical: 0.2~0.5 depending on rate.
    void set_accel_lpf_alpha(float alpha) noexcept;
    float accel_lpf_alpha() const noexcept { return accel_lpf_alpha_; }

    QuaternionWxyz quaternion() const noexcept { return q_; }

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt) noexcept;

  private:
    static inline bool finite(float v) noexcept;
    static inline float inv_sqrt(float v) noexcept;
    static inline float clamp(float v, float lo, float hi) noexcept;

  private:
    QuaternionWxyz q_{};
    float kp_{2.0f};
    float ki_{0.0f};

    // integral error
    float ex_int_{0.0f};
    float ey_int_{0.0f};
    float ez_int_{0.0f};

    // gating parameters
    float g_ref_{9.80665f};
    float gate_width_g_{0.30f};
    float gate_min_weight_{0.0f};

    // accel low-pass filter state
    float accel_lpf_alpha_{1.0f}; // 1.0 means no filtering
    bool accel_lpf_init_{false};
    float ax_lpf_{0.0f};
    float ay_lpf_{0.0f};
    float az_lpf_{0.0f};
  };

} // namespace duojin02
