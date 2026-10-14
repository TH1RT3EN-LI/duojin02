#include "duojin02_base_driver/quaternion_solution.hpp"

#include <cmath>
#include <limits>

namespace duojin02
{

inline bool QuaternionSolution::finite(float v) noexcept
{
  return std::isfinite(static_cast<double>(v));
}

inline float QuaternionSolution::inv_sqrt(float v) noexcept
{
  return 1.0f / std::sqrt(v);
}

inline float QuaternionSolution::clamp(float v, float lo, float hi) noexcept
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void QuaternionSolution::reset() noexcept
{
  q_ = QuaternionWxyz{};
  ex_int_ = ey_int_ = ez_int_ = 0.0f;
  accel_lpf_init_ = false;
  ax_lpf_ = ay_lpf_ = az_lpf_ = 0.0f;
}

void QuaternionSolution::set_gains(float kp, float ki) noexcept
{
  kp_ = (kp >= 0.0f) ? kp : 0.0f;
  ki_ = (ki >= 0.0f) ? ki : 0.0f;
}

void QuaternionSolution::set_g_ref(float g_ref) noexcept
{
  if (finite(g_ref) && g_ref > 1e-6f) {
    g_ref_ = g_ref;
  }
}

void QuaternionSolution::set_gate_width_g(float gate_width_g) noexcept
{
  if (finite(gate_width_g) && gate_width_g > 0.0f) {
    gate_width_g_ = gate_width_g;
  }
}

void QuaternionSolution::set_gate_min_weight(float w_min) noexcept
{
  if (finite(w_min)) {
    gate_min_weight_ = clamp(w_min, 0.0f, 1.0f);
  }
}

void QuaternionSolution::set_accel_lpf_alpha(float alpha) noexcept
{
  if (finite(alpha)) {
    accel_lpf_alpha_ = clamp(alpha, 0.0f, 1.0f);
  }
}

void QuaternionSolution::update(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float dt) noexcept
{
  // ---- dt guard ----
  if (!finite(dt) || dt <= 0.0f) return;
  if (dt > 0.5f) return;

  // ---- input guard ----
  if (!finite(gx) || !finite(gy) || !finite(gz) ||
      !finite(ax) || !finite(ay) || !finite(az)) {
    return;
  }

  // ---- accel LPF (optional) ----
  if (accel_lpf_alpha_ < 1.0f) {
    if (!accel_lpf_init_) {
      ax_lpf_ = ax; ay_lpf_ = ay; az_lpf_ = az;
      accel_lpf_init_ = true;
    } else {
      const float a = accel_lpf_alpha_;
      ax_lpf_ = a * ax + (1.0f - a) * ax_lpf_;
      ay_lpf_ = a * ay + (1.0f - a) * ay_lpf_;
      az_lpf_ = a * az + (1.0f - a) * az_lpf_;
    }
    ax = ax_lpf_; ay = ay_lpf_; az = az_lpf_;
  }

  // ---- accel magnitude & gating weight ----
  float a2 = ax*ax + ay*ay + az*az;
  bool accel_valid = (a2 > 1e-12f);

  float w_acc = 0.0f;   // accel reliability weight in [0,1]
  if (accel_valid && finite(a2)) {
    const float a_mag = std::sqrt(a2);

    // deviation from reference gravity
    // gate_width_g_ is expressed in "g" units: allowed deviation = gate_width_g_ * g_ref_
    const float allowed = gate_width_g_ * g_ref_;
    if (allowed > 1e-6f) {
      const float dev = std::fabs(a_mag - g_ref_);
      // linear ramp: dev=0 -> 1, dev>=allowed -> 0
      w_acc = 1.0f - (dev / allowed);
      w_acc = clamp(w_acc, gate_min_weight_, 1.0f);
    } else {
      w_acc = 1.0f;
    }

    // if weight is essentially zero, treat accel as invalid for correction
    if (w_acc <= 1e-6f) {
      accel_valid = false;
      w_acc = 0.0f;
    }
  }

  // ---- normalize accel direction if used ----
  if (accel_valid) {
    const float inv_norm = inv_sqrt(a2);
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;
  }

  // current quaternion
  float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

  // estimated gravity direction from quaternion
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // error = a x v
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;
  if (accel_valid) {
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  }

  // dynamic gains scaled by accel reliability
  const float kp_eff = kp_ * w_acc;
  const float ki_eff = ki_ * w_acc;

  // integral feedback
  if (ki_eff > 0.0f && accel_valid) {
    ex_int_ += ex * dt;
    ey_int_ += ey * dt;
    ez_int_ += ez * dt;

    // integral clamp
    constexpr float k_i_limit = 0.5f;
    ex_int_ = clamp(ex_int_, -k_i_limit, k_i_limit);
    ey_int_ = clamp(ey_int_, -k_i_limit, k_i_limit);
    ez_int_ = clamp(ez_int_, -k_i_limit, k_i_limit);

    gx += ki_eff * ex_int_;
    gy += ki_eff * ey_int_;
    gz += ki_eff * ez_int_;
  }

  // proportional feedback
  if (kp_eff > 0.0f && accel_valid) {
    gx += kp_eff * ex;
    gy += kp_eff * ey;
    gz += kp_eff * ez;
  }

  // quaternion integration
  const float half_dt = 0.5f * dt;

  const float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
  const float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
  const float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
  const float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

  // normalize quaternion
  const float qn = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (!finite(qn) || qn < 1e-12f) {
    reset();
    return;
  }
  const float inv_qn = inv_sqrt(qn);

  q_.w = q0 * inv_qn;
  q_.x = q1 * inv_qn;
  q_.y = q2 * inv_qn;
  q_.z = q3 * inv_qn;
}

} // namespace duojin02
