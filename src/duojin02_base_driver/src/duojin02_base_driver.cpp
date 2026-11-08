#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <atomic>
#include <mutex>
#include <vector>
#include <array>
#include <functional>
#include <stdexcept>

#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "duojin02_base_driver/quaternion_solution.hpp"
#include "duojin02_base_driver/driver_types.hpp"
#include "duojin02_base_driver/feedback_pipeline.hpp"

using namespace std::chrono_literals;

class Duojin02BaseDriverNode : public rclcpp::Node {
public:
  Duojin02BaseDriverNode() : Node("duojin02_base_driver") {
    rcl_interfaces::msg::ParameterDescriptor startup_parameter;
    startup_parameter.read_only = true;
    usart_port_name_ = this->declare_parameter<std::string>(
        "usart_port_name", "/dev/duojin02_controller", startup_parameter);
    serial_baud_rate_ = this->declare_parameter<int>("serial_baud_rate", 115200, startup_parameter);

    odom_frame_id_ =
        this->declare_parameter<std::string>("odom_frame_id", "odom", startup_parameter);
    robot_frame_id_ =
        this->declare_parameter<std::string>("robot_frame_id", "base_footprint", startup_parameter);
    imu_frame_id_ =
        this->declare_parameter<std::string>("imu_frame_id", "imu_link", startup_parameter);

    odom_x_scale_ = this->declare_parameter<double>("odom_x_scale", 1.0, startup_parameter);
    odom_y_scale_ = this->declare_parameter<double>("odom_y_scale", 1.0, startup_parameter);
    odom_z_scale_positive_ =
        this->declare_parameter<double>("odom_z_scale_positive", 1.0, startup_parameter);
    odom_z_scale_negative_ =
        this->declare_parameter<double>("odom_z_scale_negative", 1.0, startup_parameter);

    loop_hz_ = this->declare_parameter<int>("loop_hz", 200, startup_parameter);
    command_timeout_s_ =
        this->declare_parameter<double>("command_timeout_s", 0.25, startup_parameter);
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0)
      throw std::invalid_argument("command_timeout_s must be finite and positive");
    if (serial_baud_rate_ <= 0 || loop_hz_ <= 0)
      throw std::invalid_argument("serial_baud_rate and loop_hz must be positive");
    for (double scale : {odom_x_scale_, odom_y_scale_, odom_z_scale_positive_, odom_z_scale_negative_})
      if (!std::isfinite(scale))
        throw std::invalid_argument("odometry scales must be finite");
    serial_cfg_ = drivers::serial_driver::SerialPortConfig(
        static_cast<uint32_t>(serial_baud_rate_), drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
    imu_yaw_rate_bias_ =
        this->declare_parameter<double>("imu_yaw_rate_bias", 0.0, startup_parameter);
    const double imu_yaw_rate_variance =
        this->declare_parameter<double>("imu_yaw_rate_variance", 1e-6, startup_parameter);
    if (!std::isfinite(imu_yaw_rate_bias_))
      throw std::invalid_argument("imu_yaw_rate_bias must be finite");
    if (!std::isfinite(imu_yaw_rate_variance) || imu_yaw_rate_variance <= 0.0)
      throw std::invalid_argument("imu_yaw_rate_variance must be finite and positive");
    const auto capacity = this->declare_parameter<int>("feedback_queue_capacity", 256, startup_parameter);
    if (capacity < 1 || capacity > 4096)
      throw std::invalid_argument("feedback_queue_capacity must be in [1,4096]");
    feedback_ = duojin02::FeedbackQueue<FeedbackSample>(static_cast<size_t>(capacity));
    const auto diagnostic_hz = this->declare_parameter<double>("diagnostic_hz", 1.0, startup_parameter);
    if (!std::isfinite(diagnostic_hz) || diagnostic_hz <= 0.0 || diagnostic_hz > 100.0)
      throw std::invalid_argument("diagnostic_hz must be finite and in (0,100]");
    auto load_twist_variances = [this, startup_parameter](const std::string &name,
                                                          const std::array<double, 36> &defaults) {
      std::vector<double> diagonal;
      for (size_t axis = 0; axis < 6; ++axis)
        diagonal.push_back(defaults[axis * 7]);
      const auto values =
          this->declare_parameter<std::vector<double>>(name, diagonal, startup_parameter);
      if (values.size() != 6)
        throw std::invalid_argument(name + " must contain exactly six variances");
      std::array<double, 36> result{};
      for (size_t axis = 0; axis < 6; ++axis) {
        if (!std::isfinite(values[axis]) || values[axis] <= 0.0)
          throw std::invalid_argument(name + " variances must be finite and positive");
        result[axis * 7] = values.at(axis);
      }
      return result;
    };
    odom_twist_moving_ =
        load_twist_variances("odom_twist_variances_moving", duojin02::odom_twist_covariance);
    odom_twist_stopped_ =
        load_twist_variances("odom_twist_variances_stopped", duojin02::odom_twist_covariance2);

    voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(1),
        std::bind(&Duojin02BaseDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    imu_msg_.header.frame_id = imu_frame_id_;

    imu_msg_.orientation_covariance[0] = 1e6;
    imu_msg_.orientation_covariance[4] = 1e6;
    imu_msg_.orientation_covariance[8] = 1e-6;

    imu_msg_.angular_velocity_covariance[0] = 1e6;
    imu_msg_.angular_velocity_covariance[4] = 1e6;
    imu_msg_.angular_velocity_covariance[8] = imu_yaw_rate_variance;

    imu_msg_.linear_acceleration_covariance[0] = 1e6;
    imu_msg_.linear_acceleration_covariance[4] = 1e6;
    imu_msg_.linear_acceleration_covariance[8] = 1e-6;

    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = robot_frame_id_;
    odom_msg_.pose.pose.position.z = 0.0;

    last_frame_steady_time_ = std::chrono::steady_clock::now();
    attitude_filter_.set_g_ref(9.80665f);
    attitude_filter_.set_gate_width_g(0.30f);
    attitude_filter_.set_accel_lpf_alpha(0.30f);

    try_open_serial_port();
    if (!serial_ || !serial_->is_open()) {
      serial_retry_timer_ = this->create_wall_timer(
          500ms, std::bind(&Duojin02BaseDriverNode::try_open_serial_port, this));
    }

    auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = this->create_wall_timer(period, std::bind(&Duojin02BaseDriverNode::on_timer, this));
    diagnostics_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / diagnostic_hz),
        std::bind(&Duojin02BaseDriverNode::publish_diagnostics, this));

    RCLCPP_INFO(this->get_logger(), "duojin02_base_driver node started");

  }

  ~Duojin02BaseDriverNode() override {
    shutting_down_.store(true, std::memory_order_release);
    if (serial_retry_timer_) {
      serial_retry_timer_->cancel();
    }

    try {
      if (serial_ && serial_->is_open()) {
        send_stop_command();
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "failed to send stop command: %s", e.what());
    }
    if (serial_ && serial_->is_open())
      serial_->close();
    io_ctx_.waitForExit();

    RCLCPP_INFO(this->get_logger(), "duojin02_base_driver node shutting down");
  }

private:
  struct FeedbackSample {
    duojin02::VelPosData velocity;
    sensor_msgs::msg::Imu imu;
    float voltage;
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point received;
    uint64_t sequence;
  };

  static inline uint8_t xor_checksum(const uint8_t *data, std::size_t len) noexcept {
    uint8_t sum = 0;
    for (std::size_t i = 0; i < len; ++i)
      sum ^= data[i];
    return sum;
  }

  static inline int16_t saturate_i16_from_scaled(double value, double scale) noexcept {
    const double scaled = value * scale;
    if (!std::isfinite(scaled)) {
      return 0;
    }
    return static_cast<int16_t>(
        std::lround(std::clamp(scaled, static_cast<double>(std::numeric_limits<int16_t>::min()),
                               static_cast<double>(std::numeric_limits<int16_t>::max()))));
  }

  static inline void encode_i16_be(int16_t v, uint8_t &high, uint8_t &low) noexcept {
    const uint16_t u = static_cast<uint16_t>(v);
    high = static_cast<uint8_t>((u >> 8) & 0xFF);
    low = static_cast<uint8_t>(u & 0xFF);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (shutting_down_.load(std::memory_order_acquire) || serial_reconnecting_ || !serial_ ||
        !serial_->is_open()) {
      command_active_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Serial port not ready, ignoring cmd_vel");
      return;
    }

    const bool valid = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) &&
                       std::isfinite(msg->angular.z);
    auto &tx = tx_buf_;
    constexpr double k_mmps = 1000.0;
    const int16_t vx_mmps = valid ? saturate_i16_from_scaled(msg->linear.x, k_mmps) : 0;
    const int16_t vy_mmps = valid ? saturate_i16_from_scaled(msg->linear.y, k_mmps) : 0;
    const int16_t wz_mradps = valid ? saturate_i16_from_scaled(msg->angular.z, k_mmps) : 0;

    encode_i16_be(vx_mmps, tx[3], tx[4]);
    encode_i16_be(vy_mmps, tx[5], tx[6]);
    encode_i16_be(wz_mradps, tx[7], tx[8]);

    tx[9] = xor_checksum(tx.data(), 9);

    try {
      send_command_frame();
      last_command_time_ = std::chrono::steady_clock::now();
      command_active_ = vx_mmps != 0 || vy_mmps != 0 || wz_mradps != 0;
      if (!valid)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Invalid cmd_vel, sent zero velocity to controller");
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "serial error in cmd_vel_callback: %s", e.what());

      handle_serial_error(std::string("cmd_vel send failed: ") + e.what());
    }
  }

  void send_command_frame() {
    if (serial_->send(tx_buf_) != tx_buf_.size())
      throw std::runtime_error("incomplete velocity frame write");
  }

  void send_stop_command() {
    auto &tx = tx_buf_;
    std::fill(tx.begin() + 3, tx.begin() + 9, 0);
    tx[9] = xor_checksum(tx.data(), 9);
    send_command_frame();
  }

  void handle_serial_error(const std::string &reason) {
    command_active_ = false;
    if (shutting_down_.load(std::memory_order_acquire))
      return;
    if (serial_reconnecting_)
      return;

    serial_reconnecting_ = true;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      feedback_.clear();
    }
    sample_time_ready_ = false;
    orientation_updated_ = false;

    RCLCPP_ERROR(this->get_logger(), "Serial error: %s — closing port and scheduling reconnect.",
                 reason.c_str());

    if (serial_) {
      if (serial_->is_open()) {
        try {
          send_stop_command();
        } catch (...) {
        }
        try {
          serial_->close();
        } catch (...) {
        }
      }
    }

    {
      std::lock_guard<std::mutex> lk(receive_mtx_);
      parser_.reset();
    }

    if (!serial_retry_timer_) {
      serial_retry_timer_ = this->create_wall_timer(
          100ms, std::bind(&Duojin02BaseDriverNode::try_open_serial_port, this));
    }
  }

  void on_timer() {
    if (serial_reconnecting_ || !serial_)
      return;

    if (serial_eof_detected_.exchange(false, std::memory_order_acq_rel)) {
      handle_serial_error("EOF detected by async receiver — device disconnected");
      return;
    }

    if (!serial_->is_open()) {
      handle_serial_error("port closed unexpectedly (possible cable disconnect / EOF)");
      return;
    }

    if (command_active_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_time_)
                .count() > command_timeout_s_) {
      try {
        send_stop_command();
        command_active_ = false;
        RCLCPP_WARN(get_logger(), "Velocity stream expired; sent zero velocity to controller");
      } catch (const std::exception &e) {
        handle_serial_error(std::string("expired command stop failed: ") + e.what());
        return;
      }
    }
    std::chrono::steady_clock::time_point last_received;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      last_received = last_frame_steady_time_;
    }
    const double since_last =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_received).count();
    if (since_last > 0.5) {
      handle_serial_error("no data received for " + std::to_string(since_last) +
                          " seconds (silent failure)");
      return;
    }
    std::vector<FeedbackSample> samples;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      samples = feedback_.take(16);
    }
    for (auto &sample : samples)
      process_feedback(sample);
  }

  void process_feedback(FeedbackSample &sample) {
    auto vel = sample.velocity;
    auto &imu = sample.imu;
    const auto &received_stamp = sample.stamp;
    double dt = 0.0;
    if (sample_time_ready_) {
      if (sample.sequence != last_sample_sequence_ + 1) {
        ++sequence_gaps_;
      } else if (received_stamp <= last_sample_time_) {
        ++nonpositive_time_steps_;
      } else {
        dt = (received_stamp - last_sample_time_).seconds();
        if (dt > duojin02::MAX_DELTA_T) {
          dt = 0.0;
          ++large_time_steps_;
        }
      }
    }
    last_sample_time_ = received_stamp;
    last_sample_sequence_ = sample.sequence;
    sample_time_ready_ = true;
    orientation_updated_ = false;
    max_queue_delay_s_ = std::max(max_queue_delay_s_,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count());
    imu.angular_velocity.z -= imu_yaw_rate_bias_;
    vel.x = static_cast<float>(vel.x * odom_x_scale_);
    vel.y = static_cast<float>(vel.y * odom_y_scale_);

    if (vel.z >= 0.0f) {
      vel.z = static_cast<float>(vel.z * odom_z_scale_positive_);
    } else {
      vel.z = static_cast<float>(vel.z * odom_z_scale_negative_);
    }

    if (dt > 0.0) {
      const float yaw = robot_pos_.z;
      const float c = std::cos(yaw);
      const float s = std::sin(yaw);
      const float dt_f = static_cast<float>(dt);

      robot_pos_.x += (vel.x * c - vel.y * s) * dt_f;
      robot_pos_.y += (vel.x * s + vel.y * c) * dt_f;
      robot_pos_.z += vel.z * dt_f;

      while (robot_pos_.z > duojin02::PI)
        robot_pos_.z -= 2.0f * duojin02::PI;
      while (robot_pos_.z < -duojin02::PI)
        robot_pos_.z += 2.0f * duojin02::PI;

      orientation_updated_ = attitude_filter_.update(
          static_cast<float>(imu.angular_velocity.x), static_cast<float>(imu.angular_velocity.y),
          static_cast<float>(imu.angular_velocity.z), static_cast<float>(imu.linear_acceleration.x),
          static_cast<float>(imu.linear_acceleration.y),
          static_cast<float>(imu.linear_acceleration.z), static_cast<float>(dt));
    }

    const auto q = attitude_filter_.quaternion();
    imu.orientation.w = q.w;
    imu.orientation.x = q.x;
    imu.orientation.y = q.y;
    imu.orientation.z = q.z;
    if (!orientation_updated_) {
      imu.orientation_covariance[0] = -1.0;
      ++orientation_unavailable_;
    }

    publish_odom(received_stamp, vel);
    publish_imu(received_stamp, imu);
    publish_voltage(sample.voltage);
    ++published_feedback_;
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "base_driver/feedback";
    status.hardware_id = usart_port_name_;
    auto value = [&status](const std::string &key, const auto &item) {
      diagnostic_msgs::msg::KeyValue field;
      field.key = key;
      field.value = std::to_string(item);
      status.values.push_back(std::move(field));
    };
    {
      std::lock_guard<std::mutex> receive_lock(receive_mtx_);
      std::lock_guard<std::mutex> state_lock(state_mtx_);
      const auto age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_frame_steady_time_).count();
      status.level = age > 0.5 || serial_reconnecting_ || !orientation_updated_ ?
          diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
      value("received_frames", feedback_.received);
      value("published_frames", published_feedback_);
      value("queued_frames", feedback_.size());
      value("overflow_dropped_frames", feedback_.overflow_dropped);
      value("reset_discarded_frames", feedback_.discarded);
      value("queue_high_watermark", feedback_.high_watermark);
      value("checksum_errors", parser_.checksum_errors);
      value("tail_errors", parser_.tail_errors);
      value("discarded_bytes", parser_.bytes_discarded);
      value("last_feedback_age_s", age);
    }
    value("sequence_gaps", sequence_gaps_);
    value("nonpositive_time_steps", nonpositive_time_steps_);
    value("large_time_steps", large_time_steps_);
    value("orientation_updated", orientation_updated_);
    value("orientation_unavailable_frames", orientation_unavailable_);
    value("orientation_covariance_calibrated", false);
    value("max_queue_delay_s", max_queue_delay_s_);
    status.message = "host receive timestamps; relative attitude covariance uncalibrated";
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_pub_->publish(message);
  }

  void start_serial_receive() {
    if (!serial_ || !serial_->is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot start serial receive: port not ready");
      return;
    }
    try {
      serial_->async_receive([this](std::vector<uint8_t> &data, const size_t bytes_transferred) {
        this->on_serial_data(data, bytes_transferred);
      });
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "async_receive failed: %s", e.what());
    }
  }

  void on_serial_data(std::vector<uint8_t> &data, size_t n) {
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }
    if (n == 0) {
      serial_eof_detected_.store(true, std::memory_order_release);
      return;
    }
    if (n > data.size())
      n = data.size();

    std::lock_guard<std::mutex> lk(receive_mtx_);
    for (size_t i = 0; i < n; ++i) {
      push_byte_and_parse(data[i]);
    }
  }

  void push_byte_and_parse(uint8_t byte) {
    duojin02::FeedbackParser::Frame frame;
    if (parser_.push(byte, frame))
      decode_frame_and_update_state(frame);
  }

  void decode_frame_and_update_state(const std::array<uint8_t, duojin02::RECEIVE_DATA_SIZE> &rx) {
    const float vx = decode_vel_mps_be(rx[2], rx[3]);
    const float vy = decode_vel_mps_be(rx[4], rx[5]);
    const float wz = decode_vel_mps_be(rx[6], rx[7]);

    duojin02::IMUData imu_raw;
    imu_raw.accele_x_data = decode_i16_be(rx[8], rx[9]);
    imu_raw.accele_y_data = decode_i16_be(rx[10], rx[11]);
    imu_raw.accele_z_data = decode_i16_be(rx[12], rx[13]);
    imu_raw.gyros_x_data = decode_i16_be(rx[14], rx[15]);
    imu_raw.gyros_y_data = decode_i16_be(rx[16], rx[17]);
    imu_raw.gyros_z_data = decode_i16_be(rx[18], rx[19]);

    const int16_t v_mv =
        static_cast<int16_t>((static_cast<uint16_t>(rx[20]) << 8) | static_cast<uint16_t>(rx[21]));
    const float v = static_cast<float>(v_mv) * 0.001f;

    {
      std::lock_guard<std::mutex> lk(state_mtx_);

      robot_vel_.x = vx;
      robot_vel_.y = vy;
      robot_vel_.z = wz;

      imu_msg_.linear_acceleration.x =
          static_cast<double>(imu_raw.accele_x_data) / duojin02::ACCEl_RATIO;
      imu_msg_.linear_acceleration.y =
          static_cast<double>(imu_raw.accele_y_data) / duojin02::ACCEl_RATIO;
      imu_msg_.linear_acceleration.z =
          static_cast<double>(imu_raw.accele_z_data) / duojin02::ACCEl_RATIO;

      imu_msg_.angular_velocity.x =
          static_cast<double>(imu_raw.gyros_x_data) * duojin02::GYROSCOPE_RATIO;
      imu_msg_.angular_velocity.y =
          static_cast<double>(imu_raw.gyros_y_data) * duojin02::GYROSCOPE_RATIO;
      imu_msg_.angular_velocity.z =
          static_cast<double>(imu_raw.gyros_z_data) * duojin02::GYROSCOPE_RATIO;

      power_voltage_ = v;
      latest_frame_stamp_ = this->now();
      last_frame_steady_time_ = std::chrono::steady_clock::now();
      const auto sequence = feedback_.received + 1;
      feedback_.push(FeedbackSample{robot_vel_, imu_msg_, power_voltage_, latest_frame_stamp_,
                                    last_frame_steady_time_, sequence});
    }
  }

  static inline int16_t decode_i16_be(uint8_t high, uint8_t low) noexcept {
    const uint16_t u = (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    return static_cast<int16_t>(u);
  }

  static inline float decode_vel_mps_be(uint8_t high, uint8_t low) noexcept {
    constexpr float k_inv_1000 = 0.001f;
    return static_cast<float>(decode_i16_be(high, low)) * k_inv_1000;
  }

  void publish_imu(const rclcpp::Time &stamp, sensor_msgs::msg::Imu &imu) {
    imu.header.stamp = stamp;
    imu_pub_->publish(imu);
  }

  void publish_voltage(float voltage) {
    static int count = 0;
    if (++count <= 10)
      return;
    count = 0;
    std_msgs::msg::Float32 msg;
    msg.data = voltage;
    voltage_pub_->publish(msg);
  }

  void publish_odom(const rclcpp::Time &stamp, duojin02::VelPosData vel) {
    odom_msg_.header.stamp = stamp;

    odom_msg_.pose.pose.position.x = robot_pos_.x;
    odom_msg_.pose.pose.position.y = robot_pos_.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pos_.z);
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);

    odom_msg_.twist.twist.linear.x = vel.x;
    odom_msg_.twist.twist.linear.y = vel.y;
    odom_msg_.twist.twist.angular.z = vel.z;

    const bool is_stopped =
        (std::fabs(vel.x) < 1e-6) && (std::fabs(vel.y) < 1e-6) && (std::fabs(vel.z) < 1e-6);

    if (is_stopped) {
      std::copy(duojin02::odom_pose_covariance2.begin(), duojin02::odom_pose_covariance2.end(),
                odom_msg_.pose.covariance.begin());
      std::copy(odom_twist_stopped_.begin(), odom_twist_stopped_.end(),
                odom_msg_.twist.covariance.begin());
    } else {
      std::copy(duojin02::odom_pose_covariance.begin(), duojin02::odom_pose_covariance.end(),
                odom_msg_.pose.covariance.begin());
      std::copy(odom_twist_moving_.begin(), odom_twist_moving_.end(),
                odom_msg_.twist.covariance.begin());
    }

    odom_pub_->publish(odom_msg_);
  }

private:
  void try_open_serial_port() {
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }
    if (serial_ && serial_->is_open()) {
      return;
    }

    try {
      serial_ = std::make_unique<drivers::serial_driver::SerialPort>(io_ctx_, usart_port_name_,
                                                                     serial_cfg_);
      serial_->open();

      send_stop_command();
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cannot initialize serial port %s: %s", usart_port_name_.c_str(),
                           e.what());
      if (serial_ && serial_->is_open())
        try {
          serial_->close();
        } catch (...) {
        }
      serial_.reset();
      return;
    }

    if (serial_ && serial_->is_open()) {
      command_active_ = false;
      serial_eof_detected_.store(false, std::memory_order_release);
      RCLCPP_WARN(this->get_logger(), "connected — sent stop command to lower controller");
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        feedback_.clear();
        last_frame_steady_time_ = std::chrono::steady_clock::now();
      }
      sample_time_ready_ = false;
      orientation_updated_ = false;
      {
        std::lock_guard<std::mutex> lk(receive_mtx_);
        parser_.reset();
      }
      start_serial_receive();
      serial_reconnecting_ = false;
      if (serial_retry_timer_) {
        serial_retry_timer_->cancel();
        serial_retry_timer_.reset();
      }
      RCLCPP_INFO(this->get_logger(), "serial port opened: %s", usart_port_name_.c_str());
    }
  }

  std::string usart_port_name_;
  int serial_baud_rate_{115200};

  std::string odom_frame_id_;
  std::string robot_frame_id_;
  std::string imu_frame_id_;

  double odom_x_scale_{1.0};
  double odom_y_scale_{1.0};
  double odom_z_scale_positive_{1.0};
  double odom_z_scale_negative_{1.0};

  int loop_hz_{200};
  double imu_yaw_rate_bias_{0.0};
  std::array<double, 36> odom_twist_moving_{}, odom_twist_stopped_{};

  duojin02::FeedbackParser parser_;
  std::mutex receive_mtx_;
  std::atomic<bool> shutting_down_{false};
  bool serial_reconnecting_{false};

  std::atomic<bool> serial_eof_detected_{false};

  duojin02::FeedbackQueue<FeedbackSample> feedback_;
  std::mutex state_mtx_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr serial_retry_timer_;

  rclcpp::Time last_sample_time_, latest_frame_stamp_;
  bool sample_time_ready_{false};
  bool orientation_updated_{false};
  uint64_t last_sample_sequence_{0}, published_feedback_{0}, sequence_gaps_{0};
  uint64_t nonpositive_time_steps_{0}, large_time_steps_{0}, orientation_unavailable_{0};
  double max_queue_delay_s_{0.0};
  std::chrono::steady_clock::time_point last_frame_steady_time_;
  std::chrono::steady_clock::time_point last_command_time_;
  double command_timeout_s_{0.25};
  bool command_active_{false};

  drivers::common::IoContext io_ctx_;
  drivers::serial_driver::SerialPortConfig serial_cfg_{
      static_cast<uint32_t>(serial_baud_rate_), drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE};
  std::unique_ptr<drivers::serial_driver::SerialPort> serial_;

  duojin02::VelPosData robot_pos_;
  duojin02::VelPosData robot_vel_;

  std::vector<uint8_t> tx_buf_{duojin02::FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               duojin02::FRAME_TAIL};

  float power_voltage_{0.0f};

  sensor_msgs::msg::Imu imu_msg_;
  nav_msgs::msg::Odometry odom_msg_;

  duojin02::QuaternionSolution attitude_filter_{2.0f, 0.0f};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<Duojin02BaseDriverNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("duojin02_base_driver"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
