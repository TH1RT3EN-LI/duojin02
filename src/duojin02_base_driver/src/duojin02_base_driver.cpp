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
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "duojin02_base_driver/quaternion_solution.hpp"
#include "duojin02_base_driver/driver_types.hpp"

using namespace std::chrono_literals;

class Duojin02BaseDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    Duojin02BaseDriverNode()
        : Node("duojin02_base_driver")
    {
        rcl_interfaces::msg::ParameterDescriptor startup_parameter;
        startup_parameter.read_only = true;
        usart_port_name_ = this->declare_parameter<std::string>("usart_port_name", "/dev/duojin02_controller", startup_parameter);
        serial_baud_rate_ = this->declare_parameter<int>("serial_baud_rate", 115200, startup_parameter);

        odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom", startup_parameter);
        robot_frame_id_ = this->declare_parameter<std::string>("robot_frame_id", "base_footprint", startup_parameter);
        imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", "imu_link", startup_parameter);

        // 用于缩放里程计的尺度
        odom_x_scale_ = this->declare_parameter<double>("odom_x_scale", 1.0, startup_parameter);
        odom_y_scale_ = this->declare_parameter<double>("odom_y_scale", 1.0, startup_parameter);
        odom_z_scale_positive_ = this->declare_parameter<double>("odom_z_scale_positive", 1.0, startup_parameter);
        odom_z_scale_negative_ = this->declare_parameter<double>("odom_z_scale_negative", 1.0, startup_parameter);

        loop_hz_ = this->declare_parameter<int>("loop_hz", 200, startup_parameter);
        command_timeout_s_ = this->declare_parameter<double>("command_timeout_s", 0.25, startup_parameter);
        if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0)
            throw std::invalid_argument("command_timeout_s must be finite and positive");
        serial_cfg_ = drivers::serial_driver::SerialPortConfig(
            static_cast<uint32_t>(serial_baud_rate_), drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
        imu_yaw_rate_bias_ = this->declare_parameter<double>("imu_yaw_rate_bias", 0.0, startup_parameter);
        const double imu_yaw_rate_variance =
            this->declare_parameter<double>("imu_yaw_rate_variance", 1e-6, startup_parameter);
        auto load_twist_variances = [this, startup_parameter](const std::string &name,
            const std::array<double, 36> &defaults) {
            std::vector<double> diagonal;
            for (size_t axis = 0; axis < 6; ++axis) diagonal.push_back(defaults[axis * 7]);
            const auto values = this->declare_parameter<std::vector<double>>(name, diagonal, startup_parameter);
            std::array<double, 36> result{};
            for (size_t axis = 0; axis < 6; ++axis) {
                result[axis * 7] = values.at(axis);
            }
            return result;
        };
        odom_twist_moving_ = load_twist_variances("odom_twist_variances_moving", duojin02::odom_twist_covariance);
        odom_twist_stopped_ = load_twist_variances("odom_twist_variances_stopped", duojin02::odom_twist_covariance2);

        // -------- publishers --------
        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);

        // -------- subscriber --------
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            rclcpp::QoS(1),
            std::bind(&Duojin02BaseDriverNode::cmd_vel_callback, this, std::placeholders::_1));

        // -------- imu --------
        imu_msg_.header.frame_id = imu_frame_id_;
        // orientation covariance
        imu_msg_.orientation_covariance[0] = 1e6;
        imu_msg_.orientation_covariance[4] = 1e6;
        imu_msg_.orientation_covariance[8] = 1e-6;

        // angular velocity covariance
        imu_msg_.angular_velocity_covariance[0] = 1e6;
        imu_msg_.angular_velocity_covariance[4] = 1e6;
        imu_msg_.angular_velocity_covariance[8] = imu_yaw_rate_variance;

        // linear acceleration covariance
        imu_msg_.linear_acceleration_covariance[0] = 1e6;
        imu_msg_.linear_acceleration_covariance[4] = 1e6;
        imu_msg_.linear_acceleration_covariance[8] = 1e-6;

        // -------- odom --------
        odom_msg_.header.frame_id = odom_frame_id_;
        odom_msg_.child_frame_id = robot_frame_id_;
        odom_msg_.pose.pose.position.z = 0.0;
        // -------- serial open --------
        // Retry opening the hardware device while USB/udev discovery or reconnection completes.
        // Keep retry logic in the driver to avoid brittle launch-time sleep sequencing.
        try_open_serial_port();
        if (!serial_ || !serial_->is_open())
        {
            serial_retry_timer_ = this->create_wall_timer(
                500ms, std::bind(&Duojin02BaseDriverNode::try_open_serial_port, this));
        }
        // -------- timer loop --------
        auto period = std::chrono::duration<double>(1.0 / loop_hz_);
        timer_ = this->create_wall_timer(period, std::bind(&Duojin02BaseDriverNode::on_timer, this));

        last_frame_steady_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "duojin02_base_driver node started");

        // -------- quaternion --------
        attitude_filter_.set_g_ref(9.80665f);
        attitude_filter_.set_gate_width_g(0.30f);
        attitude_filter_.set_accel_lpf_alpha(0.30f);
    }
    /**
     * @brief 析构函数
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    ~Duojin02BaseDriverNode() override
    {
        shutting_down_.store(true, std::memory_order_release);
        if (serial_retry_timer_)
        {
            serial_retry_timer_->cancel();
        }

        try
        {
            if (serial_ && serial_->is_open())
            {
                send_stop_command();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "failed to send stop command: %s", e.what());
        }
        if (serial_ && serial_->is_open())
            serial_->close();

        RCLCPP_INFO(this->get_logger(), "duojin02_base_driver node shutting down");
    }

private:
    /**
     * @brief 异或校验
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    static inline uint8_t xor_checksum(const uint8_t *data, std::size_t len) noexcept
    {
        uint8_t sum = 0;
        for (std::size_t i = 0; i < len; ++i)
            sum ^= data[i];
        return sum;
    }
    /**
     * @brief 用米为单位的浮点数构造毫米单位的整数
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    static inline int16_t saturate_i16_from_scaled(double value, double scale) noexcept
    {
        const double scaled = value * scale;
        if (!std::isfinite(scaled))
        {
            return 0;
        }
        return static_cast<int16_t>(std::lround(std::clamp(scaled,
            static_cast<double>(std::numeric_limits<int16_t>::min()),
            static_cast<double>(std::numeric_limits<int16_t>::max()))));
    }
    /**
     * @brief 编码hl
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    static inline void encode_i16_be(int16_t v, uint8_t &high, uint8_t &low) noexcept
    {
        const uint16_t u = static_cast<uint16_t>(v);
        high = static_cast<uint8_t>((u >> 8) & 0xFF);
        low = static_cast<uint8_t>(u & 0xFF);
    }
    /**
     * @brief 回调控制下位机；对应源先的Cmd_Vel_Callback，但是这里对部分东西进行了一层抽象，作为工具方法定义在上
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (shutting_down_.load(std::memory_order_acquire) || serial_reconnecting_ ||
            !serial_ || !serial_->is_open())
        {
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

        try
        {
            send_command_frame();
            last_command_time_ = std::chrono::steady_clock::now();
            command_active_ = vx_mmps != 0 || vy_mmps != 0 || wz_mradps != 0;
            if (!valid)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Invalid cmd_vel, sent zero velocity to controller");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "serial error in cmd_vel_callback: %s", e.what());
            // 发送失败说明串口异常，立即触发重连
            handle_serial_error(std::string("cmd_vel send failed: ") + e.what());
        }
    }
    void send_command_frame()
    {
        if (serial_->send(tx_buf_) != tx_buf_.size())
            throw std::runtime_error("incomplete velocity frame write");
    }

    /**
     * @brief 构造并发送零速度帧到下位机。调用前需确保 serial_ 非空且 is_open()。
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-29
     */
    void send_stop_command()
    {
        auto &tx = tx_buf_;
        std::fill(tx.begin() + 3, tx.begin() + 9, 0);
        tx[9] = xor_checksum(tx.data(), 9);
        send_command_frame();
    }

    /**
     * @brief 检测到串口错误时，安全地关闭端口并启动重连定时器。
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-29
     */
    void handle_serial_error(const std::string &reason)
    {
        command_active_ = false;
        if (shutting_down_.load(std::memory_order_acquire))
            return;
        if (serial_reconnecting_)
            return;

        serial_reconnecting_ = true;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            frame_ready_ = false;
        }
        sample_time_ready_ = false;

        RCLCPP_ERROR(this->get_logger(),
            "Serial error: %s — closing port and scheduling reconnect.",
            reason.c_str());

        if (serial_)
        {
            if (serial_->is_open())
            {
                try { send_stop_command(); } catch (...) {}
                try { serial_->close(); } catch (...) {}
            }
        }

        rx_count_ = 0;

        if (!serial_retry_timer_)
        {
            serial_retry_timer_ = this->create_wall_timer(
                100ms, std::bind(&Duojin02BaseDriverNode::try_open_serial_port, this));
        }
    }

    /**
     * @brief 对应原Controller
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    void on_timer()
    {
        // ── 重连中：跳过所有处理，等待 try_open_serial_port 完成 ──
        if (serial_reconnecting_ || !serial_)
            return;

        // ── serial health check ──
        // 0) 异步接收检测到 EOF（来自 io_context 线程的即时通知）
        if (serial_eof_detected_.exchange(false, std::memory_order_acq_rel))
        {
            handle_serial_error("EOF detected by async receiver — device disconnected");
            return;
        }
        // 1) 端口被库关闭（EOF / 设备拔出）
        if (!serial_->is_open())
        {
            handle_serial_error("port closed unexpectedly (possible cable disconnect / EOF)");
            return;
        }
        // A stopped guard / monitor must not leave the last velocity latched.
        // Use monotonic time, independent of ROS time and feedback arrival.
        if (command_active_ && std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_command_time_).count() > command_timeout_s_)
        {
            try {
                send_stop_command();
                command_active_ = false;
                RCLCPP_WARN(get_logger(), "Velocity stream expired; sent zero velocity to controller");
            } catch (const std::exception &e) {
                handle_serial_error(std::string("expired command stop failed: ") + e.what());
                return;
            }
        }
        duojin02::VelPosData vel;
        sensor_msgs::msg::Imu imu;
        float voltage = 0.0f;
        rclcpp::Time received_stamp;
        std::chrono::steady_clock::time_point last_received;
        const bool has_frame = get_sensor_data(vel, imu, voltage, received_stamp, last_received);
        const double since_last = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_received).count();
        if (since_last > 0.5) {
            handle_serial_error(
                "no data received for " + std::to_string(since_last) + " seconds (silent failure)");
            return;
        }
        if (!has_frame) {
            return;
        }
        double dt = 0.0;
        if (sample_time_ready_ && received_stamp > last_sample_time_) {
            dt = (received_stamp - last_sample_time_).seconds();
            if (dt > duojin02::MAX_DELTA_T) dt = 0.0;
        }
        last_sample_time_ = received_stamp;
        sample_time_ready_ = true;
        imu.angular_velocity.z -= imu_yaw_rate_bias_;
        vel.x = static_cast<float>(vel.x * odom_x_scale_);
        vel.y = static_cast<float>(vel.y * odom_y_scale_);

        if (vel.z >= 0.0f)
        {
            vel.z = static_cast<float>(vel.z * odom_z_scale_positive_);
        }
        else
        {
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

            attitude_filter_.update(
                static_cast<float>(imu.angular_velocity.x),
                static_cast<float>(imu.angular_velocity.y),
                static_cast<float>(imu.angular_velocity.z),
                static_cast<float>(imu.linear_acceleration.x),
                static_cast<float>(imu.linear_acceleration.y),
                static_cast<float>(imu.linear_acceleration.z),
                static_cast<float>(dt));

        }

        const auto q = attitude_filter_.quaternion();
        imu.orientation.w = q.w;
        imu.orientation.x = q.x;
        imu.orientation.y = q.y;
        imu.orientation.z = q.z;

        publish_odom(received_stamp, vel);
        publish_imu(received_stamp, imu);
        publish_voltage(voltage);
    }

    /**
     * @brief 通过串口获取并解包下位机发来的数据；改自原 turn_on_robot::Get_Sensor_Data_New()
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    bool get_sensor_data(duojin02::VelPosData &vel, sensor_msgs::msg::Imu &imu,
                         float &voltage, rclcpp::Time &stamp,
                         std::chrono::steady_clock::time_point &last_received)
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_received = last_frame_steady_time_;
        if (!frame_ready_) {
            return false;
        }
        frame_ready_ = false;
        vel = robot_vel_;
        imu = imu_msg_;
        voltage = power_voltage_;
        stamp = latest_frame_stamp_;
        return true;
    }

    /**
     * @brief 异步读
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-27
     */
    void start_serial_receive()
    {
        if (!serial_ || !serial_->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot start serial receive: port not ready");
            return;
        }
        try
        {
            serial_->async_receive(
                [this](std::vector<uint8_t> &data, const size_t bytes_transferred)
                {
                    this->on_serial_data(data, bytes_transferred);
                });
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "async_receive failed: %s", e.what());
        }
    }

    /**
     * @brief
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-27
     */
    void on_serial_data(std::vector<uint8_t> &data, size_t n)
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }
        if (n == 0)
        {
            serial_eof_detected_.store(true, std::memory_order_release);
            return;
        }
        if (n > data.size())
            n = data.size();

        for (size_t i = 0; i < n; ++i)
        {
            push_byte_and_parse(data[i]);
        }
    }
    /**
     * @brief 获取一帧有效数据
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-27
     */
    void push_byte_and_parse(uint8_t byte)
    {
        if (rx_count_ == 0)
        {
            if (byte != duojin02::FRAME_HEADER)
            {
                return;
            }
            rx_buf_[0] = byte;
            rx_count_ = 1;
            return;
        }

        rx_buf_[rx_count_] = byte;
        rx_count_++;

        if (rx_count_ < static_cast<int>(duojin02::RECEIVE_DATA_SIZE))
        {
            return;
        }

        rx_count_ = 0;

        if (rx_buf_[duojin02::RECEIVE_DATA_SIZE - 1] != duojin02::FRAME_TAIL)
        {
            return;
        }

        const uint8_t check = xor_checksum(rx_buf_.data(), 22);
        if (check != rx_buf_[22])
        {
            return;
        }

        decode_frame_and_update_state(rx_buf_);
    }

    /**
     * @brief 把帧数据解析到对应的字段，更新时加锁
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-27
     */
    void decode_frame_and_update_state(const std::array<uint8_t, duojin02::RECEIVE_DATA_SIZE> &rx)
    {
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

        const int16_t v_mv = static_cast<int16_t>(
            (static_cast<uint16_t>(rx[20]) << 8) | static_cast<uint16_t>(rx[21]));
        const float v = static_cast<float>(v_mv) * 0.001f;

        {
            std::lock_guard<std::mutex> lk(state_mtx_);

            robot_vel_.x = vx;
            robot_vel_.y = vy;
            robot_vel_.z = wz;

            imu_msg_.linear_acceleration.x = static_cast<double>(imu_raw.accele_x_data) / duojin02::ACCEl_RATIO;
            imu_msg_.linear_acceleration.y = static_cast<double>(imu_raw.accele_y_data) / duojin02::ACCEl_RATIO;
            imu_msg_.linear_acceleration.z = static_cast<double>(imu_raw.accele_z_data) / duojin02::ACCEl_RATIO;

            imu_msg_.angular_velocity.x = static_cast<double>(imu_raw.gyros_x_data) * duojin02::GYROSCOPE_RATIO;
            imu_msg_.angular_velocity.y = static_cast<double>(imu_raw.gyros_y_data) * duojin02::GYROSCOPE_RATIO;
            imu_msg_.angular_velocity.z = static_cast<double>(imu_raw.gyros_z_data) * duojin02::GYROSCOPE_RATIO;

            power_voltage_ = v;
            latest_frame_stamp_ = this->now();
            last_frame_steady_time_ = std::chrono::steady_clock::now();
            frame_ready_ = true;
        }
    }

    /**
     * @brief 解码一个有符号16-bit整数（从大端）；对应原本的IMU_Trans
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    static inline int16_t decode_i16_be(uint8_t high, uint8_t low) noexcept
    {
        const uint16_t u = (static_cast<uint16_t>(high) << 8) |
                           static_cast<uint16_t>(low);
        return static_cast<int16_t>(u);
    }

    /**
     * @brief 把大端 mm/s 的速度解码成 m/s；对应原本的Odom_Trans，简化了无意义的乘法和取余运算
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    static inline float decode_vel_mps_be(uint8_t high, uint8_t low) noexcept
    {
        constexpr float k_inv_1000 = 0.001f;
        return static_cast<float>(decode_i16_be(high, low)) * k_inv_1000;
    }

    /**
     * @brief 把IMU数据发布到 Topic；对应原Publish_ImuSensor，去掉了其中已经在其他部分赋值过的变量
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    void publish_imu(const rclcpp::Time &stamp, sensor_msgs::msg::Imu &imu)
    {
        imu.header.stamp = stamp;
        imu_pub_->publish(imu);
    }

    /**
     * @brief 发布电池当前电压到 Topic
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    void publish_voltage(float voltage)
    {
        static int count = 0;
        if (++count <= 10)
            return;
        count = 0;
        std_msgs::msg::Float32 msg;
        msg.data = voltage;
        voltage_pub_->publish(msg);
    }

    /**
     * @brief 发布当前的位置、姿态、三轴速度、绕三轴的角速度等数据（来源里程计）；对应原Publish_Odom，优化了奇怪的赋值、浮点数与0的比较等等内容
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-01-25
     */
    void publish_odom(const rclcpp::Time &stamp, duojin02::VelPosData vel)
    {
        odom_msg_.header.stamp = stamp;

        odom_msg_.pose.pose.position.x = robot_pos_.x;
        odom_msg_.pose.pose.position.y = robot_pos_.y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot_pos_.z);
        odom_msg_.pose.pose.orientation = tf2::toMsg(q);

        // 速度
        odom_msg_.twist.twist.linear.x = vel.x;
        odom_msg_.twist.twist.linear.y = vel.y;
        odom_msg_.twist.twist.angular.z = vel.z;

        const bool is_stopped =
            (std::fabs(vel.x) < 1e-6) &&
            (std::fabs(vel.y) < 1e-6) &&
            (std::fabs(vel.z) < 1e-6);

        if (is_stopped)
        {
            std::copy(duojin02::odom_pose_covariance2.begin(),
                      duojin02::odom_pose_covariance2.end(),
                      odom_msg_.pose.covariance.begin());
            std::copy(odom_twist_stopped_.begin(),
                      odom_twist_stopped_.end(),
                      odom_msg_.twist.covariance.begin());
        }
        else
        {
            std::copy(duojin02::odom_pose_covariance.begin(),
                      duojin02::odom_pose_covariance.end(),
                      odom_msg_.pose.covariance.begin());
            std::copy(odom_twist_moving_.begin(),
                      odom_twist_moving_.end(),
                      odom_msg_.twist.covariance.begin());
        }

        odom_pub_->publish(odom_msg_);
    }

private:
    /**
     * @brief 打开底盘串口
     *
     * @author litianshun (litianshun.cn@gmail.com)
     * @date 2026-02-08
     */
    void try_open_serial_port()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }
        if (serial_ && serial_->is_open())
        {
            return;
        }

        try
        {
            serial_ = std::make_unique<drivers::serial_driver::SerialPort>(io_ctx_, usart_port_name_, serial_cfg_);
            serial_->open();
            // A connection is usable only after a complete zero frame is written.
            // Failed initialization leaves the existing retry timer active.
            send_stop_command();
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cannot initialize serial port %s: %s",
                usart_port_name_.c_str(),
                e.what());
            if (serial_ && serial_->is_open())
                try { serial_->close(); } catch (...) {}
            serial_.reset();
            return;
        }

        if (serial_ && serial_->is_open())
        {
            command_active_ = false;
            serial_eof_detected_.store(false, std::memory_order_release);
            RCLCPP_WARN(this->get_logger(), "connected — sent stop command to lower controller");
            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                frame_ready_ = false;
                last_frame_steady_time_ = std::chrono::steady_clock::now();
            }
            sample_time_ready_ = false;
            rx_count_ = 0;
            start_serial_receive();
            serial_reconnecting_ = false;
            if (serial_retry_timer_)
            {
                serial_retry_timer_->cancel();
                serial_retry_timer_.reset();
            }
            RCLCPP_INFO(this->get_logger(), "serial port opened: %s", usart_port_name_.c_str());
        }
    }

    // -------- parameters --------
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

    // --- receive state machine ---
    int rx_count_{0}; // 0..23
    std::array<uint8_t, duojin02::RECEIVE_DATA_SIZE> rx_buf_{};
    std::atomic<bool> shutting_down_{false};
    bool serial_reconnecting_{false};

    // --- serial EOF detection (set by io_context thread, read by ROS timer thread) ---
    std::atomic<bool> serial_eof_detected_{false};

    // All feedback fields, including frame_ready_, are protected by state_mtx_.
    bool frame_ready_{false};
    std::mutex state_mtx_;

    // -------- ROS entities --------
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr serial_retry_timer_;

    // -------- timing --------
    rclcpp::Time last_sample_time_, latest_frame_stamp_;
    bool sample_time_ready_{false};
    std::chrono::steady_clock::time_point last_frame_steady_time_;
    std::chrono::steady_clock::time_point last_command_time_;
    double command_timeout_s_{0.25};
    bool command_active_{false};

    // -------- serial --------
    drivers::common::IoContext io_ctx_;
    drivers::serial_driver::SerialPortConfig serial_cfg_{
        static_cast<uint32_t>(serial_baud_rate_),
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE};
    std::unique_ptr<drivers::serial_driver::SerialPort> serial_;

    // Driver runtime state
    duojin02::VelPosData robot_pos_;
    duojin02::VelPosData robot_vel_;
    // ROS callbacks run in the default mutually exclusive callback group.
    std::vector<uint8_t> tx_buf_{duojin02::FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 duojin02::FRAME_TAIL};

    float power_voltage_{0.0f};

    sensor_msgs::msg::Imu imu_msg_;
    nav_msgs::msg::Odometry odom_msg_;

    duojin02::QuaternionSolution attitude_filter_{2.0f, 0.0f};
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Duojin02BaseDriverNode>());
  rclcpp::shutdown();
  return 0;
}
