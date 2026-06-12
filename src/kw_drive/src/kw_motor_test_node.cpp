#include "kw_drive/protocol/kw_motor.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{

uint16_t checked_u16(const rclcpp::Node & node, const std::string & parameter_name)
{
  const auto value = node.get_parameter(parameter_name).as_int();
  if (value < 0 || value > 0xffff) {
    throw std::out_of_range(parameter_name + " must be in [0, 65535]");
  }
  return static_cast<uint16_t>(value);
}

uint32_t checked_u32(const rclcpp::Node & node, const std::string & parameter_name)
{
  const auto value = node.get_parameter(parameter_name).as_int();
  if (value < 0) {
    throw std::out_of_range(parameter_name + " must be non-negative");
  }
  return static_cast<uint32_t>(value);
}

}  // namespace

class KwMotorTestNode : public rclcpp::Node
{
public:
  KwMotorTestNode()
  : Node("kw_motor_test")
  {
    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int64_t>("serial_baud", 921600);
    declare_parameter<int64_t>("can_id", 0x01);
    declare_parameter<bool>("enable_on_start", true);
    declare_parameter<double>("period_ms", 1.0);
    declare_parameter<int64_t>("print_every", 100);
    declare_parameter<double>("pos_max", 12.5);
    declare_parameter<double>("vel_max", 30.0);
    declare_parameter<double>("iq_max", 10.0);
    declare_parameter<double>("kp_max", 500.0);
    declare_parameter<double>("kd_max", 5.0);
    declare_parameter<double>("kp", 0.0);
    declare_parameter<double>("kd", 0.0);
    declare_parameter<double>("q", 0.0);
    declare_parameter<double>("dq", 0.0);
    declare_parameter<double>("iq", 0.0);
    declare_parameter<std::string>("state_topic", "/kw_motor/state");

    serial_port_ = get_parameter("serial_port").as_string();
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port is empty");
    }

    can_id_ = checked_u16(*this, "can_id");
    const uint32_t serial_baud = checked_u32(*this, "serial_baud");
    const bool enable_on_start = get_parameter("enable_on_start").as_bool();
    print_every_ = static_cast<int>(get_parameter("print_every").as_int());
    if (print_every_ < 1) {
      throw std::out_of_range("print_every must be >= 1");
    }

    kp_ = static_cast<float>(get_parameter("kp").as_double());
    kd_ = static_cast<float>(get_parameter("kd").as_double());
    q_ = static_cast<float>(get_parameter("q").as_double());
    dq_ = static_cast<float>(get_parameter("dq").as_double());
    iq_ = static_cast<float>(get_parameter("iq").as_double());
    state_topic_ = get_parameter("state_topic").as_string();
    if (state_topic_.empty()) {
      throw std::invalid_argument("state_topic is empty");
    }

    const auto pos_max = get_parameter("pos_max").as_double();
    const auto vel_max = get_parameter("vel_max").as_double();
    const auto iq_max = get_parameter("iq_max").as_double();
    const auto kp_max = get_parameter("kp_max").as_double();
    const auto kd_max = get_parameter("kd_max").as_double();
    if (
      !std::isfinite(pos_max) || !std::isfinite(vel_max) || !std::isfinite(iq_max) ||
      !std::isfinite(kp_max) || !std::isfinite(kd_max) ||
      pos_max <= 0.0 || vel_max <= 0.0 || iq_max <= 0.0 || kp_max <= 0.0 || kd_max <= 0.0)
    {
      throw std::out_of_range("pos_max, vel_max, iq_max, kp_max, and kd_max must be finite and > 0");
    }
    if (!std::isfinite(kp_) || kp_ < 0.0F || kp_ > kp_max) {
      throw std::out_of_range("kp must be finite and in [0, kp_max]");
    }
    if (!std::isfinite(kd_) || kd_ < 0.0F || kd_ > kd_max) {
      throw std::out_of_range("kd must be finite and in [0, kd_max]");
    }

    kw::MotorConfig motor_config{};
    motor_config.can_id = can_id_;
    motor_config.limit = kw::LimitParam{
      static_cast<float>(pos_max),
      static_cast<float>(vel_max),
      static_cast<float>(iq_max),
      static_cast<float>(kp_max),
      static_cast<float>(kd_max),
    };
    motors_.push_back(motor_config);

    RCLCPP_INFO(
      get_logger(),
      "Opening KW serial %s @ %u, motor_id=0x%02x, command_id=0x%02x, feedback_id=0x%02x, enable_on_start=%s, max=[pos %.3f, vel %.3f, iq %.3f, kp %.3f, kd %.3f]",
      serial_port_.c_str(), serial_baud, can_id_, can_id_ | 0x70, can_id_ | 0x10,
      enable_on_start ? "true" : "false", pos_max, vel_max, iq_max, kp_max, kd_max);

    control_ = std::make_shared<kw::MotorControl>(
      serial_baud, serial_port_, &motors_, enable_on_start);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);

    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      throw std::runtime_error("configured motor was not registered");
    }

    const auto period_ms = get_parameter("period_ms").as_double();
    if (period_ms <= 0.0) {
      throw std::out_of_range("period_ms must be > 0");
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(period_ms));
    timer_ = create_wall_timer(period, [this]() { update_motor(); });
  }

  ~KwMotorTestNode() override
  {
    timer_.reset();
    control_.reset();
  }

private:
  void update_motor()
  {
    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      RCLCPP_ERROR(get_logger(), "No registered motor for can_id=0x%02x", can_id_);
      rclcpp::shutdown();
      return;
    }

    control_->control_mit(*motor, kp_, kd_, q_, dq_, iq_);
    publish_state(*motor);
    ++tick_;
    if (tick_ % print_every_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "can_id=0x%02x err=%s pos=%.5f vel=%.5f iq=%.5f dt=%.6f",
        can_id_,
        motor->Get_Error() ? "true" : "false",
        motor->Get_Position(),
        motor->Get_Velocity(),
        motor->Get_iq(),
        motor->getTimeInterval());
    }
  }

  void publish_state(const kw::Motor & motor)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.push_back("kw_motor_" + std::to_string(can_id_));
    msg.position.push_back(motor.Get_Position());
    msg.velocity.push_back(motor.Get_Velocity());
    msg.effort.push_back(motor.Get_iq());
    state_pub_->publish(msg);
  }

  std::string serial_port_;
  std::string state_topic_;
  uint16_t can_id_{0};
  int print_every_{100};
  size_t tick_{0};
  float kp_{0.0F};
  float kd_{0.0F};
  float q_{0.0F};
  float dq_{0.0F};
  float iq_{0.0F};
  std::vector<kw::MotorConfig> motors_;
  std::shared_ptr<kw::MotorControl> control_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KwMotorTestNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("kw_motor_test"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
