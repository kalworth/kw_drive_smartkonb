#include "kw_drive/protocol/kw_motor.h"

#include <algorithm>
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

double wrap_to_half_period(double value, double period)
{
  return std::remainder(value, period);
}

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

double checked_positive_double(
  const rclcpp::Node & node,
  const std::string & parameter_name)
{
  const auto value = node.get_parameter(parameter_name).as_double();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::out_of_range(parameter_name + " must be finite and > 0");
  }
  return value;
}

double checked_nonnegative_double(
  const rclcpp::Node & node,
  const std::string & parameter_name)
{
  const auto value = node.get_parameter(parameter_name).as_double();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::out_of_range(parameter_name + " must be finite and >= 0");
  }
  return value;
}

}  // namespace

class KwSmartKnobNode : public rclcpp::Node
{
public:
  KwSmartKnobNode()
  : Node("kw_smart_knob")
  {
    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int64_t>("serial_baud", 921600);
    declare_parameter<int64_t>("can_id", 0x01);
    declare_parameter<bool>("enable_on_start", false);
    declare_parameter<double>("period_ms", 5.0);
    declare_parameter<int64_t>("print_every", 100);
    declare_parameter<double>("pos_max", 12.5);
    declare_parameter<double>("vel_max", 45.0);
    declare_parameter<double>("iq_max", 2.0);
    declare_parameter<double>("iq_limit", 0.3);
    declare_parameter<int64_t>("detents_per_revolution", 0);
    declare_parameter<double>("detent_spacing", 0.25);
    declare_parameter<double>("detent_strength", 1.2);
    declare_parameter<double>("damping", 0.02);
    declare_parameter<double>("torque_sign", 1.0);
    declare_parameter<bool>("wrap_angle", true);
    declare_parameter<double>("wrap_period", 0.0);
    declare_parameter<double>("feedback_timeout_ms", 100.0);
    declare_parameter<bool>("zero_on_start", true);
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

    const auto pos_max = checked_positive_double(*this, "pos_max");
    const auto vel_max = checked_positive_double(*this, "vel_max");
    const auto iq_max = checked_positive_double(*this, "iq_max");
    iq_limit_ = checked_positive_double(*this, "iq_limit");
    if (iq_limit_ > iq_max) {
      RCLCPP_WARN(
        get_logger(),
        "iq_limit %.3f exceeds iq_max %.3f; clamping iq_limit to iq_max",
        iq_limit_, iq_max);
      iq_limit_ = iq_max;
    }

    wrap_angle_ = get_parameter("wrap_angle").as_bool();
    wrap_period_ = checked_nonnegative_double(*this, "wrap_period");
    if (wrap_angle_ && wrap_period_ == 0.0) {
      wrap_period_ = 2.0 * pos_max;
    }
    if (wrap_angle_ && wrap_period_ <= 0.0) {
      throw std::out_of_range("wrap_period must be > 0 when wrap_angle is true");
    }

    const auto detents_per_revolution = get_parameter("detents_per_revolution").as_int();
    if (detents_per_revolution < 0 || detents_per_revolution == 1) {
      throw std::out_of_range("detents_per_revolution must be 0 or >= 2");
    }
    detents_per_revolution_ = static_cast<int>(detents_per_revolution);

    detent_spacing_ = checked_positive_double(*this, "detent_spacing");
    if (detents_per_revolution_ > 0) {
      if (wrap_period_ == 0.0) {
        wrap_period_ = 2.0 * pos_max;
      }
      if (wrap_period_ <= 0.0) {
        throw std::out_of_range("wrap_period must be > 0 when detents_per_revolution is used");
      }
      detent_spacing_ = wrap_period_ / static_cast<double>(detents_per_revolution_);
    }
    if (detent_spacing_ <= 0.0) {
      throw std::out_of_range("detent_spacing must be > 0");
    }
    if (wrap_angle_ && detent_spacing_ >= wrap_period_) {
      throw std::out_of_range("detent_spacing must be smaller than wrap_period when wrap_angle is true");
    }

    detent_strength_ = checked_positive_double(*this, "detent_strength");
    damping_ = get_parameter("damping").as_double();
    if (!std::isfinite(damping_) || damping_ < 0.0) {
      throw std::out_of_range("damping must be finite and >= 0");
    }

    const auto torque_sign = get_parameter("torque_sign").as_double();
    if (!std::isfinite(torque_sign) || torque_sign == 0.0) {
      throw std::out_of_range("torque_sign must be non-zero");
    }
    torque_sign_ = torque_sign > 0.0 ? 1.0 : -1.0;

    feedback_timeout_s_ = checked_positive_double(*this, "feedback_timeout_ms") / 1000.0;
    zero_on_start_ = get_parameter("zero_on_start").as_bool();

    state_topic_ = get_parameter("state_topic").as_string();
    if (state_topic_.empty()) {
      throw std::invalid_argument("state_topic is empty");
    }

    kw::MotorConfig motor_config{};
    motor_config.can_id = can_id_;
    motor_config.limit = kw::LimitParam{
      static_cast<float>(pos_max),
      static_cast<float>(vel_max),
      static_cast<float>(iq_max),
    };
    motors_.push_back(motor_config);

    RCLCPP_INFO(
      get_logger(),
      "Opening KW smart knob serial %s @ %u, motor_id=0x%02x, enable_on_start=%s, period=%.3f ms, spacing=%.5f rad, detents_per_revolution=%d, wrap_angle=%s, wrap_period=%.5f, strength=%.3f, damping=%.3f, iq_limit=%.3f, torque_sign=%.0f",
      serial_port_.c_str(), serial_baud, can_id_, enable_on_start ? "true" : "false",
      checked_positive_double(*this, "period_ms"), detent_spacing_, detents_per_revolution_,
      wrap_angle_ ? "true" : "false", wrap_period_, detent_strength_, damping_, iq_limit_,
      torque_sign_);

    control_ = std::make_shared<kw::MotorControl>(
      serial_baud, serial_port_, &motors_, enable_on_start);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);

    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      throw std::runtime_error("configured motor was not registered");
    }

    const auto period_ms = checked_positive_double(*this, "period_ms");
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(period_ms));
    timer_ = create_wall_timer(period, [this]() { update_knob(); });
  }

  ~KwSmartKnobNode() override
  {
    timer_.reset();
    control_.reset();
  }

private:
  void update_knob()
  {
    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      RCLCPP_ERROR(get_logger(), "No registered motor for can_id=0x%02x", can_id_);
      rclcpp::shutdown();
      return;
    }

    if (!motor->HasFeedback()) {
      send_iq(*motor, 0.0);
      ++tick_;
      if (tick_ % print_every_ == 0) {
        RCLCPP_WARN(get_logger(), "Waiting for first feedback from can_id=0x%02x", can_id_);
      }
      return;
    }

    if (motor->SecondsSinceFeedback() > feedback_timeout_s_) {
      send_iq(*motor, 0.0);
      ++tick_;
      if (tick_ % print_every_ == 0) {
        RCLCPP_WARN(
          get_logger(),
          "Feedback timeout from can_id=0x%02x: %.3f s; sending zero iq",
          can_id_, motor->SecondsSinceFeedback());
      }
      publish_state(*motor);
      return;
    }

    if (!center_initialized_) {
      center_q_ = zero_on_start_ ? motor->Get_Position() : 0.0;
      center_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Smart knob center set to q=%.5f rad", center_q_);
    }

    const double q = motor->Get_Position();
    const double dq = motor->Get_Velocity();
    const double relative_q = wrap_delta(q - center_q_);
    const auto detent_index = static_cast<long>(std::llround(relative_q / detent_spacing_));
    const double target_relative_q = static_cast<double>(detent_index) * detent_spacing_;
    const double target_q = wrap_absolute(center_q_ + target_relative_q);
    const double error_q = wrap_delta(target_relative_q - relative_q);

    double iq = detent_strength_ * error_q - damping_ * dq;
    iq = std::clamp(iq, -iq_limit_, iq_limit_);
    iq *= torque_sign_;

    if (motor->Get_Error()) {
      iq = 0.0;
    }

    send_iq(*motor, iq);
    publish_state(*motor);

    ++tick_;
    if (tick_ % print_every_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "detent=%ld q=%.5f relative=%.5f target=%.5f err_q=%.5f dq=%.5f iq_cmd=%.5f feedback_iq=%.5f err=%s",
        detent_index, q, relative_q, target_q, error_q, dq, iq, motor->Get_iq(),
        motor->Get_Error() ? "true" : "false");
    }
  }

  double wrap_delta(double value) const
  {
    return wrap_angle_ ? wrap_to_half_period(value, wrap_period_) : value;
  }

  double wrap_absolute(double value) const
  {
    return wrap_angle_ ? wrap_to_half_period(value, wrap_period_) : value;
  }

  void send_iq(kw::Motor & motor, double iq)
  {
    const double limited_iq = std::clamp(iq, -iq_limit_, iq_limit_);
    control_->control_mit(
      motor,
      0.0F,
      0.0F,
      0.0F,
      0.0F,
      static_cast<float>(limited_iq));
  }

  void publish_state(const kw::Motor & motor)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.push_back("kw_smart_knob_" + std::to_string(can_id_));
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
  double iq_limit_{0.3};
  int detents_per_revolution_{24};
  double detent_spacing_{0.25};
  double detent_strength_{1.2};
  double damping_{0.02};
  double torque_sign_{1.0};
  bool wrap_angle_{true};
  double wrap_period_{0.0};
  double feedback_timeout_s_{0.1};
  double center_q_{0.0};
  bool zero_on_start_{true};
  bool center_initialized_{false};
  std::vector<kw::MotorConfig> motors_;
  std::shared_ptr<kw::MotorControl> control_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KwSmartKnobNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("kw_smart_knob"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
