#include "kw_drive/protocol/kw_motor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
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

double read_aliased_double(
  const rclcpp::Node & node,
  const std::string & canonical_name,
  const std::string & legacy_name,
  double default_value)
{
  const double canonical_value = node.get_parameter(canonical_name).as_double();
  const double legacy_value = node.get_parameter(legacy_name).as_double();
  if (canonical_value == default_value && legacy_value != default_value) {
    return legacy_value;
  }
  return canonical_value;
}

enum class KnobMode
{
  Free,
  Detent,
  Endstop,
  Spring,
};

KnobMode parse_mode(const std::string & mode)
{
  if (mode == "free" || mode == "off") {
    return KnobMode::Free;
  }
  if (mode == "detent") {
    return KnobMode::Detent;
  }
  if (mode == "endstop" || mode == "limit") {
    return KnobMode::Endstop;
  }
  if (mode == "spring") {
    return KnobMode::Spring;
  }
  throw std::invalid_argument("mode must be one of: free, detent, endstop, spring");
}

const char * mode_name(KnobMode mode)
{
  switch (mode) {
    case KnobMode::Free:
      return "free";
    case KnobMode::Detent:
      return "detent";
    case KnobMode::Endstop:
      return "endstop";
    case KnobMode::Spring:
      return "spring";
  }
  return "unknown";
}

struct KnobCommand
{
  bool active{false};
  const char * label{"free"};
  long index{0};
  double relative_q{0.0};
  double target_q{0.0};
  double error_q{0.0};
  double kp{0.0};
  double kd{0.0};
  double q_ref{0.0};
  double dq_ref{0.0};
  double iq_ff{0.0};
};

}  // namespace

class KwSmartKnobNode : public rclcpp::Node
{
public:
  KwSmartKnobNode()
  : Node("kw_smart_knob")
  {
    declare_parameter<std::string>("basic.mode", "detent");
    declare_parameter<double>("basic.period_ms", 5.0);
    declare_parameter<int64_t>("basic.print_every", 100);
    declare_parameter<double>("basic.feedback_timeout_ms", 100.0);
    declare_parameter<bool>("basic.zero_on_start", true);
    declare_parameter<std::string>("basic.state_topic", "/kw_motor/state");

    declare_parameter<std::string>("serial.port", "/dev/ttyACM0");
    declare_parameter<int64_t>("serial.baud", 921600);

    declare_parameter<int64_t>("motor.can_id", 0x01);
    declare_parameter<bool>("motor.enable_on_start", false);
    declare_parameter<double>("motor.pos_max", 12.5);
    declare_parameter<double>("motor.vel_max", 45.0);
    declare_parameter<double>("motor.iq_max", 2.0);

    declare_parameter<double>("mit.kp_max", 500.0);
    declare_parameter<double>("mit.kd_max", 5.0);

    declare_parameter<bool>("angle.wrap", true);
    declare_parameter<double>("angle.period", 0.0);

    declare_parameter<int64_t>("detent.per_revolution", 0);
    declare_parameter<double>("detent.spacing", 0.25);
    declare_parameter<double>("detent.kp", 1.2);
    declare_parameter<double>("detent.kd", 0.02);
    declare_parameter<double>("detent.iq_ff", 0.0);

    declare_parameter<double>("endstop.min", -1.57);
    declare_parameter<double>("endstop.max", 1.57);
    declare_parameter<double>("endstop.kp", 1.2);
    declare_parameter<double>("endstop.kd", 0.02);
    declare_parameter<double>("limit.min", -1.57);
    declare_parameter<double>("limit.max", 1.57);
    declare_parameter<double>("limit.kp", 1.2);
    declare_parameter<double>("limit.kd", 0.02);

    declare_parameter<double>("spring.kp", 1.2);
    declare_parameter<double>("spring.kd", 0.02);
    declare_parameter<double>("spring.iq_ff", 0.0);

    serial_port_ = get_parameter("serial.port").as_string();
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port is empty");
    }

    can_id_ = checked_u16(*this, "motor.can_id");
    const uint32_t serial_baud = checked_u32(*this, "serial.baud");
    const bool enable_on_start = get_parameter("motor.enable_on_start").as_bool();
    mode_ = parse_mode(get_parameter("basic.mode").as_string());

    print_every_ = static_cast<int>(get_parameter("basic.print_every").as_int());
    if (print_every_ < 1) {
      throw std::out_of_range("basic.print_every must be >= 1");
    }

    pos_max_ = checked_positive_double(*this, "motor.pos_max");
    vel_max_ = checked_positive_double(*this, "motor.vel_max");
    iq_max_ = checked_positive_double(*this, "motor.iq_max");
    kp_max_ = checked_positive_double(*this, "mit.kp_max");
    kd_max_ = checked_positive_double(*this, "mit.kd_max");

    wrap_angle_ = get_parameter("angle.wrap").as_bool();
    wrap_period_ = checked_nonnegative_double(*this, "angle.period");
    if (wrap_angle_ && wrap_period_ == 0.0) {
      wrap_period_ = 2.0 * pos_max_;
    }
    if (wrap_angle_ && wrap_period_ <= 0.0) {
      throw std::out_of_range("angle.period must be > 0 when angle.wrap is true");
    }

    const auto detents_per_revolution = get_parameter("detent.per_revolution").as_int();
    if (detents_per_revolution < 0 || detents_per_revolution == 1) {
      throw std::out_of_range("detent.per_revolution must be 0 or >= 2");
    }
    detents_per_revolution_ = static_cast<int>(detents_per_revolution);

    detent_spacing_ = checked_positive_double(*this, "detent.spacing");
    if (detents_per_revolution_ > 0) {
      if (wrap_period_ == 0.0) {
        wrap_period_ = 2.0 * pos_max_;
      }
      if (wrap_period_ <= 0.0) {
        throw std::out_of_range("angle.period must be > 0 when detent.per_revolution is used");
      }
      detent_spacing_ = wrap_period_ / static_cast<double>(detents_per_revolution_);
    }
    if (detent_spacing_ <= 0.0) {
      throw std::out_of_range("detent.spacing must be > 0");
    }
    if (wrap_angle_ && detent_spacing_ >= wrap_period_) {
      throw std::out_of_range(
        "detent.spacing must be smaller than angle.period when angle.wrap is true");
    }

    kp_ = checked_nonnegative_double(*this, "detent.kp");
    kd_ = checked_nonnegative_double(*this, "detent.kd");
    iq_ff_ = get_parameter("detent.iq_ff").as_double();
    if (!std::isfinite(iq_ff_)) {
      throw std::out_of_range("detent.iq_ff must be finite");
    }
    if (kp_ > kp_max_) {
      throw std::out_of_range("detent.kp must be <= mit.kp_max");
    }
    if (kd_ > kd_max_) {
      throw std::out_of_range("detent.kd must be <= mit.kd_max");
    }
    if (std::abs(iq_ff_) > iq_max_) {
      throw std::out_of_range("abs(detent.iq_ff) must be <= motor.iq_max");
    }

    endstop_min_ = read_aliased_double(*this, "endstop.min", "limit.min", -1.57);
    endstop_max_ = read_aliased_double(*this, "endstop.max", "limit.max", 1.57);
    endstop_kp_ = read_aliased_double(*this, "endstop.kp", "limit.kp", 1.2);
    endstop_kd_ = read_aliased_double(*this, "endstop.kd", "limit.kd", 0.02);
    if (!std::isfinite(endstop_min_) || !std::isfinite(endstop_max_)) {
      throw std::out_of_range("endstop.min and endstop.max must be finite");
    }
    if (endstop_min_ >= endstop_max_) {
      throw std::out_of_range("endstop.min must be smaller than endstop.max");
    }
    if (!std::isfinite(endstop_kp_) || endstop_kp_ < 0.0) {
      throw std::out_of_range("endstop.kp must be finite and >= 0");
    }
    if (!std::isfinite(endstop_kd_) || endstop_kd_ < 0.0) {
      throw std::out_of_range("endstop.kd must be finite and >= 0");
    }
    if (endstop_kp_ > kp_max_) {
      throw std::out_of_range("endstop.kp must be <= mit.kp_max");
    }
    if (endstop_kd_ > kd_max_) {
      throw std::out_of_range("endstop.kd must be <= mit.kd_max");
    }

    spring_kp_ = checked_nonnegative_double(*this, "spring.kp");
    spring_kd_ = checked_nonnegative_double(*this, "spring.kd");
    spring_iq_ff_ = get_parameter("spring.iq_ff").as_double();
    if (!std::isfinite(spring_iq_ff_)) {
      throw std::out_of_range("spring.iq_ff must be finite");
    }
    if (spring_kp_ > kp_max_) {
      throw std::out_of_range("spring.kp must be <= mit.kp_max");
    }
    if (spring_kd_ > kd_max_) {
      throw std::out_of_range("spring.kd must be <= mit.kd_max");
    }
    if (std::abs(spring_iq_ff_) > iq_max_) {
      throw std::out_of_range("abs(spring.iq_ff) must be <= motor.iq_max");
    }

    feedback_timeout_s_ = checked_positive_double(*this, "basic.feedback_timeout_ms") / 1000.0;
    zero_on_start_ = get_parameter("basic.zero_on_start").as_bool();

    state_topic_ = get_parameter("basic.state_topic").as_string();
    if (state_topic_.empty()) {
      throw std::invalid_argument("state_topic is empty");
    }

    kw::MotorConfig motor_config{};
    motor_config.can_id = can_id_;
    motor_config.limit = kw::LimitParam{
      static_cast<float>(pos_max_),
      static_cast<float>(vel_max_),
      static_cast<float>(iq_max_),
      static_cast<float>(kp_max_),
      static_cast<float>(kd_max_),
    };
    motors_.push_back(motor_config);

    RCLCPP_INFO(
      get_logger(),
      "Opening KW smart knob serial %s @ %u, motor_id=0x%02x, enable_on_start=%s, mode=%s, period=%.3f ms, spacing=%.5f rad, detents_per_revolution=%d, wrap_angle=%s, wrap_period=%.5f, kp=%.3f, kd=%.3f, iq_ff=%.3f, endstop=[%.3f, %.3f], endstop_kp=%.3f, endstop_kd=%.3f, max=[pos %.3f, vel %.3f, iq %.3f, kp %.3f, kd %.3f]",
      serial_port_.c_str(), serial_baud, can_id_, enable_on_start ? "true" : "false",
      mode_name(mode_), checked_positive_double(*this, "basic.period_ms"), detent_spacing_,
      detents_per_revolution_, wrap_angle_ ? "true" : "false", wrap_period_, kp_, kd_,
      iq_ff_, endstop_min_, endstop_max_, endstop_kp_, endstop_kd_, pos_max_, vel_max_, iq_max_,
      kp_max_, kd_max_);

    control_ = std::make_shared<kw::MotorControl>(
      serial_baud, serial_port_, &motors_, enable_on_start);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);

    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      throw std::runtime_error("configured motor was not registered");
    }

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters_changed(parameters);
      });

    const auto period_ms = checked_positive_double(*this, "basic.period_ms");
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
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    try {
      KnobMode next_mode = mode_;
      int next_print_every = print_every_;
      double next_feedback_timeout_s = feedback_timeout_s_;

      int next_detents_per_revolution = detents_per_revolution_;
      double next_detent_spacing = detent_spacing_;
      double next_detent_kp = kp_;
      double next_detent_kd = kd_;
      double next_detent_iq_ff = iq_ff_;

      double next_endstop_min = endstop_min_;
      double next_endstop_max = endstop_max_;
      double next_endstop_kp = endstop_kp_;
      double next_endstop_kd = endstop_kd_;

      double next_spring_kp = spring_kp_;
      double next_spring_kd = spring_kd_;
      double next_spring_iq_ff = spring_iq_ff_;

      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "basic.mode") {
          next_mode = parse_mode(parameter.as_string());
        } else if (name == "basic.print_every") {
          const auto value = parameter.as_int();
          if (value < 1) {
            throw std::out_of_range("basic.print_every must be >= 1");
          }
          next_print_every = static_cast<int>(value);
        } else if (name == "basic.feedback_timeout_ms") {
          const auto value = parameter.as_double();
          if (!std::isfinite(value) || value <= 0.0) {
            throw std::out_of_range("basic.feedback_timeout_ms must be finite and > 0");
          }
          next_feedback_timeout_s = value / 1000.0;
        } else if (name == "detent.per_revolution") {
          const auto value = parameter.as_int();
          if (value < 0 || value == 1) {
            throw std::out_of_range("detent.per_revolution must be 0 or >= 2");
          }
          next_detents_per_revolution = static_cast<int>(value);
        } else if (name == "detent.spacing") {
          next_detent_spacing = parameter.as_double();
        } else if (name == "detent.kp") {
          next_detent_kp = parameter.as_double();
        } else if (name == "detent.kd") {
          next_detent_kd = parameter.as_double();
        } else if (name == "detent.iq_ff") {
          next_detent_iq_ff = parameter.as_double();
        } else if (name == "endstop.min" || name == "limit.min") {
          next_endstop_min = parameter.as_double();
        } else if (name == "endstop.max" || name == "limit.max") {
          next_endstop_max = parameter.as_double();
        } else if (name == "endstop.kp" || name == "limit.kp") {
          next_endstop_kp = parameter.as_double();
        } else if (name == "endstop.kd" || name == "limit.kd") {
          next_endstop_kd = parameter.as_double();
        } else if (name == "spring.kp") {
          next_spring_kp = parameter.as_double();
        } else if (name == "spring.kd") {
          next_spring_kd = parameter.as_double();
        } else if (name == "spring.iq_ff") {
          next_spring_iq_ff = parameter.as_double();
        } else {
          throw std::invalid_argument(
            name + " cannot be changed at runtime; edit YAML and restart the node");
        }
      }

      double effective_detent_spacing = next_detent_spacing;
      if (next_detents_per_revolution > 0) {
        effective_detent_spacing =
          wrap_period_ / static_cast<double>(next_detents_per_revolution);
      }
      if (!std::isfinite(effective_detent_spacing) || effective_detent_spacing <= 0.0) {
        throw std::out_of_range("detent spacing must be finite and > 0");
      }
      if (wrap_angle_ && effective_detent_spacing >= wrap_period_) {
        throw std::out_of_range(
          "detent spacing must be smaller than the active wrap period");
      }

      validate_gain("detent.kp", next_detent_kp, kp_max_);
      validate_gain("detent.kd", next_detent_kd, kd_max_);
      validate_iq("detent.iq_ff", next_detent_iq_ff);
      validate_gain("endstop.kp", next_endstop_kp, kp_max_);
      validate_gain("endstop.kd", next_endstop_kd, kd_max_);
      validate_gain("spring.kp", next_spring_kp, kp_max_);
      validate_gain("spring.kd", next_spring_kd, kd_max_);
      validate_iq("spring.iq_ff", next_spring_iq_ff);

      if (!std::isfinite(next_endstop_min) || !std::isfinite(next_endstop_max)) {
        throw std::out_of_range("endstop.min and endstop.max must be finite");
      }
      if (next_endstop_min >= next_endstop_max) {
        throw std::out_of_range("endstop.min must be smaller than endstop.max");
      }

      mode_ = next_mode;
      print_every_ = next_print_every;
      feedback_timeout_s_ = next_feedback_timeout_s;

      detents_per_revolution_ = next_detents_per_revolution;
      detent_spacing_ = effective_detent_spacing;
      kp_ = next_detent_kp;
      kd_ = next_detent_kd;
      iq_ff_ = next_detent_iq_ff;

      endstop_min_ = next_endstop_min;
      endstop_max_ = next_endstop_max;
      endstop_kp_ = next_endstop_kp;
      endstop_kd_ = next_endstop_kd;

      spring_kp_ = next_spring_kp;
      spring_kd_ = next_spring_kd;
      spring_iq_ff_ = next_spring_iq_ff;

      RCLCPP_INFO(
        get_logger(),
        "Updated smart knob runtime params: mode=%s, detent=[spacing %.5f kp %.3f kd %.3f iq_ff %.3f], endstop=[%.3f %.3f kp %.3f kd %.3f], spring=[kp %.3f kd %.3f iq_ff %.3f]",
        mode_name(mode_), detent_spacing_, kp_, kd_, iq_ff_, endstop_min_, endstop_max_,
        endstop_kp_, endstop_kd_, spring_kp_, spring_kd_, spring_iq_ff_);

      result.successful = true;
      return result;
    } catch (const std::exception & ex) {
      result.reason = ex.what();
      return result;
    }
  }

  void validate_gain(const std::string & name, double value, double max_value) const
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::out_of_range(name + " must be finite and >= 0");
    }
    if (value > max_value) {
      throw std::out_of_range(name + " exceeds configured MIT field max");
    }
  }

  void validate_iq(const std::string & name, double value) const
  {
    if (!std::isfinite(value)) {
      throw std::out_of_range(name + " must be finite");
    }
    if (std::abs(value) > iq_max_) {
      throw std::out_of_range(name + " exceeds motor.iq_max");
    }
  }

  void update_knob()
  {
    auto motor = control_->getMotor(can_id_);
    if (!motor) {
      RCLCPP_ERROR(get_logger(), "No registered motor for can_id=0x%02x", can_id_);
      rclcpp::shutdown();
      return;
    }

    if (!motor->HasFeedback()) {
      send_zero_command(*motor);
      ++tick_;
      if (tick_ % print_every_ == 0) {
        RCLCPP_WARN(get_logger(), "Waiting for first feedback from can_id=0x%02x", can_id_);
      }
      return;
    }

    if (motor->SecondsSinceFeedback() > feedback_timeout_s_) {
      send_zero_command(*motor);
      ++tick_;
      if (tick_ % print_every_ == 0) {
        RCLCPP_WARN(
          get_logger(),
          "Feedback timeout from can_id=0x%02x: %.3f s; sending zero MIT command",
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
    const KnobCommand command = compute_command(q);

    if (motor->Get_Error()) {
      send_zero_command(*motor);
    } else if (!command.active) {
      send_zero_command(*motor);
    } else {
      send_command(*motor, command);
    }
    publish_state(*motor);

    ++tick_;
    if (tick_ % print_every_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "mode=%s active=%s index=%ld q=%.5f relative=%.5f target=%.5f err_q=%.5f dq=%.5f kp=%.5f kd=%.5f iq_ff=%.5f feedback_iq=%.5f err=%s",
        command.label, command.active ? "true" : "false", command.index, q, command.relative_q,
        command.target_q, command.error_q, dq, command.kp, command.kd, command.iq_ff,
        motor->Get_iq(), motor->Get_Error() ? "true" : "false");
    }
  }

  KnobCommand compute_command(double q) const
  {
    switch (mode_) {
      case KnobMode::Free:
        return compute_free_command(q);
      case KnobMode::Detent:
        return compute_detent_command(q);
      case KnobMode::Endstop:
        return compute_endstop_command(q);
      case KnobMode::Spring:
        return compute_spring_command(q);
    }
    return compute_free_command(q);
  }

  KnobCommand compute_free_command(double q) const
  {
    KnobCommand command{};
    command.label = "free";
    command.relative_q = wrap_delta(q - center_q_);
    command.target_q = q;
    command.q_ref = q;
    return command;
  }

  KnobCommand compute_detent_command(double q) const
  {
    KnobCommand command{};
    command.active = true;
    command.label = "detent";
    command.relative_q = wrap_delta(q - center_q_);
    command.index = static_cast<long>(std::llround(command.relative_q / detent_spacing_));
    const double target_relative_q = static_cast<double>(command.index) * detent_spacing_;
    command.target_q = wrap_absolute(center_q_ + target_relative_q);
    command.error_q = wrap_delta(target_relative_q - command.relative_q);
    command.kp = kp_;
    command.kd = kd_;
    command.q_ref = command.target_q;
    command.dq_ref = 0.0;
    command.iq_ff = iq_ff_;
    return command;
  }

  KnobCommand compute_endstop_command(double q) const
  {
    KnobCommand command{};
    command.label = "endstop";
    command.relative_q = wrap_delta(q - center_q_);

    if (command.relative_q < endstop_min_) {
      command.active = true;
      command.index = -1;
      const double target_relative_q = endstop_min_;
      command.target_q = wrap_absolute(center_q_ + target_relative_q);
      command.error_q = wrap_delta(target_relative_q - command.relative_q);
    } else if (command.relative_q > endstop_max_) {
      command.active = true;
      command.index = 1;
      const double target_relative_q = endstop_max_;
      command.target_q = wrap_absolute(center_q_ + target_relative_q);
      command.error_q = wrap_delta(target_relative_q - command.relative_q);
    } else {
      command.active = false;
      command.index = 0;
      command.target_q = q;
      command.error_q = 0.0;
    }

    command.kp = command.active ? endstop_kp_ : 0.0;
    command.kd = command.active ? endstop_kd_ : 0.0;
    command.q_ref = command.target_q;
    command.dq_ref = 0.0;
    command.iq_ff = 0.0;
    return command;
  }

  KnobCommand compute_spring_command(double q) const
  {
    KnobCommand command{};
    command.active = true;
    command.label = "spring";
    command.relative_q = wrap_delta(q - center_q_);
    command.target_q = center_q_;
    command.error_q = wrap_delta(-command.relative_q);
    command.kp = spring_kp_;
    command.kd = spring_kd_;
    command.q_ref = center_q_;
    command.dq_ref = 0.0;
    command.iq_ff = spring_iq_ff_;
    return command;
  }

  double wrap_delta(double value) const
  {
    return wrap_angle_ ? wrap_to_half_period(value, wrap_period_) : value;
  }

  double wrap_absolute(double value) const
  {
    return wrap_angle_ ? wrap_to_half_period(value, wrap_period_) : value;
  }

  void send_zero_command(kw::Motor & motor)
  {
    control_->control_mit(
      motor,
      0.0F,
      0.0F,
      0.0F,
      0.0F,
      0.0F);
  }

  void send_command(kw::Motor & motor, const KnobCommand & command)
  {
    control_->control_mit(
      motor,
      static_cast<float>(command.kp),
      static_cast<float>(command.kd),
      static_cast<float>(command.q_ref),
      static_cast<float>(command.dq_ref),
      static_cast<float>(command.iq_ff));
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
  double pos_max_{12.5};
  double vel_max_{45.0};
  double iq_max_{2.0};
  double kp_max_{500.0};
  double kd_max_{5.0};
  KnobMode mode_{KnobMode::Detent};
  int detents_per_revolution_{0};
  double detent_spacing_{0.25};
  double kp_{1.2};
  double kd_{0.02};
  double iq_ff_{0.0};
  double endstop_min_{-1.57};
  double endstop_max_{1.57};
  double endstop_kp_{1.2};
  double endstop_kd_{0.02};
  double spring_kp_{1.2};
  double spring_kd_{0.02};
  double spring_iq_ff_{0.0};
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
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
