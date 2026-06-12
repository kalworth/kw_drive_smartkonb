#include "kw_drive/protocol/kw_motor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace kw
{    
     
namespace
{

uint16_t command_id(uint16_t motor_id)
{
    return motor_id | 0x70;
}

uint16_t feedback_id(uint16_t motor_id)
{
    return motor_id | 0x10;
}

}

Motor::Motor(uint16_t can_id, LimitParam limit)
        : can_id_(can_id){
    this->limit_param = limit;
    this->last_time_= std::chrono::steady_clock::now();
}

void Motor::updateTimeInterval() 
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last_time_;
    last_time_ = now;

    delta_time_ = dt.count(); // 秒为单位
}

double Motor::getTimeInterval() 
{
    return delta_time_;
}


void Motor::receive_data(float q, float dq, float iq, bool error)
{
    this->state_q = q;
    this->state_dq = dq;
    this->state_iq = iq;
    this->state_error = error;
    this->state_received = true;
}

double Motor::SecondsSinceFeedback() const
{
    if (!state_received) {
        return std::numeric_limits<double>::infinity();
    }
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_time_).count();
}

MotorControl::MotorControl(uint32_t serial_baud, std::string serial_port,
    std::vector<MotorConfig> *data_ptr, bool enable_on_start)
    :  data_ptr_(data_ptr)
{
    for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it) {
        std::shared_ptr<Motor> motor = std::make_shared<Motor>(it->can_id, it->limit);
        addMotor(motor);
    }

    transport_ = std::make_shared<KwSerialTransport>(serial_port, serial_baud);
    transport_->setFrameCallback(
        [this](const CanFrame& frame) {
            this->canframeCallback(frame);
        });
   
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
   
    if (enable_on_start) {
        enable_all();
    }

    std::cout<<"**********KW MotorControl initialization successful**********"<<std::endl<<std::endl;
}

MotorControl::~MotorControl()
{   
    try {
        if (enabled_) {
            disable_all();
        }
    } catch (const std::exception& ex) {
        std::cerr << "[Error] Failed to disable motor during shutdown: " << ex.what() << std::endl;
    }
}

void MotorControl::addMotor(std::shared_ptr<Motor> motor)
{
    motors.insert({motor->GetCanId(), motor});
    motors.insert({feedback_id(motor->GetCanId()), motor});
}

void MotorControl::enable_all()
{   
    for (const auto & config : *data_ptr_) {
        auto motor = getMotor(config.can_id);
        if (!motor) {
            continue;
        }
        for (int j = 0; j < 5; j++) {
            control_cmd(motor->GetCanId(), 0xFC);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    enabled_ = true;
}

void MotorControl::disable_all()
{
    for (const auto & config : *data_ptr_) {
        auto motor = getMotor(config.can_id);
        if (!motor) {
            continue;
        }
        for (int j = 0; j < 5; j++) {
            control_cmd(motor->GetCanId(), 0xFD);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    enabled_ = false;
}

void MotorControl::control_cmd(uint16_t id , uint8_t cmd)
{
    const std::array<uint8_t, 8> data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    transport_->sendCanFrame(id, data);
}

void MotorControl::control_mit(Motor &motor, float kp, float kd, float q, float dq, float iq)
{
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        if (!std::isfinite(x) || !std::isfinite(xmin) || !std::isfinite(xmax)) {
            throw std::invalid_argument("float_to_uint: values must be finite");
        }
        if (xmax <= xmin) {
            throw std::invalid_argument("float_to_uint: xmax must be greater than xmin");
        }
        float span = xmax - xmin;
        float data_norm = std::clamp((x - xmin) / span, 0.0F, 1.0F);
        return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
    };
    uint16_t id = motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("control_mit: motor id is not registered");
    }
    auto& m = motors[id];
    LimitParam limit_param_cmd = m->get_limit_param();
    uint16_t kp_uint = float_to_uint(kp, 0.0F, limit_param_cmd.kp_max, 12);
    uint16_t kd_uint = float_to_uint(kd, 0.0F, limit_param_cmd.kd_max, 12);
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.pos_max, limit_param_cmd.pos_max, 16);
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.vel_max, limit_param_cmd.vel_max, 12);
    uint16_t iq_uint = float_to_uint(iq, -limit_param_cmd.iq_max, limit_param_cmd.iq_max, 12);

    uint16_t can_id = command_id(id);
    std::array<uint8_t, 8> data{};
    data[0] = (q_uint >> 8) & 0xff;
    data[1] = q_uint & 0xff;
    data[2] = dq_uint >> 4;
    data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    data[4] = kp_uint & 0xff;
    data[5]= kd_uint >> 4;
    data[6] = ((kd_uint & 0xf) << 4) | ((iq_uint >> 8) & 0xf);
    data[7] = iq_uint & 0xff;

    transport_->sendCanFrame(can_id, data);
}

void MotorControl::canframeCallback(const CanFrame& frame)
{   
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        
        return data;
    };

    uint32_t canID = frame.can_id;
    if(motors.find(canID) == motors.end())
    {
        return;
    }

    const bool error = (frame.data[1] & 0x01) != 0;
    uint16_t q_uint = (uint16_t(frame.data[2]) << 8) | frame.data[3];
    uint16_t dq_uint = (uint16_t(frame.data[4]) << 8) | frame.data[5];
    uint16_t iq_uint = (uint16_t(frame.data[6]) << 8) | frame.data[7];

    auto m = motors[canID];
    LimitParam limit_param_receive = m->get_limit_param();
    float receive_q = uint_to_float(q_uint, -limit_param_receive.pos_max, limit_param_receive.pos_max, 16);
    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.vel_max, limit_param_receive.vel_max, 16);
    float receive_iq = uint_to_float(iq_uint, -limit_param_receive.iq_max, limit_param_receive.iq_max, 16);

    m->updateTimeInterval();
    m->receive_data(receive_q, receive_dq, receive_iq, error);
}

}
        
       
