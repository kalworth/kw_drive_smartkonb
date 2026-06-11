#ifndef KW_DRIVE_PROTOCOL_KW_MOTOR_H
#define KW_DRIVE_PROTOCOL_KW_MOTOR_H

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "kw_drive/protocol/kw_serial_transport.h"

namespace kw
{

typedef struct
{
    float pos_max;
    float vel_max;
    float iq_max;
} LimitParam;

struct MotorConfig
{
    uint16_t can_id;
    LimitParam limit;
};

class Motor
{
private:
    uint16_t can_id_;
    float state_q=0.0;
    float state_dq=0.0;
    float state_iq=0.0;
    bool state_error=false;
    bool state_received=false;
    LimitParam limit_param{};

    std::chrono::steady_clock::time_point last_time_;
    double delta_time_{0.0};
public:

    std::chrono::system_clock::time_point stamp;
    double frequency;

    Motor(uint16_t can_id, LimitParam limit);
    
    void updateTimeInterval(); 
    double getTimeInterval();
    
    void receive_data(float q, float dq, float iq, bool error);
    
    LimitParam get_limit_param() { return limit_param; }
    uint16_t GetCanId() const { return this->can_id_; }
    float Get_Position() const { return this->state_q; }
    float Get_Velocity() const { return this->state_dq; }
    float Get_iq() const { return this->state_iq; }
    bool Get_Error() const { return this->state_error; }
    bool HasFeedback() const { return this->state_received; }
    double SecondsSinceFeedback() const;
};


class MotorControl
{
 public:
    MotorControl(uint32_t serial_baud, std::string serial_port,
        std::vector<MotorConfig> *data_ptr, bool enable_on_start);
    ~MotorControl();
    
    void addMotor(std::shared_ptr<Motor> motor);
    void enable_all();
    void disable_all();
    
    void control_cmd(uint16_t id , uint8_t cmd);
    void control_mit(Motor &motor, float kp, float kd, float q, float dq, float iq);
    
    void canframeCallback(const CanFrame& frame);
    
    
    std::shared_ptr<Motor> getMotor(uint16_t id) const
    {
        auto it = motors.find(id);
        if (it != motors.end()) 
        {
            return it->second;
        } else 
        {   
            std::cerr << "[Error] In getMotor,no motor with id " << id << " is registered." << std::endl;
            return nullptr;
        }
    }

    std::shared_ptr<KwSerialTransport> getTransport() const {
        return transport_;
    }
 private:
    std::unordered_map<uint16_t, std::shared_ptr<Motor>> motors;

    std::vector<MotorConfig>* data_ptr_;
    std::shared_ptr<KwSerialTransport> transport_;
    bool enabled_{false};
    
};

};

#endif
