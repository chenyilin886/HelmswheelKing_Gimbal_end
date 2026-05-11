#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP

#pragma once

#include "DmMotor.hpp"
#include "DT7.hpp"
#include "pid.hpp"
#include "can_hal.hpp"
#include "uart_hal.hpp"
#include "gimbal_to_chassis.hpp"

extern BSP::Motor::DM::J4310<2> dm_motor;
extern BSP::REMOTE_CONTROL::RemoteController dr16;
extern Comm::GimbalToChassis g2c;

extern ALG::PID::PID yaw_speed_pid;
extern ALG::PID::PID pitch_angle_pid;
extern ALG::PID::PID pitch_speed_pid;

extern float target_pitch_rad;

struct GimbalDebugData
{
    float yaw_speed_target;
    float yaw_speed;
    float yaw_torque;
    float pitch_angle;
    float pitch_speed;
    float pitch_torque;
    uint8_t yaw_connected;
    uint8_t pitch_connected;
};

extern GimbalDebugData gimbal_debug;

namespace Gimbal
{
    constexpr uint8_t PITCH_ID = 1;
    constexpr uint8_t YAW_ID = 2;
    constexpr uint16_t DBUS_BUF_SIZE = 18;
    constexpr int16_t YAW_INIT_ANGLE_DEG = 77;

    float ZeroCrossingProcessing(float expectations, float feedback, float maxpos);
    float CalcuGimbalToChassisAngle();

    struct GimbalConfig_t {
        float pitch_angle_max;
        float pitch_angle_min;
        float yaw_vel_scale;
        float pitch_sensitivity;
    };

    extern GimbalConfig_t gimbal_cfg;

    void Init();
    void ParseCANFrame(const HAL::CAN::Frame &frame);
    void StartUARTReceive();
    void EnableMotors();
    void DisableMotors();
    void Update();
    void SendToChassis();
    void ProcessUARTRx(UART_HandleTypeDef *huart, uint16_t size);
    void ProcessCANRx(CAN_HandleTypeDef *hcan);
    void ProcessCANFifo1(CAN_HandleTypeDef *hcan);
}

#endif
