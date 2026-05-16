#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP

#pragma once

#include "DmMotor.hpp"
#include "DT7.hpp"
#include "ladrc_improved.hpp"
#include "Feedforward.hpp"
#include "pid.hpp"
#include "can_hal.hpp"
#include "uart_hal.hpp"
#include "gimbal_to_chassis.hpp"
#include "HI12_imu.hpp"

extern BSP::Motor::DM::J4310<2> dm_motor;
extern BSP::REMOTE_CONTROL::RemoteController dr16;
extern Comm::GimbalToChassis g2c;
extern BSP::IMU::HI12_float imu;

extern ALG::LADRC::LADRC yaw_adrc;
extern ALG::LADRC::LADRC pitch_adrc;

extern ALG::PID::PID yaw_angle_pid;

extern Alg::Feedforward::Gravity pitch_gravity_ff;

extern float target_yaw_rad;
extern float target_pitch_deg;

struct GimbalDebugData
{
    float yaw_vel_target;
    float yaw_vel_filtered;
    float yaw_vel_feedback;
    float yaw_torque;
    float yaw_angle_target;
    float yaw_angle_feedback;
    float yaw_angle_err;
    float pitch_vel_target;
    float pitch_vel_filtered;
    float pitch_vel_feedback;
    float pitch_torque;
    float pitch_angle_deg;
    float gravity_ff;
    uint8_t yaw_connected;
    uint8_t pitch_connected;
    uint8_t imu_connected;
    uint8_t yaw_runaway;
    uint8_t pitch_runaway;
};

extern GimbalDebugData gimbal_debug;

namespace Gimbal
{
    constexpr uint8_t PITCH_ID = 1;
    constexpr uint8_t YAW_ID = 2;
    constexpr uint16_t DBUS_BUF_SIZE = 18;
    constexpr uint16_t IMU_BUF_SIZE = 82;
    constexpr int16_t YAW_INIT_ANGLE_DEG = 77;

    constexpr float PITCH_ANGLE_MAX_DEG = 30.0f;
    constexpr float PITCH_ANGLE_MIN_DEG = -30.0f;
    constexpr float DEG_TO_RAD = 0.0174532925f;

    constexpr float YAW_VEL_LIMIT_RAD = 5.0f;
    constexpr float PITCH_VEL_LIMIT_RAD = 4.0f;
    constexpr uint8_t RUNAWAY_COUNT_THRESHOLD = 10;

    float ZeroCrossingProcessing(float expectations, float feedback, float maxpos);
    float CalcuGimbalToChassisAngle();

    struct GimbalConfig_t {
        float yaw_vel_scale;
        float pitch_vel_scale;
        float pitch_angle_scale;
        float gravity_comp;
    };

    extern GimbalConfig_t gimbal_cfg;

    void Init();
    void ParseCANFrame(const HAL::CAN::Frame &frame);
    void StartUARTReceive();
    void StartIMUReceive();
    void EnableMotors();
    void DisableMotors();
    void Update();
    void SendToChassis();
    void ProcessUARTRx(UART_HandleTypeDef *huart, uint16_t size);
    void ProcessCANRx(CAN_HandleTypeDef *hcan);
    void ProcessCANFifo1(CAN_HandleTypeDef *hcan);
}

#endif
