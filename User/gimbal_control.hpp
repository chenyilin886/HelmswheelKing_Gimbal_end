#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP

#pragma once

#include "gimbal_axis.hpp"
#include "gimbal_target.hpp"
#include "gimbal_driver.hpp"
#include "vision.hpp"

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
    float pitch_angle_target;
    float pitch_angle_err;
    float gravity_ff;
    uint8_t yaw_connected;
    uint8_t pitch_connected;
    uint8_t imu_connected;
    uint8_t yaw_runaway;
    uint8_t pitch_runaway;
    uint8_t gimbal_mode;
};

struct GimbalTuneData
{
    AxisTuneParams yaw;
    AxisTuneParams pitch;
    float yaw_vel_scale;
    float pitch_vel_scale;
    float pitch_angle_scale;
};

extern GimbalDebugData gimbal_debug;
extern GimbalTuneData gimbal_tune;
extern Comm::Vision::DebugData vision_debug;

namespace Gimbal
{
    constexpr float DEG_TO_RAD = 0.0174532925f;
    constexpr float PITCH_ANGLE_MAX_DEG = 20.0f;
    constexpr float PITCH_ANGLE_MIN_DEG = -20.0f;

    void Init();
    void ParseCANFrame(const HAL::CAN::Frame &frame);
    void StartUARTReceive();
    void StartIMUReceive();
    void StartVisionReceive();
    void EnableMotors();
    void DisableMotors();
    void Update();
    void SendToChassis();
    void ProcessUARTRx(UART_HandleTypeDef *huart, uint16_t size);
    void ProcessUARTRxCplt(UART_HandleTypeDef *huart);
    void ProcessCANRx(CAN_HandleTypeDef *hcan);
    void ProcessCANFifo1(CAN_HandleTypeDef *hcan);
    float CalcuGimbalToChassisAngle();
}

#endif
