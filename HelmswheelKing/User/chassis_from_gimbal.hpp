#ifndef CHASSIS_FROM_GIMBAL_HPP
#define CHASSIS_FROM_GIMBAL_HPP

#pragma once

#include <cstdint>
#include <cstring>
#include "can_hal.hpp"

namespace Comm
{

constexpr uint32_t CAN_G2C_FRAME_ID = 0x205;

struct __attribute__((packed)) GimbalToChassisFrame
{
    uint8_t head;
    uint8_t lx;
    uint8_t ly;
    uint8_t mode;
    float yaw_angle_err;
};

struct ChassisMode
{
    uint8_t universal : 1;
    uint8_t follow : 1;
    uint8_t rotating : 1;
    uint8_t stop : 1;
};

class ChassisFromGimbal
{
public:
    ChassisFromGimbal() = default;

    void parseFrame(const HAL::CAN::Frame &frame);

    float getLeftX() const { return (rx_frame_.lx - 110) / 110.0f; }
    float getLeftY() const { return (rx_frame_.ly - 110) / 110.0f; }
    float getYawAngleErr() const { return rx_frame_.yaw_angle_err * 0.0174532925f; }

    bool isFollowMode() const { return mode_.follow; }
    bool isRotatingMode() const { return mode_.rotating; }
    bool isStopMode() const { return mode_.stop; }
    bool isUniversalMode() const { return mode_.universal; }

    bool isConnected() const;

private:
    GimbalToChassisFrame rx_frame_{};
    ChassisMode mode_{};
    uint32_t last_rx_time_ = 0;
    static constexpr uint32_t TIMEOUT_MS = 500;
};

}

#endif
