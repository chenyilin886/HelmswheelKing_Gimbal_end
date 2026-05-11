#ifndef GIMBAL_TO_CHASSIS_HPP
#define GIMBAL_TO_CHASSIS_HPP

#pragma once

#include <cstdint>
#include <cstring>

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

class GimbalToChassis
{
public:
    GimbalToChassis() = default;

    void packFrame(float left_x, float left_y, float yaw_angle_err, const ChassisMode &mode);

    const uint8_t *data() const { return reinterpret_cast<const uint8_t *>(&frame_); }
    static constexpr uint8_t size() { return sizeof(GimbalToChassisFrame); }

    float getYawAngleErr() const { return frame_.yaw_angle_err; }
    bool isRotatingMode() const { return mode_.rotating; }

private:
    static uint8_t floatToChannel(float value)
    {
        int16_t raw = static_cast<int16_t>(value * 110.0f) + 110;
        if (raw < 0) raw = 0;
        if (raw > 220) raw = 220;
        return static_cast<uint8_t>(raw);
    }

    GimbalToChassisFrame frame_{};
    ChassisMode mode_{};
};

}

#endif
