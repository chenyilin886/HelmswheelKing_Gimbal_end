#include "chassis_from_gimbal.hpp"
#include "stm32f4xx_hal.h"

namespace Comm
{

void ChassisFromGimbal::parseFrame(const HAL::CAN::Frame &frame)
{
    if (frame.id != CAN_G2C_FRAME_ID)
        return;

    if (frame.dlc < sizeof(GimbalToChassisFrame))
        return;

    std::memcpy(&rx_frame_, frame.data, sizeof(GimbalToChassisFrame));

    if (rx_frame_.head != 0xA5)
        return;

    mode_.universal = (rx_frame_.mode >> 0) & 0x01;
    mode_.follow = (rx_frame_.mode >> 1) & 0x01;
    mode_.rotating = (rx_frame_.mode >> 2) & 0x01;
    mode_.stop = (rx_frame_.mode >> 3) & 0x01;

    last_rx_time_ = HAL_GetTick();
}

bool ChassisFromGimbal::isConnected() const
{
    return (HAL_GetTick() - last_rx_time_) < TIMEOUT_MS;
}

}
