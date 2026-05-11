#include "gimbal_to_chassis.hpp"

namespace Comm
{

void GimbalToChassis::packFrame(float left_x, float left_y, float yaw_angle_err, const ChassisMode &mode)
{
    mode_ = mode;

    frame_.head = 0xA5;
    frame_.lx = floatToChannel(left_x);
    frame_.ly = floatToChannel(left_y);
    frame_.yaw_angle_err = yaw_angle_err;

    uint8_t mode_byte = 0;
    mode_byte |= (mode.universal & 0x01) << 0;
    mode_byte |= (mode.follow & 0x01) << 1;
    mode_byte |= (mode.rotating & 0x01) << 2;
    mode_byte |= (mode.stop & 0x01) << 3;
    frame_.mode = mode_byte;
}

}
