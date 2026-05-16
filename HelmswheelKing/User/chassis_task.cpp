#include "chassis_task.h"
#include "chassis.hpp"
#include "DT7.hpp"
#include "uart_bus.hpp"
#include "can_hal.hpp"
#include "can_device.hpp"
#include "chassis_from_gimbal.hpp"
#include <cmath>

extern UART_HandleTypeDef huart3;

// ================= 全局实例定义 =================
uint8_t dbus_buf[18];
BSP::REMOTE_CONTROL::RemoteController remote_control(100);

Chassis my_chassis;
Comm::ChassisFromGimbal c2g;

// ================= 串口接收回调 (桥接 HAL 库与 C++ 框架) =================
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        HAL::UART::Data rx_data = {dbus_buf, Size};
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        uart3.trigger_rx_callbacks(rx_data);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        HAL::UART::Data rx_data = {dbus_buf, 18};
        uart3.clear_ore_error(rx_data);
    }
}

void CAN1_RxCallback(const HAL::CAN::Frame &frame)
{
    if (frame.id >= 0x201 && frame.id <= 0x204) {
        my_chassis.get_wheel_motor().Parse(frame);
    }
    else if (frame.id >= 0x141 && frame.id <= 0x144) {
        my_chassis.get_steer_motor().Parse(frame);
    }
}

void CAN2_RxCallback(const HAL::CAN::Frame &frame)
{
    c2g.parseFrame(frame);
}

void Chassis_Init()
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    uart3.register_rx_callback([&](const HAL::UART::Data &data) {
        if (data.size == 18) {
            remote_control.parseData(data.buffer);
        }
        HAL::UART::Data next_rx_data = {dbus_buf, 18};
        auto &u3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        u3.receive_dma_idle(next_rx_data);
    });

    HAL::UART::Data initial_rx_data = {dbus_buf, 18};
    uart3.receive_dma_idle(initial_rx_data);

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    can1.register_rx_callback(CAN1_RxCallback);
    HAL::CAN::get_can_bus_instance().get_can1();

    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can2.register_rx_callback(CAN2_RxCallback);
    HAL::CAN::get_can_bus_instance().get_can2();

    my_chassis.Init();
}

void Chassis_Control_Task()
{
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_vw = 0.0f;

    if (c2g.isConnected())
    {
        float max_linear_speed = 1.5f;
        float max_angular_speed = 1.2f;

        float lx = c2g.getLeftX();
        float ly = c2g.getLeftY();
        float yaw_err = c2g.getYawAngleErr();

        while (yaw_err > 3.14159265f) yaw_err -= 2.0f * 3.14159265f;
        while (yaw_err < -3.14159265f) yaw_err += 2.0f * 3.14159265f;

        float cos_theta = cosf(yaw_err + 3.14159265f);
        float sin_theta = sinf(yaw_err + 3.14159265f);

        float raw_vx = ly * max_linear_speed;
        float raw_vy = -lx * max_linear_speed;

        target_vx = raw_vx * cos_theta - raw_vy * sin_theta;
        target_vy = raw_vx * sin_theta + raw_vy * cos_theta;

        if (c2g.isRotatingMode())
        {
            target_vw = max_angular_speed;
        }
        else if (c2g.isFollowMode())
        {
            float follow_kp = 0.8f;
            if (fabsf(yaw_err) > 0.12f)
            {
                target_vw = -yaw_err * follow_kp;
                if (target_vw > max_angular_speed) target_vw = max_angular_speed;
                if (target_vw < -max_angular_speed) target_vw = -max_angular_speed;
            }
        }

        if (c2g.isStopMode())
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vw = 0.0f;
        }
    }
    else
    {
        if (remote_control.isConnected())
        {
            uint8_t s2 = remote_control.get_s2();
            if (s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
            {
                target_vx = 0.0f;
                target_vy = 0.0f;
                target_vw = 0.0f;
            }
            else
            {
                float max_linear_speed = 1.5f;
                float max_angular_speed = 1.2f;
                target_vx = remote_control.get_left_y() * max_linear_speed;
                target_vy = -remote_control.get_left_x() * max_linear_speed;
                if (s2 == BSP::REMOTE_CONTROL::RemoteController::UP)
                {
                    target_vw = max_angular_speed;
                }
            }
        }
    }

    my_chassis.Set_Velocity(target_vx, target_vy, target_vw);
    my_chassis.Update();
}
