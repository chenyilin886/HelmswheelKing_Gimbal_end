#include "chassis_task.h"
#include "chassis.hpp"
#include "DT7.hpp"
#include "uart_bus.hpp"
#include "can_hal.hpp"
#include "can_device.hpp"
#include "chassis_from_gimbal.hpp"
#include "SlopePlanning.hpp"
#include <cmath>

extern UART_HandleTypeDef huart3;

// ================= 全局实例定义 =================
uint8_t dbus_buf[18];
BSP::REMOTE_CONTROL::RemoteController remote_control(100);

Chassis my_chassis;
Comm::ChassisFromGimbal c2g;
//斜坡规划
Alg::Utility::SlopePlanning slope_vx(0.004f, 0.008f);//加速斜坡  减速斜坡
Alg::Utility::SlopePlanning slope_vy(0.004f, 0.008f);
Alg::Utility::SlopePlanning slope_vw(0.002f, 0.002f);

//底盘运动参数
volatile float max_linear_speed = 1.0f;
volatile float max_angular_speed = 1.0f;
volatile float yaw_err_feedforward_delay = 0.08f;
volatile float gyro_steer_offset = 0.56f;

// ================= 串口接收回调 (桥接 HAL 库与 C++ 框架) =================

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        HAL::UART::Data rx_data = {dbus_buf, Size};
        // 获取串口3的 C++ 单例对象，并触发它的接收回调逻辑
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        uart3.trigger_rx_callbacks(rx_data);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//处理 ORE 溢出错误
{
    if (huart->Instance == USART3)
    {
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        HAL::UART::Data rx_data = {dbus_buf, 18};
        uart3.clear_ore_error(rx_data);
    }
}
//接收回调函数
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
    // 注册串口3的解析逻辑
    uart3.register_rx_callback([&](const HAL::UART::Data &data) {
        if (data.size == 18) {
            remote_control.parseData(data.buffer);
        }
        // 重新开启下一次 DMA 接收
        HAL::UART::Data next_rx_data = {dbus_buf, 18};
        auto &u3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        u3.receive_dma_idle(next_rx_data);
    });
    // 第一次启动遥控器 DMA 接收
    HAL::UART::Data initial_rx_data = {dbus_buf, 18};
    uart3.receive_dma_idle(initial_rx_data);
    // 绑定并初始化CAN
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
    //板间通信判断优先
    if (c2g.isConnected())
    {
        float lx = c2g.getLeftX();
        float ly = c2g.getLeftY();
        float yaw_err = c2g.getYawAngleErr();

        while (yaw_err > 3.14159265f) yaw_err -= 2.0f * 3.14159265f;
        while (yaw_err < -3.14159265f) yaw_err += 2.0f * 3.14159265f;

        float raw_vx = ly * max_linear_speed;
        float raw_vy = -lx * max_linear_speed;

        float current_vw = 0.0f;
        if (c2g.isRotatingMode())
        {
            current_vw = max_angular_speed;
            target_vw = max_angular_speed;

            if (gyro_steer_offset != 0.0f)
            {
                float cos_off = cosf(gyro_steer_offset);
                float sin_off = sinf(gyro_steer_offset);
                float tmp_vx = raw_vx * cos_off - raw_vy * sin_off;
                float tmp_vy = raw_vx * sin_off + raw_vy * cos_off;
                raw_vx = tmp_vx;
                raw_vy = tmp_vy;
            }
        }
        else if (c2g.isFollowMode())
        {
            float follow_kp = 0.8f;
            if (fabsf(yaw_err) > 0.12f)
            {
                current_vw = -yaw_err * follow_kp;
                if (current_vw > max_angular_speed) current_vw = max_angular_speed;
                if (current_vw < -max_angular_speed) current_vw = -max_angular_speed;
                target_vw = current_vw;
            }
        }

        float compensated_yaw_err = yaw_err - current_vw * yaw_err_feedforward_delay;
        float cos_theta = cosf(compensated_yaw_err + 3.14159265f);
        float sin_theta = sinf(compensated_yaw_err + 3.14159265f);

        target_vx = raw_vx * cos_theta - raw_vy * sin_theta;
        target_vy = raw_vx * sin_theta + raw_vy * cos_theta;
        //急停
        if (c2g.isStopMode())
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vw = 0.0f;
            slope_vx.Reset();
            slope_vy.Reset();
            slope_vw.Reset();
        }
        else
        {
            slope_vx.TIM_Calculate_PeriodElapsedCallback(target_vx, my_chassis.get_vx());
            slope_vy.TIM_Calculate_PeriodElapsedCallback(target_vy, my_chassis.get_vy());
            slope_vw.TIM_Calculate_PeriodElapsedCallback(target_vw, my_chassis.get_vw());
            target_vx = slope_vx.GetOut();
            target_vy = slope_vy.GetOut();
            target_vw = slope_vw.GetOut();
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
                slope_vx.Reset();
                slope_vy.Reset();
                slope_vw.Reset();
            }
            else
            {
                float raw_vx = remote_control.get_left_y() * max_linear_speed;
                float raw_vy = -remote_control.get_left_x() * max_linear_speed;
                if (s2 == BSP::REMOTE_CONTROL::RemoteController::UP)
                {
                    target_vw = max_angular_speed;

                    if (gyro_steer_offset != 0.0f)
                    {
                        float cos_off = cosf(gyro_steer_offset);
                        float sin_off = sinf(gyro_steer_offset);
                        float tmp_vx = raw_vx * cos_off - raw_vy * sin_off;
                        float tmp_vy = raw_vx * sin_off + raw_vy * cos_off;
                        raw_vx = tmp_vx;
                        raw_vy = tmp_vy;
                    }
                }
                target_vx = raw_vx;
                target_vy = raw_vy;
                slope_vx.TIM_Calculate_PeriodElapsedCallback(target_vx, my_chassis.get_vx());
                slope_vy.TIM_Calculate_PeriodElapsedCallback(target_vy, my_chassis.get_vy());
                slope_vw.TIM_Calculate_PeriodElapsedCallback(target_vw, my_chassis.get_vw());
                target_vx = slope_vx.GetOut();
                target_vy = slope_vy.GetOut();
                target_vw = slope_vw.GetOut();
            }
        }
    }
    //将算好的目标速度下发给底盘运动学对象
    my_chassis.Set_Velocity(target_vx, target_vy, target_vw);
    my_chassis.Update();
}
