#ifndef GIMBAL_DRIVER_HPP
#define GIMBAL_DRIVER_HPP

#pragma once

#include "DmMotor.hpp"
#include "DT7.hpp"
#include "HI12_imu.hpp"
#include "can_hal.hpp"
#include "uart_hal.hpp"
#include "gimbal_to_chassis.hpp"
#include "vision.hpp"
#include <cstring>

class GimbalDriver
{
public:
    static constexpr uint8_t PITCH_ID = 1;
    static constexpr uint8_t YAW_ID = 2;
    static constexpr uint16_t DBUS_BUF_SIZE = 18;
    static constexpr uint16_t IMU_BUF_SIZE = 82;
    static constexpr uint16_t CONNECT_WAIT_MAX = 150;

    GimbalDriver() = default;

    void init()//初始化视觉模块
    {
        Comm::vision.setEnemyColor(0x52);
        Comm::vision.setVisionMode(1);
        Comm::vision.setBulletSpeed(26.0f);
        //startVisionReceive(); // 启动 UART6 接收视觉数据
    }

    void enableMotors() { startMotorEnable(); }

    void disableMotors()
    {
        dm_motor_.Off(YAW_ID, BSP::Motor::DM::Model::MIT);
        dm_motor_.Off(PITCH_ID, BSP::Motor::DM::Model::MIT);
        motors_enabled_ = false;
        enable_step_ = 0;
        encoder_initialized_ = false;
        connect_wait_counter_ = 0;
    }

    bool processEnable()
    {
        if (enable_step_ == 0) return false;

        switch (enable_step_)
        {
            case 1:
                dm_motor_.ClearErr(PITCH_ID, BSP::Motor::DM::Model::MIT);
                enable_wait_ = 0;
                enable_step_ = 2;
                break;
            case 2:
                if (++enable_wait_ >= 4)
                {
                    dm_motor_.ClearErr(YAW_ID, BSP::Motor::DM::Model::MIT);
                    enable_wait_ = 0;
                    enable_step_ = 3;
                }
                break;
            case 3:
                if (++enable_wait_ >= 4)
                {
                    dm_motor_.On(PITCH_ID, BSP::Motor::DM::Model::MIT);
                    enable_wait_ = 0;
                    enable_step_ = 4;
                }
                break;
            case 4:
                if (++enable_wait_ >= 4)
                {
                    dm_motor_.On(YAW_ID, BSP::Motor::DM::Model::MIT);
                    enable_wait_ = 0;
                    enable_step_ = 5;
                }
                break;
            case 5:
                if (++enable_wait_ >= 4)
                {
                    enable_step_ = 0;
                    return true;
                }
                break;
            default:
                enable_step_ = 0;
                break;
        }
        return false;
    }

    bool processEncoderInit()
    {
        if (encoder_initialized_) return true;

        if (dm_motor_.isConnected(PITCH_ID, 100) && dm_motor_.isConnected(YAW_ID, 100))
        {
            pitch_offset_ = dm_motor_.getAngleRad(PITCH_ID);
            encoder_initialized_ = true;
            connect_wait_counter_ = 0;
            return true;
        }

        if (enable_step_ == 0)
        {
            if (motors_enabled_ && connect_wait_counter_ < CONNECT_WAIT_MAX)
            {
                connect_wait_counter_++;
            }
            else
            {
                startMotorEnable();
                connect_wait_counter_ = 0;
            }
        }
        return false;
    }

    void sendTorque(uint8_t motor_id, float torque)
    {
        dm_motor_.ctrl_Mit(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, torque);
    }

    void sendZeroTorque(uint8_t motor_id)
    {
        dm_motor_.ctrl_Mit(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    void sendToChassis(float left_x, float left_y, float yaw_err,
                       const Comm::ChassisMode &mode)
    {
        g2c_.packFrame(left_x, left_y, yaw_err, mode);

        auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
        HAL::CAN::Frame frame{};
        frame.id = Comm::CAN_G2C_FRAME_ID;
        frame.dlc = Comm::GimbalToChassis::size();
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        std::memcpy(frame.data, g2c_.data(), Comm::GimbalToChassis::size());
        can2.send(frame);
    }

    void parseCANFrame(const HAL::CAN::Frame &frame) { dm_motor_.Parse(frame); }

    void processUARTRx(UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart->Instance == USART3)
        {
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
                __HAL_UART_CLEAR_OREFLAG(huart);

            if (size == DBUS_BUF_SIZE)
                dr16_.parseData(dbus_rx_buffer_);

            auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
            HAL::UART::Data data{dbus_rx_buffer_, DBUS_BUF_SIZE};
            uart3.receive_dma_idle(data);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
        else if (huart->Instance == USART1)
        {
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
                __HAL_UART_CLEAR_OREFLAG(huart);

            if (size <= IMU_BUF_SIZE)
            {
                imu_.DataUpdate(imu_rx_buffer_);
                imu_initialized_ = true;
            }

            auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
            HAL::UART::Data data{imu_rx_buffer_, IMU_BUF_SIZE};
            uart1.receive_dma_idle(data);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    void processUARTRxCplt(UART_HandleTypeDef *huart)
    {
        // if (huart->Instance == USART6)
        // {
        //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        //         __HAL_UART_CLEAR_OREFLAG(huart);

        //     if (huart->ErrorCode != HAL_UART_ERROR_NONE)
        //         huart->ErrorCode = HAL_UART_ERROR_NONE;

        //     Comm::vision.receive();

        //     auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
        //     HAL::UART::Data data{Comm::vision.getRxBuffer(), Comm::Vision::getRxSize()};
        //     if (!uart6.receive(data))
        //     {
        //         HAL_UART_Receive_IT(huart, Comm::vision.getRxBuffer(), Comm::Vision::getRxSize());
        //     }
        // }
        
        // 视觉接收已迁移至 USB CDC
        (void)huart;
    }

    void processCANRx(CAN_HandleTypeDef *hcan) { (void)hcan; }

    bool isMotorsEnabled() const { return motors_enabled_; }
    bool isEncoderInitialized() const { return encoder_initialized_; }
    bool isImuInitialized() const { return imu_initialized_; }
    bool isEnableInProgress() const { return enable_step_ != 0; }
    bool isMotorConnected(uint8_t id, uint8_t id_ring) { return dm_motor_.isConnected(id, id_ring); }
    bool isRcConnected() { return dr16_.isConnected(); }
    bool isImuConnected() { return imu_.isConnected(); }

    BSP::REMOTE_CONTROL::RemoteController &getRC() { return dr16_; }
    BSP::IMU::HI12_float &getIMU() { return imu_; }
    BSP::Motor::DM::J4310<2> &getMotor() { return dm_motor_; }

    void setMotorsEnabled(bool en) { motors_enabled_ = en; }

    void startUARTReceive()
    {
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        HAL::UART::Data data{dbus_rx_buffer_, DBUS_BUF_SIZE};
        uart3.receive_dma_idle(data);
        __HAL_DMA_DISABLE_IT(uart3.get_handle()->hdmarx, DMA_IT_HT);
    }

    void startIMUReceive()
    {
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        HAL::UART::Data data{imu_rx_buffer_, IMU_BUF_SIZE};
        uart1.receive_dma_idle(data);
        __HAL_DMA_DISABLE_IT(uart1.get_handle()->hdmarx, DMA_IT_HT);
    }

private:
    void startMotorEnable()
    {
        enable_step_ = 1;
        enable_wait_ = 0;
    }

    void startVisionReceive()// 重新开启接收视觉数据的中断
    {
        auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
        HAL::UART::Data data{Comm::vision.getRxBuffer(), Comm::Vision::getRxSize()};
        uart6.receive(data);
    }

    BSP::Motor::DM::J4310<2> dm_motor_{0x00, {6, 8}, {4, 7}};
    BSP::REMOTE_CONTROL::RemoteController dr16_;
    Comm::GimbalToChassis g2c_;
    BSP::IMU::HI12_float imu_;

    bool encoder_initialized_ = false;
    bool imu_initialized_ = false;
    bool motors_enabled_ = false;
    uint8_t enable_step_ = 0;
    uint8_t enable_wait_ = 0;
    uint16_t connect_wait_counter_ = 0;
    float pitch_offset_ = 0.0f;

    uint8_t dbus_rx_buffer_[DBUS_BUF_SIZE] = {};
    uint8_t imu_rx_buffer_[IMU_BUF_SIZE] = {};
};

#endif
