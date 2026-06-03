#include "vision.hpp"
#include "uart_bus.hpp"
#include "gimbal_to_chassis.hpp"
// 引入 USB CDC 接口头文件
#include "usbd_cdc_if.h"
#include "cmsis_os.h" // 引入 FreeRTOS 的 API
extern "C" {
    // 引用在 CubeMX 里生成的队列句柄
    extern osMessageQueueId_t visionTxQueueHandle; 
}
namespace Comm
{

Vision vision;
\
void Vision::send(float quat_w, float quat_x, float quat_y, float quat_z)
{
    tx_gimbal_.quat_w = quat_w;
    tx_gimbal_.quat_x = quat_x;
    tx_gimbal_.quat_y = quat_y;
    tx_gimbal_.quat_z = quat_z;
    tx_gimbal_.timestamp = tx_timestamp_++;

    tx_other_.tail = TX_FRAME_TAIL;

    packTxFrame();

    // auto &uart = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    // HAL::UART::Data data{tx_buffer_, TX_FRAME_SIZE};
    // uart.transmit(data); // 通过 UART6 发出


    // 使用 USB CDC 发送数据
    // 直接把打包好的数据扔进传送带 (队列)
    if (visionTxQueueHandle != NULL)
    {
        // 0 表示如果队列满了（比如拔掉USB线），绝不死等，直接丢弃这帧数据，保护云台主控循环！
        osStatus_t status = osMessageQueuePut(visionTxQueueHandle, tx_buffer_, 0, 0);
        if (status == osOK)
        {
            debug_data_.tx_count++; // 成功放入队列，计数增加
        }
    }

    debug_data_.tx_quat_w = tx_gimbal_.quat_w;
    debug_data_.tx_quat_x = tx_gimbal_.quat_x;
    debug_data_.tx_quat_y = tx_gimbal_.quat_y;
    debug_data_.tx_quat_z = tx_gimbal_.quat_z;
    debug_data_.tx_bullet_speed = tx_other_.bullet_speed;
    debug_data_.tx_enemy_color = tx_other_.enemy_color;
    debug_data_.tx_vision_mode = tx_other_.vision_mode;
    debug_data_.tx_timestamp = tx_gimbal_.timestamp;
    std::memcpy(debug_data_.tx_buffer, tx_buffer_, TX_FRAME_SIZE);

}

void Vision::receive()
{
    debug_data_.rx_frame_valid = parseRxFrame();
    
    if (!debug_data_.rx_frame_valid)
    {
        return;
    }// 帧头不对就丢弃

    vision_flag_ = (rx_other_.vision_ready != 0); // 是否识别到目标

     target_yaw_ = rx_target_.yaw_angle * -1.0f;
     target_pitch_ = rx_target_.pitch_angle * -1.0f;

    if (!fire_initialized_)
    {
        last_fire_value_ = rx_other_.fire;
        fire_initialized_ = true;
    }
    else if (rx_other_.fire != last_fire_value_)
    {
        last_fire_value_ = rx_other_.fire;
        fire_update_count_++;
    }
    
    debug_data_.rx_count++;
    debug_data_.rx_pitch_angle = rx_target_.pitch_angle;
    debug_data_.rx_yaw_angle = rx_target_.yaw_angle;
    debug_data_.rx_vision_ready = rx_other_.vision_ready;
    debug_data_.rx_fire = rx_other_.fire;
    debug_data_.rx_aim_x = rx_other_.aim_x;
    debug_data_.rx_aim_y = rx_other_.aim_y;
    debug_data_.rx_timestamp = rx_target_.timestamp;
    debug_data_.vision_flag = vision_flag_;
    std::memcpy(debug_data_.rx_buffer, rx_buffer_, RX_FRAME_SIZE);
}

void Vision::packTxFrame()//打包发送帧
{
    size_t offset = 0;

    tx_buffer_[offset++] = TX_FRAME_HEAD1;
    tx_buffer_[offset++] = TX_FRAME_HEAD2;

    std::memcpy(&tx_buffer_[offset], &tx_gimbal_.quat_w, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&tx_buffer_[offset], &tx_gimbal_.quat_x, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&tx_buffer_[offset], &tx_gimbal_.quat_y, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&tx_buffer_[offset], &tx_gimbal_.quat_z, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&tx_buffer_[offset], &tx_other_.bullet_speed, sizeof(float));
    offset += sizeof(float);

    tx_buffer_[offset++] = tx_other_.enemy_color;
    tx_buffer_[offset++] = tx_other_.vision_mode;

    tx_buffer_[offset++] = static_cast<uint8_t>(tx_gimbal_.timestamp >> 24);
    tx_buffer_[offset++] = static_cast<uint8_t>(tx_gimbal_.timestamp >> 16);
    tx_buffer_[offset++] = static_cast<uint8_t>(tx_gimbal_.timestamp >> 8);
    tx_buffer_[offset++] = static_cast<uint8_t>(tx_gimbal_.timestamp);

    tx_buffer_[offset++] = tx_other_.tail;
}

bool Vision::parseRxFrame()//解析接收帧
{
    if (rx_buffer_[0] != RX_FRAME_HEAD1 || rx_buffer_[1] != RX_FRAME_HEAD2)
    {
        return false;
    }

    int32_t pitch_raw = (rx_buffer_[2] << 24) | (rx_buffer_[3] << 16) | (rx_buffer_[4] << 8) | rx_buffer_[5];
    int32_t yaw_raw = (rx_buffer_[6] << 24) | (rx_buffer_[7] << 16) | (rx_buffer_[8] << 8) | rx_buffer_[9];

    rx_target_.pitch_angle = static_cast<float>(pitch_raw) / 100.0f;
    rx_target_.yaw_angle = static_cast<float>(yaw_raw) / 100.0f;

    rx_other_.vision_ready = rx_buffer_[10];
    rx_other_.fire = rx_buffer_[11];
    rx_other_.tail = rx_buffer_[12];

    rx_target_.timestamp = (rx_buffer_[13] << 24) | (rx_buffer_[14] << 16) | (rx_buffer_[15] << 8) | rx_buffer_[16];

    rx_other_.aim_x = rx_buffer_[17];
    rx_other_.aim_y = rx_buffer_[18];

    return true;
}

}
