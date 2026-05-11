#include "CommunicationTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "can.h"
#include "cmsis_os2.h"
// #include "Variable.hpp"
// #include "State.hpp"
#include "tim.h"
#include "../BSP/Remote/Dbus.hpp"
#include "usart.h"
#include "../HAL/UART/uart_hal.hpp"
#include "../BSP/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "../APP/UI/UI_RefreshBridge.hpp"
#define SIZE 8
extern uint8_t dbus_rx_buffer[18];
int a=0;

UI::Refresh::Probe UI::Refresh::g_probe = {};

namespace
{
constexpr uint32_t kCan2NotifyMask = CAN_IT_RX_FIFO1_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_OVERRUN |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_LAST_ERROR_CODE;
}

// 添加状态监视器，500ms超时
BSP::WATCH_STATE::StateWatch state_watch_(800);
void CommunicationTask(void *argument)
{
	osDelay(500);
    for (;;)
    {
        Gimbal_to_Chassis_Data.PollLinkRecovery();
        if (Gimbal_to_Chassis_Data.ShouldTransmit())
        {
            Gimbal_to_Chassis_Data.Transmit();
        }
        osDelay(4);
    }
}
Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

namespace Communicat
{
void Gimbal_to_Chassis::Init()
{
    ResetRxAssembly();
    const uint32_t now = HAL_GetTick();
    last_frame_time = now;
    last_recovery_attempt_time = last_frame_time;
    pending_can_error = 0;
    last_can_error = 0;
    recovery_count = 0;
    allow_transmit = false;
    // 初始化状态监视器的时间戳，避免启动时误判为离线
    state_watch_.UpdateLastTime();
    state_watch_.UpdateTime();
    state_watch_.CheckStatus();
}
void Gimbal_to_Chassis::ResetRxAssembly()
{
    frame1_received = false;
    frame2_received = false;
    std::memset(can_rx_buffer, 0, sizeof(can_rx_buffer));
}

void Gimbal_to_Chassis::NotifyCanError(uint32_t error)
{
    pending_can_error = error;
    last_can_error = error;
    allow_transmit = false;
}

bool Gimbal_to_Chassis::ShouldTransmit() const
{
    if (!allow_transmit || pending_can_error != 0U)
    {
        return false;
    }

    return (HAL_GetTick() - last_frame_time) < RECOVERY_TRIGGER_MS;
}

void Gimbal_to_Chassis::HandleCANMessage(uint32_t std_id, uint8_t* data)
{
    ParseCANFrame(std_id, data);
}

void Gimbal_to_Chassis::ParseCANFrame(uint32_t std_id, uint8_t* data)
{
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_frame_time > FRAME_TIMEOUT) {
        ResetRxAssembly();
    }
    
    // 检查超时，如果超时则重置接收状态
    // if (current_time - last_frame_time > FRAME_TIMEOUT) {
    //     frame1_received = false;
    //     frame2_received = false;
    // }
    // this->last_frame_time = current_time;

    switch(std_id) {
        case CAN_G2C_FRAME1_ID:
            std::memcpy(can_rx_buffer, data, 8);
            frame1_received = true;
            break;
        case CAN_G2C_FRAME2_ID:
            std::memcpy(can_rx_buffer + 8, data, 8);
            frame2_received = true;
            break;
        default:
            return;
    }
    last_frame_time = current_time;
    pending_can_error = 0;
    allow_transmit = true;

    // Refresh link liveness on every valid board-to-board CAN frame.
    state_watch_.UpdateLastTime();
    state_watch_.UpdateTime();
    state_watch_.CheckStatus();

    
    // Only parse when both CAN frames are present; otherwise ui_list/chassis_mode
    // may be decoded from half-old, half-new bytes and cause random UI state.
    if (frame1_received && frame2_received) {
        ProcessReceivedData();

        // 重置接收状态
        ResetRxAssembly();
    }
}
void Gimbal_to_Chassis::ProcessReceivedData()
{
    const uint8_t EXPECTED_HEAD = 0xA5; // 根据发送端设置的头字节
    if (can_rx_buffer[0] != EXPECTED_HEAD) {
        ResetRxAssembly();
        return;
    }
    auto ptr = can_rx_buffer + 1; // 跳过头字节

    std::memcpy(&direction, ptr, sizeof(direction));
    ptr += sizeof(direction);

    std::memcpy(&chassis_mode, ptr, sizeof(chassis_mode));
    ptr += sizeof(chassis_mode);

    std::memcpy(&ui_list, ptr, sizeof(ui_list));
    ptr += sizeof(ui_list);

    UI::Refresh::NotifyUiDataUpdated();
}

void Gimbal_to_Chassis::SlidingWindowRecovery()
{

    const int window_size = sizeof(pData);
    int found_pos = -1;

    // 遍历整个缓冲区寻找有效头
    for (int i = 0; i < window_size; i++) // 修正循环条件
    {
        if (pData[i] == 0xA5)
        {
            found_pos = i;
            break;
        }
    }

    if (found_pos > 0)
    {
        // 使用memmove处理可能重叠的内存区域
        std::memmove(pData, &pData[found_pos], window_size - found_pos);

        // 可选：重新配置DMA接收剩余空间
        int remaining_space = window_size - found_pos;
        HAL_UART_Receive_DMA(&huart1, pData + remaining_space, found_pos);
    }
}

bool Gimbal_to_Chassis::isConnectOnline()
{
    // 先更新当前时间，再检查状态，确保判断基于最新时间
    state_watch_.UpdateTime();
    state_watch_.CheckStatus();
    return (state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
}

void Gimbal_to_Chassis::PollLinkRecovery()
{
    const uint32_t now = HAL_GetTick();
    const uint32_t time_since_last_frame = now - last_frame_time;
    const uint32_t time_since_last_recovery = now - last_recovery_attempt_time;
    const bool has_partial_packet = frame1_received || frame2_received;
    const bool has_can_error = (pending_can_error != 0U);
    const bool link_timed_out = (time_since_last_frame >= FRAME_TIMEOUT);
    const bool partial_packet_stalled = has_partial_packet && (time_since_last_frame >= RECOVERY_TRIGGER_MS);

    if (has_can_error || time_since_last_frame >= RECOVERY_TRIGGER_MS)
    {
        allow_transmit = false;
    }

    if (!has_can_error && !link_timed_out && !partial_packet_stalled)
    {
        return;
    }

    if (time_since_last_recovery < RECOVERY_RETRY_INTERVAL)
    {
        return;
    }

    RecoverCanReceiver();
    last_recovery_attempt_time = now;
}

void Gimbal_to_Chassis::RecoverCanReceiver()
{
    ResetRxAssembly();
    pending_can_error = 0;
    recovery_count++;
    allow_transmit = false;

    CAN_FilterTypeDef filter = {};
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = 14;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterIdHigh = 0x0;
    filter.FilterIdLow = 0x0;
    filter.FilterMaskIdHigh = 0x0;
    filter.FilterMaskIdLow = 0x0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    HAL_CAN_DeactivateNotification(&hcan2, kCan2NotifyMask);
    HAL_CAN_Stop(&hcan2);
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, kCan2NotifyMask);
    last_frame_time = HAL_GetTick();
}

void Gimbal_to_Chassis::Transmit()
{
    // 使用临时指针将数据拷贝到缓冲区
    booster.heat_one = 0x21;
    booster.heat_two = 0x12;

    setNowBoosterHeat(ext_power_heat_data_0x0202.shooter_id1_17mm_cooling_heat);
    setBoosterMAX(ext_power_heat_data_0x0201.shooter_barrel_heat_limit);
    setBoosterCd(ext_power_heat_data_0x0201.shooter_barrel_cooling_value);
    setLaunchSpeed(ext_shoot_data_0x0207.initial_speed);

    // 使用临时指针将数据拷贝到缓冲区
    uint8_t tx_payload[sizeof(booster)] = {0};
    auto temp_ptr = tx_payload;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };
    memcpy_safe(booster.heat_one);       
    memcpy_safe(booster.heat_two);       
    memcpy_safe(booster.booster_heat_cd);  
    memcpy_safe(booster.booster_heat_max);  
    memcpy_safe(booster.booster_now_heat);  
    memcpy_safe(booster.launch_speed);
    HAL::CAN::Frame frame1 = {};
    frame1.dlc = 8;
    frame1.is_extended_id = false;
    frame1.is_remote_frame = false;
    frame1.id = CAN_C2G_FRAME1_ID;
    std::memcpy(frame1.data, tx_payload, frame1.dlc);

    HAL::CAN::Frame frame2 = {};
    frame2.dlc = 8;
    frame2.is_extended_id = false;
    frame2.is_remote_frame = false;
    frame2.id = CAN_C2G_FRAME2_ID;
    std::memcpy(frame2.data, tx_payload + 8, sizeof(tx_payload) - 8);

    auto& can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can2.send(frame1);
    can2.send(frame2);

}

}; // namespace Communicat
