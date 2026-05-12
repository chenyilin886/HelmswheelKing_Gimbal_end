#include "gimbal_control.hpp"
#include "gimbal_c_api.h"
#include "cmsis_os.h"
#include <cmath>

BSP::Motor::DM::J4310<2> dm_motor(0x00, {6, 8}, {4, 7});
BSP::REMOTE_CONTROL::RemoteController dr16;
Comm::GimbalToChassis g2c;

ALG::PID::PID yaw_speed_pid(1.0f, 0.0f, 0.05f, 3.0f, 1.0f, 5.0f);
ALG::PID::PID pitch_angle_pid(20.0f, 0.01f, 0.0f, 30.0f, 5.0f, 3.0f);
ALG::PID::PID pitch_speed_pid(1.0f, 0.01f, 0.00f, 3.0f, 1.0f, 5.0f);

float target_pitch_rad = 0.0f;
GimbalDebugData gimbal_debug = {};

static float pitch_offset = 0.0f;
static bool encoder_initialized = false;
static bool motors_enabled = false;
static uint8_t dbus_rx_buffer[Gimbal::DBUS_BUF_SIZE];

static float Clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
namespace Gimbal
{
    GimbalConfig_t gimbal_cfg = {
        0.6f,   // pitch_angle_max
        -0.5f,  // pitch_angle_min
        4.0f,   // yaw_vel_scale 
        0.003f  // pitch_sensitivity
    };

void Init()
{
}

void ParseCANFrame(const HAL::CAN::Frame &frame)
{
    dm_motor.Parse(frame);

    if (!encoder_initialized
        && dm_motor.isConnected(PITCH_ID, 100)
        && dm_motor.isConnected(YAW_ID, 100))
    {
        pitch_offset = dm_motor.getAngleRad(PITCH_ID);
        target_pitch_rad = 0.0f;
        encoder_initialized = true;
    }
}

void StartUARTReceive()
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    HAL::UART::Data data{dbus_rx_buffer, DBUS_BUF_SIZE};
    uart3.receive_dma_idle(data);
    __HAL_DMA_DISABLE_IT(uart3.get_handle()->hdmarx, DMA_IT_HT);
}

void EnableMotors()
{
    osDelay(100);
    dm_motor.ClearErr(PITCH_ID, BSP::Motor::DM::Model::MIT);
    osDelay(4);
    dm_motor.ClearErr(YAW_ID, BSP::Motor::DM::Model::MIT);
    osDelay(10);
    dm_motor.On(PITCH_ID, BSP::Motor::DM::Model::MIT);
    osDelay(4);
    dm_motor.On(YAW_ID, BSP::Motor::DM::Model::MIT);
    osDelay(10);
    motors_enabled = true;
}

void DisableMotors()
{
    dm_motor.Off(YAW_ID, BSP::Motor::DM::Model::MIT);
    dm_motor.Off(PITCH_ID, BSP::Motor::DM::Model::MIT);
    motors_enabled = false;
}

void Update()
{
    // 1. 编码器未就绪时，不允许控制
    if (!encoder_initialized)
        return;

    uint8_t s1 = dr16.get_s1();
    uint8_t s2 = dr16.get_s2();

    // 2. 【最高优先级】检测急停与唤醒
    if (s1 == BSP::REMOTE_CONTROL::RemoteController::DOWN && 
        s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
    {
        if (motors_enabled) {
            DisableMotors(); // 触发急停
        }
        return; // 急停状态下，不进行PID计算
    }
    else 
    {
        if (!motors_enabled) {
            EnableMotors(); // 如果拨杆恢复正常，重新唤醒电机！
        }
    }

    // 3. 如果还没使能成功（比如正在执行 EnableMotors 的 delay），先跳过本次控制
    if (!motors_enabled)
        return;

    float yaw_speed_target = dr16.get_right_x() * gimbal_cfg.yaw_vel_scale;
    float yaw_speed = dm_motor.getVelocityRads(YAW_ID);
    float yaw_torque = yaw_speed_pid.UpDate(yaw_speed_target, yaw_speed);
    yaw_torque = Clamp(yaw_torque, -3.0f, 3.0f);
    dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_torque);

    target_pitch_rad += dr16.get_right_y() * gimbal_cfg.pitch_sensitivity;
    target_pitch_rad = Clamp(target_pitch_rad, gimbal_cfg.pitch_angle_min, gimbal_cfg.pitch_angle_max);

    float pitch_angle = dm_motor.getAngleRad(PITCH_ID) - pitch_offset;
    float pitch_speed_target = pitch_angle_pid.UpDate(target_pitch_rad, pitch_angle);
    float pitch_speed = dm_motor.getVelocityRads(PITCH_ID);
    float pitch_torque = pitch_speed_pid.UpDate(pitch_speed_target, pitch_speed);
    pitch_torque = Clamp(pitch_torque, -3.0f, 3.0f);
    dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_torque);

    gimbal_debug.yaw_speed_target = yaw_speed_target;
    gimbal_debug.yaw_speed = yaw_speed;
    gimbal_debug.yaw_torque = yaw_torque;
    gimbal_debug.pitch_angle = pitch_angle;
    gimbal_debug.pitch_speed = pitch_speed;
    gimbal_debug.pitch_torque = pitch_torque;
    gimbal_debug.yaw_connected = dm_motor.isConnected(YAW_ID, 8) ? 1 : 0;
    gimbal_debug.pitch_connected = dm_motor.isConnected(PITCH_ID, 6) ? 1 : 0;
}

void SendToChassis()
{
    float left_x = 0.0f;
    float left_y = 0.0f;
    float yaw_angle_err = 0.0f;
    Comm::ChassisMode mode{};

    if (!dr16.isConnected() || !encoder_initialized)
    {
        mode.stop = 1;
    }
    else
    {
        left_x = dr16.get_left_x();
        left_y = dr16.get_left_y();
        yaw_angle_err = CalcuGimbalToChassisAngle();

        uint8_t s1 = dr16.get_s1();
        uint8_t s2 = dr16.get_s2();

        if (s1 == BSP::REMOTE_CONTROL::RemoteController::DOWN
         && s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
        {
            mode.stop = 1;
        }
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::UP)
            mode.rotating = 1;
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
            mode.follow = 1;
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
            mode.stop = 1;
        else
            mode.stop = 1;
    }

    g2c.packFrame(left_x, left_y, yaw_angle_err, mode);

    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    HAL::CAN::Frame frame{};
    frame.id = Comm::CAN_G2C_FRAME_ID;
    frame.dlc = Comm::GimbalToChassis::size();
    frame.is_extended_id = false;
    frame.is_remote_frame = false;
    std::memcpy(frame.data, g2c.data(), Comm::GimbalToChassis::size());

    can2.send(frame);
}

void ProcessUARTRx(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART3)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

        if (size == DBUS_BUF_SIZE)
            dr16.parseData(dbus_rx_buffer);

        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        HAL::UART::Data data{dbus_rx_buffer, DBUS_BUF_SIZE};
        uart3.receive_dma_idle(data);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void ProcessCANRx(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL::CAN::ICanDevice &can1 = HAL::CAN::get_can_bus_instance().get_can1();
        HAL::CAN::Frame frame;
        while (can1.receive(frame)) {}
    }
}

void ProcessCANFifo1(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        HAL::CAN::Frame rx_frame;
        auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
        while (can2.receive(rx_frame)) {}
    }
}

float ZeroCrossingProcessing(float expectations, float feedback, float maxpos)
{
    float tempcin = expectations;
    if (maxpos != 0)
    {
        tempcin = (float)fmod(expectations, maxpos);
        if (tempcin < 0)
            tempcin += maxpos;
        if (tempcin - feedback < -maxpos / 2)
            tempcin += maxpos;
        if (tempcin - feedback > maxpos / 2)
            tempcin -= maxpos;
    }
    return tempcin;
}

float CalcuGimbalToChassisAngle()
{
    float encoder_angle = dm_motor.getAngle0_360(YAW_ID);
    return ZeroCrossingProcessing(static_cast<float>(YAW_INIT_ANGLE_DEG), encoder_angle, 360.0f) - encoder_angle;
}

}

extern "C" void Gimbal_Init(void)               { Gimbal::Init(); }
extern "C" void Gimbal_InitCANBus(void)          { HAL::CAN::get_can_bus_instance(); }
extern "C" void Gimbal_ParseCANFrame(void *frame) { Gimbal::ParseCANFrame(*static_cast<HAL::CAN::Frame*>(frame)); }
extern "C" void Gimbal_ProcessCANFifo0(void *hcan)
{
    if (static_cast<CAN_HandleTypeDef*>(hcan)->Instance == CAN1)
    {
        HAL::CAN::Frame rx_frame;
        auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        while (can1.receive(rx_frame))
        {
            Gimbal::ParseCANFrame(rx_frame);
        }
    }
}
extern "C" void Gimbal_StartUARTReceive(void)    { Gimbal::StartUARTReceive(); }
extern "C" void Gimbal_EnableMotors(void)        { Gimbal::EnableMotors(); }
extern "C" void Gimbal_DisableMotors(void)       { Gimbal::DisableMotors(); }
extern "C" void Gimbal_Update(void)              { Gimbal::Update(); }
extern "C" void Gimbal_SendToChassis(void)       { Gimbal::SendToChassis(); }
extern "C" void Gimbal_ProcessUARTRx(void *huart, uint16_t size) { Gimbal::ProcessUARTRx((UART_HandleTypeDef *)huart, size); }
extern "C" void Gimbal_ProcessCANRx(void *hcan)                  { Gimbal::ProcessCANRx((CAN_HandleTypeDef *)hcan); }
extern "C" void Gimbal_ProcessCANFifo1(void *hcan)
{
    Gimbal::ProcessCANFifo1((CAN_HandleTypeDef *)hcan);
}
