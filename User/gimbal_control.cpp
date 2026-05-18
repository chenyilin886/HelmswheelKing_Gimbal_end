#include "gimbal_control.hpp"
#include "gimbal_c_api.h"
#include "cmsis_os.h"
#include <cmath>
#include "vofa.h"

BSP::Motor::DM::J4310<2> dm_motor(0x00, {6, 8}, {4, 7});
BSP::REMOTE_CONTROL::RemoteController dr16;
Comm::GimbalToChassis g2c;
BSP::IMU::HI12_float imu;
ALG::LADRC::LADRC pitch_adrc(200.0f, 6.0f, 0.0f, 35.0f, 20.0f, 0.001f, 3.5f);  

ALG::LADRC::LADRC yaw_adrc(200.0f, 10.0f, 0.0f, 35.0f, 22.0f, 0.001f, 3.5f);
ALG::PID::PID yaw_angle_pid(8.0f, 0.02f, 0.3f, 8.0f, 2.0f, 0.04f);

GimbalMode gimbal_mode = GimbalMode::ANGLE;
ALG::LADRC::LADRC yaw_vel_adrc(200.0f, 5.0f, 0.0f, 35.0f, 22.0f, 0.001f, 3.5f);
ALG::LADRC::LADRC pitch_angle_adrc(200.0f, 4.0f, 0.0f, 35.0f, 20.0f, 0.001f, 3.5f);
ALG::PID::PID pitch_angle_pid(4.5f, 0.0f, 1.2f, 5.0f, 1.5f, 0.05f);
Alg::Feedforward::Gravity pitch_gravity_ff(1.1f, 0.0f);

float target_yaw_rad = 0.0f;
float target_pitch_deg = 0.0f;
GimbalDebugData gimbal_debug = {};

static float pitch_offset = 0.0f;
static bool encoder_initialized = false;
static bool imu_initialized = false;
static bool motors_enabled = false;
static uint8_t enable_step = 0;
static uint8_t enable_wait = 0;
static uint8_t yaw_runaway_counter = 0;
static uint8_t pitch_runaway_counter = 0;
static bool yaw_runaway_flag = false;
static bool pitch_runaway_flag = false;
static uint16_t connect_wait_counter = 0;
static constexpr uint16_t CONNECT_WAIT_MAX = 150;
static uint8_t dbus_rx_buffer[Gimbal::DBUS_BUF_SIZE];
static uint8_t imu_rx_buffer[Gimbal::IMU_BUF_SIZE];

static float Clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static constexpr float PI_F = 3.14159265f;

static float NormalizeAngle(float angle)
{
    while (angle > PI_F) angle -= 2.0f * PI_F;
    while (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

static void StartMotorEnable()
{
    enable_step = 1;
    enable_wait = 0;
}

static bool ProcessMotorEnable()
{
    if (enable_step == 0) return false;

    switch (enable_step)
    {
        case 1:
            dm_motor.ClearErr(Gimbal::PITCH_ID, BSP::Motor::DM::Model::MIT);
            enable_wait = 0;
            enable_step = 2;
            break;
        case 2:
            if (++enable_wait >= 4)
            {
                dm_motor.ClearErr(Gimbal::YAW_ID, BSP::Motor::DM::Model::MIT);
                enable_wait = 0;
                enable_step = 3;
            }
            break;
        case 3:
            if (++enable_wait >= 4)
            {
                dm_motor.On(Gimbal::PITCH_ID, BSP::Motor::DM::Model::MIT);
                enable_wait = 0;
                enable_step = 4;
            }
            break;
        case 4:
            if (++enable_wait >= 4)
            {
                dm_motor.On(Gimbal::YAW_ID, BSP::Motor::DM::Model::MIT);
                enable_wait = 0;
                enable_step = 5;
            }
            break;
        case 5:
            if (++enable_wait >= 4)
            {
                enable_step = 0;
                return true;
            }
            break;
        default:
            enable_step = 0;
            break;
    }
    return false;
}

namespace Gimbal
{
    GimbalConfig_t gimbal_cfg = {
        4.0f,    // yaw_vel_scale (rad/s per unit joystick)
        1.2f,    // pitch_vel_scale (rad/s per unit joystick)
        0.3f,    // pitch_angle_scale (deg per loop, for angle tracking)
        1.15f    // gravity_comp
    };

void Init()
{
}

void ParseCANFrame(const HAL::CAN::Frame &frame)
{
    dm_motor.Parse(frame);
}

void StartUARTReceive()
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    HAL::UART::Data data{dbus_rx_buffer, DBUS_BUF_SIZE};
    uart3.receive_dma_idle(data);
    __HAL_DMA_DISABLE_IT(uart3.get_handle()->hdmarx, DMA_IT_HT);
}

void StartIMUReceive()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    HAL::UART::Data data{imu_rx_buffer, IMU_BUF_SIZE};
    uart1.receive_dma_idle(data);
    __HAL_DMA_DISABLE_IT(uart1.get_handle()->hdmarx, DMA_IT_HT);
}

void EnableMotors()
{
    StartMotorEnable();
}

void DisableMotors()
{
    dm_motor.Off(YAW_ID, BSP::Motor::DM::Model::MIT);
    dm_motor.Off(PITCH_ID, BSP::Motor::DM::Model::MIT);
    motors_enabled = false;
    enable_step = 0;
    encoder_initialized = false;
    connect_wait_counter = 0;
    yaw_adrc.reset();
    pitch_adrc.reset();
    yaw_angle_pid.reset();
    pitch_angle_pid.reset();
    yaw_vel_adrc.reset();
    pitch_angle_adrc.reset();
    yaw_runaway_counter = 0;
    pitch_runaway_counter = 0;
    yaw_runaway_flag = false;
    pitch_runaway_flag = false;
}

void Update()
{
    // ========== Emergency Stop: Highest Priority ==========
    if (dr16.get_s1() == BSP::REMOTE_CONTROL::RemoteController::DOWN &&
        dr16.get_s2() == BSP::REMOTE_CONTROL::RemoteController::DOWN)
    {
        if (motors_enabled || enable_step != 0)
        {
            DisableMotors();
        }
        return;
    }

    if (enable_step != 0)
    {
        if (ProcessMotorEnable())
            motors_enabled = true;
    }

    if (!encoder_initialized)
    {
        if (dm_motor.isConnected(PITCH_ID, 100)
            && dm_motor.isConnected(YAW_ID, 100))
        {
            pitch_offset = dm_motor.getAngleRad(PITCH_ID);
            if (imu_initialized)
            {
                target_yaw_rad = imu.GetAngle(2) * DEG_TO_RAD;
                target_pitch_deg = imu.GetAngle(1);
                target_pitch_deg = Clamp(target_pitch_deg, PITCH_ANGLE_MIN_DEG, PITCH_ANGLE_MAX_DEG);
            }
            else
            {
                target_pitch_deg = (PITCH_ANGLE_MIN_DEG + PITCH_ANGLE_MAX_DEG) / 2.0f;
            }
            encoder_initialized = true;
            connect_wait_counter = 0;
        }
        else if (enable_step == 0)
        {
            if (motors_enabled && connect_wait_counter < CONNECT_WAIT_MAX)
            {
                connect_wait_counter++;
            }
            else
            {
                StartMotorEnable();
                connect_wait_counter = 0;
            }
        }
        return;
    }

    if (!motors_enabled && enable_step == 0)
    {
        StartMotorEnable();
    }

    if (!motors_enabled)
        return;

    // ========== Remote Controller Safety Check ==========
    if (!dr16.isConnected())
    {
        dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        yaw_adrc.reset();
        pitch_adrc.reset();
        yaw_angle_pid.reset();
        pitch_angle_pid.reset();
        yaw_vel_adrc.reset();
        pitch_angle_adrc.reset();
        return;
    }

    // ========== IMU Safety Check ==========
    bool imu_ok = imu.isConnected() && imu_initialized;
    if (!imu_ok)
    {
        dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        yaw_adrc.reset();
        pitch_adrc.reset();
        yaw_angle_pid.reset();
        pitch_angle_pid.reset();
        yaw_vel_adrc.reset();
        pitch_angle_adrc.reset();
        return;
    }

    // ========== Motor Safety Check ==========
    bool yaw_motor_ok = dm_motor.isConnected(YAW_ID, 8);
    bool pitch_motor_ok = dm_motor.isConnected(PITCH_ID, 6);

    if (!yaw_motor_ok)
    {
        dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        yaw_adrc.reset();
        yaw_angle_pid.reset();
        yaw_vel_adrc.reset();
    }
    if (!pitch_motor_ok)
    {
        dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        pitch_adrc.reset();
        pitch_angle_pid.reset();
        pitch_angle_adrc.reset();
    }
    if (!yaw_motor_ok && !pitch_motor_ok)
    {
        return;
    }

    // ========== IMU Data Sanity Check ==========
    float cur_yaw_rad = imu.GetAngle(2) * DEG_TO_RAD;
    float cur_yaw_vel = imu.GetGyro(2) * DEG_TO_RAD;
    float pitch_angle_deg = imu.GetAngle(1);
    float cur_pitch_vel = imu.GetGyro(1) * DEG_TO_RAD;

    if (std::isnan(cur_yaw_rad) || std::isnan(cur_yaw_vel) ||
        std::isnan(pitch_angle_deg) || std::isnan(cur_pitch_vel))
    {
        dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        yaw_adrc.reset();
        pitch_adrc.reset();
        yaw_angle_pid.reset();
        pitch_angle_pid.reset();
        yaw_vel_adrc.reset();
        pitch_angle_adrc.reset();
        return;
    }

    // ========== Gimbal Mode Switching ==========
    GimbalMode new_gimbal_mode;
    uint8_t s1 = dr16.get_s1();
    if (s1 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
        new_gimbal_mode = GimbalMode::VELOCITY;
    else
        new_gimbal_mode = GimbalMode::ANGLE;

    if (new_gimbal_mode != gimbal_mode)
    {
        if (new_gimbal_mode == GimbalMode::VELOCITY)
        {
            yaw_vel_adrc.reset();
            pitch_adrc.reset();
        }
        else
        {
            target_yaw_rad = imu.GetAngle(2) * DEG_TO_RAD;
            yaw_angle_pid.reset();
            yaw_adrc.reset();
            target_pitch_deg = pitch_angle_deg;
            pitch_angle_pid.reset();
            pitch_angle_adrc.reset();
        }
        gimbal_mode = new_gimbal_mode;
    }

    // ========== Target Generation ==========

    float yaw_vel_target = 0.0f;
    float yaw_angle_err = 0.0f;
    float pitch_vel_target = 0.0f;
    float pitch_angle_err = 0.0f;

    if (gimbal_mode == GimbalMode::ANGLE)
    {
        // Yaw: joystick -> angle target -> angle PID -> velocity target
        float raw_yaw_input = dr16.get_right_x();
        if (std::fabs(raw_yaw_input) > 0.02f)
        {
            target_yaw_rad -= raw_yaw_input * gimbal_cfg.yaw_vel_scale * 0.001f;
            target_yaw_rad = NormalizeAngle(target_yaw_rad);
        }
        yaw_angle_err = NormalizeAngle(target_yaw_rad - cur_yaw_rad);
        yaw_vel_target = yaw_angle_pid.UpDate(yaw_angle_err, 0.0f);

        // Pitch: joystick -> angle target -> angle PID -> velocity target
        float raw_pitch_input = dr16.get_right_y();
        if (std::fabs(raw_pitch_input) > 0.02f)
        {
            target_pitch_deg += raw_pitch_input * gimbal_cfg.pitch_angle_scale;
            target_pitch_deg = Clamp(target_pitch_deg, PITCH_ANGLE_MIN_DEG, PITCH_ANGLE_MAX_DEG);
        }
        pitch_angle_err = (target_pitch_deg - pitch_angle_deg) * DEG_TO_RAD;
        pitch_vel_target = pitch_angle_pid.UpDate(pitch_angle_err, 0.0f);
    }
    else
    {
        // Yaw: joystick -> velocity target (direct mapping)
        yaw_vel_target = dr16.get_right_x() * gimbal_cfg.yaw_vel_scale;

        // Pitch: joystick -> velocity target (direct mapping)
        pitch_vel_target = dr16.get_right_y() * gimbal_cfg.pitch_vel_scale;

        // Pitch angle-based velocity limiting (safety)
        if (pitch_angle_deg >= PITCH_ANGLE_MAX_DEG && pitch_vel_target > 0.0f)
            pitch_vel_target = 0.0f;
        if (pitch_angle_deg <= PITCH_ANGLE_MIN_DEG && pitch_vel_target < 0.0f)
            pitch_vel_target = 0.0f;
    }

    // ========== Yaw: ADRC Speed Loop ==========
    float yaw_torque = 0.0f;

    if (yaw_motor_ok)
    {
        if (std::fabs(cur_yaw_vel) > YAW_VEL_LIMIT_RAD)
        {
            yaw_runaway_counter++;
            if (yaw_runaway_counter >= RUNAWAY_COUNT_THRESHOLD)
            {
                yaw_runaway_flag = true;
            }
        }
        else
        {
            if (yaw_runaway_counter > 0)
                yaw_runaway_counter--;
            if (std::fabs(cur_yaw_vel) < YAW_VEL_LIMIT_RAD * 0.5f)
                yaw_runaway_flag = false;
        }

        if (!yaw_runaway_flag)
        {
            if (gimbal_mode == GimbalMode::ANGLE)
            {
                yaw_adrc.setTarget(yaw_vel_target);
                yaw_torque = -yaw_adrc.update(cur_yaw_vel);
            }
            else
            {
                yaw_vel_adrc.setTarget(yaw_vel_target);
                yaw_torque = -yaw_vel_adrc.update(cur_yaw_vel);
            }
            yaw_torque = Clamp(yaw_torque, -3.0f, 3.0f);
        }
        else
        {
            yaw_adrc.reset();
            yaw_vel_adrc.reset();
        }
    }
    dm_motor.ctrl_Mit(YAW_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_torque);

    // ========== Pitch: ADRC Speed Loop + Gravity Feedforward ==========
    float pitch_torque = 0.0f;

    if (pitch_motor_ok)
    {
        if (std::fabs(cur_pitch_vel) > PITCH_VEL_LIMIT_RAD)
        {
            pitch_runaway_counter++;
            if (pitch_runaway_counter >= RUNAWAY_COUNT_THRESHOLD)
            {
                pitch_runaway_flag = true;
            }
        }
        else
        {
            if (pitch_runaway_counter > 0)
                pitch_runaway_counter--;
            if (std::fabs(cur_pitch_vel) < PITCH_VEL_LIMIT_RAD * 0.5f)
                pitch_runaway_flag = false;
        }

        if (!pitch_runaway_flag)
        {
            if (gimbal_mode == GimbalMode::ANGLE)
            {
                pitch_angle_adrc.setTarget(pitch_vel_target);
                pitch_torque = pitch_angle_adrc.update(cur_pitch_vel);
            }
            else
            {
                pitch_adrc.setTarget(pitch_vel_target);
                pitch_torque = pitch_adrc.update(cur_pitch_vel);
            }
            pitch_gravity_ff.GravityFeedforward(pitch_angle_deg);
            pitch_torque += pitch_gravity_ff.getFeedforward();
            pitch_torque = Clamp(pitch_torque, -3.0f, 3.0f);
        }
        else
        {
            pitch_adrc.reset();
            pitch_angle_adrc.reset();
        }
    }
    dm_motor.ctrl_Mit(PITCH_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_torque);

    // ========== Debug Data ==========
    gimbal_debug.yaw_vel_target = yaw_vel_target;
    gimbal_debug.yaw_vel_filtered = (gimbal_mode == GimbalMode::ANGLE) ? yaw_adrc.getTD_X1() : yaw_vel_adrc.getTD_X1();
    gimbal_debug.yaw_vel_feedback = cur_yaw_vel;
    gimbal_debug.yaw_torque = yaw_torque;
    gimbal_debug.yaw_angle_target = target_yaw_rad;
    gimbal_debug.yaw_angle_feedback = cur_yaw_rad;
    gimbal_debug.yaw_angle_err = yaw_angle_err;
    gimbal_debug.pitch_vel_target = pitch_vel_target;
    gimbal_debug.pitch_vel_filtered = (gimbal_mode == GimbalMode::ANGLE) ? pitch_angle_adrc.getTD_X1() : pitch_adrc.getTD_X1();
    gimbal_debug.pitch_vel_feedback = cur_pitch_vel;
    gimbal_debug.pitch_torque = pitch_torque;
    gimbal_debug.pitch_angle_deg = pitch_angle_deg;
    gimbal_debug.pitch_angle_target = target_pitch_deg;
    gimbal_debug.pitch_angle_err = pitch_angle_err;
    gimbal_debug.gravity_ff = pitch_gravity_ff.getFeedforward();
    gimbal_debug.yaw_connected = dm_motor.isConnected(YAW_ID, 8) ? 1 : 0;
    gimbal_debug.pitch_connected = dm_motor.isConnected(PITCH_ID, 6) ? 1 : 0;
    gimbal_debug.imu_connected = imu.isConnected() ? 1 : 0;
    gimbal_debug.yaw_runaway = yaw_runaway_flag ? 1 : 0;
    gimbal_debug.pitch_runaway = pitch_runaway_flag ? 1 : 0;
    gimbal_debug.gimbal_mode = (gimbal_mode == GimbalMode::ANGLE) ? 0 : 1;

    vofa_send(pitch_angle_deg, target_pitch_deg, pitch_vel_target, 
              cur_pitch_vel, pitch_torque, gimbal_debug.gravity_ff);
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
    else if (huart->Instance == USART1)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

        if (size <= IMU_BUF_SIZE)
        {
            imu.DataUpdate(imu_rx_buffer);
            imu_initialized = true;
        }

        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        HAL::UART::Data data{imu_rx_buffer, IMU_BUF_SIZE};
        uart1.receive_dma_idle(data);
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
extern "C" void Gimbal_StartIMUReceive(void)     { Gimbal::StartIMUReceive(); }
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
