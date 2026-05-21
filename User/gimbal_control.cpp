#include "gimbal_control.hpp"
#include "gimbal_c_api.h"
#include "cmsis_os.h"
#include <cmath>
#include "vofa.h"

static GimbalDriver driver_;
static GimbalAxis yaw_axis_;
static GimbalAxis pitch_axis_;
static GimbalTarget target_gen_;
static GimbalMode gimbal_mode = GimbalMode::ANGLE;

GimbalDebugData gimbal_debug = {};
GimbalTuneData gimbal_tune = {};
Comm::Vision::DebugData vision_debug = {};

static void InitAxes()
{
    gimbal_tune.yaw = {
        .vel_kp = 5.0f, .vel_kd = 0.0f, .vel_wc = 35.0f, .vel_b0 = 22.0f, .vel_max = 3.5f,
        .ang_kp = 10.0f, .ang_kd = 0.0f, .ang_wc = 35.0f, .ang_b0 = 20.0f, .ang_max = 3.5f,
        .pid_kp = 20.0f, .pid_ki = 0.02f, .pid_kd = 0.3f,
        .pid_max = 8.0f, .pid_ilimit = 2.0f, .pid_isep = 0.04f,//积分分离阈值
        .td_r = 80.0f,//跟踪微分器快速性
        .gravity_k = 0.0f, .gravity_phi = 0.0f,
        .vel_limit = 5.0f,
        .runaway_thresh = 10,//飞车判定次数
        .torque_limit = 3.0f,//扭矩限幅
        .torque_sign = -1.0f,//扭矩方向
        .angle_max_deg = 20.0f,
        .angle_min_deg = -20.0f,
    };
    gimbal_tune.pitch = {
        .vel_kp = 80.0f, .vel_kd = 0.0f, .vel_wc = 40.0f, .vel_b0 = 15.0f, .vel_max = 0.53f,
        .ang_kp = 8.0f, .ang_kd = 0.0f, .ang_wc = 38.0f, .ang_b0 = 6.0f, .ang_max = 0.5f,
        .pid_kp = 10.0f, .pid_ki = 0.0f, .pid_kd = 80.0f,
        .pid_max = 0.8f, .pid_ilimit = 1.0f, .pid_isep = 0.1f,
        .td_r = 80.0f,
        .gravity_k = 0.95f, .gravity_phi = 0.0f,
        .vel_limit = 4.0f,
        .runaway_thresh = 10,
        .torque_limit = 3.0f,
        .torque_sign = 1.0f,
        .angle_max_deg = 20.0f,
        .angle_min_deg = -15.0f,
    };

    gimbal_tune.yaw_vel_scale = 4.0f;
    gimbal_tune.pitch_vel_scale = 3.0f;
    gimbal_tune.pitch_angle_scale = 0.3f;

    yaw_axis_.init({
        .tune = gimbal_tune.yaw,
    });

    pitch_axis_.init({
        .tune = gimbal_tune.pitch,
        .use_td_filter = true,
        .use_gravity_ff = true,
    });

    target_gen_.init({});
}

namespace Gimbal
{

void Init()
{
    InitAxes();
    driver_.init();
}

void ParseCANFrame(const HAL::CAN::Frame &frame)
{
    driver_.parseCANFrame(frame);
}

void StartUARTReceive()
{
    driver_.startUARTReceive();
}

void StartIMUReceive()
{
    driver_.startIMUReceive();
}

void StartVisionReceive()
{
    driver_.init();
}

void EnableMotors()
{
    driver_.enableMotors();
}

void DisableMotors()
{
    driver_.disableMotors();
    yaw_axis_.resetControllers();
    pitch_axis_.resetControllers();
}

static GimbalMode getGimbalMode()
{
    uint8_t s1 = driver_.getRC().get_s1();
    uint8_t s2 = driver_.getRC().get_s2();

    if (s2 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE && Comm::vision.getVisionFlag())
        return GimbalMode::VISION;
    if (s1 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
        return GimbalMode::VELOCITY;
    return GimbalMode::ANGLE;
}

static void handleModeSwitch(GimbalMode new_mode, float cur_yaw_rad, float cur_pitch_deg)
{
    if (new_mode == gimbal_mode) return;

    if (new_mode == GimbalMode::VISION)
    {
        target_gen_.resetTargets(cur_yaw_rad, cur_pitch_deg);
        yaw_axis_.resetControllers();
        pitch_axis_.resetControllers();
    }
    else if (new_mode == GimbalMode::VELOCITY)
    {
        yaw_axis_.resetControllers();
        pitch_axis_.resetControllers();
    }
    else
    {
        target_gen_.resetTargets(cur_yaw_rad, cur_pitch_deg);
        yaw_axis_.resetControllers();
        pitch_axis_.resetControllers();
    }
    gimbal_mode = new_mode;
}

static void resetAllControllers()
{
    yaw_axis_.resetControllers();
    pitch_axis_.resetControllers();
}

void Update()
{
    if (driver_.getRC().get_s1() == BSP::REMOTE_CONTROL::RemoteController::DOWN &&
        driver_.getRC().get_s2() == BSP::REMOTE_CONTROL::RemoteController::DOWN)
    {
        if (driver_.isMotorsEnabled() || driver_.isEnableInProgress())
            DisableMotors();
        return;
    }

    if (driver_.isEnableInProgress())
    {
        if (driver_.processEnable())
            driver_.setMotorsEnabled(true);
    }

    if (!driver_.processEncoderInit())
        return;

    if (!driver_.isMotorsEnabled() && !driver_.isEnableInProgress())
    {
        driver_.enableMotors();
    }

    if (!driver_.isMotorsEnabled())
        return;

    if (!driver_.isRcConnected())
    {
        driver_.sendZeroTorque(GimbalDriver::YAW_ID);
        driver_.sendZeroTorque(GimbalDriver::PITCH_ID);
        resetAllControllers();
        return;
    }

    bool imu_ok = driver_.isImuConnected() && driver_.isImuInitialized();
    if (!imu_ok)
    {
        driver_.sendZeroTorque(GimbalDriver::YAW_ID);
        driver_.sendZeroTorque(GimbalDriver::PITCH_ID);
        resetAllControllers();
        return;
    }

    bool yaw_motor_ok = driver_.isMotorConnected(GimbalDriver::YAW_ID, 8);
    bool pitch_motor_ok = driver_.isMotorConnected(GimbalDriver::PITCH_ID, 6);

    if (!yaw_motor_ok)
    {
        driver_.sendZeroTorque(GimbalDriver::YAW_ID);
        yaw_axis_.resetControllers();
    }
    if (!pitch_motor_ok)
    {
        driver_.sendZeroTorque(GimbalDriver::PITCH_ID);
        pitch_axis_.resetControllers();
    }
    if (!yaw_motor_ok && !pitch_motor_ok)
        return;

    float cur_yaw_rad = driver_.getIMU().GetAngle(2) * DEG_TO_RAD;
    float cur_yaw_vel = driver_.getIMU().GetGyro(2) * DEG_TO_RAD;
    float pitch_angle_deg = driver_.getIMU().GetAngle(1);
    float cur_pitch_vel = driver_.getIMU().GetGyro(1) * DEG_TO_RAD;

    if (std::isnan(cur_yaw_rad) || std::isnan(cur_yaw_vel) ||
        std::isnan(pitch_angle_deg) || std::isnan(cur_pitch_vel))
    {
        driver_.sendZeroTorque(GimbalDriver::YAW_ID);
        driver_.sendZeroTorque(GimbalDriver::PITCH_ID);
        resetAllControllers();
        return;
    }

    GimbalMode new_mode = getGimbalMode();
    handleModeSwitch(new_mode, cur_yaw_rad, pitch_angle_deg);

    yaw_axis_.applyTuneParams(gimbal_tune.yaw);
    pitch_axis_.applyTuneParams(gimbal_tune.pitch);
    target_gen_.applyScaleParams(gimbal_tune.yaw_vel_scale, gimbal_tune.pitch_vel_scale, gimbal_tune.pitch_angle_scale);

    auto target = target_gen_.update(gimbal_mode, cur_yaw_rad, pitch_angle_deg,
                                     yaw_axis_, pitch_axis_,
                                     driver_.getRC(), Comm::vision);

    if (gimbal_mode == GimbalMode::VELOCITY)
    {
        target.pitch_vel_target = pitch_axis_.applyAngleSafetyLimit(
            target.pitch_vel_target, pitch_angle_deg);
    }

    float yaw_torque = 0.0f;
    if (yaw_motor_ok)
    {
        yaw_axis_.checkRunaway(cur_yaw_vel);
        yaw_torque = yaw_axis_.computeTorque(gimbal_mode, target.yaw_vel_target, cur_yaw_vel);
    }
    // 发送yaw角度目标
    driver_.sendTorque(GimbalDriver::YAW_ID, yaw_torque);

    float pitch_torque = 0.0f;
    if (pitch_motor_ok)
    {
        pitch_axis_.checkRunaway(cur_pitch_vel);
        pitch_torque = pitch_axis_.computeTorque(gimbal_mode, target.pitch_vel_target,
                                                  cur_pitch_vel, pitch_angle_deg);
    }
    // 发送pitch角度目标
    driver_.sendTorque(GimbalDriver::PITCH_ID, pitch_torque);

    gimbal_debug.yaw_vel_target = target.yaw_vel_target;
    gimbal_debug.yaw_vel_filtered = yaw_axis_.getVelFiltered(gimbal_mode);
    gimbal_debug.yaw_vel_feedback = cur_yaw_vel;
    gimbal_debug.yaw_torque = yaw_torque;
    gimbal_debug.yaw_angle_target = target_gen_.getTargetYawRad();
    gimbal_debug.yaw_angle_feedback = cur_yaw_rad;
    gimbal_debug.yaw_angle_err = target.yaw_angle_err;
    gimbal_debug.pitch_vel_target = target.pitch_vel_target;
    gimbal_debug.pitch_vel_filtered = pitch_axis_.getVelFiltered(gimbal_mode);
    gimbal_debug.pitch_vel_feedback = cur_pitch_vel;
    gimbal_debug.pitch_torque = pitch_torque;
    gimbal_debug.pitch_angle_deg = pitch_angle_deg;
    gimbal_debug.pitch_angle_target = target_gen_.getTargetPitchDeg();
    gimbal_debug.pitch_angle_err = target.pitch_angle_err;
    gimbal_debug.gravity_ff = pitch_axis_.getGravityFF();
    gimbal_debug.yaw_connected = driver_.isMotorConnected(GimbalDriver::YAW_ID, 8) ? 1 : 0;
    gimbal_debug.pitch_connected = driver_.isMotorConnected(GimbalDriver::PITCH_ID, 6) ? 1 : 0;
    gimbal_debug.imu_connected = driver_.isImuConnected() ? 1 : 0;
    gimbal_debug.yaw_runaway = yaw_axis_.isRunaway() ? 1 : 0;
    gimbal_debug.pitch_runaway = pitch_axis_.isRunaway() ? 1 : 0;
    gimbal_debug.gimbal_mode = (gimbal_mode == GimbalMode::ANGLE) ? 0 : 1;

    vofa_send(cur_yaw_rad, target_gen_.getTargetYawRad(), target.pitch_vel_target,
              cur_pitch_vel, pitch_torque, gimbal_debug.gravity_ff);

    Comm::vision.send(driver_.getIMU().GetQuaternion(0),
                       driver_.getIMU().GetQuaternion(1),
                       driver_.getIMU().GetQuaternion(2),
                       driver_.getIMU().GetQuaternion(3));
    vision_debug = Comm::vision.getDebugData();
}

void SendToChassis()
{
    float left_x = 0.0f;
    float left_y = 0.0f;
    float yaw_angle_err = 0.0f;
    Comm::ChassisMode mode{};

    if (!driver_.isRcConnected() || !driver_.isEncoderInitialized())
    {
        mode.stop = 1;
    }
    else
    {
        left_x = driver_.getRC().get_left_x();
        left_y = driver_.getRC().get_left_y();
        yaw_angle_err = CalcuGimbalToChassisAngle();

        uint8_t s1 = driver_.getRC().get_s1();
        uint8_t s2 = driver_.getRC().get_s2();

        if (s1 == BSP::REMOTE_CONTROL::RemoteController::DOWN
         && s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
            mode.stop = 1;
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::UP)
            mode.rotating = 1;
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
        {
        }
        else if (s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
            mode.stop = 1;
        else
            mode.stop = 1;
    }

    driver_.sendToChassis(left_x, left_y, yaw_angle_err, mode);
}

void ProcessUARTRx(UART_HandleTypeDef *huart, uint16_t size)
{
    driver_.processUARTRx(huart, size);
}

void ProcessUARTRxCplt(UART_HandleTypeDef *huart)
{
    driver_.processUARTRxCplt(huart);
}

void ProcessCANRx(CAN_HandleTypeDef *hcan)
{
    driver_.processCANRx(hcan);
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

static float ZeroCrossingProcessing(float expectations, float feedback, float maxpos)
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
    float encoder_angle = driver_.getMotor().getAngle0_360(GimbalDriver::YAW_ID);
    return ZeroCrossingProcessing(77.0f, encoder_angle, 360.0f) - encoder_angle;
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
extern "C" void Gimbal_ProcessUARTRxCplt(void *huart) { Gimbal::ProcessUARTRxCplt((UART_HandleTypeDef *)huart); }
extern "C" void Gimbal_ProcessCANRx(void *hcan)                  { Gimbal::ProcessCANRx((CAN_HandleTypeDef *)hcan); }
extern "C" void Gimbal_ProcessCANFifo1(void *hcan)
{
    Gimbal::ProcessCANFifo1((CAN_HandleTypeDef *)hcan);
}

extern "C" uint8_t* Gimbal_GetVisionRxBuffer(void)
{
    return Comm::vision.getRxBuffer();
}

extern "C" uint8_t Gimbal_GetVisionRxSize(void)
{
    return Comm::Vision::getRxSize();
}
