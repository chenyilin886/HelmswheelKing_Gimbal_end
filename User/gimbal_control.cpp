//视觉发送:Gimbal::Update() 的末尾
#include "gimbal_control.hpp"
#include "gimbal_c_api.h"
#include "cmsis_os.h"
#include <cmath>
#include "vofa.h"
#include <cstring>
#include "usbd_cdc_if.h"
//实例化
static GimbalDriver driver_;//底层驱动
static GimbalAxis yaw_axis_;//两个轴控制器
static GimbalAxis pitch_axis_;
static GimbalTarget target_gen_;//目标生成器
static GimbalMode gimbal_mode = GimbalMode::ANGLE;
//static，意味着只在这个文件内可见
GimbalDebugData gimbal_debug = {};
GimbalTuneData gimbal_tune = {};
Comm::Vision::DebugData vision_debug = {};

static void InitAxes()
{
    gimbal_tune.yaw = {
        .vel_kp = 6.5f, .vel_kd = 0.2f, .vel_wc = 35.0f, .vel_b0 = 22.0f, .vel_max = 3.5f,
        .ang_kp = 6.0f, .ang_kd = 0.0f, .ang_wc = 35.0f, .ang_b0 = 20.0f, .ang_max = 3.5f,
        .pid_kp = 10.0f, .pid_ki = 0.0f, .pid_kd = 0.1f,
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
        .vel_kp = 100.0f, .vel_kd = 0.0f, .vel_wc = 50.0f, .vel_b0 = 8.0f, .vel_max = 0.53f,
        .ang_kp = 10.0f, .ang_kd = 0.3f, .ang_wc = 40.0f, .ang_b0 = 8.0f, .ang_max = 0.5f,
        .pid_kp = 30.0f, .pid_ki = 0.2f, .pid_kd = 80.0f,
        .pid_max = 2.5f, .pid_ilimit = 1.0f, .pid_isep = 0.1f,
        .td_r = 80.0f,
        .gravity_k = 1.0f, .gravity_phi = 0.0f,
        .vel_limit = 2.0f,
        .runaway_thresh = 10,
        .torque_limit = 1.8f,
        .torque_sign = 1.0f,
        .angle_max_deg = 20.0f,
        .angle_min_deg = -15.0f,
    };

    gimbal_tune.yaw_vel_scale = 4.0f;
    gimbal_tune.pitch_vel_scale = 3.0f;
    gimbal_tune.pitch_angle_scale = 0.3f;
    //初始化
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
//对外公共接口（API）
namespace Gimbal
{
//启动与初始化
void Init()
{
    InitAxes();
    driver_.init();
}

//数据搬运工 (CAN 与 串口收发)
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
//视觉接收
void StartVisionReceive()
{
    driver_.init();
}
//电机启停与安全机制
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
        return GimbalMode::VISION;// 右拨杆居中且视觉连上：自瞄模式
    if (s1 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
        return GimbalMode::VELOCITY;// 左拨杆居中：速度模式
    else
    return GimbalMode::ANGLE;// 默认：角度打杆模式
}
\
static void handleModeSwitch(GimbalMode new_mode, float cur_yaw_rad, float cur_pitch_deg)
{
    if (new_mode == gimbal_mode) return;

    if (new_mode == GimbalMode::VISION)
    {   //把目标角度重置为当前的真实物理角度
        target_gen_.resetTargets(cur_yaw_rad, cur_pitch_deg);
        //清除底层的 PID 积分和 ADRC 历史数据
        yaw_axis_.resetControllers();
        pitch_axis_.resetControllers();
    }
    else if (new_mode == GimbalMode::VELOCITY)
    {   //把目标速度重置为0
        target_gen_.resetTargets(0.0f, 0.0f);
        //清除底层的 PID 积分和 ADRC 历史数据
        yaw_axis_.resetControllers();
        yaw_axis_.resetControllers();
        pitch_axis_.resetControllers();
    }
    else
    {
        //默认的 ANGLE 模式
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
    Gimbal_ProcessUSBFromTask();
    //双下急停与一大堆检测
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
    // 读陀螺仪当前角度和角速度
    float cur_yaw_rad = driver_.getIMU().GetAngle(2) * DEG_TO_RAD;
    float cur_yaw_vel = driver_.getIMU().GetGyro(2) * DEG_TO_RAD;
    float pitch_angle_deg = driver_.getIMU().GetAngle(1);//单位deg
    float cur_pitch_vel = driver_.getIMU().GetGyro(1) * DEG_TO_RAD;
    // 检查读出的数据是不是 NaN (Not a Number)，安全保护机制
    if (std::isnan(cur_yaw_rad) || std::isnan(cur_yaw_vel) ||
        std::isnan(pitch_angle_deg) || std::isnan(cur_pitch_vel))
    {
        driver_.sendZeroTorque(GimbalDriver::YAW_ID);
        driver_.sendZeroTorque(GimbalDriver::PITCH_ID);
        resetAllControllers();
        return;
    }
    // 获取并切换模式
    GimbalMode new_mode = getGimbalMode();
    handleModeSwitch(new_mode, cur_yaw_rad, pitch_angle_deg);
    // 应用新的参数
    yaw_axis_.applyTuneParams(gimbal_tune.yaw);
    pitch_axis_.applyTuneParams(gimbal_tune.pitch);
    target_gen_.applyScaleParams(gimbal_tune.yaw_vel_scale, gimbal_tune.pitch_vel_scale, gimbal_tune.pitch_angle_scale);
    // 把当前状态喂给目标生成器，更新目标角度
    auto target = target_gen_.update(gimbal_mode, cur_yaw_rad, pitch_angle_deg,
                                     yaw_axis_, pitch_axis_,
                                     driver_.getRC(), Comm::vision);

    if (gimbal_mode == GimbalMode::VELOCITY)
    {
        // 软限位
        target.pitch_vel_target = pitch_axis_.applyAngleSafetyLimit(
            target.pitch_vel_target, pitch_angle_deg);
    }

    float yaw_torque = 0.0f;
    if (yaw_motor_ok)
    {
        // 计算Yaw轴力矩 (经过防飞车检查 -> ADRC运算)
        yaw_axis_.checkRunaway(cur_yaw_vel);
        yaw_torque = yaw_axis_.computeTorque(gimbal_mode, target.yaw_vel_target, cur_yaw_vel);
    }
    // 发送给达妙电机
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
    
    // 发送Vofa数据
    vofa_send(cur_yaw_rad, target_gen_.getTargetYawRad(), target.pitch_vel_target,
              cur_pitch_vel, pitch_torque, gimbal_debug.gravity_ff);
    // 发送视觉数据
    Comm::vision.receive();
    Comm::vision.send(driver_.getIMU().GetQuaternion(0),
                       driver_.getIMU().GetQuaternion(1),
                       driver_.getIMU().GetQuaternion(2),
                       driver_.getIMU().GetQuaternion(3));
    vision_debug = Comm::vision.getDebugData();
}
//将云台数据发送给底盘
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
    // 发送给底盘
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
// 过零点处理算法
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
//计算云台相对于底盘的相对偏航角差值，处理了角度在 360 度到 0 度之间的“突变”问题
{
    float encoder_angle = driver_.getMotor().getAngle0_360(GimbalDriver::YAW_ID);
    return ZeroCrossingProcessing(77.0f, encoder_angle, 360.0f) - encoder_angle;
}
//常数77.0f代表云台正对着底盘前方时，偏航轴电机编码器的真实读数
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



//USB CDC与视觉通信的接收部分
#define RX_STREAM_BUF_SIZE 256
static uint8_t rx_stream_buf[RX_STREAM_BUF_SIZE];
static uint16_t rx_stream_len = 0;

extern "C" void Gimbal_ProcessUSBFromTask(void)
{
    uint8_t tmp[64];//USB FS（全速）协议一次最多传 64 字节
    uint16_t n = CDC_ReadRxData(tmp, sizeof(tmp));
    if (n == 0)
        return;

    if (rx_stream_len + n > RX_STREAM_BUF_SIZE)
    {
        rx_stream_len = 0;
        // 如果数据太多装不下了，说明之前的数据卡死了，直接清空重来
    }

    std::memcpy(&rx_stream_buf[rx_stream_len], tmp, n);
    rx_stream_len += n;

    uint8_t frame_size = Comm::Vision::getRxSize();

    while (rx_stream_len >= frame_size)
    {
        // 检查头两个字节是不是协议规定的“帧头”
        if (rx_stream_buf[0] == Comm::Vision::RX_FRAME_HEAD1 &&
            rx_stream_buf[1] == Comm::Vision::RX_FRAME_HEAD2)
        {
            // 提取数据：把这一整帧拷贝给视觉模块
            std::memcpy(Comm::vision.getRxBuffer(), rx_stream_buf, frame_size);
            Comm::vision.receive();
            // 把这一帧的长度从总长度里扣除
            rx_stream_len -= frame_size;
            if (rx_stream_len > 0)
            {
                // 如果传送带上还有剩下的数据，就把它们“往前平移”，填补刚才拿走的空位
                std::memmove(rx_stream_buf, &rx_stream_buf[frame_size], rx_stream_len);
            }
        }
        else
        
        {
            rx_stream_len -= 1;
            if (rx_stream_len > 0)
            {
                std::memmove(rx_stream_buf, &rx_stream_buf[1], rx_stream_len);
            }
        }
    }
}