#include "ChassisTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../Task/CommunicationTask.hpp"
#include "../APP/State.hpp"
#include "../APP/Variable.hpp"
#include "cmsis_os2.h"
#include "../Task/PowerTask.hpp"
#include "../APP/Remote/KeyBroad.hpp"
#include "../APP/Remote/Mode.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../Algorithm/ChassisCalculation/StringWheel.hpp"
#include "../Task/CallBack.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"

TaskManager taskManager;
#define p 3.14159
#define p3 2.35619
#define p2 1.570795
#define p4 0.7853975
// 四个轮子在底盘坐标系下的安装方位角。
float Wheel_Azimuth[4] = {5 * My_PI / 4, 7 * My_PI / 4, My_PI / 4, 3 * My_PI / 4};

// 四个舵轮零位标定值，直接保留成当前实测数值。
float phase[4] = {1.645865 - p4, 4.333016 - p3, 1.252015 + p3 , 1.427656 + p4};
//1.645865  4.333016 1.252015  1.427656
void ChassisTask(void *argument)
{
    osDelay(500);
    taskManager.addTask<Chassis_Task>();

    for (;;)
    {
        taskManager.updateAll();
        osDelay(1);
    }
}

static constexpr float FOLLOW_YAW_DEADZONE_DEG = 2.0f;
static constexpr float KEYBOARD_ROTATION_DEADZONE = 0.05f;
static constexpr float STEER_ANGLE_SLOPE_STEP_DEG = 0.4f;

namespace
{

float WrapAngle360(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

float NormalizeAngleNearReference(float angle, float reference)
{
    const float wrapped_angle = WrapAngle360(angle);
    const float wrapped_reference = WrapAngle360(reference);
    float delta = wrapped_angle - wrapped_reference;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    return reference + delta;
}

// 统一更新底盘目标速度。
// 这里直接写入原始输入，不再经过 TD。
// 同时对平移合速度做限幅，避免斜向速度比单轴更快。
void UpdateFilteredTargets(float vx_target, float vy_target, float vw_target)
{
    const float planar_limit = fmaxf(fabsf(vx_target), fabsf(vy_target));
    const float planar_speed = sqrtf(vx_target * vx_target + vy_target * vy_target);

    if (planar_limit > 0.0f && planar_speed > planar_limit)
    {
        const float scale = planar_limit / planar_speed;
        vx_target *= scale;
        vy_target *= scale;
    }

    tar_vx.u = vx_target;
    tar_vx.x1 = vx_target;
    tar_vx.x2 = 0.0f;

    tar_vy.u = vy_target;
    tar_vy.x1 = vy_target;
    tar_vy.x2 = 0.0f;

    tar_vw.u = vw_target;
    tar_vw.x1 = vw_target;
    tar_vw.x2 = 0.0f;
}

} // namespace

// ==================== 状态处理器实现 ==================== //

class Chassis_Task::UniversalHandler : public StateHandler
{
    Chassis_Task &m_task;

  public:
    explicit UniversalHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void UniversalTarget()
    {
        const float cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);
        const float sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);

        UpdateFilteredTargets(TAR_LX * 880.0f, TAR_LY * 880.0f, TAR_VW * 550.0f);

        const float vx_slope = ApplySlope(slope_vx, tar_vx.x1, Chassis_Data.vx);
        const float vy_slope = ApplySlope(slope_vy, tar_vy.x1, Chassis_Data.vy);

        Chassis_Data.vx = vx_slope * cos_theta - vy_slope * sin_theta;
        Chassis_Data.vy = vx_slope * sin_theta + vy_slope * cos_theta;
    }

    void handle() override
    {
        // 通用模式只处理平移，旋转直接使用输入目标。
        UniversalTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::FollowHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit FollowHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void FllowTarget()
    {
        // 跟随模式下，vw 由云台和底盘夹角闭环得到。
        float yaw_err = Gimbal_to_Chassis_Data.getEncoderAngleErr();

        while (yaw_err > 1.570796f) yaw_err -= 2 * 1.570796f;
        while (yaw_err < -1.570796f) yaw_err += 2 * 1.570796f;

        if (fabsf(yaw_err) <= (FOLLOW_YAW_DEADZONE_DEG * 0.017453f))
        {
            yaw_err = 0.0f;
            pid_vw.clearPID();
        }
        else
        {
            pid_vw.GetPidPos(Kpid_vw, 0, yaw_err, 10000);
        }

        const float cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);
        const float sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);
        const float vw_target = (yaw_err == 0.0f) ? 0.0f : pid_vw.GetCout();

        UpdateFilteredTargets(TAR_LX * 880.0f, TAR_LY * 880.0f, vw_target);

        const float vx_slope = tar_vx.x1;
        const float vy_slope = tar_vy.x1;
        const float vw_slope = tar_vw.x1;

        Chassis_Data.vx = vx_slope * cos_theta - vy_slope * sin_theta;
        Chassis_Data.vy = vx_slope * sin_theta + vy_slope * cos_theta;
        Chassis_Data.vw = vw_slope;

        td_FF_Tar.Calc(tar_vx.x1);
    }

    void handle() override
    {
        FllowTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::KeyBoardHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit KeyBoardHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void FllowTarget()
    {
        // 键鼠模式下，平移直接来自板间通信。
        // vw 优先使用小陀螺输入；没有输入时退回跟随闭环。
        float yaw_err = Gimbal_to_Chassis_Data.getEncoderAngleErr();

        while (yaw_err > 1.570796f) yaw_err -= 2 * 1.570796f;
        while (yaw_err < -1.570796f) yaw_err += 2 * 1.570796f;

        const float cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);
        const float sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);

        UpdateFilteredTargets(TAR_LX * 660.0f, TAR_LY * 660.0f, 0.0f);

        const uint8_t rotating_vel_raw = Gimbal_to_Chassis_Data.getRotatingVel();
        float keyboard_vw_input = 0.0f;

        // 兼容云台当前的 0/220 开关量写法：
        // 0 认为是停转，220 认为是正向小陀螺。
        if (rotating_vel_raw != 0U)
        {
            keyboard_vw_input = (rotating_vel_raw - 110) / 110.0f;
        }

        float vw_target = 0.0f;
        if (fabsf(keyboard_vw_input) > KEYBOARD_ROTATION_DEADZONE)
        {
            vw_target = keyboard_vw_input;
        }
        else
        {
            if (!Gimbal_to_Chassis_Data.getFollow())
            {
                pid_vw.clearPID();
                vw_target = 0.0f;
            }
            else if (fabsf(yaw_err) <= (FOLLOW_YAW_DEADZONE_DEG * 0.017453f))
            {
                yaw_err = 0.0f;
                pid_vw.clearPID();
                vw_target = 0.0f;
            }
            else
            {
                pid_vw.GetPidPos(Kpid_vw, 0, yaw_err, 10000);
                vw_target = pid_vw.GetCout();
            }
        }

        const float vx_slope = ApplySlope(slope_vx, tar_vx.x1, Chassis_Data.vx);
        const float vy_slope = ApplySlope(slope_vy, tar_vy.x1, Chassis_Data.vy);

        Chassis_Data.vx = vx_slope * cos_theta - vy_slope * sin_theta;
        Chassis_Data.vy = vx_slope * sin_theta + vy_slope * cos_theta;
        Chassis_Data.vw = vw_target;

        td_FF_Tar.Calc(tar_vx.x1);
    }

    void handle() override
    {
        FllowTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::RotatingHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit RotatingHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void RotatingTarget()
    {
        // 小陀螺模式下，平移仍然按云台坐标系输入，vw 直接取旋转目标。
        const float cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);
        const float sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + 3.1415926535f);

        UpdateFilteredTargets(TAR_LX * 880.0f, TAR_LY * 880.0f, TAR_VW * 550.0f);

        const float vx_slope = ApplySlope(slope_vx, tar_vx.x1, slope_vx.Get_Out());
        const float vy_slope = ApplySlope(slope_vy, tar_vy.x1, slope_vy.Get_Out());
        const float vw_slope = ApplySlope(slope_vw, tar_vw.x1, Chassis_Data.vw);

        Chassis_Data.vx = vx_slope * cos_theta - vy_slope * sin_theta;
        Chassis_Data.vy = vx_slope * sin_theta + vy_slope * cos_theta;
        Chassis_Data.vw = vw_slope;

        td_FF_Tar.Calc(tar_vx.x1);
    }

    void handle() override
    {
        RotatingTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::StopHandler : public StateHandler
{
    Chassis_Task &m_task;

  public:
    explicit StopHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 急停时清掉所有 PID 积分，并下发零目标。
        UpdateFilteredTargets(0.0f, 0.0f, 0.0f);
        Chassis_Data.vx = ApplySlope(slope_vx, 0.0f, Chassis_Data.vx);
        Chassis_Data.vy = ApplySlope(slope_vy, 0.0f, Chassis_Data.vy);
        Chassis_Data.vw = ApplySlope(slope_vw, 0.0f, Chassis_Data.vw);
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();

        if (fabsf(Chassis_Data.vx) < 1.0f &&
            fabsf(Chassis_Data.vy) < 1.0f &&
            fabsf(Chassis_Data.vw) < 1.0f)
        {
            pid_vel_String[0].clearPID();
            pid_vel_String[1].clearPID();
            pid_vel_String[2].clearPID();
            pid_vel_String[3].clearPID();

            pid_vel_Wheel[0].clearPID();
            pid_vel_Wheel[1].clearPID();
            pid_vel_Wheel[2].clearPID();
            pid_vel_Wheel[3].clearPID();
        }

        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

// ==================== 任务主体实现 ==================== //

Chassis_Task::Chassis_Task()
    : slope_speed{
          Class_Slope(2, 6, Slope_First_REAL),
          Class_Slope(2, 6, Slope_First_REAL),
          Class_Slope(2, 6, Slope_First_REAL),
          Class_Slope(2, 6, Slope_First_REAL)},
      steer_angle_slope{
          Class_Slope(STEER_ANGLE_SLOPE_STEP_DEG, STEER_ANGLE_SLOPE_STEP_DEG, Slope_First_REAL),
          Class_Slope(STEER_ANGLE_SLOPE_STEP_DEG, STEER_ANGLE_SLOPE_STEP_DEG, Slope_First_REAL),
          Class_Slope(STEER_ANGLE_SLOPE_STEP_DEG, STEER_ANGLE_SLOPE_STEP_DEG, Slope_First_REAL),
          Class_Slope(STEER_ANGLE_SLOPE_STEP_DEG, STEER_ANGLE_SLOPE_STEP_DEG, Slope_First_REAL)},
      stringIk(0.22f, 0.09f, Wheel_Azimuth, phase)
{
    updateState();
}

void Chassis_Task::executeState()
{
    if (m_stateHandler)
    {
        m_stateHandler->handle();
    }
}

void Chassis_Task::updateState()
{
    if (Mode::Chassis::Move())
    {
        m_currentState = State::MoveState;
    }
    else if (Mode::Chassis::Universal())
    {
        m_currentState = State::UniversalState;
    }
    else if (Mode::Chassis::Follow())
    {
        m_currentState = State::FollowState;
    }
    else if (Mode::Chassis::Rotating())
    {
        m_currentState = State::RotatingState;
    }
    else if (Mode::Chassis::KeyBoard())
    {
        m_currentState = State::KeyBoardState;
    }
    else if (Mode::Chassis::Stop())
    {
        m_currentState = State::StopState;
    }

    // 根据当前模式选择对应的状态处理器。
    switch (m_currentState)
    {
    case State::UniversalState:
        m_stateHandler = std::make_unique<UniversalHandler>(*this);
        break;
    case State::FollowState:
        m_stateHandler = std::make_unique<FollowHandler>(*this);
        break;
    case State::RotatingState:
        m_stateHandler = std::make_unique<RotatingHandler>(*this);
        break;
    case State::KeyBoardState:
        m_stateHandler = std::make_unique<KeyBoardHandler>(*this);
        break;
    case State::StopState:
        m_stateHandler = std::make_unique<StopHandler>(*this);
        break;
    default:
        break;
    }
}

// Stop 模式下直接把当前输入写回目标，避免上一次模式残留。
void Chassis_Task::Tar_Updata()
{
    UpdateFilteredTargets(TAR_LX * 880.0f, TAR_LY * 880.0f, TAR_VW * 550.0f);

    Chassis_Data.vx = tar_vx.x1;
    Chassis_Data.vy = tar_vy.x1;
    Chassis_Data.vw = tar_vw.x1;

    td_FF_Tar.Calc(tar_vx.x1);
}

// 根据底盘目标速度做舵轮逆解，得到四个轮子的速度和角度目标。
void Chassis_Task::Wheel_UpData()
{
    for (int i = 0; i < 4; ++i)
    {
        slope_speed[i].TIM_Calculate_PeriodElapsedCallback();
    }

    for (int i = 0; i < 4; i++)
    {
        const float current_angle = BSP::Motor::LK::Motor4005.getAngleRad(i + 1);
        stringIk.Set_current_steer_angles(current_angle, i);
    }

    stringIk.StringInvKinematics(
        Chassis_Data.vx / 660.0f,
        Chassis_Data.vy / 660.0f,
        Chassis_Data.vw / 660.0f,
        0.0f,
        25.0f,
        35.0f);

    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_speed[i] = stringIk.GetMotor_wheel(i);
        Chassis_Data.tar_angle[i] = stringIk.GetMotor_direction(i) * 57.2957795f;
    }
}

// 轮向测速先经过 TD，减小反馈抖动。
void Chassis_Task::Filtering()
{
    for (int i = 0; i < 4; i++)
    {
        td_3508_speed[i].Calc(BSP::Motor::Dji::Motor3508.getVelocityRads(i + 1));
    }
}

float Chassis_Task::SmoothSteerTarget(uint8_t index, float raw_target_angle, float current_feedback)
{
    // 舵向目标做一层角度限坡，避免快速切换时直接跳角。
    Class_Slope &planner = steer_angle_slope[index];
    const float wrapped_feedback = WrapAngle360(current_feedback);

    if (!steer_angle_slope_initialized_[index])
    {
        planner.Reset(wrapped_feedback);
        steer_angle_slope_initialized_[index] = true;
    }

    const float planner_output = planner.Get_Out();
    const float feedback_near_planner = NormalizeAngleNearReference(wrapped_feedback, planner_output);
    const float target_near_planner = NormalizeAngleNearReference(raw_target_angle, planner_output);
    const float smoothed_target = ApplySlope(planner, target_near_planner, feedback_near_planner);

    return NormalizeAngleNearReference(smoothed_target, current_feedback);
}

void Chassis_Task::PID_Updata()
{
    // 先算舵向双环：角度环输出给速度环。
    for (int i = 0; i < 4; i++)
    {
        feed_4005[i].UpData(Chassis_Data.FF_Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_4005[i].cout, feed_4005[i].target_e, 360);

        const float current_feedback = BSP::Motor::LK::Motor4005.getAngleDeg(i + 1);
        const float target_angle = SmoothSteerTarget(i, Chassis_Data.tar_angle[i], current_feedback);

        pid_angle_String[i].GetPidPos(Kpid_4005_angle, target_angle, current_feedback, 500);
        pid_vel_String[i].GetPidPos(
            Kpid_4005_vel,
            pid_angle_String[i].pid.cout,
            BSP::Motor::LK::Motor4005.getVelocityRpm(i + 1),
            500);
    }

    // 再算轮向速度环。
    for (int i = 0; i < 4; i++)
    {
        pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[i], td_3508_speed[i].x1, 16384.0f);
    }
}

void Chassis_Task::CAN_Setting()
{
    // 先取 PID 原始输出，后面再按功率限幅重新收缩。
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
        Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
    }

    const float wheel_total_power_limit = PowerControl.Wheel_PowerData.MAXPower;
    const float steer_base_power_limit = PowerControl.String_PowerData.MAXPower;

    float steer_alloc_power = PowerControl.String_PowerData.EstimatedPower;
    if (steer_alloc_power > steer_base_power_limit) steer_alloc_power = steer_base_power_limit;
    if (steer_alloc_power < 0.0f) steer_alloc_power = 0.0f;

    // 当前策略：
    // 舵向只吃裁判功率上限中的固定份额；
    // 轮向吃剩余的裁判功率份额和能量环补偿。
    float wheel_alloc_power = wheel_total_power_limit - steer_base_power_limit;
    if (wheel_alloc_power < 0.0f) wheel_alloc_power = 0.0f;

    PowerControl.String_PowerData.MAXPower = steer_alloc_power;
    PowerControl.Wheel_PowerData.MAXPower = wheel_alloc_power;

    PowerControl.String_PowerData.UpScaleMaxPow(pid_vel_String);
    PowerControl.String_PowerData.UpCalcMaxTorque(
        Chassis_Data.final_4005_Out,
        pid_vel_String,
        toque_const_4005,
        rpm_to_rads_4005);

    PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel);
    PowerControl.Wheel_PowerData.UpCalcMaxTorque(
        Chassis_Data.final_3508_Out,
        pid_vel_Wheel,
        toque_const_3508,
        rpm_to_rads_3508);

    PowerControl.String_PowerData.MAXPower = steer_base_power_limit;
    PowerControl.Wheel_PowerData.MAXPower = wheel_total_power_limit;

    for (int i = 0; i < 4; i++)
    {
        BSP::Motor::Dji::Motor3508.setCAN(Tools.clamp(Chassis_Data.final_3508_Out[i], 16384, -16384), i + 1);
    }
}

void Chassis_Task::CAN_Send()
{
    // 3508 和 4005 交替发送，超级电容每次循环都发。
    if (Send_ms == 0)
    {
        int16_t iq_control[4];

        for (int i = 0; i < 4; i++)
        {
            Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
            iq_control[i] = Tools.clamp((int16_t)Chassis_Data.final_4005_Out[i], 200, -200);
        }
        BSP::Motor::LK::Motor4005.ctrl_Multi(iq_control);
    }

    if (Send_ms == 1)
    {
        BSP::Motor::Dji::Motor3508.sendCAN();
    }

    BSP::SuperCap::cap.sendCAN();
    Send_ms++;
    Send_ms %= 2;

    // Tools.vofaSend(
    //     BSP::Motor::LK::Motor4005.getTorque(1),
    //     BSP::Motor::LK::Motor4005.getTorque(2),
    //     BSP::Motor::LK::Motor4005.getTorque(3),
    //     BSP::Motor::LK::Motor4005.getTorque(4),
    //     0,
    //     0);
}
