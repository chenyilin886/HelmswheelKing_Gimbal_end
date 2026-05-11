#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "DjiMotor.hpp"
#include "Lk_motor.hpp"
#include "pid.hpp"
#include "StringWheel.hpp"

class Chassis {
public:
    // 构造函数：用于初始化那些一出生就必须要有参数的成员（比如运动学解算器）
    Chassis();

    // 初始化硬件（清空错误、启动电机等）
    void Init();

    // 唯一的控制接口：接收上层传来的目标速度
    void Set_Velocity(float vx, float vy, float vw);

    // 核心更新函数：放在 RTOS 任务中以固定频率循环调用 (例如 1ms)
    void Update();

    // 提供给 CAN 中断的回调接口，让中断能把数据喂给电机
    BSP::Motor::Dji::GM3508<4>& get_wheel_motor() { return wheel_motor_; }
    BSP::Motor::LK::LK4005<4>& get_steer_motor() { return steer_motor_; }

private:
    // 1. 硬件实例
    BSP::Motor::Dji::GM3508<4> wheel_motor_;
    BSP::Motor::LK::LK4005<4> steer_motor_;

    // 2. 闭环控制器
    ALG::PID::PID wheel_pid_[4];
    ALG::PID::PID steer_angle_pid_[4];
    ALG::PID::PID steer_velocity_pid_[4];

    // 3. 运动学解算实例
    Alg::CalculationBase::String_IK string_ik_;

    // 4. 内部状态变量
    float target_vx_;
    float target_vy_;
    float target_vw_;

    // 5. 运动学参数
    float debug_phase_[4];
    float debug_linear_gain_;
    float debug_angular_gain_;
};

#endif // CHASSIS_HPP