#include "chassis.hpp"
#include <cmath>

// 常量定义，避免在代码里写死魔法数字 (Magic Numbers)
#define M_PI_F 3.14159265f
#define P3 2.35619f
#define P2 1.570795f
#define P4 0.7853975f

// ================== 构造函数 ==================
// 利用 C++ 的初始化列表，把原来的全局参数填进去
Chassis::Chassis() 
    : wheel_motor_(0x200, {1, 2, 3, 4}, 0x200),
      steer_motor_(0x140, {1, 2, 3, 4}, {1, 2, 3, 4}),
      // 初始化轮向 PID
      wheel_pid_{
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 150.0f}
      },
      // 初始化舵向角度环 PID
      steer_angle_pid_{
          {10.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 5.0f},
          {10.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f},
          {10.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f},
          {10.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 20.0f}
      },
      // 初始化舵向速度环 PID
      steer_velocity_pid_{
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f},
          {1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f}
      },
      // 初始化运动学解算器 (注意这里用临时数组给指针传参的方式)
      string_ik_(0.17f, 0.055f, 
                 (float[]){5.0f*M_PI_F/4.0f, 7.0f*M_PI_F/4.0f, M_PI_F/4.0f, 3.0f*M_PI_F/4.0f}, 
                 (float[]){4.804619f - P4, 1.424031f - P3, 4.734439f + P3, 4.575001f + P4}),
      target_vx_(0.0f), target_vy_(0.0f), target_vw_(0.0f),
      debug_phase_{-0.13f, 0.0f, 0.0f, 0.0f},
      debug_linear_gain_(0.5f), debug_angular_gain_(1.5f)
{
    // 构造函数体留空
}

// ================== 初始化 ==================
void Chassis::Init() {
    for (int i = 1; i <= 4; i++) {
        steer_motor_.On(i);
        steer_motor_.setAllowAccumulate(i, true);
        steer_motor_.ClearErr(i);
    }
}

// ================== 接口：设置速度 ==================
void Chassis::Set_Velocity(float vx, float vy, float vw) {
    target_vx_ = vx;
    target_vy_ = vy;
    target_vw_ = vw;
}

// ================== 核心：运算与控制 ==================
void Chassis::Update() {
    // 1. 更新当前舵向角度给运动学模型
    for (int i = 0; i < 4; i++) {
        float steer_angle_rad = steer_motor_.getAngleRad(i + 1);
        string_ik_.Set_current_steer_angles(steer_angle_rad - debug_phase_[i], i);
    }
    
    // 2. 逆运动学解算
    string_ik_.StringInvKinematics(
        target_vx_, target_vy_, target_vw_, 
        0.0f, debug_linear_gain_, debug_angular_gain_
    );
    
    // 3. 执行闭环控制
    int16_t steer_outputs[4];
    bool is_zero_speed = (std::fabs(target_vx_) < 0.01f && 
                          std::fabs(target_vy_) < 0.01f && 
                          std::fabs(target_vw_) < 0.01f);
    
    for (int i = 0; i < 4; i++) {
        if (is_zero_speed) {
            steer_outputs[i] = 0;
            wheel_motor_.setCAN(0, i + 1);
        } else {
            // --- 舵向电机控制 (LK4005) ---
            float target_steer = (string_ik_.GetMotor_direction(i) + debug_phase_[i]) * 57.29578f;
            float current_steer = steer_motor_.getAngleDeg(i + 1);
            
            steer_angle_pid_[i].UpDate(target_steer, current_steer);
            steer_velocity_pid_[i].UpDate(steer_angle_pid_[i].getOutput(), steer_motor_.getVelocityRpm(i + 1));
            
            int16_t steer_output = (int16_t)steer_velocity_pid_[i].getOutput();
            // 限制幅值
            if(steer_output > 2048) steer_output = 2048;
            if(steer_output < -2048) steer_output = -2048;
            steer_outputs[i] = steer_output;

            // --- 轮向电机控制 (3508) ---
            float target_wheel = string_ik_.GetMotor_wheel(i);
            float current_wheel = wheel_motor_.getVelocityRpm(i + 1);
            
            wheel_pid_[i].UpDate(target_wheel, current_wheel);
            int16_t wheel_output = (int16_t)wheel_pid_[i].getOutput();
            
            wheel_motor_.setCAN(wheel_output, i + 1);
        }
    }
    
    // 4. 发送电流值
    steer_motor_.ctrl_Multi(steer_outputs);
    wheel_motor_.sendCAN();
}