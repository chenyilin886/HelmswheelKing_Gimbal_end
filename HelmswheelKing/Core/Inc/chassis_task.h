#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdint.h>

// 1. 结构体定义放在 extern "C" 之外（使用 typedef 兼容 C 语言）
typedef struct ChassisTarget {
    float vx;      // X方向速度 (m/s)
    float vy;      // Y方向速度 (m/s)  
    float vw;      // 旋转角速度 (rad/s)
} ChassisTarget;

// 2. 跨语言接口区
#ifdef __cplusplus
extern "C" {
#endif

// 暴露给外部调用的变量
extern ChassisTarget chassis_target;

// PID 参数（用于 RTT 实时调参）
extern float wheel_pid_kp;
extern float wheel_pid_ki;
extern float wheel_pid_kd;

extern float steer_angle_pid_kp;
extern float steer_angle_pid_ki;
extern float steer_angle_pid_kd;

extern float steer_velocity_pid_kp;
extern float steer_velocity_pid_ki;
extern float steer_velocity_pid_kd;

// 函数声明
void Chassis_Init(void);           
void Chassis_Control_Task(void);   

#ifdef __cplusplus
}  
#endif

#endif // CHASSIS_TASK_H
