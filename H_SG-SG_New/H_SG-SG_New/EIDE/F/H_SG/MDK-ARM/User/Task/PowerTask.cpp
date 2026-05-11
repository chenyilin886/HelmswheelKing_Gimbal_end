
// 添加PD控制器类定义
class PDController {
private:
    float Kp, Kd;
    float prev_error;

public:
    PDController(float kp, float kd) : Kp(kp), Kd(kd), prev_error(0.0f) {}

    float update(float error, float dt) {
        float derivative = (error - prev_error) / dt;
        float output = Kp * error + Kd * derivative;
        prev_error = error;
        return output;
    }
};


void RLSTask(void *argument)
{
    osDelay(500);
    
    // 初始化电机接口
    PowerControl.InitMotorInterfaces(BSP::Motor::Dji::Motor3508, BSP::Motor::LK::Motor4005);
    
    // 初始化PD控制器
    PDController powerPD_base(0.1f, 0.01f); // 调整参数以适应实际需求
    PDController powerPD_full(0.1f, 0.01f); // 调整参数以适应实际需求

    for (;;) {
        // 轮向电机功率计算 (DJI 3508)
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, toque_const_3508, rpm_to_rads_3508);
        
        // 舵向电机功率计算 (LK 4005)
        PowerControl.String_PowerData.UpRLS(pid_vel_String, toque_const_4005, rpm_to_rads_4005);

        // 获取当前时间戳（用于计算dt）
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        // 计算能量误差
        float Es = sqrtf(manager.baseBuffSet); // 目标能量
        float Ef = sqrtf(manager.powerBuff);  // 实际能量
        float e_t = Es - Ef;

        // 使用PD控制器计算功率调整量
        float power_adjustment = powerPD_base.update(e_t, dt);

        // 动态调整功率上限
        float lim_cin_power = ext_power_heat_data_0x0201.chassis_power_limit - 1.0f;
        float P_max_base = fmax(manager.refereeMaxPower - power_adjustment, MIN_MAXPOWER_CONFIGURED);
        float P_max_full = fmax(manager.fullMaxPower - power_adjustment, MIN_MAXPOWER_CONFIGURED);

        // 设置功率限制
        BSP::SuperCap::cap.SetSendValue(P_max_base);
        BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);

        osDelay(5);
    }
}