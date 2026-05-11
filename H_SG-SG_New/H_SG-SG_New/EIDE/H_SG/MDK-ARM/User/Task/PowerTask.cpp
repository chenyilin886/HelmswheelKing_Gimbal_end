
void PowerUpData_t::InitEnergyRing(float setpoint, float kp, float kd)
{
    energy_setpoint = setpoint;
    kp_energy = kp;
    kd_energy = kd;
    energy_feedback = 0.0f;
    energy_error = 0.0f;
    energy_derivative = 0.0f;
    energy_integral = 0.0f;
    energy_pd_output = 0.0f;
    energy_max_power = MAXPower;
    energy_min_power = MIN_MAXPOWER_CONFIGURED;
}

void PowerUpData_t::UpdateEnergyRing(float feedback, float dt)
{
    // 更新能量反馈值
    energy_feedback = feedback;
    
    // 计算能量误差
    energy_error = energy_setpoint - energy_feedback;
    
    // 计算能量变化率
    energy_derivative = (energy_error - energy_error_prev) / dt;
    
    // 更新前一次误差
    energy_error_prev = energy_error;
    
    // 计算PD控制器输出
    energy_pd_output = kp_energy * energy_error + kd_energy * energy_derivative;
    
    // 计算功率上限值
    energy_max_power = MAXPower + energy_pd_output;
    
    // 限制功率上限值在合理范围内
    energy_max_power = fmax(energy_max_power, energy_min_power);
    energy_max_power = fmin(energy_max_power, MAXPower * 1.5f); // 上限不超过1.5倍最大功率
    
    // 计算功率下限值
    energy_min_power = MAXPower - energy_pd_output;
    energy_min_power = fmax(energy_min_power, MIN_MAXPOWER_CONFIGURED);
}


void RLSTask(void *argument)
{
    osDelay(500);
    
    // 初始化电机接口
    PowerControl.InitMotorInterfaces(BSP::Motor::Dji::Motor3508, BSP::Motor::LK::Motor4005);
    
    float last_time = 0.0f;
    float dt = 0.0f;
    
    for (;;) {
        // 获取当前时间
        float current_time = HAL_GetTick() / 1000.0f;
        dt = current_time - last_time;
        last_time = current_time;
        
        // 轮向电机功率计算 (DJI 3508)
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, toque_const_3508, rpm_to_rads_3508);
        
        // 舵向电机功率计算 (LK 4005)
        PowerControl.String_PowerData.UpRLS(pid_vel_String, toque_const_4005, rpm_to_rads_4005);

        // 获取电容能量
        float supercap_energy = BSP::SuperCap::cap.getOutPower();
        
        // 更新能量环
        PowerControl.Wheel_PowerData.UpdateEnergyRing(supercap_energy, dt);
        PowerControl.String_PowerData.UpdateEnergyRing(supercap_energy, dt);
        
        // 使用能量环的功率上限值
        float lim_cin_power = ext_power_heat_data_0x0201.chassis_power_limit - 1.0f;
        
        // 应用能量环的功率限制
        lim_cin_power = fmin(lim_cin_power, PowerControl.Wheel_PowerData.GetMaxPower());
        lim_cin_power = fmax(lim_cin_power, PowerControl.Wheel_PowerData.GetMinPower());
        
        BSP::SuperCap::cap.SetSendValue(lim_cin_power);
        BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);

        osDelay(5);
    }
}