
void Chassis_Task::CAN_Setting()
{
    // 检查遥控器是否在线，如果不在线则将所有输出设置为0
    if (!BSP::Remote::dr16.isDrOnline()) {
        for (int i = 0; i < 4; i++) {
            Chassis_Data.final_3508_Out[i] = 0;
        }
    } else {
        // 正常计算输出值
        for (int i = 0; i < 4; i++)
        {
            Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
        }
    }

    // 功率控制部分
    // if (Dir_Event.GetDir_String() == false)
    // {
    //     PowerControl.String_PowerData.UpScaleMaxPow(pid_vel_String);
//         PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_4005_Out, pid_vel_String,
//                                                      toque_const_4005, rpm_to_rads_4005);
    // }

    // if (Dir_Event.GetDir_Wheel() == false)
    // {
    //     PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel, Motor3508);
    //     PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, Motor3508, pid_vel_Wheel,
    //                                                  toque_const_3508, rpm_to_rads_3508);
    // }
    // 设置CAN消息数据但不立即发送
    // for (int i = 0; i < 4; i++) {
    //     BSP::Motor::LK::Motor4005.setCAN(Chassis_Data.final_4005_Out[i], (1 + i));
    //     osDelay(1);
    // }

    for(int i = 0; i < 4; i++)
    {
        BSP::Motor::Dji::Motor3508.setCAN(Chassis_Data.final_3508_Out[i], (1 + i));
        osDelay(1);
    }

}


void Chassis_Task::CAN_Send()
{
    // 发送数据
    if (Send_ms == 0)
    {
        BSP::Power::pm01.PM01SendFun(); 
    }
    else if (Send_ms == 1)    
    {   
        // 发送3508电机的CAN数据
        BSP::Motor::Dji::Motor3508.sendCAN(&hcan1, CAN_TX_MAILBOX0);
    }
    Send_ms++; 
    Send_ms %= 2;

    // Tools.vofaSend(BSP::Motor::Dji::Motor3508.getVelocityRpm(1),
    //                  BSP::Motor::Dji::Motor3508.getVelocityRpm(2), BSP::Motor::Dji::Motor3508.getVelocityRpm(3),
	// 										BSP::Motor::Dji::Motor3508.getVelocityRpm(4), 0, 0);
}