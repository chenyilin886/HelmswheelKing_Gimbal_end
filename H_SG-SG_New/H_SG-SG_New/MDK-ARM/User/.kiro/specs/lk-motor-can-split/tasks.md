# 实现计划: LK电机CAN总线分离

## 概述

将四个LK4005电机从单一CAN1总线分配到CAN1和CAN2两条总线，电机1、2保留在CAN1，电机3、4迁移到CAN2。保持上层控制逻辑不变。

## 任务

- [ ] 1. 修改LkMotorBase类支持双CAN总线
  - [ ] 1.1 添加CAN总线引用成员变量和映射数组
    - 在LkMotorBase类中添加`can_bus_1_`和`can_bus_2_`指针
    - 添加`motor_to_can_[N]`映射数组
    - 添加`BROADCAST_ID`常量（0x280）
    - _Requirements: 1.1, 1.2, 2.3_
  - [ ] 1.2 添加CAN总线设置和获取方法
    - 实现`setCAN1()`和`setCAN2()`方法
    - 实现`getCanBus(uint8_t motor_id)`方法，根据电机ID返回对应CAN总线
    - _Requirements: 4.1, 4.2, 5.3_

- [ ] 2. 修改单电机控制函数
  - [ ] 2.1 修改On/Off/ClearErr函数使用getCanBus
    - 将硬编码的`get_can1()`替换为`getCanBus(id)`
    - _Requirements: 4.1, 4.2_
  - [ ] 2.2 修改ctrl_Torque和ctrl_Position函数
    - 将硬编码的`get_can1()`替换为`getCanBus(id)`
    - _Requirements: 4.1, 4.2_

- [ ] 3. 修改多电机广播控制函数
  - [ ] 3.1 重构ctrl_Multi函数支持双总线广播
    - 向CAN1发送电机1、2的控制数据（使用0x280）
    - 向CAN2发送电机3、4的控制数据（使用0x280）
    - 分别检查两条总线的邮箱状态
    - _Requirements: 2.1, 2.2, 2.3_

- [ ] 4. 修改电机实例定义
  - [ ] 4.1 更新Motor4005实例构造
    - 添加CAN映射参数`{0, 0, 1, 1}`（电机1,2->CAN1, 电机3,4->CAN2）
    - 修改LK4005构造函数接受CAN映射参数
    - _Requirements: 1.1, 1.2, 1.3_

- [ ] 5. 修改初始化代码
  - [ ] 5.1 在Init.cpp中设置CAN总线引用
    - 调用`Motor4005.setCAN1(&can1)`
    - 调用`Motor4005.setCAN2(&can2)`
    - _Requirements: 1.1, 1.2_
  - [ ] 5.2 注册CAN2回调处理电机反馈
    - 为CAN2注册回调，调用`Motor4005.Parse(frame)`
    - 确保CAN1回调也正确注册
    - _Requirements: 3.1, 3.2_

- [ ] 6. 检查点 - 验证编译和基本功能
  - 确保代码编译通过
  - 确保所有电机控制函数接口保持不变
  - 如有问题请询问用户

## 注意事项

- 上层控制逻辑（ChassisTask、EvenTask等）无需修改
- 电机ID（1-4）保持不变，仅CAN总线路由发生变化
- 断联检测逻辑保持不变，每个电机独立维护状态监视器
