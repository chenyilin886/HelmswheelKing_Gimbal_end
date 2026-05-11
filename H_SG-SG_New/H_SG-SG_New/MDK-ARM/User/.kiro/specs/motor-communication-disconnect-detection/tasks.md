# Implementation Plan: 电机和板间通信断联检测

## Overview

本实现计划将断联检测功能分解为增量式的编码任务，从底层电机时间戳更新开始，逐步实现状态检测、LED显示和蜂鸣器报警功能。

## Tasks

- [x] 1. 增强MotorBase基类的状态监控接口
  - [x] 1.1 在MotorBase中添加isMotorOnline()方法
    - 实现单个电机在线状态检查
    - 调用UpdateTime()、CheckStatus()并返回状态
    - _Requirements: 1.1, 1.2, 2.1, 2.2_
  - [x] 1.2 在MotorBase中添加getFirstOfflineMotorId()方法
    - 遍历所有电机返回第一个离线电机ID
    - 返回0表示全部在线
    - _Requirements: 1.4, 2.4_

- [x] 2. 实现LK4005舵向电机时间戳更新
  - [x] 2.1 修改LK4005的Parse函数
    - 在成功解析CAN数据后调用state_watch_[i].UpdateLastTime()
    - _Requirements: 6.1_

- [x] 3. 实现GM3508轮向电机时间戳更新
  - [x] 3.1 修改GM3508的Parse函数
    - 在成功解析CAN数据后调用state_watch_[i].UpdateLastTime()
    - _Requirements: 6.1_

- [x] 4. 实现EvenTask中的状态检测函数
  - [x] 4.1 实现Dir_String()函数
    - 遍历4个舵向电机检查在线状态
    - 更新DirData.String数组
    - 离线时调用BuzzerManager请求报警
    - _Requirements: 1.3, 1.4, 5.1_
  - [x] 4.2 实现Dir_Wheel()函数
    - 遍历4个轮向电机检查在线状态
    - 更新DirData.Wheel数组
    - 离线时调用BuzzerManager请求报警
    - _Requirements: 2.3, 2.4, 5.2_
  - [x] 4.3 实现Dir_Communication()函数
    - 检查板间通信在线状态
    - 更新DirData.Communication字段
    - 离线时调用BuzzerManager请求报警
    - _Requirements: 3.3, 3.4, 5.3_
  - [x] 4.4 更新UpEvent()函数
    - 启用Dir_String()、Dir_Wheel()、Dir_Communication()调用
    - _Requirements: 1.3, 2.3, 3.3_

- [x] 5. Checkpoint - 验证状态检测功能
  - 确保电机和通信状态能正确检测
  - 验证DirData结构体正确更新
  - 如有问题请询问用户

- [x] 6. 更新LED显示逻辑
  - [x] 6.1 修改LED::Update()函数
    - 实现优先级显示：舵向(红) > 轮向(蓝) > 板间通信(粉)
    - 所有设备在线时显示正常流水灯
    - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5_

- [x] 7. 更新BuzzerManager板间通信报警
  - [x] 7.1 修改processRing()函数中0xFE的处理
    - 将板间通信报警改为3次长音(500ms)
    - _Requirements: 5.3_

- [x] 8. 添加必要的头文件包含
  - [x] 8.1 在EvenTask.cpp中添加buzzer_manager.hpp包含
    - 确保BuzzerManagerSimple可访问
    - _Requirements: 5.1, 5.2, 5.3_
  - [x] 8.2 在EvenTask.cpp中添加电机头文件包含
    - 包含Lk_motor.hpp和DjiMotor.hpp
    - _Requirements: 1.4, 2.4_

- [x] 9. Final Checkpoint - 完整功能验证
  - 确保所有断联检测功能正常工作
  - 验证LED颜色显示正确
  - 验证蜂鸣器报警正确
  - 如有问题请询问用户

## Notes

- 本项目为嵌入式C++项目，运行在STM32平台
- 所有任务按顺序执行，确保依赖关系正确
- 电机超时时间为100ms，板间通信超时时间为50ms
- 蜂鸣器报警：电机鸣叫ID次短音(100ms)，板间通信鸣叫3次长音(500ms)
