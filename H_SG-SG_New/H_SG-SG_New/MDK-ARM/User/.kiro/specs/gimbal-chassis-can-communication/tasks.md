# Implementation Plan: Gimbal-Chassis CAN Communication

## Overview

本实现计划基于最小修改原则，统一云台和底盘的CAN通信配置。主要修改包括：统一CAN ID定义、确保数据结构一致、修复帧头校验逻辑。

## Tasks

- [ ] 1. 统一CAN ID定义
  - [ ] 1.1 修改云台CommunicationTask.hpp中的CAN ID定义
    - 将CAN_C2G_FRAME1_ID改为0x505（与底盘发送ID一致）
    - 确保CAN_G2C_FRAME1_ID为0x501，CAN_G2C_FRAME2_ID为0x502
    - _Requirements: 5.1, 5.2_
  - [ ] 1.2 修改底盘CommunicationTask.hpp中的CAN ID定义
    - 确保CAN_G2C_FRAME1_ID为0x501，CAN_G2C_FRAME2_ID为0x502（用于接收）
    - 确保CAN_C2G_FRAME1_ID为0x505（用于发送）
    - _Requirements: 5.1, 5.2_

- [ ] 2. 修改云台发送逻辑
  - [ ] 2.1 检查并修正云台Data_send()函数
    - 确保帧头为0xA5
    - 确保使用CAN ID 0x501和0x502发送两帧
    - 验证数据打包顺序：帧头 + Direction + ChassisMode + UiList
    - _Requirements: 1.1, 1.2, 1.3, 1.4_

- [ ] 3. 修改底盘接收逻辑
  - [ ] 3.1 修改底盘ParseCANFrame()函数
    - 监听CAN ID 0x501和0x502
    - 实现两帧数据重组
    - 添加50ms超时重置逻辑
    - _Requirements: 2.1, 2.2, 2.6_
  - [ ] 3.2 修改底盘ProcessReceivedData()函数
    - 验证帧头为0xA5
    - 正确解析Direction、ChassisMode、UiList数据
    - _Requirements: 2.3, 2.4, 2.5_

- [ ] 4. 修改底盘发送逻辑
  - [ ] 4.1 检查并修正底盘Transmit()函数
    - 确保帧头为0x21和0x12
    - 确保使用CAN ID 0x505发送
    - 验证数据打包顺序：帧头 + heat_cd + heat_max + now_heat
    - _Requirements: 3.1, 3.2, 3.3, 3.4_

- [ ] 5. 修改云台接收逻辑
  - [ ] 5.1 修改云台HandleCANMessage()和ParseCANFrame()函数
    - 监听CAN ID 0x505
    - 验证帧头为0x21和0x12
    - 正确解析裁判系统数据
    - _Requirements: 4.1, 4.2, 4.3_

- [ ] 6. 确保数据结构一致性
  - [ ] 6.1 对比并统一Direction结构体定义
    - 确保云台和底盘使用相同的packed属性和字段顺序
    - _Requirements: 6.1_
  - [ ] 6.2 对比并统一ChassisMode结构体定义
    - 确保位字段定义一致
    - _Requirements: 6.2_
  - [ ] 6.3 对比并统一UiList结构体定义
    - 确保位字段定义一致
    - _Requirements: 6.3_
  - [ ] 6.4 对比并统一Booster/RxRefree结构体定义
    - 确保packed属性和字段顺序一致
    - _Requirements: 6.4_

- [ ] 7. Checkpoint - 代码编译验证
  - 确保云台和底盘代码都能正常编译
  - 检查是否有未定义的符号或类型不匹配

- [ ]* 8. 属性测试（可选）
  - [ ]* 8.1 编写数据往返一致性测试
    - **Property 1: 数据往返一致性**
    - **Validates: Requirements 1.1, 1.5, 1.6, 1.7, 2.5, 6.1, 6.2, 6.3**
  - [ ]* 8.2 编写裁判系统数据往返测试
    - **Property 2: 裁判系统数据往返一致性**
    - **Validates: Requirements 3.1, 3.4, 4.3, 6.4**
  - [ ]* 8.3 编写帧头验证测试
    - **Property 3: 帧头验证正确性**
    - **Validates: Requirements 2.3, 2.4, 4.1, 4.2**

- [ ] 9. Final Checkpoint - 功能验证
  - 确保所有修改完成
  - 验证CAN ID配置一致性
  - 验证数据结构定义一致性

## Notes

- Tasks marked with `*` are optional and can be skipped for faster MVP
- 本实现基于最小修改原则，不重写现有代码逻辑
- 主要修改集中在CAN ID统一和数据结构对齐
- 属性测试需要在有测试框架支持的环境中进行
