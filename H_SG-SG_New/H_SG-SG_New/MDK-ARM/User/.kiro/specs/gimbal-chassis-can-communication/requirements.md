# Requirements Document

## Introduction

本文档定义了云台与底盘之间的CAN通信协议需求。云台通过两帧发送控制指令、模式和UI状态数据到底盘，同时接收底盘发送的裁判系统数据。底盘通过一帧发送裁判系统数据，通过两帧接收云台的控制数据。所有通信帧都包含帧头校验以确保数据完整性。

## Glossary

- **Gimbal（云台）**: 机器人上层控制板，负责发送控制指令和接收裁判系统数据
- **Chassis（底盘）**: 机器人下层控制板，负责接收控制指令和发送裁判系统数据
- **CAN_Bus**: Controller Area Network总线，用于云台和底盘之间的通信
- **Frame_Header（帧头）**: 数据帧开头的标识字节，用于验证数据有效性
- **Direction_Data**: 方向控制数据，包含LX、LY、旋转速度、Yaw编码器角度误差等
- **Chassis_Mode**: 底盘模式数据，包含万向、跟随、小陀螺、键鼠、停止等模式标志
- **UI_List**: UI状态数据，包含MCL、BP、F5、Shift、Vision等状态标志
- **Referee_Data**: 裁判系统数据，包含枪口热量冷却值、热量上限、当前热量等

## Requirements

### Requirement 1: 云台发送数据到底盘

**User Story:** 作为云台控制系统，我需要将控制指令、模式和UI状态发送到底盘，以便底盘能够执行相应的运动控制。

#### Acceptance Criteria

1. WHEN 云台准备发送数据 THEN Gimbal SHALL 将数据打包为两个CAN帧（Frame1: 8字节，Frame2: 剩余字节补0至8字节）
2. WHEN 发送第一帧 THEN Gimbal SHALL 使用CAN ID 0x501发送，包含帧头0xA5和Direction数据的前7字节
3. WHEN 发送第二帧 THEN Gimbal SHALL 使用CAN ID 0x502发送，包含剩余的Direction数据、ChassisMode和UiList数据
4. THE Gimbal SHALL 在每个发送周期内按顺序发送两帧数据
5. THE Direction_Data SHALL 包含LX、LY、Rotating_vel、Yaw_encoder_angle_err、target_offset_angle和Power字段
6. THE Chassis_Mode SHALL 包含Universal_mode、Follow_mode、Rotating_mode、KeyBoard_mode和stop位字段
7. THE UI_List SHALL 包含MCL、BP、UI_F5、Shift、Vision、aim_x和aim_y字段

### Requirement 2: 底盘接收云台数据

**User Story:** 作为底盘控制系统，我需要接收并解析云台发送的控制指令，以便执行正确的运动控制。

#### Acceptance Criteria

1. WHEN 底盘接收到CAN ID 0x501的帧 THEN Chassis SHALL 将数据存入接收缓冲区的前8字节
2. WHEN 底盘接收到CAN ID 0x502的帧 THEN Chassis SHALL 将数据存入接收缓冲区的第8-15字节
3. WHEN 两帧数据都接收完成 THEN Chassis SHALL 验证帧头是否为0xA5
4. IF 帧头验证失败 THEN Chassis SHALL 丢弃该数据包并重置接收状态
5. WHEN 帧头验证成功 THEN Chassis SHALL 解析Direction、ChassisMode和UiList数据
6. IF 接收帧间隔超过50ms THEN Chassis SHALL 重置接收状态并丢弃不完整数据

### Requirement 3: 底盘发送裁判系统数据到云台

**User Story:** 作为底盘控制系统，我需要将裁判系统数据发送到云台，以便云台能够获取热量信息进行射击控制。

#### Acceptance Criteria

1. WHEN 底盘准备发送裁判系统数据 THEN Chassis SHALL 将数据打包为一个CAN帧（8字节）
2. WHEN 发送裁判系统帧 THEN Chassis SHALL 使用CAN ID 0x505发送
3. THE Referee_Data帧 SHALL 包含帧头0x21和0x12作为前两个字节
4. THE Referee_Data帧 SHALL 包含booster_heat_cd（2字节）、booster_heat_max（2字节）和booster_now_heat（2字节）

### Requirement 4: 云台接收裁判系统数据

**User Story:** 作为云台控制系统，我需要接收底盘发送的裁判系统数据，以便进行热量管理和射击控制。

#### Acceptance Criteria

1. WHEN 云台接收到CAN ID 0x505的帧 THEN Gimbal SHALL 验证帧头是否为0x21和0x12
2. IF 帧头验证失败 THEN Gimbal SHALL 丢弃该数据帧
3. WHEN 帧头验证成功 THEN Gimbal SHALL 解析booster_heat_cd、booster_heat_max和booster_now_heat数据
4. THE Gimbal SHALL 提供getter方法获取解析后的裁判系统数据

### Requirement 5: CAN ID配置一致性

**User Story:** 作为系统集成工程师，我需要确保云台和底盘使用一致的CAN ID配置，以便通信能够正常工作。

#### Acceptance Criteria

1. THE Gimbal发送帧ID SHALL 与Chassis接收帧ID保持一致（0x501、0x502）
2. THE Chassis发送帧ID SHALL 与Gimbal接收帧ID保持一致（0x505）
3. THE 帧头标识 SHALL 在发送端和接收端保持一致（云台发送0xA5，底盘发送0x21+0x12）
4. WHEN CAN ID配置不一致 THEN 通信 SHALL 失败且接收端无法解析数据

### Requirement 6: 数据结构对齐

**User Story:** 作为开发者，我需要确保云台和底盘的数据结构定义一致，以便数据能够正确解析。

#### Acceptance Criteria

1. THE Direction结构体 SHALL 在云台和底盘使用相同的packed属性和字段顺序
2. THE ChassisMode结构体 SHALL 在云台和底盘使用相同的位字段定义
3. THE UiList结构体 SHALL 在云台和底盘使用相同的位字段定义
4. THE Referee_Data结构体 SHALL 在云台和底盘使用相同的packed属性和字段顺序
