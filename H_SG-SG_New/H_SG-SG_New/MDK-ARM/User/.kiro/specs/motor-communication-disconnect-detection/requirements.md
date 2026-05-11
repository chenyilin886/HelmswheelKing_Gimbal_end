# Requirements Document

## Introduction

本功能实现电机和板间通信的断联检测系统，通过StateWatch监控设备在线状态，并通过LED颜色显示和蜂鸣器报警提示用户设备掉线情况。系统支持舵向电机（LK4005）、轮向电机（GM3508）和板间CAN通信的断联检测。

## Glossary

- **StateWatch**: 设备在线状态监视器，通过定时检查设备数据更新时间来判断设备是否在线
- **BuzzerManager**: 蜂鸣器管理器，提供基于队列的蜂鸣器管理功能，支持多个设备的响铃请求排队处理
- **String_Motor**: 舵向电机，使用LK4005电机，共4个（ID 1-4）
- **Wheel_Motor**: 轮向电机，使用GM3508电机，共4个（ID 1-4）
- **Board_Communication**: 板间CAN通信，云台板与底盘板之间的数据交换
- **EvenTask**: 事件任务，负责更新和检测各设备的在线状态
- **LED_Indicator**: LED指示灯，通过不同颜色显示设备状态

## Requirements

### Requirement 1: 舵向电机断联检测

**User Story:** As a 机器人操作员, I want 系统能够检测舵向电机的断联状态, so that 我能及时发现电机通信故障。

#### Acceptance Criteria

1. WHEN 舵向电机CAN数据超过100ms未更新, THEN THE StateWatch SHALL 将该电机状态标记为OFFLINE
2. WHEN 舵向电机CAN数据正常接收, THEN THE StateWatch SHALL 将该电机状态标记为ONLINE
3. THE EvenTask SHALL 每5ms检查一次所有舵向电机的在线状态
4. WHEN 舵向电机状态从ONLINE变为OFFLINE, THEN THE Dir_Data_t SHALL 更新对应String数组元素为false

### Requirement 2: 轮向电机断联检测

**User Story:** As a 机器人操作员, I want 系统能够检测轮向电机的断联状态, so that 我能及时发现电机通信故障。

#### Acceptance Criteria

1. WHEN 轮向电机CAN数据超过100ms未更新, THEN THE StateWatch SHALL 将该电机状态标记为OFFLINE
2. WHEN 轮向电机CAN数据正常接收, THEN THE StateWatch SHALL 将该电机状态标记为ONLINE
3. THE EvenTask SHALL 每5ms检查一次所有轮向电机的在线状态
4. WHEN 轮向电机状态从ONLINE变为OFFLINE, THEN THE Dir_Data_t SHALL 更新对应Wheel数组元素为false

### Requirement 3: 板间通信断联检测

**User Story:** As a 机器人操作员, I want 系统能够检测板间CAN通信的断联状态, so that 我能及时发现上下板通信故障。

#### Acceptance Criteria

1. WHEN 板间通信CAN数据超过50ms未更新, THEN THE StateWatch SHALL 将通信状态标记为OFFLINE
2. WHEN 板间通信CAN数据正常接收, THEN THE StateWatch SHALL 将通信状态标记为ONLINE
3. THE EvenTask SHALL 每5ms检查一次板间通信的在线状态
4. WHEN 板间通信状态从ONLINE变为OFFLINE, THEN THE Dir_Data_t SHALL 更新Communication字段为false

### Requirement 4: LED状态显示

**User Story:** As a 机器人操作员, I want LED能够通过不同颜色显示设备掉线情况, so that 我能快速识别故障类型。

#### Acceptance Criteria

1. WHEN 任意舵向电机处于OFFLINE状态, THEN THE LED_Indicator SHALL 显示红色（0xFFFF0000）
2. WHEN 任意轮向电机处于OFFLINE状态且所有舵向电机在线, THEN THE LED_Indicator SHALL 显示蓝色（0xFF0000FF）
3. WHEN 板间通信处于OFFLINE状态且所有电机在线, THEN THE LED_Indicator SHALL 显示粉色（0xFFFFC0CB）
4. WHEN 所有设备均处于ONLINE状态, THEN THE LED_Indicator SHALL 显示正常流水灯效果
5. THE LED_Indicator SHALL 按照舵向电机 > 轮向电机 > 板间通信的优先级显示故障颜色

### Requirement 5: 蜂鸣器报警

**User Story:** As a 机器人操作员, I want 蜂鸣器能够通过不同鸣叫模式提示设备掉线情况, so that 我能通过声音识别故障设备。

#### Acceptance Criteria

1. WHEN 舵向电机ID N（1-4）处于OFFLINE状态, THEN THE BuzzerManager SHALL 鸣叫N次短音（100ms每次）
2. WHEN 轮向电机ID N（1-4）处于OFFLINE状态, THEN THE BuzzerManager SHALL 鸣叫N次短音（100ms每次）
3. WHEN 板间通信处于OFFLINE状态, THEN THE BuzzerManager SHALL 鸣叫3次长音（500ms每次）
4. THE BuzzerManager SHALL 按照队列顺序依次处理报警请求，每次报警间隔不少于500ms
5. THE BuzzerManager SHALL 避免重复添加相同设备的报警请求到队列中

### Requirement 6: 状态更新时间戳

**User Story:** As a 系统开发者, I want 电机和通信模块在接收到有效数据时更新时间戳, so that StateWatch能够准确判断设备在线状态。

#### Acceptance Criteria

1. WHEN 电机CAN回调函数接收到有效数据帧, THEN THE MotorBase SHALL 调用UpdateLastTime更新对应电机的时间戳
2. WHEN 板间通信接收到完整的三帧数据, THEN THE Gimbal_to_Chassis SHALL 调用UpdateLastTime更新通信时间戳
3. THE StateWatch SHALL 在CheckStatus时比较当前时间与上次更新时间的差值来判断超时
