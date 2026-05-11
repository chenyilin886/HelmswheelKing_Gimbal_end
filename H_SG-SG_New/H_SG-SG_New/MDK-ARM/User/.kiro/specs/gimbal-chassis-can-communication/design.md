# Design Document: Gimbal-Chassis CAN Communication

## Overview

本设计文档描述了云台与底盘之间CAN通信的实现方案。基于现有代码结构，通过最小化修改实现双向通信：
- 云台 → 底盘：两帧发送（控制指令、模式、UI状态）
- 底盘 → 云台：一帧发送（裁判系统数据）

设计原则是**不重写代码**，仅修改现有实现以统一CAN ID和数据结构。

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         CAN Bus (CAN1)                          │
└─────────────────────────────────────────────────────────────────┘
        ▲                                           ▲
        │ 0x501, 0x502                              │ 0x505
        │ (Direction, Mode, UI)                     │ (Referee Data)
        │                                           │
┌───────┴───────┐                           ┌───────┴───────┐
│    Gimbal     │                           │    Chassis    │
│  (发送端)      │                           │   (发送端)     │
│               │                           │               │
│  Data_send()  │                           │  Transmit()   │
└───────────────┘                           └───────────────┘
        │                                           │
        │ 0x505                                     │ 0x501, 0x502
        │ (Referee Data)                           │ (Direction, Mode, UI)
        ▼                                           ▼
┌───────────────┐                           ┌───────────────┐
│    Gimbal     │                           │    Chassis    │
│  (接收端)      │                           │   (接收端)     │
│               │                           │               │
│ HandleCAN()   │                           │ HandleCAN()   │
└───────────────┘                           └───────────────┘
```

## Components and Interfaces

### 1. CAN ID 定义（统一配置）

云台和底盘需要使用相同的CAN ID定义：

```cpp
// 云台 → 底盘
#define CAN_G2C_FRAME1_ID 0x501  // 第一帧：帧头 + Direction前7字节
#define CAN_G2C_FRAME2_ID 0x502  // 第二帧：Direction剩余 + ChassisMode + UiList

// 底盘 → 云台
#define CAN_C2G_FRAME1_ID 0x505  // 裁判系统数据帧
```

### 2. 云台发送模块 (Gimbal Data_send)

**现有代码位置**: 云台 `CommunicationTask.cpp` 中的 `Gimbal_to_Chassis::Data_send()`

**修改要点**:
- 保持现有数据打包逻辑
- 确保CAN ID使用0x501和0x502
- 帧头使用0xA5

**数据帧结构**:
```
Frame1 (0x501): [0xA5][LX][LY][Rotating_vel][Yaw_err(4bytes)]
Frame2 (0x502): [target_offset][Power][ChassisMode(1)][UiList(3)][0][0][0]
```

### 3. 底盘接收模块 (Chassis ParseCANFrame)

**现有代码位置**: 底盘 `CommunicationTask.cpp` 中的 `Gimbal_to_Chassis::ParseCANFrame()`

**修改要点**:
- 监听CAN ID 0x501和0x502
- 两帧接收完成后验证帧头0xA5
- 50ms超时重置接收状态

### 4. 底盘发送模块 (Chassis Transmit)

**现有代码位置**: 底盘 `CommunicationTask.cpp` 中的 `Gimbal_to_Chassis::Transmit()`

**修改要点**:
- 使用CAN ID 0x505
- 帧头使用0x21和0x12

**数据帧结构**:
```
Frame (0x505): [0x21][0x12][heat_cd(2)][heat_max(2)][now_heat(2)]
```

### 5. 云台接收模块 (Gimbal HandleCANMessage)

**现有代码位置**: 云台 `CommunicationTask.cpp` 中的 `Gimbal_to_Chassis::HandleCANMessage()`

**修改要点**:
- 监听CAN ID 0x505
- 验证帧头0x21和0x12
- 解析裁判系统数据

## Data Models

### Direction 结构体 (10 bytes)
```cpp
struct __attribute__((packed)) Direction {
    uint8_t LX;                    // 1 byte - 左摇杆X
    uint8_t LY;                    // 1 byte - 左摇杆Y
    uint8_t Rotating_vel;          // 1 byte - 旋转速度
    float Yaw_encoder_angle_err;   // 4 bytes - Yaw编码器角度误差
    uint8_t target_offset_angle;   // 1 byte - 目标偏移角度
    int8_t Power;                  // 1 byte - 功率
};  // Total: 9 bytes
```

### ChassisMode 结构体 (1 byte)
```cpp
struct __attribute__((packed)) ChassisMode {
    uint8_t Universal_mode : 1;    // 万向模式
    uint8_t Follow_mode : 1;       // 跟随模式
    uint8_t Rotating_mode : 1;     // 小陀螺模式
    uint8_t KeyBoard_mode : 1;     // 键鼠模式
    uint8_t stop : 1;              // 停止模式
};  // Total: 1 byte
```

### UiList 结构体 (3 bytes)
```cpp
struct __attribute__((packed)) UiList {
    uint8_t MCL : 1;
    uint8_t BP : 1;
    uint8_t UI_F5 : 1;
    uint8_t Shift : 1;
    uint8_t Vision : 2;
    uint8_t aim_x;                 // 1 byte
    uint8_t aim_y;                 // 1 byte
};  // Total: 3 bytes
```

### RxRefree/Booster 结构体 (8 bytes)
```cpp
struct __attribute__((packed)) Booster {
    uint8_t heat_one;              // 1 byte - 帧头1 (0x21)
    uint8_t heat_two;              // 1 byte - 帧头2 (0x12)
    uint16_t booster_heat_cd;      // 2 bytes - 冷却值
    uint16_t booster_heat_max;     // 2 bytes - 热量上限
    uint16_t booster_now_heat;     // 2 bytes - 当前热量
};  // Total: 8 bytes
```

### 数据帧布局

**云台 → 底盘 (总计14字节)**:
```
Byte 0:     帧头 (0xA5)
Byte 1-9:   Direction (9 bytes)
Byte 10:    ChassisMode (1 byte)
Byte 11-13: UiList (3 bytes)

Frame1 (0x501): Byte 0-7
Frame2 (0x502): Byte 8-13 + padding
```

**底盘 → 云台 (总计8字节)**:
```
Byte 0:     帧头1 (0x21)
Byte 1:     帧头2 (0x12)
Byte 2-3:   booster_heat_cd
Byte 4-5:   booster_heat_max
Byte 6-7:   booster_now_heat

Frame (0x505): Byte 0-7
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system-essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: 数据往返一致性 (Round-trip)

*For any* valid Direction、ChassisMode和UiList数据组合，云台打包发送后，底盘接收解析得到的数据应与原始数据完全一致。

**Validates: Requirements 1.1, 1.5, 1.6, 1.7, 2.5, 6.1, 6.2, 6.3**

### Property 2: 裁判系统数据往返一致性 (Round-trip)

*For any* valid Booster/RxRefree数据，底盘打包发送后，云台接收解析得到的热量数据应与原始数据完全一致。

**Validates: Requirements 3.1, 3.4, 4.3, 6.4**

### Property 3: 帧头验证正确性

*For any* 接收到的CAN数据帧，如果帧头不匹配预期值（云台发送0xA5，底盘发送0x21+0x12），则数据应被丢弃且不更新内部状态。

**Validates: Requirements 2.3, 2.4, 4.1, 4.2**

### Property 4: 数据打包帧数量正确

*For any* 云台发送操作，应产生恰好两个CAN帧（ID 0x501和0x502）；*For any* 底盘发送操作，应产生恰好一个CAN帧（ID 0x505）。

**Validates: Requirements 1.1, 1.2, 1.3, 3.1, 3.2**

## Error Handling

### 帧头验证失败
- 底盘接收到的数据帧头不是0xA5时，丢弃数据包
- 云台接收到的数据帧头不是0x21+0x12时，丢弃数据包

### 超时处理
- 底盘接收两帧数据间隔超过50ms时，重置接收状态
- 丢弃不完整的数据包

### CAN发送失败
- 保持现有的CAN HAL错误处理机制
- 发送失败不影响下一周期的发送

## Testing Strategy

### 单元测试
1. 数据结构大小验证 - 确保packed结构体大小符合预期
2. 帧头常量验证 - 确保帧头值定义正确
3. CAN ID常量验证 - 确保CAN ID定义一致

### 属性测试 (Property-Based Testing)
使用C++的属性测试框架（如RapidCheck）进行测试：

1. **Property 1测试**: 生成随机Direction、ChassisMode、UiList数据，验证打包-解包往返一致性
2. **Property 2测试**: 生成随机Booster数据，验证打包-解包往返一致性
3. **Property 3测试**: 生成带有效/无效帧头的数据，验证帧头验证逻辑
4. **Property 4测试**: 验证发送操作产生的帧数量

### 集成测试
1. 实际CAN总线通信测试
2. 双向通信延迟测试
3. 超时恢复测试

**测试配置**:
- 每个属性测试运行至少100次迭代
- 测试标签格式: **Feature: gimbal-chassis-can-communication, Property N: [property_text]**
