# Design Document

## Overview

本设计文档描述电机和板间通信断联检测系统的架构和实现细节。系统基于现有的StateWatch和BuzzerManager组件，通过观察者模式在EvenTask中统一管理设备状态检测，并通过LED和蜂鸣器向用户反馈设备状态。

## Architecture

系统采用分层架构，各层职责明确：

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │    LED      │  │   Buzzer    │  │     EvenTask        │  │
│  │  (Observer) │  │  (Observer) │  │  (Subject/Manager)  │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                      BSP Layer                               │
│  ┌─────────────────────────────────────────────────────────┐│
│  │                   StateWatch                             ││
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐  ││
│  │  │String[4] │ │Wheel[4]  │ │Board Comm│ │BuzzerMgr   │  ││
│  │  └──────────┘ └──────────┘ └──────────┘ └────────────┘  ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│                      Motor Layer                             │
│  ┌─────────────────────┐  ┌─────────────────────────────┐   │
│  │  LK4005 (舵向电机)   │  │  GM3508 (轮向电机)          │   │
│  │  - state_watch_[4]  │  │  - state_watch_[4]          │   │
│  └─────────────────────┘  └─────────────────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│                   Communication Layer                        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              Gimbal_to_Chassis                           ││
│  │              - state_watch_                              ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

## Components and Interfaces

### 1. MotorBase 增强

在电机基类中添加状态监控接口：

```cpp
// BSP/Motor/MotorBase.hpp
template <uint8_t N> class MotorBase
{
protected:
    BSP::WATCH_STATE::StateWatch state_watch_[N];  // 已存在

public:
    // 新增：更新指定电机的时间戳
    void updateTimestamp(uint8_t id)
    {
        if (id >= 1 && id <= N) {
            state_watch_[id - 1].UpdateLastTime();
        }
    }
    
    // 新增：检查指定电机是否在线
    bool isMotorOnline(uint8_t id)
    {
        if (id >= 1 && id <= N) {
            state_watch_[id - 1].UpdateTime();
            state_watch_[id - 1].CheckStatus();
            return state_watch_[id - 1].GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }
        return false;
    }
    
    // 新增：获取掉线电机ID（返回第一个掉线的电机ID，0表示全部在线）
    uint8_t getFirstOfflineMotorId()
    {
        for (uint8_t i = 0; i < N; i++) {
            state_watch_[i].UpdateTime();
            state_watch_[i].CheckStatus();
            if (state_watch_[i].GetStatus() == BSP::WATCH_STATE::Status::OFFLINE) {
                return i + 1;
            }
        }
        return 0;
    }
};
```

### 2. LK4005 电机时间戳更新

在Parse函数中添加时间戳更新：

```cpp
// BSP/Motor/Lk/Lk_motor.hpp - Parse函数
void Parse(const HAL::CAN::Frame &frame) override
{
    for (uint8_t i = 0; i < N; ++i)
    {
        if (frame.id == init_address + recv_idxs_[i])
        {
            // ... 现有解析代码 ...
            Configure(i, feedback_[i]);
            
            // 新增：更新时间戳
            this->state_watch_[i].UpdateLastTime();
        }
    }
}
```

### 3. GM3508 电机时间戳更新

在Parse函数中添加时间戳更新：

```cpp
// BSP/Motor/Dji/DjiMotor.hpp - Parse函数
void Parse(const HAL::CAN::Frame &frame) override
{
    const uint16_t received_id = frame.id;

    for (uint8_t i = 0; i < N; ++i)
    {
        if (received_id == init_address + recv_idxs_[i])
        {
            // ... 现有解析代码 ...
            Configure(i);
            
            // 新增：更新时间戳
            this->state_watch_[i].UpdateLastTime();
        }
    }
}
```

### 4. EvenTask 状态更新

增强Dir类的状态检测功能：

```cpp
// Task/EvenTask.cpp
bool Dir::Dir_String()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.String[i] = BSP::Motor::LK::Motor4005.isMotorOnline(i + 1);
        if (!DirData.String[i]) {
            allOnline = false;
            // 请求蜂鸣器报警
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

bool Dir::Dir_Wheel()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1);
        if (!DirData.Wheel[i]) {
            allOnline = false;
            // 请求蜂鸣器报警（轮向电机ID偏移4，使用5-8）
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

bool Dir::Dir_Communication()
{
    DirData.Communication = Gimbal_to_Chassis_Data.isConnectOnline();
    if (!DirData.Communication) {
        // 请求板间通信报警
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
    }
    return DirData.Communication;
}

void Dir::UpEvent()
{
    Dir_Remote();
    Dir_String();
    Dir_Wheel();
    Dir_Communication();
    Init_Flag();
}
```

### 5. LED 显示逻辑

更新LED的Update函数实现优先级显示：

```cpp
// APP/LED.cpp
bool LED::Update(void)
{
    Dir *dir = static_cast<Dir *>(sub);

    // 优先级1：舵向电机掉线 - 红色
    if (dir->isAnyStringOffline()) {
        aRGB_led_show(RED);
        return false;
    }
    
    // 优先级2：轮向电机掉线 - 蓝色
    if (dir->isAnyWheelOffline()) {
        aRGB_led_show(BLUE);
        return false;
    }
    
    // 优先级3：板间通信掉线 - 粉色
    if (!dir->getDir_Communication()) {
        aRGB_led_show(PINK);
        return false;
    }
    
    // 所有设备在线 - 正常流水灯
    Normal_State();
    return true;
}
```

### 6. BuzzerManager 板间通信报警

板间通信报警设计为3次长音（500ms），与电机报警区分：

```cpp
// BSP/Common/StateWatch/buzzer_manager.cpp - processRing函数
void BuzzerManagerSimple::processRing(uint8_t id)
{
    if (id == 0xFF)  // 遥控器
    {
        controlBuzzer(true);
        osDelay(LONG_BEEP_MS);
        controlBuzzer(false);
        osDelay(LONG_BEEP_MS);
    }
    else if(id == 0xFE) // 板间通讯 - 3次长音
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            controlBuzzer(true);
            osDelay(LONG_BEEP_MS);  // 500ms
            controlBuzzer(false);
            
            if (i < 2)
                osDelay(BETWEEN_BEEP_MS);
        }
        osDelay(AFTER_BEEP_MS);
    }
    else  // 电机，id=1-8
    {
        for (uint8_t i = 0; i < id; i++)
        {
            controlBuzzer(true);
            osDelay(SHORT_BEEP_MS);  // 100ms
            controlBuzzer(false);
            
            if (i < id - 1)
                osDelay(BETWEEN_BEEP_MS);
        }
        osDelay(AFTER_BEEP_MS);
    }
}
```

## Data Models

### 设备状态数据结构

```cpp
// Task/EvenTask.hpp
struct Dir_Data_t
{
    bool Dr16;              // 遥控器在线状态
    bool MeterPower;        // 功率计在线状态
    bool String[4];         // 舵向电机在线状态 [0-3] 对应 ID 1-4
    bool Wheel[4];          // 轮向电机在线状态 [0-3] 对应 ID 1-4
    bool Communication;     // 板间通信在线状态
    bool SuperCap;          // 超级电容在线状态
    bool InitFlag;          // 初始化标志
};
```

### 蜂鸣器报警ID映射

| ID值 | 设备类型 | 报警模式 |
|------|----------|----------|
| 1-4  | 舵向电机 | N次短音(100ms) |
| 1-4  | 轮向电机 | N次短音(100ms) |
| 0xFE | 板间通信 | 3次长音(500ms) |
| 0xFF | 遥控器   | 1次长音(500ms) |
| 0xFD | 陀螺仪   | 1次超长音(1000ms) |

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system-essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: 超时检测正确性

*For any* StateWatch实例，当当前时间与上次更新时间的差值大于等于超时阈值时，CheckStatus应将状态设置为OFFLINE；当差值小于超时阈值时，状态应为ONLINE。

**Validates: Requirements 1.1, 1.2, 2.1, 2.2, 3.1, 3.2, 6.3**

### Property 2: 状态同步正确性

*For any* 电机或通信设备，当其StateWatch状态为OFFLINE时，对应的Dir_Data_t字段（String[i]、Wheel[i]或Communication）应为false；当状态为ONLINE时，对应字段应为true。

**Validates: Requirements 1.4, 2.4, 3.4**

### Property 3: LED优先级显示正确性

*For any* 设备状态组合，LED显示颜色应遵循以下优先级规则：
- 若任意String[i]为false，显示RED
- 否则若任意Wheel[i]为false，显示BLUE
- 否则若Communication为false，显示PINK
- 否则显示正常流水灯

**Validates: Requirements 4.1, 4.2, 4.3, 4.5**

### Property 4: 蜂鸣器报警请求正确性

*For any* 电机ID（1-8），当该电机离线时，BuzzerManager队列中应包含该ID的报警请求，且processRing应鸣叫ID次短音。

**Validates: Requirements 5.1, 5.2**

### Property 5: 队列去重正确性

*For any* 设备ID，多次调用requestMotorRing或requestCommunicationRing添加相同ID时，队列中该ID只应出现一次。

**Validates: Requirements 5.5**

### Property 6: 时间戳更新正确性

*For any* 有效的CAN数据帧，Parse函数处理后应调用UpdateLastTime更新对应设备的last_update_time_为当前系统时间。

**Validates: Requirements 6.1, 6.2**

## Error Handling

### 超时处理

- StateWatch在计时器溢出时（32位计数器回绕）正确计算时间差
- 当设备长时间离线后重新上线，状态应正确恢复为ONLINE

### 队列溢出处理

- BuzzerManager队列满（12个请求）时，新的报警请求将被忽略
- 队列处理采用FIFO顺序，确保先到的报警先处理

### 无效ID处理

- requestMotorRing对无效ID（<1或>8）不做处理
- isMotorOnline对无效ID返回false

## Testing Strategy

### 单元测试

由于本项目是嵌入式C++项目，运行在STM32平台上，传统的单元测试框架（如Google Test）难以直接应用。建议采用以下测试策略：

1. **模拟测试**：在PC端使用模拟HAL_GetTick()的方式测试StateWatch逻辑
2. **硬件在环测试**：通过实际硬件验证电机和通信的断联检测

### 属性测试

对于可在PC端模拟的组件，可使用属性测试验证：

1. **StateWatch超时检测**：生成随机的时间差值，验证状态判断正确性
2. **BuzzerManager队列去重**：生成随机的ID序列，验证队列中无重复
3. **LED优先级逻辑**：生成随机的设备状态组合，验证颜色选择正确性

### 集成测试

1. 断开舵向电机CAN线，验证LED变红、蜂鸣器鸣叫对应次数
2. 断开轮向电机CAN线，验证LED变蓝、蜂鸣器鸣叫对应次数
3. 断开板间通信CAN线，验证LED变粉、蜂鸣器鸣叫3次长音
4. 同时断开多个设备，验证优先级显示正确

