# HelmswheelKing 底盘代码文档

## 1. 项目概述

本项目为基于 STM32F407 的全向/舵轮底盘控制系统，运行在 FreeRTOS 上。项目采用分层架构：**HAL层 → BSP层 → Alg层 → Core层**，支持全向轮（麦克纳姆轮）和舵轮两种底盘类型。

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────┐
│                   Core 层                        │
│  chassis_task.h / main.c / freertos.c           │
│  (FreeRTOS任务、底盘目标结构体、PID外部调参接口)    │
├─────────────────────────────────────────────────┤
│                   Alg 算法层                      │
│  ┌──────────┬──────────┬──────────┬──────────┐  │
│  │运动学解算 │   PID    │  ADRC    │ 功率控制  │  │
│  │(全向/舵轮)│          │(1/2阶LADRC)│(衰减法) │  │
│  ├──────────┼──────────┼──────────┼──────────┤  │
│  │  前馈补偿 │  滤波器   │ 斜坡规划  │  FSM     │  │
│  │(上坡前馈) │(KF/LPF等) │          │(状态机)  │  │
│  └──────────┴──────────┴──────────┴──────────┘  │
├─────────────────────────────────────────────────┤
│                   BSP 板级支持层                  │
│  ┌────────┬────────┬────────┬────────┬────────┐ │
│  │ DJI电机 │ DM电机  │ LK电机  │ DT7遥控 │  IMU  │ │
│  │3508/   │J4310/  │LK4005  │        │ HI12  │ │
│  │6020/   │S2325   │        │        │       │ │
│  │2006    │        │        │        │       │ │
│  ├────────┴────────┴────────┴────────┴────────┤ │
│  │  StateWatch(在线监测) │ Buzzer(蜂鸣器)      │ │
│  │  SimpleKey(按键)     │ FSM(有限状态机)      │ │
│  └─────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────┤
│                   HAL 硬件抽象层                  │
│  ┌──────────┬──────────┬──────────┬──────────┐  │
│  │ CAN总线   │ UART总线  │  DWT定时器 │ Logger  │  │
│  │(收发封装) │(收发封装) │(高精度计时) │(RTT日志)│  │
│  └──────────┴──────────┴──────────┴──────────┘  │
├─────────────────────────────────────────────────┤
│              STM32 HAL Driver / CMSIS            │
└─────────────────────────────────────────────────┘
```

---

## 3. Core 层 — 底盘任务接口

**文件**: `Core/Inc/chassis_task.h`

### 3.1 底盘目标结构体

```c
typedef struct ChassisTarget {
    float vx;   // X方向速度 (m/s)
    float vy;   // Y方向速度 (m/s)
    float vw;   // 旋转角速度 (rad/s)
} ChassisTarget;
```

### 3.2 外部接口

| 函数 | 说明 |
|------|------|
| `Chassis_Init()` | 底盘初始化 |
| `Chassis_Control_Task()` | 底盘控制任务（FreeRTOS周期调用） |

### 3.3 RTT实时调参变量

| 变量 | 说明 |
|------|------|
| `wheel_pid_kp/ki/kd` | 轮速PID参数 |
| `steer_angle_pid_kp/ki/kd` | 舵向角度PID参数 |
| `steer_velocity_pid_kp/ki/kd` | 舵向速度PID参数 |

---

## 4. Alg 算法层

### 4.1 运动学解算

**基础类**: `Alg/ChassisCalculation/CalculationBase.hpp`

| 基类 | 功能 | 核心成员 |
|------|------|----------|
| `ForwardKinematicsBase` | 正向运动学基类 | `w[4]` 四轮角速度 |
| `InverseDynamicsBase` | 逆向动力学基类 | `Fx, Fy, Torque` |
| `InverseKinematicsBase` | 逆向运动学基类 | `Signal_x/y/w, Phase, SpeedGain, RotationalGain` |

#### 4.1.1 全向轮解算

**文件**: `Alg/ChassisCalculation/OmniCalculation.hpp`

**命名空间**: `Alg::CalculationBase`

| 类 | 功能 | 关键方法 |
|----|------|----------|
| `Omni_FK` | 正向运动学：轮速→底盘速度 | `OmniForKinematics(w0,w1,w2,w3)` |
| `Omni_ID` | 逆向动力学：力/力矩→轮扭矩 | `OmniInvDynamics(fx,fy,torque)` |
| `Omni_IK` | 逆向运动学：底盘速度→轮速 | `OmniInvKinematics(vx,vy,vw,phase,speed_gain,rotate_gain)` |

**构造参数**:
- `R`: 中心到轮子投影点距离
- `S`: 轮子半径
- `Wheel_Azimuth[4]`: 轮子滚动方向角（相对车体X轴），典型值 `{0, π/2, π, 3π/2}`
- `Wheel_Direction[4]` / `Wheel_Coordinate[4][2]`: 轮子位置方向角或坐标

**IK计算流程**: 设置信号→旋转矩阵计算速度分量(`CalculateVelocities`)→逆运动学解算(`InvKinematics`)

#### 4.1.2 舵轮解算

**文件**: `Alg/ChassisCalculation/StringWheel.hpp`

**命名空间**: `Alg::CalculationBase`

| 类 | 功能 | 关键方法 |
|----|------|----------|
| `String_FK` | 正向运动学 | `OmniForKinematics(w0,w1,w2,w3)` + `Set_current_steer_angles()` |
| `String_ID` | 逆向动力学 | `OmniInvDynamics(fx,fy,torque)` + `Set_current_steer_angles()` |
| `String_IK` | 逆向运动学 | `StringInvKinematics(vx,vy,vw,phase,speed_gain,rotate_gain)` |

**舵轮特有机制**:
- **就近转位** (`_Steer_Motor_Kinematics_Nearest_Transposition`): 确保舵向电机以最短路径转动，±π/2内直接转动，超出则反转轮速+偏转π
- **双输出**: IK同时输出 `Motor_wheel[4]`（轮速RPM）和 `Motor_direction[4]`（舵向角度）
- **使用前必须调用** `Set_current_steer_angles(angle, index)` 设置当前舵向角度

---

### 4.2 PID 控制器

**文件**: `Alg/PID/pid.hpp` / `Alg/PID/pid.cpp`

**命名空间**: `ALG::PID`

| 特性 | 说明 |
|------|------|
| 积分限幅 | `integral_limit_` 限制积分累积绝对值 |
| 积分分离 | `integral_separation_threshold_` 误差大于阈值时停止积分（设0禁用） |
| 输出限幅 | `max_` / `min_` 限制输出范围 |
| 抗积分饱和 | 输出饱和时回退积分累积 |

**核心接口**:
- `UpDate(target, feedback)` → 计算PID输出
- `setK(kp, ki, kd)` / `setMax(max)` / `setIntegralLimit()` / `setIntegralSeparation()`

---

### 4.3 ADRC 自抗扰控制

**文件**: `Alg/ADRC/adrc.hpp`

**命名空间**: `ALG::ADRC`

| 类 | 阶数 | 核心方法 | 关键参数 |
|----|------|----------|----------|
| `FirstLADRC` | 一阶 | `LADRC_1(input, feedback)` | `wc, w0, b0, h, max` |
| `SecondLADRC` | 二阶 | `LADRC_2(input, feedback)` | `wc, w0, b0, h, r, max` |

**参数说明**:
- `wc`: 控制器带宽 → 决定KP/KD
- `w0`: 观测器带宽 → 决定ESO增益Beta
- `b0`: 被控对象补偿因子
- `h`: 采样周期
- `r`: 跟踪微分器速度因子（仅二阶）

**二阶LADRC包含**: 跟踪微分器(TD) + 线性扩张状态观测器(LESO) + 线性状态误差反馈(LSEF)

---

### 4.4 功率控制

**文件**: `Alg/PowerControl/PowerControlBase.hpp` / `Alg/PowerControl/PowerControl.hpp`

**命名空间**: `ALG::PowerControl`

**功率模型**: `P = k0 + k1·I + k2·ω + k3·I·ω + k4·I² + k5·ω²`

| 方案 | 方法 | 特点 |
|------|------|------|
| A: 功率衰减法 | `AttenuatedPower(I, V, K, CorrectionConstant, PowerMax)` | 局部衰减，每个电机独立解方程，**可能改变电机间受力比例** |
| B: 电流衰减法 | `DecayingCurrent(I, V, K, I_other, CorrectionConstant, PowerMax)` | 全局衰减，求统一衰减系数η，**保持电流比例不变，推荐使用** |

**方案B流程**:
1. 计算各电机原始功率，区分发电/耗电
2. 若总功率未超限，直接输出原始电流
3. 若超限，构建全局方程 `A·η² + B·η + C = 0`，求解η∈[0,1]
4. 所有电机电流乘以η

---

### 4.5 前馈补偿

**文件**: `Alg/Feedforward/Feedforward.hpp`

**命名空间**: `Alg::Feedforward`

| 类 | 功能 |
|----|------|
| `Uphill` | 上坡前馈力计算 |

**原理**: 根据坡度计算总前馈力 `F = m·g·sin(θ)`，再通过三次多项式系数矩阵分配到各轮，最后转换为扭矩。

**支持的底盘类型力-扭矩转换**:
- `Omni_ForceToTorque()` — 全向轮
- `Mecanum_ForceToTorque()` — 麦克纳姆轮
- `steering_ForceToTorque()` — 舵轮

---

### 4.6 滤波器

**文件**: `Alg/Filter/Filter.hpp`

| 类 | 功能 | 核心参数 |
|----|------|----------|
| `KalmanFilter` | 卡尔曼滤波 | Q(过程噪声), R(观测噪声) |
| `TDFilter` | 跟踪微分器滤波 | R(速度因子), H(步长) |
| `LPFFilter` | 一阶低通滤波 | Ratio(0~1，越大响应越快) |
| `LMFFilter` | 限幅滤波 | Limit_Ratio(最大变化量) |

---

### 4.7 斜坡规划

**文件**: `Alg/UtilityFunction/SlopePlanning.hpp`

**命名空间**: `Alg::Utility`

平滑控制输出变化，限制上升/下降斜率，避免突变。核心方法: `TIM_Calculate_PeriodElapsedCallback(target, feedback)`

---

### 4.8 底盘状态机

**文件**: `BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp` / `FiniteStateMachine_chassis.cpp`

**类**: `Chassis_FSM`

| 状态 | 含义 | 触发条件 |
|------|------|----------|
| `STOP` | 停止 | 左右开关均在中位 / 设备离线 |
| `FOLLOW` | 跟随云台 | 左开关↑ 右开关中 |
| `NOTFOLLOW` | 非跟随 | 左开关中 右开关↑ |
| `KEYBOARD` | 键盘控制 | 左右开关均↑ |

**功能**: 状态切换统计（进入次数、运行时间）、定时更新时间计数。

---

## 5. BSP 板级支持层

### 5.1 电机驱动

**基类**: `BSP/Motor/MotorBase.hpp` — `BSP::Motor::MotorBase<N>`

统一数据结构 `UnitData`（角度/弧度/速度/电流/扭矩/温度），集成 `StateWatch` 在线检测。

#### 5.1.1 DJI电机

**文件**: `BSP/Motor/Dji/DjiMotor.hpp`

**命名空间**: `BSP::Motor::Dji`

| 电机型号 | 类名 | 减速比 | 力矩常数(Nm/A) | 最大电流(A) | 编码器分辨率 |
|----------|------|--------|----------------|-------------|-------------|
| GM2006 | `GM2006<N>` | 36 | 0.18/36 | 10 | 8192 |
| GM3508 | `GM3508<N>` | 1 | 0.3 | 20 | 8192 |
| GM6020 | `GM6020<N>` | 1 | 0.7 | 3 | 8192 |

**CAN通信**: 接收ID = `Init_id + recv_idxs[i]`，发送通过 `setCAN(data, id)` + `sendCAN()`

**构造示例**: `GM3508<4> motor(0x200, {1,2,3,4}, 0x200)`

#### 5.1.2 DM达妙电机

**文件**: `BSP/Motor/DM/DmMotor.hpp`

**命名空间**: `BSP::Motor::DM`

| 电机型号 | 类名 | 位置范围(rad) | 速度范围(rad/s) | 扭矩范围(Nm) |
|----------|------|---------------|-----------------|-------------|
| J4310 | `J4310<N>` | ±12.56 | ±45 | ±18 |
| S2325 | `S2325<N>` | ±12.5 | ±50 | ±10 |

**控制模式**:

| 模式 | 枚举值 | 方法 | CAN ID |
|------|--------|------|--------|
| MIT | `MIT=0` | `ctrl_Mit(id, pos, vel, KP, KD, torq)` | `send_idxs[id]` |
| 角度速度 | `ANGLEVELOCITY=1` | `ctrl_AngleVelocity(id, pos, vel)` | `0x100 + send_idxs[id]` |
| 速度 | `VELOCITY=2` | `ctrl_Velocity(id, vel)` | `0x200 + send_idxs[id]` |

**使能/失能/清错**: `On(id, mod)` / `Off(id, mod)` / `ClearErr(id, mod)`

#### 5.1.3 LK电机

**文件**: `BSP/Motor/LK/Lk_motor.hpp`

**命名空间**: `BSP::Motor::LK`

| 电机型号 | 类名 | 减速比 | 扭矩常数 | 最大电流(A) | 编码器分辨率 |
|----------|------|--------|----------|-------------|-------------|
| LK4005 | `LK4005<N>` | 10 | 0.06 | 4 | 65536 |

**控制方法**:
- `ctrl_Position(id, angle, speed)` — 位置控制
- `ctrl_Torque(id, torque)` — 扭矩控制（±2048限幅）
- `ctrl_Multi(iqControl[4])` — 多电机电流控制（ID=0x280）

**特有功能**: 多圈角度累计 (`getMultiAngle`, `setAllowAccumulate`)

---

### 5.2 DT7遥控器

**文件**: `BSP/RemoteControl/DT7.hpp`

**命名空间**: `BSP::REMOTE_CONTROL`

| 数据类型 | 接口 | 范围 |
|----------|------|------|
| 通道原始值 | `get_ch0~ch3()` | 364~1684 (中值1024) |
| 摇杆位置 | `get_left/right_x/y()` | -1.0~1.0 |
| 坐标值 | `get_left/right_stick_x/y()` | -660~660 |
| 开关 | `get_s1()` / `get_s2()` | 1(上)/2(下)/3(中) |
| 鼠标 | `get_mouseLeft/Right()` | bool |
| 键盘 | `get_key(KEY_W)` 等 | bool |

**死区补偿**: `SetDeadzone(value)` + `DeadzoneCompensation(value)`

**在线检测**: `isConnected()` — 离线时触发蜂鸣器报警

---

### 5.3 HI12 IMU

**文件**: `BSP/IMU/HI12_imu.hpp` / `BSP/IMU/HI12Base.hpp`

**命名空间**: `BSP::IMU`

| 类 | 功能 |
|----|------|
| `HI12Base` | 帧头校验、CRC16校验、在线检测、ORE清错 |
| `HI12_float` | 浮点数据解析 |

**数据接口**:

| 数据 | 方法 | 单位 |
|------|------|------|
| 加速度 | `GetAcc(0/1/2)` | g |
| 角速度 | `GetGyro(0/1/2)` | °/s |
| 欧拉角 | `GetAngle(0:roll/1:pitch/2:yaw)` | ° |
| 四元数 | `GetQuaternion(0:w/1:x/2:y/3:z)` | - |
| 累计Yaw | `GetAddYaw()` | ° |

**协议**: 帧头 `0x5A 0xA5`，负载长度76字节，CRC16-CCITT校验

---

### 5.4 状态监控与蜂鸣器

**文件**: `BSP/Common/StateWatch/state_watch.hpp`

`StateWatch` 类通过超时阈值判断设备在线/离线状态，被电机、IMU、遥控器统一使用。

---

### 5.5 按键

**文件**: `BSP/SimpleKey/SimpleKey.hpp`

`SimpleKey` 类支持：上升沿/下降沿检测、点击/长按判定（阈值500ms）、翻转状态。

---

## 6. HAL 硬件抽象层

### 6.1 CAN总线

**文件**: `HAL/CAN/can_hal.hpp`

统一封装CAN收发接口，提供 `HAL::CAN::Frame` 结构体和 `can_bus` / `can_device` 抽象，支持CAN1/CAN2。

### 6.2 DWT定时器

**文件**: `HAL/DWT/DWT.hpp`

单例模式，基于Cortex-M4 DWT周期计数器，提供：
- `GetDeltaT()` / `GetDeltaT64()` — 高精度时间差
- `GetTimeline_s/ms/us()` — 系统时间轴
- `Delay(seconds)` — 秒级延时

---

## 7. 底盘控制数据流

```
DT7遥控器 ──→ RemoteController.parseData()
                    │
                    ├── 通道/开关/键盘数据
                    │
                    ▼
            Chassis_FSM.StateUpdate() ──→ 状态切换(STOP/FOLLOW/NOTFOLLOW/KEYBOARD)
                    │
                    ▼
            ChassisTarget {vx, vy, vw}  ←── 遥控器摇杆映射 / 键盘控制
                    │
                    ▼
            String_IK.StringInvKinematics()  (舵轮逆运动学)
            或 Omni_IK.OmniInvKinematics()   (全向轮逆运动学)
                    │
                    ├── 轮速目标 → PID.UpDate() → 电机电流
                    ├── 舵向角度 → PID.UpDate() → 舵向电机角度
                    │
                    ▼
            PowerControl.DecayingCurrent()  (功率限制)
                    │
                    ▼
            电机驱动层 sendCAN()  →  CAN总线  →  电机执行
```

---

## 8. 关键设计要点

1. **舵轮就近转位**: 舵向电机角度差超过±π/2时，反转轮速并偏转π，避免整圈旋转
2. **功率控制推荐方案B**: 电流衰减法保持各电机电流比例，确保底盘运动方向不偏移
3. **PID抗积分饱和**: 输出饱和时回退积分项，防止超调
4. **统一在线检测**: 所有外设（电机/IMU/遥控器）通过 `StateWatch` 统一管理，离线触发蜂鸣器
5. **RTT实时调参**: PID参数暴露为全局变量，可通过SEGGER RTT实时调整
