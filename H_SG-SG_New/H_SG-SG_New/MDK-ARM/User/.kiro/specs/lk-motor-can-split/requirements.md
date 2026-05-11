# 需求文档

## 简介

将四个LK4005舵向电机从单一CAN1总线分配到CAN1和CAN2两条总线上，以优化CAN总线负载和提高系统可靠性。电机1、2保留在CAN1，电机3、4迁移到CAN2。控制逻辑保持不变。

## 术语表

- **LK_Motor_System**: LK4005电机控制系统，负责管理四个舵向电机
- **CAN_Bus**: CAN通信总线，系统中有CAN1和CAN2两条总线
- **Motor_Group**: 电机组，按CAN总线分为CAN1组（电机1、2）和CAN2组（电机3、4）
- **Broadcast_Mode**: 广播模式，使用单一CAN ID（0x280/0x281）同时控制多个电机
- **Control_Logic**: 控制逻辑，包括扭矩控制、位置控制等电机控制算法

## 需求

### 需求 1：电机CAN总线分配

**用户故事：** 作为系统开发者，我希望将四个LK电机分配到两条CAN总线上，以便降低单条总线负载并提高通信可靠性。

#### 验收标准

1. THE LK_Motor_System SHALL 将电机1和电机2配置在CAN1总线上
2. THE LK_Motor_System SHALL 将电机3和电机4配置在CAN2总线上
3. WHEN 电机配置完成后，THE LK_Motor_System SHALL 保持每个电机的原有ID映射关系不变

### 需求 2：多总线广播控制

**用户故事：** 作为系统开发者，我希望能够通过广播模式同时控制两条总线上的电机，以便保持原有的控制效率。

#### 验收标准

1. WHEN 调用多电机控制函数时，THE LK_Motor_System SHALL 向CAN1发送电机1、2的控制数据
2. WHEN 调用多电机控制函数时，THE LK_Motor_System SHALL 向CAN2发送电机3、4的控制数据
3. THE LK_Motor_System SHALL 在两条CAN总线上均使用0x280作为广播ID

### 需求 3：数据接收与解析

**用户故事：** 作为系统开发者，我希望系统能够正确接收和解析来自两条CAN总线的电机反馈数据，以便准确获取所有电机状态。

#### 验收标准

1. WHEN CAN1收到电机1或电机2的反馈帧时，THE LK_Motor_System SHALL 正确解析并更新对应电机的状态数据
2. WHEN CAN2收到电机3或电机4的反馈帧时，THE LK_Motor_System SHALL 正确解析并更新对应电机的状态数据
3. THE LK_Motor_System SHALL 为每个电机维护独立的状态监视器用于断联检测

### 需求 4：单电机控制兼容

**用户故事：** 作为系统开发者，我希望单电机控制函数（On/Off/ClearErr/ctrl_Torque/ctrl_Position）能够自动选择正确的CAN总线，以便保持API兼容性。

#### 验收标准

1. WHEN 对电机1或电机2执行单电机控制时，THE LK_Motor_System SHALL 通过CAN1发送控制命令
2. WHEN 对电机3或电机4执行单电机控制时，THE LK_Motor_System SHALL 通过CAN2发送控制命令
3. THE LK_Motor_System SHALL 保持所有单电机控制函数的接口签名不变

### 需求 5：控制逻辑无感知

**用户故事：** 作为系统开发者，我希望上层控制逻辑无需修改即可正常工作，以便最小化代码改动范围。

#### 验收标准

1. THE Control_Logic SHALL 继续使用相同的电机ID（1-4）访问所有电机
2. THE Control_Logic SHALL 继续使用相同的API接口控制电机
3. WHEN 上层调用电机控制函数时，THE LK_Motor_System SHALL 自动处理CAN总线路由
4. THE LK_Motor_System SHALL 保持电机数据结构（unit_data_）的访问方式不变

### 需求 6：断联检测支持

**用户故事：** 作为系统开发者，我希望断联检测功能能够正确识别两条总线上的电机状态，以便及时发现通信故障。

#### 验收标准

1. WHEN 电机1或电机2在CAN1上超时未响应时，THE LK_Motor_System SHALL 将对应电机标记为离线
2. WHEN 电机3或电机4在CAN2上超时未响应时，THE LK_Motor_System SHALL 将对应电机标记为离线
3. THE LK_Motor_System SHALL 为每条CAN总线独立维护通信状态
