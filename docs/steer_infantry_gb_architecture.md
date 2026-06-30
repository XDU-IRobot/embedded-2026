# steer_infantry_gb 信息流向与状态机文档

## 1. 系统概述

该目标平台为 RoboMaster 步兵机器人云台（单板+妙算NUC）。主控芯片运行 1kHz 主循环，通过 CAN 总线与底盘、NUC（妙算）、电机通信。NUC 运行视觉算法，给出目标位置和开火建议。

---

## 2. 信息流向

### 2.1 硬件拓扑与总线分配

```
┌─────────────────────────────────────────────────────────────┐
│  主控 (STM32)                                                │
│                                                              │
│  SPI1 ─── BMI088 (IMU)                                       │
│  USART3 ── DR16 (遥控器)                                      │
│  USART6 ── 裁判系统                                           │
│  USART1 ── 调试串口                                           │
│                                                              │
│  CAN1 ───┬── GM6020 (yaw电机, ID:4)                           │
│          ├── M3508 (拨盘电机, ID:5)                            │
│          ├── 超级电容                                          │
│          └── 底盘通信 (ChassisCommunicator)                    │
│                                                              │
│  CAN2 ───┬── DM4310 (pitch电机)                               │
│          ├── M3508 (左摩擦轮, ID:1)                            │
│          ├── M3508 (右摩擦轮, ID:2)                            │
│          └── NUC通信 (AimbotCanCommunicator)                  │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 主循环数据流 (1kHz)

```
SubLoop500Hz (每周期执行)
│
├─ ① IMU更新 → AHRS姿态解算
│     imu->Update() → ahrs.Update()
│     产出: yaw, pitch, roll (欧拉角)
│
├─ ② 发送控制数据给 NUC (CAN2)
│     aimbot_communicator->UpdateControl(yaw, pitch, roll, robot_id, aim_mode, imu_count, ammo_speed)
│     产出: 告知NUC当前姿态和模式
│
├─ ③ 遥控器状态更新 → 状态机转换
│     RCStateUpdate()
│     产出: StateMachine_, GimbalMove_
│
├─ ④ 底盘/UI 状态更新
│     ChassisStateUpdate()
│     产出: chassis_move_x, chassis_move_y, chassis_state, aim_mode, ui_refresh_flag
│
├─ ⑤ 发送底盘控制指令 (CAN1)
│     chassis_communicator->SendChassisCommand(...)
│     产出: 底盘运动、UI显示、热量数据
│
├─ ⑥ 云台任务 (GimbalTask)
│     GimbalStateUpdate()
│     ├─ 电机使能/失能判断
│     ├─ 目标角度计算 (自瞄/遥控/辨识/前馈验证)
│     ├─ 位置+速度PID控制
│     ├─ 发射机构控制 (ShootEnableUpdate/ShootDisableUpdate)
│     └─ 电流输出
│
└─ ⑦ CAN 发送电机指令
      DjiMotorBase::SendCommand(CAN1)  → yaw电机、拨盘电机
      DjiMotorBase::SendCommand(CAN2)  → pitch电机、摩擦轮
```

### 2.3 NUC ↔ 主控 数据交换 (CAN2)

```
主控 → NUC (UpdateControl):
  - yaw, pitch, roll       : 云台当前姿态角 (来自AHRS)
  - robot_id               : 机器人ID (红方=3, 蓝方=103)
  - aim_mode               : 自瞄模式 (0x01=普通, 0x02=大符, 0x03=小符)
  - imu_count              : IMU计数
  - bullet_speed           : 弹速 (m/s)

NUC → 主控 (RxCallback):
  - aimbot_state()         : 状态位域
      bit0: get_target_flag  (是否有有效目标)
      bit1: suggest_fire_flag (是否建议开火)
  - yaw(), pitch()         : 目标绝对角度 (弧度)
  - yaw_vel(), pitch_vel() : 目标角速度 (rad/s, 前馈)
  - yaw_acc(), pitch_acc() : 目标角加速度 (rad/s², 前馈)
  - nuc_start_flag()       : NUC启动标志
```

### 2.4 底盘 ↔ 主控 数据交换 (CAN1)

```
主控 → 底盘 (SendChassisCommand):
  - chassis_move_x, chassis_move_y : 底盘目标速度
  - chassis_state     : 底盘状态 (小陀螺、大/小符模式等)
  - ui_refresh_flag   : UI刷新标志
  - get_target_flag   : 有目标标志 (回传用于UI)
  - suggest_fire_flag : 建议开火标志 (回传用于UI)
  - aim_speed_change  : 弹速调整量
  - robot_hp[5]       : 机器人血量

底盘 → 主控 (RxCallback):
  - heat_limit(), heat_real() : 热量上限 / 实时热量
  - ammo_speed()              : 底盘供弹速度 (m/s)
  - gimbal/chassis/ammo_power_state() : 各模块供电状态
  - robot_id()                : 机器人ID
```

### 2.5 裁判系统数据流

```
裁判系统串口 → RxReferee → VT03 (image_data)
  - keyboard_key  : 键鼠按键状态 (16位, 每位对应一个键)
  - mouse_x/y     : 鼠标坐标
  - mouse_button_left/right : 鼠标左右键
  - robot_custom_data_3 : 自定义数据 (含血量)

image_update_flag = device_referee.all_device_ok()
  为 true 时, 优先使用裁判系统键鼠数据
  为 false 时, 降级使用遥控器 DR16 键鼠数据
```

---

## 3. 状态机

### 3.1 状态枚举 (main.hpp:17-28)

```
kUnable  (0)  断电模式 — 遥控器离线或供电异常
kNoForce (1)  无力模式 — 所有电机失能, 初始化倒计时
kTest    (2)  调试模式 — 云台使能, 发射机构有条件使能
kMatch   (3)  比赛模式 — 云台+发射机构全使能

kGbRemote   (4)  云台遥控 (手动)
kGbAimbot   (5)  云台自瞄 (NUC视觉跟踪)
kGbAimbotFu (6)  云台打符 (能量机关)
kGbIdentify (7)  参数辨识 (扫频)
kGbFfVerify (8)  前馈验证
```

前4个是系统级状态 (`StateMachine_`)，后5个是云台运动子状态 (`GimbalMove_`)。

### 3.2 系统级状态转换 (main.cc:110-172)

触发源：遥控器右拨杆

```
                  ┌──────────┐
      设备离线/    │          │  初始化完成
      供电异常     │ kUnable  │──────────────────┐
      ┌──────────→│  (断电)   │                  │
      │           └──────────┘                  │
      │                                         ▼
      │                                    ┌──────────┐
      │                  右拨杆=下           │ kNoForce │
      │              ┌─────────────────────│  (无力)   │
      │              │                     └──────────┘
      │              │                          │
      │              │         右拨杆=上/中      │
      │              ▼                          ▼
      │         ┌──────────┐             ┌──────────┐
      │         │  kMatch  │             │  kTest   │
      │         │  (比赛)   │             │  (调试)   │
      │         └──────────┘             └──────────┘
      │              │                          │
      └──────────────┴──────────────────────────┘
              供电/设备异常
```

**RCStateUpdate() 完整映射表：**

| 右拨杆 | 左拨杆 | StateMachine_ | GimbalMove_ |
|:------:|:------:|:-------------:|:-----------:|
| Up     | Down   | kMatch        | (不设置)    |
| Up     | Mid    | kTest         | kGbFfVerify |
| Up     | Up     | kTest         | kGbIdentify |
| Mid    | Down   | kTest         | kGbRemote   |
| Mid    | Mid    | kTest         | kGbAimbotFu |
| Mid    | Up     | kTest         | kGbAimbot   |
| Down   | Up     | kNoForce      | (不变)      |
| Down   | Mid    | kNoForce      | (不变)      |
| Down   | Down   | kNoForce      | (不变)      |

特殊情况：
- 初始化倒计时 `init_time > 0` → 强制进入 kNoForce
- 遥控器设备离线 或 底盘未给云台供电 → 强制进入 kUnable
- 进入 kMatch 时不设置 GimbalMove_，由其内部逻辑动态决定

### 3.3 云台运动子状态转换（动态）

#### 3.3.1 比赛模式 kMatch (Gimbal.cc:315-322)

```
GimbalMatchUpdate() 每周期执行:

    aimbot_state bit0 == 1 ?         (NUC 有目标)
       YES → GimbalMove_ = kGbAimbot (→ 自瞄跟踪)
       NO  → GimbalMove_ = kGbRemote (→ 退回手动遥控)

这是动态闭环: 每一帧根据 NUC 数据自动切换
```

#### 3.3.2 测试模式 kTest (Gimbal.cc:324-366)

```
GimbalMove_ 由遥控器进入 kTest 时一次性设定，保持不变:

    kGbRemote   → GimbalRCTargetUpdate()    (手动摇杆/鼠标控制)
    kGbAimbot   → GimbalAimbotTargetUpdate() (NUC目标跟踪)
    kGbAimbotFu → GimbalAimbotTargetUpdate() (打符跟踪)
    kGbIdentify → GimbalIdentifyUpdate()     (参数辨识扫频)
    kGbFfVerify → GimbalFfVerifyUpdate()     (前馈验证)

无动态切换 — 需要切换时需通过遥控器离开再进入 kTest
```

### 3.4 云台跟踪模式内部逻辑 (Gimbal.cc:195-213)

```
GimbalAimbotTargetUpdate():

  kTest 模式:
    NUC有目标(bit0==1) → 使用NUC下发角度+速度/加速度前馈
    NUC无目标         → 回退到GimbalRCTargetUpdate (手动)

  kMatch 模式:
    鼠标右键按下 → 使用NUC下发角度+速度/加速度前馈
    鼠标右键松开 → 回退到GimbalRCTargetUpdate (手动)
```

### 3.5 发射机构状态转换 (Gimbal.cc:141-166)

```
Shoot 线控判断 (每周期):

  kMatch:
    始终 → ShootEnableUpdate() (允许开火)

  kTest:
    GimbalMove_ == kGbAimbot   → ShootEnableUpdate() (允许开火)
    GimbalMove_ == kGbAimbotFu → ShootEnableUpdate() (允许开火)
    GimbalMove_ == kGbRemote   → ShootDisableUpdate() (禁止开火)
    其他                       → ShootDisableUpdate() (禁止开火)

  kNoForce / 默认:
    → ShootDisableUpdate() (禁止开火)

设备离线/供电异常 始终 → ShootDisableUpdate()
```

---

## 4. 开火决策流程 (Gimbal.cc:398-449)

```
ShootEnableUpdate() 每周期执行

优先级: 单发 > 全自动 > 停火

┌─ 条件1: 单发 ─────────────────────────────────────────────┐
│ dial <= -650                                               │
│ 或 GimbalMove_==kGbAimbotFu && NUC建议开火(bit1)           │
│ 或 大/小符模式 && 右键 && 热量余量>30                       │
│                                                            │
│ → SetMode(kSingleShot) + Fire()                            │
│ → 200ms debounce 防止连发                                   │
└────────────────────────────────────────────────────────────┘

┌─ 条件2: 全自动连发 ───────────────────────────────────────┐
│                                                            │
│ kTest:                                                     │
│   (NUC有目标(bit0) && NUC建议开火(bit1))   ← 全自动         │
│   或 dial >= 650                           ← 手动强制       │
│                                                            │
│ kMatch:                                                    │
│   (鼠标左键 && !鼠标右键)                  ← 手动           │
│   或 (鼠标右键 && NUC有目标 && NUC建议开火) ← 半自动        │
│                                                            │
│ → SetMode(kFullAuto)                                       │
│ → 射速 = f(热量余量):                                      │
│     热量余量 > 100 → 20Hz                                   │
│     热量余量 < 30  → 0Hz (过热停射)                         │
│     其他          → (热量余量)/5 Hz (线性)                  │
└────────────────────────────────────────────────────────────┘

┌─ 条件3: 停火 ─────────────────────────────────────────────┐
│ 以上都不满足                                                │
│ → SetShootFrequency(0.0f)                                  │
│ → 清除单发标志                                              │
└────────────────────────────────────────────────────────────┘
```

---

## 5. aim_mode 切换逻辑 (main.cc:245-258)

| 场景 | aim_mode | 含义 |
|:----|:-------:|:----|
| kMatch + F键 | 0x02 | 大符模式 |
| kMatch + G键 | 0x03 | 小符模式 |
| kMatch 无按键 | 0x01 | 普通自瞄 |
| kTest + kGbAimbot | 0x01 | 普通自瞄 |
| kTest + kGbAimbotFu + 拨轮切换 | 0x02 ↔ 0x03 | 大/小符切换 |
| 失能时 | 0x01 | 复位 |

`aim_mode` 通过 CAN2 发送给 NUC，影响 NUC 端的视觉处理策略。

---

## 6. 关键信号速查表

| 信号 | 来源 | 含义 |
|:----|:----|:----|
| `rc->switch_r()` | DR16 遥控器 | 右拨杆位置 (Up/Mid/Down) |
| `rc->switch_l()` | DR16 遥控器 | 左拨杆位置 (Up/Mid/Down) |
| `rc->dial()` | DR16 遥控器 | 拨轮值 (-660~660) |
| `rc->mouse_button_left()` | DR16 / 裁判系统 | 鼠标左键 |
| `rc->mouse_button_right()` | DR16 / 裁判系统 | 鼠标右键 |
| `aimbot_state() bit0` | NUC (CAN2) | get_target_flag — 是否有有效目标 |
| `aimbot_state() bit1` | NUC (CAN2) | suggest_fire_flag — 是否建议开火 |
| `aimbot_state() yaw/pitch` | NUC (CAN2) | 目标绝对角度 (弧度) |
| `heat_limit()` | 底盘 (CAN1) | 热量上限 |
| `heat_real()` | 底盘 (CAN1) | 实时热量 |
| `ammo_speed()` | 底盘 (CAN1) | 供弹速度 (m/s) |
| `gimbal_power_state()` | 底盘 (CAN1) | 云台供电状态 |
| `ammo_power_state()` | 底盘 (CAN1) | 发射机构供电状态 |
| `image_update_flag` | 裁判系统 | 裁判系统链路是否在线 |
| `device_*.all_device_ok()` | 内部 | 各模块设备是否全部在线 |
