# Steer Infantry（舵轮步兵）代码解析

## 概述

本项目为 RoboMaster 舵轮步兵机器人的嵌入式固件，基于 STM32F407 平台开发。舵轮步兵采用**双板架构**，分为两个独立的 CMake Target：

| Target | 分支 | 功能 | 定位 |
|--------|------|------|------|
| `steer_infantry_gb` | `target/steer_infantry_gb` | 云台控制板（Gimbal Board） | 上部云台、发射机构、遥控器接收、自瞄通信 |
| `steer_infantry_cs` | `target/steer_infantry_cs` | 底盘控制板（Chassis/Steering Board） | 四舵轮底盘运动控制、裁判系统处理、UI 交互 |

两块板之间通过 **CAN 总线** 进行数据交互。云台板向下发送底盘运动指令和 UI 刷新数据，底盘板向上回传热量、弹速、供能状态等信息。

---

## 项目目录结构

```
embedded-2026/
├── app/
│   ├── CMakeLists.txt                  # 所有 target 的构建配置
│   ├── common/                         # 公共模块（多 target 共用）
│   │   ├── controllers/                # 运动控制算法
│   │   │   ├── gimbal_2dof.hpp         # 二轴云台 PID 控制器
│   │   │   ├── quad_steering_chassis.hpp # 四舵轮底盘运动学+控制
│   │   │   ├── shoot_3fric.hpp         # 三摩擦轮发射机构控制器
│   │   │   └── ...
│   │   ├── aimbot_comm_can.hpp          # 自瞄 CAN 通信协议
│   │   ├── rgb_led.hpp                 # RGB LED 控制
│   │   ├── buzzer.hpp                  # 蜂鸣器控制
│   │   └── ...
│   └── targets/
│       ├── steer_infantry_gb/          # 云台板固件
│       │   ├── main.hpp / main.cc      # 入口 + 全局仓库
│       │   ├── Gimbal.hpp / Gimbal.cc  # 云台控制逻辑
│       │   ├── ChassisCommunicator.hpp # 底盘 CAN 通信（发送端）
│       │   └── Referee.hpp / Referee.cc # 裁判系统串口接收
│       └── steer_infantry_cs/          # 底盘板固件
│           ├── main.hpp / main.cc      # 入口 + 全局仓库
│           ├── Chassis.hpp / Chassis.cc # 底盘控制逻辑
│           ├── GimbalCommunicator.hpp  # 云台 CAN 通信（接收端）
│           ├── Referee.hpp / Referee.cc # 裁判系统串口接收
│           ├── UI/                     # 裁判系统可视化 UI
│           │   ├── UIInfantry.hpp/cc   # 己方机器人状态 UI
│           │   └── UIuser1.hpp/cc      # 敌方/友方信息 UI（红蓝差分）
│           └── subReferee/             # 裁判系统 0x301 子协议
│               ├── protocol_user.hpp   # 自定义子协议数据结构
│               ├── referee_user.hpp    # 子协议解析与组包
│               └── TaskScheduler.hpp   # UI 任务调度器
├── libs/librm/                         # 通用机器人库（子模块）
├── Core/                               # STM32 HAL 层代码
├── Drivers/                            # STM32 驱动
├── Middlewares/                        # 中间件
└── CMakeLists.txt                      # 顶层 CMake
```

---

## 一、steer_infantry_cs（底盘控制板）

### 1.1 功能总览

底盘控制板负责：
- **四舵轮运动控制** — 解析目标速度，执行舵轮运动学正解 + PID 控制
- **CAN 通信** — 接收云台板下发的运动指令，回传底盘状态（热量、弹速、供能）
- **裁判系统处理** — 解析裁判串口数据，提取比赛信息（敌方位置、血量、金币、Buff 等）
- **UI 渲染** — 通过裁判系统 0x301 自定义协议，向操作手屏幕绘制图形界面

### 1.2 核心文件解析

#### `main.hpp` / `main.cc` — 全局仓库与主循环

定义 `GlobalWarehouse` 结构体，持有所有硬件、设备、控制器的指针。这是整个固件的"全局上下文"。

**硬件清单：**

| 设备 | 型号 | 用途 |
|------|------|------|
| can1 / can2 | CAN 总线 | 与云台板、电机通信 |
| referee_uart | UART | 接收裁判系统数据 |
| imu | BMI088 | 底盘姿态检测 |
| super_cap | 港科超级电容 | 功率缓冲 |
| yaw_motor | GM6020 | 底盘上 Yaw 轴控制 |
| steer_lf/rf/lb/rb | GM6020 ×4 | 四轮转向舵电机 |
| wheel_lf/rf/lb/rb | M3508 ×4 | 四轮驱动电机 |
| buzzer + led | — | 状态指示 |

**主循环调度（分频执行）：**
```
SubLoop500Hz()   → 设备状态检查、电机控制输出
SubLoop250Hz()   → 底盘运动学更新、UI 刷新
SubLoop100Hz()   → 裁判系统数据更新、遥控器状态更新
SubLoop50Hz()    → 
SubLoop30Hz()    → UI 任务调度
SubLoop10Hz()    → 音乐播放
```

**状态机：**
```
kUnable  → 断电模式
kNoForce → 无力模式（电机不输出）
kTest    → 调试模式
kMatch   → 比赛模式
  ├── kFollow    → 跟随模式
  ├── kRotate    → 旋转（小陀螺）模式
  └── kReRotate  → 反旋转模式
      ├── kNormalSpeed → 正常速度
      └── kHighSpeed   → 高速模式
          ├── kNormal  → 正常 buff
          ├── kDaFu    → 大符 buff
          └── kXiaoFu  → 小符 buff
```

#### `Chassis.hpp` / `Chassis.cc` — 底盘运动控制

核心控制器为 `QuadSteeringChassis`（位于 `app/common/controllers/quad_steering_chassis.hpp`）。

**控制架构：**
1. **运动学正解** — 根据目标 vx/vy/w 和当前舵角，计算每个舵轮的目标角度和轮速
2. **舵角控制** — 支持位置环（单环）和速度-位置串级（双环）两种模式
3. **轮速控制** — 速度环 PID 跟踪运动学解算的目标轮速
4. **功率限制** — 根据裁判系统反馈动态限速

#### `GimbalCommunicator.hpp/cc` — 云台 CAN 通信

接收云台板通过 CAN 下发的数据：
- `remote_speed_x/y` — 底盘目标速度
- `chassis_mode` — 底盘运动模式
- `UI_show_flag` — UI 显示标志
- `get_target_flag` — 目标检测标志
- `suggest_fire_flag` — 建议开火标志
- `aim_speed_change` — 弹速调整值
- `robot_hp[]` — 5 台机器人血量

发送给云台的数据：
- `current_heat` / `heat_limit` — 热量状态
- `ammo_speed` — 当前弹速
- `power_state` — 供能状态
- `robot_id` — 机器人 ID

#### `subReferee/` — 裁判系统子协议

##### `protocol_user.hpp`
定义裁判系统 **0x301** 用户自定义子协议的全部数据结构：

- **雷达信息** — `EnemyRobotPosition`（敌方位置）、`EnemyRobotHP`（敌方血量）、`EnemyGoldCoinRFID`（金币/场地交互）
- **机器人信息** — `AllyRobotPosition`（友方位置）、`EnemyRobotBuff`（增益状态）、`MapRobotPosition`（小地图位置）
- **联动信息** — `Hero2Drone`/`Drone2Hero`（英雄-无人机大吊射数据）
- **UI 图形** — `UIFigure`（基础图形元素，支持直线/矩形/圆/椭圆/弧/文本）、`UIFigure1/2/5/7`（批量图形容器）、`UICharacter`（字符串）
- **命令码映射** — 通过 `TypeToCmd<T>` 模板自动推导子命令 ID
- **内存映射** — `RefereeSubProtocolMemoryMap` 利用 `offsetof` 和 `eternal::map` 实现字节流零拷贝反序列化

##### `referee_user.hpp`
- `RefereeUser` 类：裁判系统用户协议处理器，将串口字节流按 `cmd_id` 映射拷贝到对应的结构体字段
- `Referee0x301Prepare()`：组包函数，将数据结构序列化为裁判系统 0x301 协议帧（SOF + 长度 + 序列号 + CRC8 + CMD_ID + 子协议 + CRC16）
- `RefereePrepare()`：专门为 `MapRobotPosition` 组包的函数

##### `TaskScheduler.hpp`
UI 任务调度器，用于管理裁判系统 UI 图形的周期性发送（限 30Hz，最多 30 个任务）：
- `UITask` — 单个 UI 任务（函数指针 + 频率）
- 支持**动态任务**（按频率轮流调度）和**静态任务**（一次性发送，适合初始化图形元素）
- 采用**累积优先级调度算法**：每次调用 `schedule()` 选择 `acc` 值最大的任务执行，确保低频任务不被饿死

#### `UI/` — 可视化界面

##### `UIInfantry.hpp/cc`
绘制己方机器人状态的自定义 UI：
- `UIInfantryAdd1/2/3/4()` — 添加初始图形元素（底盘模式、速度模式、热量条等）
- `UIInfantryEdit()` — 更新动态数据

##### `UIuser1.hpp/cc`
绘制红/蓝双方敌方信息的 UI（"红蓝差分"）：
- 红方信息波：对手机器人 ID、血量、弹丸允许量
- 蓝方信息波：对手机器人 ID、血量、弹丸允许量
- 通过裁判系统传递的 `EnemyRobotHP` 和 `EnemyRobotProjectileAllowance` 数据动态更新

---

## 二、steer_infantry_gb（云台控制板）

### 2.1 功能总览

云台控制板负责：
- **云台姿态控制** — Yaw/Pitch 双轴 PID 控制，支持遥控跟随、自瞄跟随、打符模式
- **发射机构控制** — 三摩擦轮速度控制 + 拨盘单发/连发控制
- **遥控器接收** — DR16 接收机 DBUS 协议解析
- **自瞄通信** — 与 NUC（迷你电脑）通过 CAN 交换自瞄数据
- **底盘指令下发** — 通过 CAN 向底盘板发送运动命令
- **裁判系统接收** — 解析裁判数据，提取血量、热量、图像等信息

### 2.2 核心文件解析

#### `main.hpp` / `main.cc` — 全局仓库

**硬件清单：**

| 设备 | 型号 | 用途 |
|------|------|------|
| can1 / can2 | CAN 总线 | 与底盘板、NUC 通信 |
| dbus | UART | DR16 遥控器接收 |
| referee_uart | UART | 裁判系统数据 |
| imu | BMI088 | 云台姿态 |
| yaw_motor | GM6020 | Yaw 轴电机 |
| pitch_motor | DM Motor (MIT) | Pitch 轴电机 |
| friction_left/right | M3508 ×2 | 摩擦轮 |
| dial_motor | M3508 | 拨盘供弹 |
| nuc | CAN | 自瞄迷你电脑 |
| super_cap | 港科超级电容 | 功率缓冲 |

**状态机：**
```
kUnable        → 断电模式
kNoForce       → 无力模式
kTest          → 调试模式
kMatch         → 比赛模式
  ├── kGbRemote   → 遥控模式（手动控制）
  ├── kGbAimbot   → 自瞄模式（NUC 自动瞄准）
  └── kGbAimbotFu → 打符模式（能量机关）
kGbIdentify    → 系统辨识模式
kGbFfVerify    → 前馈验证模式
```

#### `Gimbal.hpp` / `Gimbal.cc` — 云台控制

核心控制器为 `Gimbal2Dof`（位于 `app/common/controllers/gimbal_2dof.hpp`）：
- Yaw 轴：环形 PID（支持模糊积分），位置-速度串级控制
- Pitch 轴：限幅 PID，位置-速度串级控制
- 支持遥控跟随（`GimbalRCTargetUpdate`）、自瞄跟随（`GimbalAimbotTargetUpdate`）、扫描模式（`GimbalScanTargetUpdate`）

发射控制使用 `Shoot3Fric`（位于 `app/common/controllers/shoot_3fric.hpp`）：
- 三摩擦轮速度闭环控制
- 拨盘支持**单发模式**（位置-速度串级，精确控制单发角度）和**全自动模式**（速度环，按射频连续供弹）
- 热量限制逻辑

**特殊功能：**
- **系统辨识**（`GimbalIdentifyUpdate`）— 扫频正弦信号激励，用于系统模型辨识
- **前馈验证**（`GimbalFfVerifyUpdate`）— 阶跃响应测试，验证前馈参数

#### `ChassisCommunicator.hpp/cc` — 底盘 CAN 通信

向底盘板发送：
- `chassis_move_x/y` — 底盘目标速度
- `chassis_state` — 底盘模式
- `ui_refresh_flag` — UI 刷新标志
- `get_target_flag` / `suggest_fire_flag` — 自瞄状态
- `aim_speed_change` — 弹速调整
- `hp1~hp5` — 机器人血量

从底盘板接收：
- `heat_real/limit` — 热量状态
- `ammo_speed` — 弹速
- `robot_id` — 机器人 ID
- `power_state` — 供能状态

#### `Referee.hpp/cc` — 裁判系统接收

与底盘板的 Referee 类似，但额外包含 `VT03` 图像数据接收（裁判系统图像信道）。

---

## 三、双板通信架构

```
┌─────────────────────────────────────────────────────────┐
│                    CAN Bus                               │
│                                                          │
│  ┌───────────────────┐       ┌───────────────────┐      │
│  │ steer_infantry_gb │──────▶│ steer_infantry_cs │      │
│  │   (云台板)         │◀──────│   (底盘板)          │      │
│  │                    │       │                    │      │
│  │ ChassisCommunicator│      │ GimbalCommunicator │      │
│  │  发送:              │      │  接收:              │      │
│  │  · 底盘目标速度     │      │  · 底盘目标速度     │      │
│  │  · 底盘模式         │      │  · 底盘模式         │      │
│  │  · 自瞄状态         │      │  · 自瞄状态         │      │
│  │  · 弹速调整         │      │  · 弹速调整         │      │
│  │  · 机器人血量       │      │  · 机器人血量       │      │
│  │  接收:              │      │  发送:              │      │
│  │  · 热量/热量上限    │      │  · 热量/热量上限    │      │
│  │  · 弹速             │      │  · 弹速             │      │
│  │  · 供能状态         │      │  · 供能状态         │      │
│  │  · 机器人 ID        │      │  · 机器人 ID        │      │
│  └───────────────────┘       └───────────────────┘      │
│           │                          │                   │
│           │ UART                     │ UART              │
│           ▼                          ▼                   │
│     ┌──────────┐              ┌──────────┐              │
│     │ 裁判系统  │              │ 裁判系统  │              │
│     │ (VT03图像)│              │ (0x301子协议)│           │
│     └──────────┘              └──────────┘              │
│                                                        │
│     ┌──────────┐                                       │
│     │ DR16遥控  │── DBUS ──▶ 云台板                     │
│     └──────────┘                                       │
│     ┌──────────┐                                       │
│     │ NUC(自瞄) │── CAN ──▶ 云台板                      │
│     └──────────┘                                       │
└─────────────────────────────────────────────────────────┘
```

- 云台板接收遥控器和 NUC 自瞄数据，综合处理后下发底盘运动指令
- 底盘板执行运动控制，并通过裁判系统 0x301 协议向操作手屏幕绘制 UI
- 两块板独立接收裁判系统串口数据（云台板额外接收 VT03 图像）

---

## 四、公共控制器详解

### 4.1 `QuadSteeringChassis` — 四舵轮底盘控制器

```
输入: vx, vy, w (目标速度/角速度)
  │
  ▼
运动学正解 (SteeringChassis::Forward)  → 每个舵轮的目标角度 + 目标轮速
  │
  ├──▶ 舵角控制 (位置环 / 速度-位置双环) → 舵电机电流
  │
  └──▶ 轮速控制 (速度环) → 轮电机电流
```

- 特殊逻辑：当轮速高于阈值且无平移指令时，锁死当前舵角（避免低速抖动引起舵角摆动）

### 4.2 `Gimbal2Dof` — 二轴云台控制器

```
目标 Yaw/Pitch 位置 + Yaw 前馈
  │
  ├──▶ Yaw: 环形 PID 位置环 → 速度环 → 电流输出
  │      (支持模糊积分 + 速度前馈 + 力矩前馈)
  │
  └──▶ Pitch: PID 位置环 → 速度环 → 电流输出
         (支持模糊积分)
```

### 4.3 `Shoot3Fric` — 三摩擦轮发射机构控制器

```
模式选择:
  ├── 单发: 拨盘位置环 → 拨盘速度环 → 精确旋转一个子弹间距
  │        摩擦轮速度保持
  └── 全自动: 摩擦轮速度环 + 拨盘速度环持续旋转
             射频决定拨盘 RPM
```

---

## 五、关键设计特点

1. **双板 CAN 通信** — 云台和底盘物理分离，通过 CAN 总线实现低延迟（1kHz）数据同步
2. **零拷贝反序列化** — 裁判系统子协议利用 `eternal::map` + `offsetof` 实现 O(1) 字节流到结构体的映射，无需 switch-case 解析
3. **UI 任务调度** — 裁判系统 UI 带宽有限（30Hz），采用累积优先级调度确保多任务公平分配
4. **红蓝差分 UI** — 根据己方颜色自动切换显示红方/蓝方信息，适配比赛场景
5. **系统辨识模式** — 云台板内置正弦扫频 + 阶跃响应测试功能，用于系统建模和参数整定
6. **功率管理** — 底盘板根据裁判系统功率限制 + 超级电容状态动态调整速度上限

---

## 六、构建与编译

本项目使用 **CMake + ARM GCC** 工具链，STM32CubeMX 生成 HAL 层代码。

```bash
# 配置构建
cmake --preset default

# 编译特定 target
cmake --build build --target steer_infantry_gb
cmake --build build --target steer_infantry_cs
```

Target 在 `app/CMakeLists.txt` 中通过 `add_exe_target` 宏定义，自动扫描 targets 目录下的 `.cc` 源文件并链接公共模块。

---

## 七、依赖关系

```
steer_infantry_gb  ──依赖──▶  steer_infantry_cs (CAN 通信)
       │                            │
       ├── librm (机器人库)  ◀──────┤
       ├── common/controllers       │
       ├── common/aimbot_comm_can   │
       └── common/buzzer/led        │
                                    │
                            common/controllers/quad_steering_chassis (独享)
                            subReferee/ (独享)
                            UI/ (独享)
```
