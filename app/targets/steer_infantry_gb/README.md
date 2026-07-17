# steer_infantry_gb 开火延迟测量 & 单发模式

## 一、开火延迟测量

### 1.1 测量定义

> **自瞄发送开火命令 → 摩擦轮转速下降 100 RPM** 的时间间隔，单位 ms。

每次发射后统计，取平均值存入全局变量 `globals->AdelayTime`，在 100Hz 主循环中更新。

### 1.2 状态机设计

```
                  fire_cmd
     ┌─────────┐  detected   ┌──────────┐  rpm drop ≥100  ┌───────────┐
     │ kFdIdle │ ──────────> │ kFdArmed │ ──────────────> │ kFdDropped │
     └─────────┘             └──────────┘                 └───────────┘
          ^                        │                            │
          │      timeout 500ms     │   rpm recovered           │
          └────────────────────────┘   (peak - 50)              │
          ^                        │    & still firing          │
          │                        └───────────────────────────┘
          │                                      │
          │          fire stopped + timeout       │
          └──────────────────────────────────────┘
```

**三个状态：**

| 状态 | 含义 | 行为 |
|---|---|---|
| `kFdIdle` | 空闲，等待开火 | 检测到开火 → 记录 RPM 基线 & 起始 tick → 进入 `kFdArmed` |
| `kFdArmed` | 已下令开火，等待 RPM 下降 | 持续追踪 RPM 峰值；若 RPM 下降 ≥100 → 计算延迟 → 进入 `kFdDropped`；超时 500ms → 放弃 → `kFdIdle` |
| `kFdDropped` | RPM 已下降，等待恢复 | 若 RPM 恢复到 `peak - 50` 且仍在开火 → 记录新基线 → `kFdArmed`（下一发）；若停火超时 → `kFdIdle` |

### 1.3 开火判定

在 `Gimbal::ShootEnableUpdate()` 中（500Hz），通过 shoot controller 状态区分：

| 模式 | 判定条件 |
|---|---|
| **单发** | `single_shoot_flag_ == true` 且 `shoot_controller.shoot_flag() == false`（即 `Fire()` 被调用后，拨盘正在送弹） |
| **全自动** | `shoot_controller.target().loader_speed != 0.0f`（拨盘持续转动中） |

### 1.4 计时精度

| 参数 | 值 |
|---|---|
| 采样频率 | 500Hz（`ShootEnableUpdate` 调用频率） |
| 每 tick | 2ms |
| 超时保护 | 250 tick = 500ms |
| RPM 下降阈值 | 100 RPM（绝对值） |
| RPM 恢复阈值 | peak - 50 RPM |
| 延迟有效范围 | 0 ~ 500ms（超范围丢弃） |

### 1.5 数据流

```
ShootEnableUpdate() [500Hz]
    │
    ├─ 检测开火命令
    ├─ 追踪摩擦轮 RPM（取左摩擦轮绝对值）
    ├─ 状态机判定 RPM 下降
    ├─ 计算 delay_ms = (fd_tick_ - fd_arm_tick_) × 2.0
    ├─ 累加至 fd_sum_ms_ / fd_count_
    │
    v
SubLoop100Hz() [100Hz, 每 10ms]
    │
    └─ globals->AdelayTime = gimbal->GetFireDelayAvg()
       (fd_sum_ms_ / fd_count_, fd_count_==0 时返回 0.0)
```

### 1.6 涉及文件

| 文件 | 修改点 |
|---|---|
| `main.hpp:94` | `float AdelayTime = 0.0f` 全局变量 |
| `Gimbal.hpp:58-65` | 状态机变量（`fd_state_`, `fd_tick_`, `fd_peak_rpm_`, `fd_arm_tick_`, `fd_sum_ms_`, `fd_count_`） |
| `Gimbal.hpp:87` | `GetFireDelayAvg()` 公共访问器 |
| `Gimbal.cc:449-494` | 状态机核心逻辑（`ShootEnableUpdate` 末尾） |
| `Gimbal.cc:498` | `ShootDisableUpdate` 中 `fd_state_ = kFdIdle` 安全重置 |
| `main.cc:350-351` | 100Hz 更新 `AdelayTime` |

---

## 二、单发模式实现

### 2.1 参数总览

| 参数 | 值 | 来源 |
|---|---|---|
| 拨盘每圈子弹数 | 9 | `Shoot3Fric{9, 19.2f, true}` |
| 拨盘减速比 | 19.2:1 | 同上 |
| 电机编码器分辨率 | 8192 counts/rev | M3508 编码器 |
| 每发子弹的电机位移 | 17474 counts | `19.2 / 9 × 8191` |
| 摩擦轮目标转速 | ~6200 RPM | `ammo_speed_` in Gimbal.hpp |
| 单发冷却时间 | 200 tick = 400ms | `single_shoot_time_ = 200` at 500Hz |
| 拨盘位置到达窗口 | ±5000 counts | shoot_3fric.hpp Update() |
| 送弹中间速度 | ±2000 RPM | shoot_3fric.hpp Update() |

### 2.2 触发条件

在 `ShootEnableUpdate()` 中（500Hz），以下条件之一满足且 `single_shoot_flag_ == false` 时进入单发：

1. **遥控器手动单发**：`rc->dial() <= -650`
2. **自瞄打符**：`GimbalMove_ == kGbAimbotFu && suggest_fire_flag == 1`
3. **符模式右键射击**：`heat_remain > 30 && (df_state || xf_state) && mouse_right`

单发优先级 **高于** 全自动——三个 if-else 分支中单发在最前面。

### 2.3 两步时序

```
│  Fire()  │<────────── single_shoot_time_ 倒数 ──────────>│  Fire()  │
│  调用     │        200 → 199 → ... → 1 → 0                │  可再调用  │
│          │   (400ms 冷却期, 阻止连发)                      │          │
└──────────┘                                                └──────────┘
     ^                                                            ^
single_shoot_flag_ = true                              single_shoot_flag_ = false
```

代码（Gimbal.cc:409-418）：
```cpp
if (!gimbal->single_shoot_flag_) {
    globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
    globals->shoot_controller.Fire();
    gimbal->single_shoot_flag_ = true;
    gimbal->single_shoot_time_ = 200;
} else if (gimbal->single_shoot_time_ > 0) {
    gimbal->single_shoot_time_--;
} else if (gimbal->single_shoot_time_ == 0) {
    gimbal->single_shoot_flag_ = false;
}
```

### 2.4 Fire() — 拨盘目标位置计算

`Fire()` 在 `shoot_3fric.hpp:78-96`：

```cpp
void Fire() {
    if (!armed_) return;  // 摩擦轮未解锁 → 不开火
    if (mode_ == kSingleShot) {
        if (single_shoot_complete_) {  // 上一发已完成
            // direction_ = true: 正转推弹
            target_.loader_position = state_.loader_position
                + 19.2f / 9.0f * 8191.0f;  // ≈ +17474 counts
            single_shoot_complete_ = false;  // 标记"一发进行中"
        }
    }
}
```

计算：`(19.2 / 9) × 8191 = 2.133 × 8191 ≈ 17474` encoder counts。即拨盘电机转 2.133 圈（拨盘转 40°），刚好推送 1 发子弹。

### 2.5 Update() — 拨盘串级PID & 完成判断

`Update()` 在 `shoot_3fric.hpp:17-73`，每 500Hz 执行：

**完成判断（先于 PID 执行）：**
```cpp
// direction_ = true
if (loader_position >= target_.loader_position - 5000.0f) {
    single_shoot_complete_ = true;  // 到达目标附近 → 认为完成
}
```

**拨盘串级 PID（仅单发进行中时）：**
```
target_.loader_position (绝对位置)
        │
        ▼
  [位置环 PID]  ──→  中间速度 ±2000 RPM
        │
        ▼
  [速度环 PID]  ──→  output_.loader (电流值 → M3508)
```

**单发完成后：** `target_.loader_speed = 0`（`SetShootFrequency` 作用），速度环将拨盘减速至零。

**摩擦轮 PID 始终运行：**
```
target_.fric_speed ≈ 6200  ──→ [速度环PID] ──→ output_.fric_1 → friction_left
-target_.fric_speed ≈ -6200 ──→ [速度环PID] ──→ output_.fric_2 → friction_right
```

### 2.6 完整时间轴（一发子弹的视角）

```
t=0    条件满足, single_shoot_flag_=false
       → SetMode(kSingleShot)
       → Fire():
           target_.loader_position = cur_pos + 17474
           single_shoot_complete_ = false
       → single_shoot_flag_ = true, single_shoot_time_ = 200

t=1~N  Update() 每 2ms 执行:
       → 位置环 PID 驱动拨盘电机转动
       → 拨盘旋转, 子弹被推入摩擦轮间隙
       → 子弹接触高速旋转的摩擦轮 → 摩擦轮 RPM 瞬间下降 ~100-200 RPM
       → 子弹获得动能射出

t=K    loader_position 到达 target - 5000 窗口内
       → single_shoot_complete_ = true (子弹已射出)
       → 位置环退出, 速度环目标=0, 拨盘停止

t=1~200  single_shoot_time_ 从 200 递减 (400ms 冷却)
       → 期间单发条件被忽略, 防止连发

t=200  single_shoot_time_ == 0 → single_shoot_flag_ = false
       → 可以发射下一发
```

### 2.7 输出到电机的路径

```
ShootEnableUpdate()
    → shoot_controller.Update(...)  产生 output_ 结构体
    → SetMotorCurrent() [Gimbal.cc:544]
        → dial_motor->SetCurrent(output_.loader)       // 拨盘 CAN1
        → friction_left->SetCurrent(output_.fric_1)     // 左摩擦轮 CAN2
        → friction_right->SetCurrent(output_.fric_2)    // 右摩擦轮 CAN2
    → SubLoop500Hz() [main.cc:337-338]
        → DjiMotorBase::SendCommand(*can1)  // 发送拨盘电流
        → DjiMotorBase::SendCommand(*can2)  // 发送摩擦轮电流
```

### 2.8 与开火延迟测量的关联

在单发模式下，开火延迟测量状态机的触发链条：

```
Fire() 调用 → single_shoot_complete_=false
    → is_firing=true
    → fd_state_: kFdIdle → kFdArmed (记录 fd_peak_rpm_, fd_arm_tick_)
    → 等待摩擦轮 RPM 下降 100
    → kFdArmed → kFdDropped (计算 delay_ms, 累加平均)
    → 等待 RPM 恢复
    → 单发完成, is_firing=false
    → kFdDropped → kFdIdle
```
