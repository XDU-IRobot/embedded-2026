# 电机控制异常 — 待修复问题清单

> 基于 2026-05-23 代码审查，按优先级排序。问题一已由用户自行修复，问题三已完成。

---

## 严重

- [ ] **CAN2 总线关闭自动恢复** (`Core/Src/can.c:80`)
  `hcan2.Init.AutoBusOff = DISABLE;` → 改为 `ENABLE`（与 CAN1 一致）。同时在 `bxcan.cc` 注册 `CAN_IT_ERROR` / `CAN_IT_BUSOFF` 中断，仿照 `fdcan.cc` 已有的 `Restart()` 逻辑。
  **后果**: CAN2 挂载 yaw(GM6020)、拨盘(M2006)、镜头(M2006)，总线故障后全部永久离线。

- [ ] **BxCan::Write() 无超时自旋锁死** (`libs/librm/src/librm/hal/stm32/bxcan.cc:149-150`)
  `while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0) { }` 无超时。添加 ~500μs 超时（约 10000 次迭代 @168MHz），超时返回 false。仿照 `fdcan.cc:217-237` 的超时+重试模式。
  **后果**: CAN 拥塞时整个系统永久挂起，无看门狗触发。

- [x] ~~**DjiMotorBase::SendCommand 脏标志复制粘贴错误**~~ ✅ 已修复
  `libs/librm/src/librm/device/actuator/dji_motor.hpp:119,123` — `dirty_2ff` → `dirty_1fe` / `dirty_2fe`

---

## 高

- [ ] **状态转换时 PID 未清除** (`app/targets/drone_gb_new/ControllerPidGimbal.hpp:31-35`)
  `enabled_` 为 false 时输出清零但 PID 内部状态保留。重新使能时残留积分项引发瞬态跳动。在 `Enable(true)` 时调用 `ClearAllPid()`（参考 `ControllerPidGimbalV1.0.hpp:226-236`）。
  **后果**: 模式切换时云台意外跳动。

- [ ] **偏航前馈硬编码偏移与类成员不一致** (`Acontrol.cc:70` vs `gimbal.hpp:46`)
  `5.14` vs `yaw_center_encoder = 5.174`，差约 0.034 rad ≈ 2°。改用 `drone_gb.yaw_center_encoder` 或定义单一命名常量。
  **后果**: 前馈扭矩计算偏离约 2°，校准后不会自动同步。

- [ ] **镜头堵转检测阈值过于严格** (`app/targets/drone_gb_new/Acontrol.cc:399-401`)
  `kStallThreshold = 3` 编码器计数 / 8ms。M2006 @500RPM 每 8ms 移动约 544 计数。改为 ≥50 计数，或改用基于时间的超时（如 100ms 内无运动）。
  **后果**: 加速瞬态、CAN 帧丢失、低速运行（<22 RPM）时误触发堵转。

- [ ] **CAN ISR 共享数据缺少 volatile** (`libs/librm/src/librm/device/actuator/dji_motor.hpp:304-310`)
  `encoder_`, `rpm_`, `current_`, `temperature_` 由 CAN ISR 写入、主循环读取，均非 volatile。同样的问题存在于 `app/common/aimbot_comm_can.hpp:25-31`（`aimbot_state_`, `yaw_`, `pitch_` 等）。全部添加 volatile 或改用 atomic 访问。
  **后果**: LTO 优化下编译器可能缓存陈旧值，导致反馈回路失控。

---

## 中

- [ ] **模糊 PID pitch 位置 d_error_scale 未设置** (`app/targets/drone_gb_new/ControllerPidGimbal.hpp:19`)
  `.SetFuzzy(true).SetFuzzyErrorScale(M_PI)` 缺少 `.SetFuzzyDErrorScale(...)`。参考偏航位置已设置 `SetFuzzyDErrorScale(M_PI * 100)`。
  **后果**: Pitch 位置模糊推理被完全绕过（`ec` 恒为 0），模糊整定对该轴无效。

- [ ] **PID 积分 windup 无反计算钳位** (`libs/librm/src/librm/modules/pid.cc:87`)
  总输出 `Clamp(P+I+D)` 后，I 项未在钳位时反向修正。实现反计算钳位（back-calculation clamping）。
  **后果**: 长时间饱和后恢复时过冲。

- [ ] **DM 电机 FloatToInt 缺少输入限幅** (`libs/librm/src/librm/modules/utils.cc:90-94`)
  缩放前用 `std::clamp(x, x_min, x_max)` 包裹输入。同样检查 `dm_motor.hpp:158-162` 的调用处。
  **后果**: 超出范围的位置/速度值产生截断的 CAN 数据。

- [ ] **裁判系统子协议 handler 在未知子命令时崩溃** (`app/targets/drone_gb_new/UI/referee_user.hpp:64`)
  `mapSize.at(subCmdID)` → 改用 `mapSize.find(subCmdID)`，未知 ID 时 return。
  **后果**: 格式错误的 0x301 数据包使整个系统自旋挂起。

- [ ] **裁判系统子协议 memcpy 缺少缓冲区大小** (`app/targets/drone_gb_new/UI/referee_user.hpp:93,112`)
  `Referee0x301Prepare()` 和 `RefereePrepare()` 对调用方提供的 `u8 *data` 无大小参数。添加大小参数并在写入前验证。
  **后果**: 调用方缓冲区过小时溢出。

- [ ] **所有堆分配缺少 OOM 检查** (`app/targets/drone_gb_new/gimbal.hpp:180-205`)
  ~15 个 `new` 分配均无空指针检查。STM32 + `-fno-exceptions` 下 `new` 失败行为未定义。添加空指针检查，或迁移到静态/栈分配。
  **后果**: 堆耗尽时未定义行为（典型表现为空指针解引用 crash）。

---

## 低

- [ ] **共用状态枚举可能导致交叉污染** (`app/targets/drone_gb_new/gimbal.hpp:135-146`)
  `StateMachineType` 的值来自两个不同语义域：`kNoForce=0/kManual=1/kAuto=2`（云台）和 `kStop=0/kReady=1/kFire=2`（发射机构）。拆分为两个独立枚举（`enum class`）。

- [ ] **Shoot2Fric::mode_ 从未被读取** (`app/common/controllers/shoot_2firc.hpp:44-46`)
  `SetMode()` 存储但 `Update()` 从未使用。删除死代码或完成实现。

- [ ] **pitch_torque 对 CONTROLLER_CHOICE==1 是死代码** (`Acontrol.cc:61-62`)
  500Hz 计算 `cos(pitch)` 但结果未被使用。添加 `#if` 守卫或删除。

- [ ] **WS2812b 亮度除零风险** (`WS2812b.cc:27`)
  当 brightness > 90 时 `tan(angle)` 可能为 0 或负值。添加 bounds check。

- [ ] **CAN FIFO1 未使用** (`bxcan.cc:162`)
  可利用 FIFO1 分散 RX 负载，降低高总线流量下的溢出风险。非紧迫。

- [ ] **DMA2_Stream1(pri0) 可抢占 CAN IRQ(pri1)** (`dma.c:54`)
  考虑降低 DMA 优先级或缩短其 ISR 执行时间。
