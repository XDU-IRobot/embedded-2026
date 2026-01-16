# 2026 赛季嵌入式组软件仓库

为最大化代码复用和管理效率，除工程机械臂、硬件等特殊项目外，赛季内大部分嵌入式软件开发工作集中在此仓库进行。

## CI status

| 模块        | 编译测试状态                                                                                                                                                                                                                       |
|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 舵轮底盘      | [![build-steer_infantry_cs](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_cs.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_cs.yml) |
| 全向轮靶车底盘   | [![build-omni_target](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-omni_target.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-omni_target.yml)                   |
| 舵轮云台      | [![build-steer_infantry_cs](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_gb.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_gb.yml) |
| 无人机云台     | [![build-drone_gb](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-drone_gb.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-drone_gb.yml)                            |
| 2025赛季老哨兵 | [![build-old_sentry](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-old_sentry.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-old_sentry.yml)                      |
| 测试平台云台    | [![build-test_gb](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-test_gb.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-test_gb.yml)                               |
| 镖架        | [![build-dart_rack](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-dart_rack.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-dart_rack.yml)                       |

## 开发指南

### Software packages

- CubeMX 6.15.0

- STM32Cube FW_F4_V1.28.3

### 目录结构

开发**_严格执行_**非侵入式原则，在 CubeMX 生成代码的基础上，所有用户代码均放置在 `app/` 目录下，对 CubeMX 生成的代码仅做最小化的改动：在
`main()` 函数里调用用户程序的入口点 `AppMain()`。

`app/` 目录包含 `common/` 和 `targets/` 两个子目录：

- `common/`: 存放通用代码模块，这些模块会被所有目标共享使用。例如：“二轴云台控制器”、“定时任务执行器”这种所有兵种开发时都会用的模块，应放在此目录下。
- `targets/`: 存放特定“目标”的代码，每一块 C 板控制的一个实体称作一个目标。例如：“3
  号老舵轮的底盘”、“新串腿底盘”、“某架无人机云台”称作一个目标。每个目标都有自己独立的代码目录，目录名即为目标名。

### 分支规则

- `main` ：主分支，始终保持可编译、可运行状态。已设置保护规则，禁止直接 push。

- `target/<目标名>`： 每个目标对应一个分支。例如 `target/steer_infantry_cs`。大多数情况下每个目标都由一个人负责，如果有例外情况，要多人合作开发一辆车，自行协调分支规则。

### 工作流程

1. 每次开始在自己的分支上工作之前，先执行一次 `git pull`、`git merge main`，确保自己的分支是基于最新的 `main` 分支进行开发的。

2. merge main之后：如果有冲突，解决冲突；如果自己的代码break掉了，想办法修好；如果自己的改动把别人的代码break掉了，协商沟通，修好之后再继续工作。

3. 阶段性完成工作，自己的分支稳定下来之后，立刻发起 Pull Request 合并回 `main` 分支。

### [指南 1：添加新模块](app/common/new_common_module.md)

### [指南 2：添加新目标](app/targets/new_target.md)

### [指南 3：OpenOCD 调试配置](openocd/README.md)
