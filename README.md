# 2026赛季嵌入式组软件仓库

为最大化代码复用和管理效率，除工程机械臂、硬件等特殊项目外，赛季内大部分嵌入式软件开发工作集中在此仓库进行。

## CI status

| 模块    | 编译测试状态                                                                                                                                                                                                                       |
|-------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 舵轮底盘  | [![build-steer_infantry_cs](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_cs.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_cs.yml) |
| 舵轮云台  | [![build-steer_infantry_cs](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_gb.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-steer_infantry_gb.yml) |
| 无人机云台 | [![build-drone_gb](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-drone_gb.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026/actions/workflows/build-drone_gb.yml)                            |

## 开发指南

### Software packages

- CubeMX 6.15.0

- STM32Cube FW_F4_V1.28.3

### 目录结构

工程开发务必***严格执行***非侵入式原则，在CubeMX生成代码的基础上，所有用户代码均放置在 `app/` 目录下，对CubeMX生成的代码仅做最小化的改动：在
`main()` 函数里调用用户程序的入口点 `AppMain()`。

`app/` 目录包含 `common/` 和 `targets/` 两个子目录：

- `common/`: 存放通用代码模块，这些模块会被所有目标共享使用。例如：“二轴云台控制器”、“定时任务执行器”这种所有兵种开发时都会用的模块，应放在此目录下。
- `targets/`: 存放特定“目标”的代码，每一块C板控制的一个实体称作一个目标。例如：“3号老舵轮的底盘”、“新串腿底盘”、“某架无人机云台”称作一个目标。每个目标都有自己独立的代码目录，目录名即为目标名。

### 添加新模块

比如造了一个新构型的东西：“大yaw轴+小二轴的云台“、”登岛月球车底盘“，需要一个新的控制器类：

把新添加的代码放到 `app/common/` 目录下即可，无需做任何其他改动。

### 添加新目标

添加一个新的可执行目标（例如，造新车了，要给他写程序），按以下步骤操作：

1. 在 `app/targets/` 目录下创建一个新的子目录，名称为你的目标名称（例如 `engineer_chassis_new`）。

2. 在新创建的目录中，编写你的应用程序代码。必须包含一个 `main.cc` 文件，其中定义了 `AppMain()` 函数作为程序的入口点。

3. 打开 `app/CMakeLists.txt` 文件，在文件末尾添加以下代码来定义你的新目标：

   ```cmake
   # My Robot
   file(GLOB_RECURSE ENGINEER_CHASSIS_NEW_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/engineer_chassis_new/*.cc)
   add_exe_target(engineer_chassis_new "${ENGINEER_CHASSIS_NEW_SOURCES};${COMMON_SOURCES}")
   ```

   将 `engineer_chassis_new` 替换为具体的名字即可，
   ***注意命名规范保持一致！！！注意命名规范保持一致！！！注意命名规范保持一致！！！***

完成以上步骤后，重新生成 CMake 项目，新的目标就会出现在CLion的可执行目标列表中，可以进行编译和调试。

### 增量开发：改目标还是改通用模块？

如果你发现了一些bug需要修复，或者看到了一些优化点，例如：发现了一个底盘功率控制算法、一个很好的滤波器、一个更优雅的状态机实现方式等：

优先考虑把这些改动放到 `common/` 目录下的通用模块里，而不是只管自己的车能跑就行。尽量把这些改动做成通用模块的改进，所有车都能受益。

### OpenOCD调试配置

`openocd/` 目录下存放了C板对应的stlink和daplink的OpenOCD配置文件，以及C板对应的svd外设描述文件，调试时自行配置使用。