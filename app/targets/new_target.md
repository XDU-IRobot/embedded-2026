# 添加新目标

什么时候该添加一个新目标？

当造出来一辆新车（产生了一个具体的 C 板控制的实体），或者需要为现有的某辆车做一个完全全新、区别巨大的软件版本（例如：底盘控制器要从 PID 改成 MPC，或者要换一个全新的控制架构），就需要添加一个新目标。

比如：造了一个新的无人机云台，就需要建一个新的目标 `drone_gb_new`；要为 3 号老舵轮底盘做一个全新的控制器软件，就需要建一个新的目标 `steer_infantry_3_cs_v2`。

## 命名规范/缩写对照表

目标名称采用小写字母和下划线的组合。例： `steer_infantry_cs`。

常用缩写对照表：

| 缩写    | 解释                          |
| ------- | ----------------------------- |
| `gb`    | gimbal，云台                  |
| `cs`    | chassis，底盘                 |
| `meca`  | mecanum，麦克纳姆轮           |
| `steer` | steering wheel, 舵轮          |
| `omni`  | omnidirectional wheel, 全向轮 |
| `drone` | drone, 无人机                 |

## 怎么添加新目标？

添加一个新的可执行目标，按以下步骤操作：

1. 在 `app/targets/` 目录下创建一个新的子目录，名称为你的目标名称（例如 `engineer_chassis_new`）。

2. 在新创建的目录中，编写你的应用程序代码。必须包含一个 `main.cc` 文件，其中定义了 `AppMain()` 函数作为程序的入口点。

3. 打开 `app/CMakeLists.txt` 文件，在文件末尾添加以下代码来定义你的新目标：

   ```cmake
   # 新工程底盘
   file(GLOB_RECURSE ENGINEER_CHASSIS_NEW_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/engineer_chassis_new/*.cc)
   add_exe_target(engineer_chassis_new "${ENGINEER_CHASSIS_NEW_SOURCES};${COMMON_SOURCES}")
   ```

   将 `engineer_chassis_new`、`ENGINEER_CHASSIS_NEW_SOURCES` 替换为你的目标名称和对应的源文件变量名。
   **_注意命名规范！！！注意命名规范！！！注意命名规范！！！_**

完成以上步骤后，重新生成 CMake 项目，新的目标就会出现在 CLion 的可执行目标列表中，可以进行编译和调试。

![](https://github.com/user-attachments/assets/c80dcff9-7144-459c-a46f-059de8d46362)
