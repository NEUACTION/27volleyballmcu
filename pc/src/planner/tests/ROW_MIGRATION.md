# Row单逻辑轴数学核心迁移

> 目录已按机构整理；本文保留迁移时的范围和验证记录，当前接口、构建接入、调参和模式选择请看 [planner 使用指南](../README.md)。通用 ScrewPositionPlanner 现在与历史丝杆接口同在 slide/position 中。

只新增serve/arm_trajectory.hpp、serve/arm_trajectory.cpp、本说明及verify_row_reference.py；未修改既有实现、CMake、MCU或主控。当前不会自动编入已有库，也不发送总线命令。

## 来源

参考：`D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c`，WAY=0分支。

- 767–807：EvaluateQuinticTrajectory，归一化五次多项式，位置/速度/加速度表达式逐句保留float运算顺序。
- 812–892：PlanRowPositionTrajectory，方向、有限性、时间和提前量校验；两段时长均为`2*|位移|/|击球速度|`，**仅第一段101个采样点检查速度/加速度**，不增加第二段检查。
- 897–939：GetRowPositionReference，ERROR/WAIT/SWING/FOLLOW/HOLD阶段，错误时不改写输出，击球时刻开始FOLLOW，结束时刻开始HOLD。
- 944–1006：一般轨迹，目标68圈/秒、加速4圈、恒速1.5圈、每次0.005秒；保留先更新速度后积分，进入STOPPING的周期仍积分非零速度，下一周期置零并发出失能意图，不擅自钳制到5.5圈。
- 46–49：一般轨迹状态值0、1、2、3。
- 646–647：绝对逻辑圈映射，arm1=q，arm2=-q-0.009。
- 658–665：一般累计圈映射，arm1=-ARMSTART+positionSum，arm2=ARMSTART-positionSum-0.009，ARMSTART必填。
- 原头`User_Ctrl\Inc\user_task.h`162–187：FIVE阶段枚举及输入/输出字段。

## 用法与单位

GeneralRowPlanner::Step每次现算一份5ms参考，返回一个逻辑row累计圈位置、速度、左右独立增益和失能意图。Reset显式重置原静态状态，两个实例互不干扰。一般轨迹STOPPING的两个SetStateAndPosCtrl调用转为本次disable_requested=true；调用方必须负责实际执行失能。-1增益哨兵保留，代表原驱动“不发期望”。

FIVE先显式填入PosRowPlan_t五个输入，调用PlanRowPositionTrajectory，再按相同时间原点每5ms调用GetRowPositionReference。输入角度rad、速度rad/s、加速度rad/s²、时间秒；不给准备角/击球角/结束角/击球时刻/速度/运动限幅填默认硬件数。ARMSTART、FIVE目标和时钟均由调用者明确提供。

规划期101次求值只用于限制校验，不保存点列；周期阶段按当前时间直接求值，没有预上传点列。FIVE实际时间由调用方传入，数学核心不自行推进假定时钟。

MapLogicalRowTurns接收绝对逻辑圈位置；若来自FIVE的rad输出，调用方先显式除以同一twoPi再映射。MapGeneralRowTurns接收一般轨迹的累计圈及明确的ARMSTART圈值。两个映射都独立携带arm1、arm2的kp/kd，保留原表达式和-0.009圈偏置，不在核心换成编码计数、发送PD或合并增益。

## 验证证据

只读语句对照：FIVE三个函数体有效语句完全一致。一般轨迹在明确替换实例状态、左右增益聚合赋值、两次CAN失能为一个失能意图后，状态转移、积分、增益、常量和顺序全部相同。

ARM编译：`C:\Users\city\.eide\tools\gcc_arm\bin\arm-none-eabi-g++.exe`，`-std=c++17 -Wall -Wextra -Werror`，serve/arm_trajectory.cpp和serve/plate_distance.cpp分别成功编译为临时目录对象。row原函数oracle差分夹具亦通过同参数语法检查；交叉产物无法在Windows本机执行，未声称数值运行通过。

```text
python tests/verify_row_reference.py
python tests/verify_row_reference.py --compiler <本机g++或clang++>
python tests/verify_row_reference.py --compiler <ARM g++> --compile-only
```

可执行差分测试包含150帧一般轨迹、两方向各1401帧FIVE、左右增益、仅一次失能意图、Reset、非法计划/输出不改写、两个映射表达式。仅指定可在本机运行的编译器时才执行测试程序。

## 未迁移/未完成

- WAY1全部High/HighDown/117/1110/Low118A/118B/119轨迹变体。
- FIVE外层553–588的目标选择/计划调度；593–606的击球后恒速段时长与提前失能；624–643将核心FOLLOW改为恒速并选择驱动增益的外层策略。**当前FIVE核心FOLLOW仍为原897–939函数的第二段五次轨迹，不代表完整外层发球动作已迁完。**
- 行程/时间任务触发、5ms循环运行器、双臂CAN命令、4096计数换算、电机限位、回零/到位反馈及GUI/ROS/主控连接。

当前完成的是逐句可核对的数学核心和显式双臂映射；实际控制必须由后续获准的调用层消费当前帧并发送命令。
