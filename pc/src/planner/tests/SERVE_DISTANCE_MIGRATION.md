# 发球路程与配置位置接口迁移记录

> 目录已按机构整理；本文保留迁移时的范围和验证记录，当前接口、构建接入、调参和模式选择请看 [planner 使用指南](../README.md)。通用 ScrewPositionPlanner 现在与历史丝杆接口同在 slide/position 中。

> 以下为初次迁移历史记录。本轮已将11份内部函数收敛为参数表和共用求值；重复赋值不再保留。当前结构、验证方式和限制以 [精简审查记录](REFACTOR_REVIEW.md) 为准，旧“逐句对照”描述不代表重构后的验证结果。

本数学核心任务只维护独立头文件、实现、本说明与只读差分测试，不修改CMake、ROS节点、串口或MCU。调用层及构建接入由集成任务另行处理，本文只记录数学核心本身的行为与验证。

## 实际来源与分支

参考文件为 `D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c`。
`#if WAY == 0` 位于 51 行，`#elif WAY == 1` 位于 1590 行，结束于 4053 行。

| 显式模式 | 源函数和行号 | 实际参数与分支 |
| --- | --- | --- |
| Way0High | GetPlateVelocityHigh 1186–1285 | 目标 10.0，a/d=900；非正路程归零；≤1.60 加速限幅；[1.2,1.4)、[1.4,1.6] PID；(1.60,1.70] 恒速；(1.70,2.95] 减速及负被开方量/近零检查；>2.95 停止 |
| Way0Try | GetPlateVelocityTry 1290–1397 | 目标 30.0，a/d=1500；非正归零；≤1.0 首段，达到目标时返回 4/3×目标即40；(1.0,1.50] 返回目标30；(1.5,2.95] 减速仍减去1.70；>2.95停止 |
| Way0Low | GetPlateVelocityLow 1402–1464 | 目标11.2，a/d=200；非正归零；≤1.60 加速（≥0.6改变ki）；(1.60,1.70] 仅写kp；(1.70,2.95] 减速；>2.95仅写ki |
| Way1High | GetPlateVelocityHigh 3367–3438 | 目标32.5，a/d=1500；相同路程段，1.2/1.4的PID与WAY0不同，减速段ki=200；保留函数末尾返回0 |
| Way1HighDown | GetPlateVelocityHighDown 3443–3513 | 目标35.80，a/d=1500；路程及PID分段保持源式，与Way1High同一段边界，减速段ki=200 |
| Way1HighDown117 | GetPlateVelocityHighDown117 3518–3591 | 目标33.5，a/d=1500；不用注释掉的35.4/50.0；完整保留各段PID及减速检查 |
| Way1HighDown1110 | GetPlateVelocityHighDown1110 3596–3663 | 目标58.0，a/d=1500；保留≤1.60、(1.60,1.70]、(1.70,2.95]、>2.95及内部PID边界 |
| Way1Low118A | GetPlateVelocityLow118A 3668–3729 | 目标11.2，a/d=200；≤1.60加速且≥0.6改ki；(1.60,1.70]仅写kp；减速写kp/ki；>2.95仅写ki |
| Way1Low118B | GetPlateVelocityLow118B 3734–3797 | 目标7.6，a/d=200；不用注释掉的9.2/10.5；分段与写入增益字段保留源式 |
| Way1Low119 | GetPlateVelocityLow119 3802–3862 | 目标9.0，a/d=200；不用注释掉的10.0；恒速段ki、停止段kp继续继承前次值 |
| Way1Low | GetPlateVelocityLow 3867–3926 | 目标20.0，a/d=400；≤1.60仅在≥1.2时写kp，不写ki；(1.60,1.70]仅写kp；减速写kp、ki；>2.95仅写ki；保留末尾返回0 |

源代码有效语句逐句迁移，未采用已注释的候选值。重复的PID赋值也保留。Try首段内1.2/1.4条件在≤1.0外层下不可达，仍照抄，未“修复”。使用float保存源中参数、计算表达式和返回值，平方根显式按C源sqrt的double调用语义处理。

## 接口及状态语义

EvaluateServeDistance 的模式、盘路程（转）与 previous_gains 均由调用方显式提供，返回盘速（转/秒）与本次完整增益。调用方若要复现历史调用序列，应将上次结果的 gains 传入下次调用；切换模式也沿用同一份增益。初始增益不从其它初始化代码猜测。返回 nullopt 表示非法枚举或非有限路程/增益，这是新接口的输入约定；源WAY0及WAY1 Low118A/Low118B/Low119的NaN路径没有返回值，不声称迁移了该未定义行为。这六个函数补充的最终return对有限输入不可达。新增六个枚举在既有Way1Low之后追加，不改变既有五个枚举值。

指定源中没有名为GetPlatePosition的函数。初版因此只提供无默认目标的ScrewPositionPlanner；后续已明确真实名称为GetSlidePosition，并在独立slide_plan模块迁移其全部历史实验预设。本模块不把这些预设混入通用配置接口。

ScrewPositionPlanner 仅提供显式配置算法：Configure 必须给出目标位置、每次正向步长及初始位置；Step每次增加一个配置步长并在目标处保持。单位由调用方统一，周期不隐含为5ms。未配置或配置失败时Step返回nullopt；反向目标、非正步长和非有限配置拒绝。该算法用于可审查的独立接口，**不是已确认的历史GetPlatePosition迁移**。反向运动、丝杆行程限位和回零状态需另行明确，当前未实现。

示例（数字仅为调用者提供的测试配置，不是硬件目标）：

```cpp
#include "planner/serve/plate_distance.hpp"
// 初始PID必须由调用方从实际驱动配置取得。
auto sample = planner::EvaluateServeDistance(
  planner::ServeDistanceProfile::Way0Low, actual_plate_turns, actual_previous_gains);
```

## 验证

使用Python运行 `tests/verify_distance_reference.py`：默认只读上述参考源，核对11个函数全部有效语句、常量和顺序；不改写生成实现。

提供 `--compiler <g++或clang++路径>` 可编译独立临时测试，原始源函数作为oracle。测试覆盖各段边界及相邻float、速度下限/饱和/减速停止阈值、从4转到负路程的密集逆序扫描、增益历史保持，以及非法输入与配置位置的推进/保持/重配置。每个模式约4.1万点，11个模式约45.2万次速度与PID对照；新增六模式各自饱和/停止阈值及相邻float也覆盖。Low118A/Low118B/Low119另有恒速段继承ki、停止段继承kp的断言。临时测试不依赖ROS或原工程CMake。

实际验证：11个函数体逐项源码对照PASS。使用`C:\Users\city\.eide\tools\gcc_arm\bin\arm-none-eabi-g++.exe`，`-std=c++17 -Wall -Wextra -Werror`成功编译扩展后的实现为ARM对象，临时原函数oracle差分夹具也通过语法检查。交叉产物未在Windows执行，因此未声称运行时数值差分通过。交叉检查用`--compiler <ARM g++> --compile-only`。

```text
python tests/verify_distance_reference.py
python tests/verify_distance_reference.py --compiler /path/to/g++
```

## 未覆盖

本源文件GetPlateVelocity全部11个实际函数体已覆盖。本模块不迁移时间盘速GetPlateVel/GetPlateVelNew、滑轨/双臂轨迹、计数到转数换算或驱动PID应用逻辑。后续澄清真实丝杆函数名为GetSlidePosition，其历史实验预设迁移位于独立slide_plan模块；本模块ScrewPositionPlanner仍仅为无预设的配置算法。本次数学核心扩展未修改现有serve/plate_time.cpp、构建配置、MCU或主控。
