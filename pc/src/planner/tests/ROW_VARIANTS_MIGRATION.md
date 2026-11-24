# WAY1 row 原实验轨迹迁移

> 目录已按机构整理；本文保留迁移时的范围和验证记录，当前接口、构建接入、调参和模式选择请看 [planner 使用指南](../README.md)。通用 ScrewPositionPlanner 现在与历史丝杆接口同在 slide/position 中。

> 本轮七模式参数已集中到同一表，公开兼容入口保持。新增可持续执行的 verify_row_variants_reference.py：核对七模式完整表与三个数学函数，构建原函数交错差分夹具。当前证据及未执行限制见 [精简审查记录](REFACTOR_REVIEW.md)，下文“没有新增验证脚本”仅为初次迁移历史。

## 范围与完整性结论

本次只新增 [serve/arm_legacy.hpp](../include/planner/serve/arm_legacy.hpp)、[serve/arm_legacy.cpp](../src/serve/arm_legacy.cpp) 和本文；未改现有文件或 CMake。命名空间 `planner::legacy_way1`，无 ROS、HAL、CAN 依赖。

**请求指定的 WAY1 一般 row 轨迹及 FIVE1 数学/执行路径均已迁移。** 不是整个 WAY1 固件迁移，也没有把历史实验参数解释成新上手/下手配置。

实际先 grep/read 的原参考：`D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c`，WAY1 从 1590 行开始；另已完整读取原 `User_Ctrl/Inc/user_task.h`、`User_Ctrl/Inc/user_control.h`。原头中 WAY=0，FIVE/FIVE1 注释关闭，因此这些是历史可选实验路径，不声称原默认固件当前正运行它们。

|原函数/区间（参考文件行号）|新接口/覆盖|
|---|---|
|GetRowPositionReference，2139–2168|同名成员，保留七模式分派及未知模式零位移、不发位置意图|
|GetRowPositionHighTrajectory，2173–2232|同名成员，50 圈/s，s1=4，s2=1.50 圈|
|GetRowPositionHighDownTrajectory，2237–2296|同名成员，57 圈/s，4.3/1.70 圈|
|GetRowPositionHighDown117Trajectory，2301–2360|同名成员，60 圈/s，4.3/1.80 圈|
|GetRowPositionHighDown1110Trajectory，2365–2421|同名成员，45 圈/s，4.3/1.80 圈|
|GetRowPositionLow118ATrajectory，2426–2484|同名成员，45 圈/s，4.3/1.80 圈|
|GetRowPositionLow118BTrajectory，2489–2548|同名成员，57 圈/s，4.3/1.80 圈|
|GetRowPositionLow119Trajectory，2553–2609|同名成员，64 圈/s，4.3/1.80 圈|
|EvaluateFive1QuinticState，2827–2867|同名数学函数，原多项式表达式|
|PlanFive1Trajectory，2872–2948|同名数学函数，原输入/方向/预留时间检查及起挥段 101 点检查|
|EvaluateFive1Trajectory，2950–2992|同名数学函数，两段五次和 WAIT/SWING/FOLLOW/HOLD/ERROR|
|ControlArmPosition2HighDown1110，2997–3099|同名成员，实例化所有 static，保留原控制器特有匀速 follow 与提前失能|
|ControlArmPosition2，3105–3159|同名成员，绝对位置、双电机方向、0.009 圈零偏与 FIVE1 分派|

原阶段常量 0/1/2/3 来自参考源 46–49；模式值、STARTHIGH=-1.75、STARTHIGHDOWN=-2.0、STARTLOW118A/B/119=-1.6 均来自 user_control.h 41–84。所有增益保留原乘法表达式，不自行解释成 SI 单位。

## 周期与状态

- 每次周期接口调用现算一次 5 ms，不提前生成数组；不要同周期先调 GetRowPositionReference 再调 ControlArmPosition2，否则推进两次。
- 一个 RowVariants 对象保存七个一般轨迹各自的 referencePosition/vel/stage，以及原共享 armTar 增益、位置和 FIVE1 loopCount/plan/planReady/planFailed/complete。无可变全局或可变 static；多实例互不干扰。
- `reset()` 重置整个实例，可用于重新执行。原程序没有统一重置入口；此接口不额外发送命令。
- 一般轨迹保留先 `vel += a1*dt`、再限幅、再 `referencePosition += vel*dt`。匀速阶段先用旧累计位置判断阈值，下一周期才停止；不改成解析位置或无超调停靠。
- 一般轨迹输出 stage 是本次执行后的 stage。进入 STOPPING 的当次仍积分，下一次 STOPPING 才发一次 disableIntent，随后 COMPLETE 继续保持位置意图。

## 输出语义和硬件边界

`referencePosition`：一般轨迹正累计位移（圈），FIVE1 为 arm1 绝对位置（圈）；`vel`：一般轨迹正位移速度/FIVE1 arm1 速度（圈/s）。arm1/arm2 各自输出绝对位置、带方向速度和原 pos_kp/pos_kd。速度为参考，不是反馈，不表示已经到位。

`positionIntent` 和 `disableIntent` 是原控制发送动作的意图，**这里不发送任何 CAN**。尤其：

1. 一般轨迹原停止分支先以位置 0 失能双电机，外层 ControlArmPosition2 仍继续 PosCtrl。因此该周期两个意图可能同时为 true；如调用者需要重放，必须保持“失能后位置”的顺序，而不能假设互斥。后续 COMPLETE 原外层仍调用 PosCtrl，增益 -1 表示不发该参数。
2. FIVE1 数学求值器的 FOLLOW 是第二段五次减速；但原 ControlArmPosition2HighDown1110 **覆盖它为 62.5 圈/s 匀速**，到 `hitTime+1.8/62.5` 失能，早于数学计划 endTime。本迁移保留这一差异，不偷换成平滑减速。
3. FIVE1 固定起点 -2 圈，击球增量 4.4 圈、后续 1.8 圈，最小预留 0.035 s，额外 0.005 s；速度 62.5 圈/s、最大加速度 9000 rad/s²；arm2=-arm1-0.009 圈。增益 wait/swing 与 follow 不同，均保留。
4. FIVE1 失能位置按原 `(int32_t)(圈*4096)` 截断后折回圈，输出在 disablePosition1/2；一般失能位置均为原值 0。正常 positionIntent 的位置仍是未量化浮点圈数。
5. FIVE1 完成当次只有失能意图，之后无命令；失败时 failed=true、stage=FIVE1_ERROR，保持原失败即返回行为，不擅自增加原来没有的失能。原蜂鸣器和 DEBUG 不在数学模块执行，调用者可按 failed 提示。

## 准备角控制（追加覆盖）

已进一步实际 read 源 2625–2800、3165–3314，新增同名 `ControlArmPosition1(mode, arm1MeasuredRevolutions)` 和七个 `UpdateRowPosition*`。输入 arm1 实测圈数，原脉冲到圈转换交给调用者。

- 每个 Update 函数独立保存从 0 圈起的参考位置；每次先按 0.05 圈递减，再执行原内层夹紧 if 和后续独立的第二个 if。双臂准备目标为 q、-q，不加发球执行阶段的 0.009 圈零偏。
- 控制函数依原顺序先更新准备角，再检查实测 arm1 与固定准备角的绝对误差 `<=0.02` 圈。每模式独立 zCount（包括 F 模式），累计到 200 次后置为 201 并将共享 positionOk 标记置真。**不是连续 200 次**：原代码误差超限没有 else，不清计数，也不清已经置位的 positionOk；原行为照留。未知模式仅清 positionOk、不发送位置、不清其它计数。
- FHIGHDOWN1110 和 HIGHDOWN1110 共享同一个 UpdateRowPositionHighDown1110 渐进位置，但到位计数各自独立，保持源 case 的 static 分布。
- ControlArmPosition1 输出两个位置控制意图，pos_kp=`1.2*1000*60`、pos_kd=-1；这些原来直接作为 PosCtrl 参数，不改写共享 armTar 增益，因此迁移仅覆盖本次输出增益。单独调用 Update 函数只更新目标，positionIntent=false。
- 新字段 positionOk 是原调用者共享标记的实例状态；readyCount 是当前模式累计次数。准备阶段 velocityRps/vel 是参考位置的 5 ms 差分辅助量，并非原 CAN 的速度命令。
- **实际函数没有使能/失能调用**，故准备控制 enableIntent/disableIntent 均为 false。源 UserTask 1958 行“row2 失能”注释和这些函数实际代码不符；迁移以读到的函数为准，不按注释编造 CAN 意图。外围复位/使能配置仍不属于本模块。蜂鸣器不执行。
- 同一周期不要先调用 Update 再调用 ControlArmPosition1，否则准备角递减两次。正常用法：逐周期 ControlArmPosition1，观察 positionOk 后由调用方转到 ControlArmPosition2；这里不增加启动或安全状态机。

## 验证与已知限制

已使用指定 ARM 编译器完成独立 C++11 编译：

```powershell
& 'C:\Users\city\.eide\tools\gcc_arm\bin\arm-none-eabi-g++.exe' -std=c++11 -Wall -Wextra -Werror -Ipc/src/planner/include -c pc/src/planner/src/serve/arm_legacy.cpp -o "$env:TEMP\row_variants.o"
```

结果：exit 0，无警告。另用临时 Python 检查：三个 FIVE1 数学函数的完整函数体去掉空白后与参考源逐字符一致；七个一般函数去掉行注释后提取的有效 targetVelocity/s1/s2 均与上述迁移参数一致。追加准备角接口后重新 ARM 编译仍 exit 0；临时 Python 进一步核对七个 Update 原函数各自初值 0、步长 0.05、三个 if 顺序/双臂镜像，以及 ControlArmPosition1 八个独立计数、0.02/200 门限、双臂常量增益和无使能/失能调用。没有新增验证脚本文件。尚未完成宿主可执行逐周期差分测试。仅编译不等于运行或上机验证；未验证真实电机跟踪、CAN 协议打包、限位或击球效果。数学规划依旧只检查起挥段 101 采样点，不保证采样间极值，也不新增 follow 的速度/加速度约束。EvaluateFive1QuinticState 保留原底层调用前提：有效正时长、有效输出指针；应经 PlanFive1Trajectory 后调用上层 EvaluateFive1Trajectory。

不在本次请求范围：传感器等待/启动状态机、PlateCtrl、GetPlateVelocity*、滑轨/阀门、复位/使能/速度限制 CAN 配置、WAY0/FIVE。一般轨迹的 gains 和位置命令迁移不包含原驱动底层对 -1 参数的实现。没有改 CMake，调用方需要另行把新增 cpp 加入自己的构建；本次不做集成或上机安全认证。
