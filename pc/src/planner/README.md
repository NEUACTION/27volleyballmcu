# planner：接口、调参位置与模式选择

这个包只负责**计算当前周期的目标**。它不创建 ROS 节点、不收串口、不发 CAN，也不保存整段轨迹点列。

实际运行链路是：`actionPlannerNode` 每 5ms 选择并调用本包接口 → 发布 `/mcu/transmit` → `mcuNode` 编码发送 → 主控执行。**切模式在动作节点中完成，不在本包的头文件里改宏。**

## 1. 先按机构找文件

头文件与实现使用相同目录和名称；`include/planner/` 放接口，`src/` 放实现。

```text
planner/
├── README.md
├── include/planner/
│   ├── catch/                 接球四腿
│   │   ├── timing.hpp         三种接球算法共用的时间结果
│   │   ├── height_profile.hpp 高度域 T / FIVE1
│   │   └── angle_five2.hpp    角度域 FIVE2
│   ├── serve/                 发球盘、双臂及组合顺序
│   │   ├── plate_time.hpp     盘速度随时间变化
│   │   ├── plate_distance.hpp 盘速度随实测路程变化
│   │   ├── arm_trajectory.hpp 通用双臂轨迹、WAY0 FIVE
│   │   ├── arm_legacy.hpp     WAY1 历史双臂模式与准备角
│   │   └── sequence.hpp       准备 → 光电等待 → 盘 → 双臂的调用顺序
│   └── slide/                 丝杆
│       └── position.hpp       历史位置预设与显式配置的定步长位置接口
├── src/                       与上述目录对应的八个 .cpp
└── tests/                     原工程对照脚本与历史迁移记录
```

`serve` 表示机构归属，不代表它下面所有文件都同时执行。盘、双臂可以在 `test_way0` 下单独测试；组合发球才由 `serve_way1` 调用它们。

原文件与新位置的对应关系：

| 原名称 | 现在的接口 / 实现 | 职责 |
| --- | --- | --- |
| `catch_height` | [catch/height_profile.hpp](include/planner/catch/height_profile.hpp) / [实现](src/catch/height_profile.cpp) | 高度域接球 |
| `catch_five2` | [catch/angle_five2.hpp](include/planner/catch/angle_five2.hpp) / [实现](src/catch/angle_five2.cpp) | 角度域 FIVE2 接球 |
| `catch_timing` | [catch/timing.hpp](include/planner/catch/timing.hpp) | 公共时间结果，无单独实现 |
| `serve_plan` | [serve/plate_time.hpp](include/planner/serve/plate_time.hpp) / [实现](src/serve/plate_time.cpp) | 只负责盘的时间轨迹 |
| `serve_distance_plan` | [serve/plate_distance.hpp](include/planner/serve/plate_distance.hpp) / [实现](src/serve/plate_distance.cpp) | 只负责盘的路程轨迹 |
| `row_plan` | [serve/arm_trajectory.hpp](include/planner/serve/arm_trajectory.hpp) / [实现](src/serve/arm_trajectory.cpp) | 通用双臂轨迹 |
| `row_variants` | [serve/arm_legacy.hpp](include/planner/serve/arm_legacy.hpp) / [实现](src/serve/arm_legacy.cpp) | WAY1 历史双臂实验 |
| `serve_sequence` | [serve/sequence.hpp](include/planner/serve/sequence.hpp) / [实现](src/serve/sequence.cpp) | 组合发球调度 |
| `slide_plan` | [slide/position.hpp](include/planner/slide/position.hpp) / [实现](src/slide/position.cpp) | 丝杆位置 |

只改了文件组织和 include 路径，C++ 类名、命名空间、ROS 包名、库名以及已有模式字符串继续沿用，避免把目录整理变成算法重写。比如原 `RowVariants` 名称仍保留，但文件已明确叫 `arm_legacy`。

## 2. 模式到底在哪里切

**ROS 参数入口**在 [动作节点实现](../action_planner/src/action_planner_node.cpp)。

- `category`：唯一大类。
- `submode`：该大类中的算法。
- `run_successed`：运行门控；默认 `false`。
- 源码里的 `kModes` 表：限定哪些大类/子模式组合合法，以及参与哪些电机。
- `Tick()`：读取这三个参数，并统一选择规划算法、处理复位/ready握手、发布一帧目标。

| category | submode 示例 | 参与机构 |
| --- | --- | --- |
| `idle` | 空字符串 | 全轴禁用 |
| `catch` | `catch_t`、`catch_five1`、`catch_five2` | 四条接球腿 |
| `serve_way1` | `legacy_way1_high`、`legacy_way1_low118a` 等八种组合 | 双臂＋盘 |
| `test_way0` | `plate_time`、`plate_distance_way0_high` | 单独盘 |
| `test_way0` | `row_general`、`row_legacy_way0_five`、`row_legacy_way1_high` | 单独双臂机构 |
| `test_way0` | `row_prepare_legacy_way1_high` | 单独双臂准备位置 |
| `test_way0` | `slide_legacy_way0_high`、`slide_legacy_way0_low` | 单独丝杆 |

`test_way0` 是当前测试类别名称，类别内也有用于隔离重放的 WAY1 历史算法，不能把它理解成所有算法都来自原 `WAY=0`。

完整子模式清单及模式专用约束见 [动作节点使用说明](../action_planner/README.md)。`catch_five2_pvt_experimental` 是额外许可控制的实验入口，普通接球无需选择它。

### 启动时选择

在配置好 ROS2 的 Linux 环境，从 `pc` 工作区执行：

```sh
colcon build --packages-up-to action_planner mcu
. install/setup.sh

# 启动通信与高度梯形接球模式；运行门控保持关闭。
ros2 launch action_planner action_planner.launch.py category:=catch submode:=catch_t run_successed:=false
```

### 已运行时切换

```sh
# 先关闭运行门控，再修改类别和子模式，避免分步修改时使用旧组合。
ros2 param set /actionPlannerNode run_successed false
ros2 param set /actionPlannerNode category catch
ros2 param set /actionPlannerNode submode catch_five2

# 查看当前选择和运行门控。
ros2 param get /actionPlannerNode category
ros2 param get /actionPlannerNode submode
ros2 param get /actionPlannerNode run_successed

# 配置好所选模式的必填参数、确认参与机构后，开启运行。
ros2 param set /actionPlannerNode run_successed true

# 停止时明确关闭运行门控。
ros2 param set /actionPlannerNode run_successed false
```

同一个 `submode` 重复赋值不代表重新开始。切类别/子模式，或者让 `run_successed` 从 `false` 变为 `true`，会重置规划状态。开启后还要等待当前机构的复位事务和实际配置/使能反馈，不要求无关电机在线。

不要在这里把“软件 `Reset()`”理解成“电机复位”。电机复位由主控合并模块执行；ARM2沿用 ARM1 完成后发送置零、无需检查 ARM2 复位回报的规则。PC退出或断线也不等于已发送停止命令。

## 3. 调参数：先判断是运行参数还是源码预设

**ROS 参数改完不需要重编译。源码中的预设、公式常量改完需要重新构建并重启节点。** 目前还没有把所有历史预设都做成 YAML 配置，不能只改一个 YAML 就改变所有轨迹。

### 可以在 ROS 中设置的参数

这些参数声明和读取位置都在 [动作节点实现](../action_planner/src/action_planner_node.cpp)。

| 参数 | 含义 / 单位 | 生效方式 |
| --- | --- | --- |
| `category`、`submode`、`run_successed` | 模式和运行门控 | `Tick()` 每周期读取 |
| `allow_unvalidated_pvt` | FIVE2 PVT实验许可，默认关闭 | 每周期读取；主控也有实验门控 |
| `plate_previous_kp`、`plate_previous_ki` | 盘速度环增益，填写驱动 SetVelKpKi 内部乘1000之前的原值（原程序1.8/0.4即填1.8/0.4） | 切模式或重新开启运行时读取；默认NaN，必须明确填写有限值 |
| `slide_pos_kp`、`slide_pos_kd` | 丝杆位置环增益，填写驱动 PosCtrl 内部乘 ×60000/×1000 之前的原值 | 切模式或重新开启运行时读取；默认NaN，必须明确填写有限值 |
| `row_five_home_rad` | WAY0 FIVE准备角，rad | 切模式或重新开启运行时读取 |
| `row_five_hit_rad` | WAY0 FIVE击球角，rad | 同上 |
| `row_five_end_rad` | WAY0 FIVE收杆角，rad | 同上 |
| `row_five_hit_velocity_rad_s` | WAY0 FIVE击球角速度，rad/s | 同上 |
| `row_five_max_speed_rad_s` | WAY0 FIVE速度上限，rad/s | 同上 |
| `row_five_max_acceleration_rad_s2` | WAY0 FIVE加速度上限，rad/s² | 同上 |
| `row_five_minimum_lead_s` | WAY0 FIVE起挥最少预留时间，s | 同上 |

七项 `row_five_*` 默认也为NaN，选 `row_legacy_way0_five` 时需要全部明确配置。它们不控制 WAY1 历史 FIVE1 的硬编码预设。组合发球 `serve_way1` 使用盘路程算法，同样需要明确设置 `plate_previous_kp`、`plate_previous_ki`，并不是只切子模式就能跳过增益配置。

当前 launch 只转发 `category`、`submode`、`run_successed`、`allow_unvalidated_pvt`，以及给通信节点的串口参数。**不能在 launch 命令末尾追加 `plate_previous_kp:=...` 就认为已经传进节点。** 其余参数可在启动后通过 `ros2 param set` 设置，或者使用 `ros2 run action_planner action_planner_node --ros-args -p 参数名:=数值` 直接启动规划节点；直接启动时通信节点需另启。

配置时先设 `run_successed=false`，设置该模式需要的参数，再重新开启。可用 `ros2 param list /actionPlannerNode` 查看参数，使用 `ros2 param set /actionPlannerNode 参数名 数值` 修改；这里不提供未经机构确认的通用增益值。

### 必须改源码的位置

| 想改什么 | 改哪里 | 找什么 |
| --- | --- | --- |
| 盘时间轨迹的目标速度、行程分界 | [plate_time.cpp](src/serve/plate_time.cpp) | `kTargetSpeed`、`kS1`～`kS4`；分段增益在 `PlateTimePlanner::Step()` |
| 盘路程各模式的速度、加减速度 | [plate_distance.cpp](src/serve/plate_distance.cpp) | `kDistancePresets`，顺序对应 `ServeDistanceProfile` |
| 盘路程的路程分界、分段增益规则 | [plate_distance.cpp](src/serve/plate_distance.cpp) | `EvaluateServeDistance()` 内各分支；只改速度表不会自动改变所有路程分界 |
| 通用双臂的速度、行程、增益 | [arm_trajectory.cpp](src/serve/arm_trajectory.cpp) | `GeneralRowPlanner::Step()`；初始增益还要同步看 `Reset()` |
| WAY1 各历史双臂模式的速度、行程、增益 | [arm_legacy.cpp](src/serve/arm_legacy.cpp) | 历史预设表及 `stepGeneral()` / FIVE1调用分支 |
| WAY1 各模式准备角 | [arm_legacy.hpp](include/planner/serve/arm_legacy.hpp) | `STARTHIGH`、`STARTHIGHDOWN`、`STARTLOW118A/B/119` |
| 发球触发等待、盘臂间隔 | [sequence.hpp](include/planner/serve/sequence.hpp) | `SEQUENCE_START_COUNT`、`SEQUENCE_WAITING_COUNT2`、`GAP_TIME118A` |
| 历史丝杆目标位置和每周期步长 | [position.cpp](src/slide/position.cpp) | `kSlidePresets`；枚举顺序见 `HistoricalSlideProfile` |
| 接球高度、平台速度、连杆几何 | [height_profile.cpp](src/catch/height_profile.cpp) / [angle_five2.cpp](src/catch/angle_five2.cpp) | 相关几何常量和轨迹输入；共同机构常量需核对两份实现，不能只改其中一份 |
| 当前发送的接球位置环增益、WAY0准备位置等调用层参数 | [动作节点实现](../action_planner/src/action_planner_node.cpp) | `CatchStep()`、`RowFiveStep()` 等具体调用；不在 `timing.hpp` 里改 |
| 新增一个可选子模式 | [动作节点实现](../action_planner/src/action_planner_node.cpp) | `kModes`、对应模式映射及 `Tick()` 分派；只有新增枚举还不能从ROS选中 |

接球几何尤其要注意：两份实现不仅各有 `LINK_LENGTH`、`OFFSET_RADIUS` 等常量，正逆解中还保留了 `210`、`20`、`44100` 等原式。换机构尺寸时必须一起核对这些表达式；只改一个同名常量不会自动更新全部几何。`TOTAL_H` 为目标高度、`V_CONST` 为平台线速度；`timing` 和多项式系数是它们推导的结果。

三个顺序计数单位是“符合条件时的调用次数”。`1000×5ms`、`1400×5ms`、`22×5ms` 可用于理解量级，但不能直接当作从启动到动作的墙钟延时：到位、光电、分支条件以及真实调度都会影响推进。

历史丝杆当前五组目标依次为 `241.16 / 11.42 / 241.16 / 711.42 / 241.17` 圈；第二组 `11.42` 是当前用户调整值。步长是“圈/次调用”，不是速度字段。比如改成 `0.1f` 表示每个有效5ms逻辑周期增加0.1圈。

## 4. 接口怎么调用

除 `planner::legacy_way1` 下的历史双臂与顺序接口外，其余类型和函数都在 `planner` 命名空间。

| 接口 | 输入 | 输出和调用顺序 |
| --- | --- | --- |
| `CatchHeightPlanner(mode)` | `Trapezoidal` 或 `Quintic` | 构造即初始化几何、时间和系数；重新计算用 `InitThetaProfile()` |
| `GetMotorRevsThetaPlan(t)` / `GetMotorVelAtTime(t)` | 从轨迹起点开始的秒数 `t` | 圈、圈/秒；查询不自己增加时钟，调用者推进 `t` |
| `CatchFive2Planner` / `InitThetaProfile()` | 使用实现中的几何与目标常量 | `GetMotorRevsThetaPlan(t)` / `GetMotorVelAtTime(t)` 输出位置和速度；同样由调用者维护时间 |
| `CatchTiming timing` | 初始化函数写入 | 共用的分段时间结果，单位秒；它不是独立调参配置 |
| `PlateTimePlanner::Reset()` / `Step()` | 无反馈输入 | `Step()` 返回当前盘速及 `kp/ki`，之后内部时间增加5ms；重新开始调用 `Reset()` |
| `EvaluateServeDistance(profile, distance, previous_gains)` | 模式、盘实测位置（圈）、前次速度环增益 | `optional<ServeDistanceTarget>`：速度（圈/秒）及本次增益；将返回增益保存为下次输入；非法输入为空 |
| `GeneralRowPlanner::Reset()` / `Step()` | 无反馈输入 | 当前逻辑圈位置、速度、双臂增益、阶段和失能意图；每次 `Step()` 推进一次5ms |
| `PlanRowPositionTrajectory(...)` / `GetRowPositionReference(...)` | 调用者填写的 `PosRowPlan_t`、时间和限幅 | 先规划，再按时刻求角度/角速度；位置为rad，速度为rad/s；不是直接把rad当圈发送 |
| `MapLogicalRowTurns(...)` / `MapGeneralRowTurns(...)` | 逻辑位置、双臂增益，后者另有准备零点 | 返回双臂圈目标，应用各自方向和历史偏置；不推进轨迹 |
| `RowVariants::ControlArmPosition1(mode, measured)` | 历史模式、ARM1实测圈位置 | 准备目标、累计到位状态和控制意图 |
| `RowVariants::ControlArmPosition2(mode)` | 历史模式 | 普通或FIVE1双臂轨迹；每次调用推进原逻辑周期，返回位置/失能等意图 |
| `RowVariants::reset()` | 无 | 清理各模式的独立累计状态；不操作电驱 |
| `ServeSequence::step(mode, before, after, ps_num3)` | 模式、准备控制前/后的到位标记、光电原始值 | 只返回准备/盘/双臂/跳过后续步骤的意图；调用者再调用对应规划器；`reset()`重启顺序 |
| `HistoricalSlidePlanner(profile)` / `Step()` | 一个明确的历史丝杆模式 | 首次即增加一步，返回圈位置；超目标后输出目标，内部仍累计；`Reset()`清软件位置为0 |
| `GetHistoricalSlidePreset(profile)` | 历史模式 | 目标圈数与每周期增量；非法枚举返回空 |
| `ScrewPositionPlanner::Configure(config, initial)` / `Step()` | 显式目标、正向步长和初始位置，单位由调用者统一 | 配置成功后每次推进一步；未配置返回空，内部位置在目标处夹紧；当前ROS子模式没有自动接入此接口 |

盘路程接口传入的是主控反馈中盘的原始有符号圈位置。当前调用层**不减进入模式时的位置，不取绝对值，也不再除4096**。其坐标起点来自实际复位流程。

WAY1准备接口的“到位”沿用历史累计规则，不应改读为本周期全部电机实时到位。`ServeSequence` 的输出也是“本周期应该调用什么”的意图，不等于机械动作完成。完整的当前拼接逻辑在 [动作节点](../action_planner/src/action_planner_node.cpp) 的 `CombinedServeStep()`。

### 几个容易误读的旧接口名字

- `row` 在这里是双臂挥杆，不是矩阵的一行；文件名已经改为 `arm_*`，旧类名保留以维持调用兼容。
- `CatchHeightPlanner::GetPlateVelocity()` 的 `Plate` 指接球平台，输出 mm/s；发球盘接口输出圈/秒，两者不能混用。
- 接球的 `FIVE1` 是高度五次轨迹；WAY1双臂的 `FIVE1` 是另一套起挥/收杆实验，不是同一个模式。
- `ControlArmPosition1` 表示准备控制，`ControlArmPosition2` 表示执行控制；末尾数字不是电机编号。
- `WAY0/WAY1`、`117/1110/118A` 等后缀用于追溯历史预设，不自动代表新的上手/下手分类。

两套五次轨迹的规划检查沿用原实现，只对起挥段101个采样点检查限速/限加速度；不要将返回成功解读为全轨迹连续极值保证。

## 5. 哪些不应该当作普通调参项

- `timing`、五次多项式系数、阶段计数等计算结果：先改输入，再让初始化/规划函数重算，不直接修改中间结果。
- `0.005` 周期：与动作节点定时器、积分、原历史计数及主控执行节奏有关，不能只改某个规划器中的一处就认为系统频率变了。
- 圈、rad、4096计数的换算和协议字段：它们属于接口单位/驱动适配，不是用来改变击球动作的倍率。
- 通信序号、复位事务号、轴掩码、ready位：它们控制实际执行握手，不是轨迹参数。

## 6. 修改后怎样确认

1. 改ROS参数：先关闭运行，修改参数，再重新开启；进入模式时读取的参数要触发一次重新进入或运行上升沿。
2. 改源码：在 `pc` 中重新 `colcon build --packages-up-to action_planner mcu`，重新加载环境并重启对应节点。
3. 整理了 include 后，外部调用要使用新路径，例如 `#include "planner/serve/plate_time.hpp"`；库名仍是 `planner`。
4. [tests](tests/) 保存原工程对照脚本。它们用于检查来源和差异；`--compile-only` 只编译夹具，不代表实际执行过数值对照或已经上机。
5. 本包的5ms是调用约定。ROS墙钟定时器不是硬实时保证；每次有效回调只推进一步，不补算丢失周期。

历史迁移文档记录当时版本，当前目录与调参入口以本README、现有接口和动作节点为准。
