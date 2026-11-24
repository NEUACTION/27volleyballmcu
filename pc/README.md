# New PC Control Workspace

This is a fresh ROS 2 workspace for the merged volleyball robot. It follows the reference package/message/node naming (`base_interfaces`, `mcu`, `mcuNode`, `/mcu/transmit`, `/mcu/receive`) without copying the reference robot's unrelated mechanisms.

规划接口、参数该改哪里、运行时如何切模式，以及按机构整理后的文件位置，见
[planner 接口与调参指南](src/planner/README.md)（仓库内路径 `pc/src/planner/README.md`）。
打不开是因为上面的链接指向仓库里的文件，而你的 Markdown 客户端没有把它解析成可点击链接。内容我直接放在下面，不需要点任何东西。

## 1. 文件位置（按机构整理后）

```text
pc/src/planner/
├── include/planner/
│   ├── catch/   timing.hpp  height_profile.hpp  angle_five2.hpp
│   ├── serve/   plate_time.hpp  plate_distance.hpp
│   │            arm_trajectory.hpp  arm_legacy.hpp  sequence.hpp
│   └── slide/   position.hpp
├── src/         与上面一一对应（8 个 .cpp）
└── tests/       原工程对照脚本
```

旧名对应新名：

| 旧文件 | 新文件 | 职责 |
| --- | --- | --- |
| `serve_plan` | `serve/plate_time` | 盘按时间变速 |
| `serve_distance_plan` | `serve/plate_distance` | 盘按路程变速 |
| `row_plan` | `serve/arm_trajectory` | 通用双臂轨迹 |
| `row_variants` | `serve/arm_legacy` | WAY1 历史双臂 |
| `serve_sequence` | `serve/sequence` | 准备→光电→盘→臂 顺序 |
| `catch_height` | `catch/height_profile` | 高度域接球 |
| `catch_five2` | `catch/angle_five2` | 角度域接球 |
| `slide_plan` | `slide/position` | 丝杆位置 |

## 2. 模式在哪切

模式在 `pc/src/action_planner/src/action_planner_node.cpp` 里切，**不在 planner 里**。三个 ROS 参数：

- `category`：大类
- `submode`：子模式
- `run_successed`：运行门控（默认 false）

| category | 可用 submode | 动的机构 |
| --- | --- | --- |
| `idle` | 空 | 全禁用 |
| `catch` | `catch_t`、`catch_five1`、`catch_five2` | 四腿 |
| `serve_way1` | `legacy_way1_high` 等 8 种 | 双臂+盘 |
| `test_way0` | `plate_time`、`plate_distance_*` | 单独盘 |
| `test_way0` | `row_general`、`row_legacy_*` | 单独双臂 |
| `test_way0` | `slide_legacy_*` | 单独丝杆 |
| `test_way0` | **`wheel_virtual`**（新增） | 四舵轮 |

**运行中切换命令：**

```sh
ros2 param set /actionPlannerNode run_successed false
ros2 param set /actionPlannerNode category catch
ros2 param set /actionPlannerNode submode catch_five2
ros2 param set /actionPlannerNode run_successed true
```

同一个 submode 重复设置不会重开；切模式或 run 从 false 变 true 才会重置。

## 3. 参数该改哪里

**ROS 参数**（不用重编译）——在 `action_planner_node.cpp` 的构造函数里声明：

| 参数 | 用途 |
| --- | --- |
| `plate_previous_kp` / `_ki` | 盘速度环，填驱动内部乘 1000 **之前**的原值（1.8/0.4） |
| `slide_pos_kp` / `_kd` | 丝杆位置环，填驱动乘 **之前**的原值 |
| `row_five_*`（7 个） | WAY0 FIVE 的角度、速度、限幅 |
| `wheel_virtual_turns` | **虚拟主轮位置（圈）** |
| `wheel1..4_pos_kp` / `_pos_kd` | 每轮位置环（4×2） |
| `wheel1..4_vel_kp` / `_vel_ki` | 每轮速度环（4×2） |
| `wheel1..4_ff_current` | 每轮前馈电流（4 个，A） |

**源码常量**（要重编译）：

| 改什么 | 改哪个文件 |
| --- | --- |
| 盘时间轨迹速度/行程 | `src/serve/plate_time.cpp` 的 `kTargetSpeed`、`kS1~kS4` |
| 盘路程各模式速度 | `src/serve/plate_distance.cpp` 的 `kDistancePresets` |
| 双臂速度/行程/增益 | `src/serve/arm_trajectory.cpp` |
| WAY1 历史模式和准备角 | `src/serve/arm_legacy.cpp` / `.hpp` 的 `STARTHIGH` 等 |
| 发球触发等待次数 | `src/serve/sequence.hpp` 的 `SEQUENCE_START_COUNT` 等 |
| 丝杆目标位置 | `src/slide/position.cpp` 的 `kSlidePresets` |
| 接球几何 | `src/catch/*.cpp` 的几何常量 |
| 腿 PVT 速度上限 | `merged_control.c` 的 `MCU_LEG_VELOCITY_LIMIT`（现 1.05） |

## 4. 这次舵轮改动后的新参数

**一个虚拟主轮 + 每轮独立**：

```sh
# 四轮位置目标（圈），四个轮子当前同步跟随
ros2 param set /actionPlannerNode wheel_virtual_turns 0.5

# 每轮独立增益（12 个参数，示例填 1~4 号轮）
ros2 param set /actionPlannerNode wheel1_pos_kp 1.0
ros2 param set /actionPlannerNode wheel1_pos_kd 0.0
ros2 param set /actionPlannerNode wheel1_vel_kp 1.0
ros2 param set /actionPlannerNode wheel1_vel_ki 0.0
ros2 param set /actionPlannerNode wheel1_ff_current 0.0
# wheel2/3/4 同理

ros2 param set /actionPlannerNode category test_way0
ros2 param set /actionPlannerNode submode wheel_virtual
ros2 param set /actionPlannerNode run_successed true
```

16 个增益和 4 个电流**缺任何一个有限值就整体拒绝**，不会让部分轮子执行。

## 5. 总结

- **切换模式**：修改 `category` 和 `submode`，然后重新开启 `run_successed`
- **调整运行参数**：修改 ROS 参数，不用重编译
- **调整算法预设**：修改 `planner` 源码，需要重编译
- **舵轮**：使用 `wheel_virtual` 模式 + 17 个参数
- **腿 PVT**：已打开，速度上限仍为 ±1.05 圈/秒



## Current packages

- `src/base_interfaces`: reference-style command and feedback message types adapted to arm, plate, slide, catch legs, and reserved steering wheels.
- `src/mcu`: 5 ms POSIX串口节点，SP/QR 261/133字节编解码、分包接收与短写续发。
- `src/planner`: 接球T/FIVE1/FIVE2、盘时间/11路程模式、一般/五次/历史变体双臂、5种丝杆和8种历史发球组合时序；已做参数表精简，所有规划只求当前帧。
- `src/action_planner`: 参考命名的 `actionPlannerNode`，每5ms现算现发，默认idle/禁用，不缓存整段轨迹。

The `/dev/mcu` device and 921600 baud are ROS parameters. The serial implementation uses Linux POSIX APIs, like the reference deployment.

## Build (on the target ROS 2 Linux system)

```sh
cd pc
colcon build --symlink-install
source install/setup.bash
```

当前主机未发现 `colcon`/`ros2`，因此尚未执行ROS构建。已找到ARM GCC并用 `-std=c++17 -Wall -Wextra -Werror -c` 检查独立规划库；该交叉编译不等于在小电脑Linux上完成ROS构建或运行测试。

## Hardware warning

主控已接入261/133字节帧及八轴执行适配，完成真实ARM逐文件编译与FLASH布局链接检查（未刷机）；这些不是ROS构建或实机动作验证。规划库8个实现严格ARM编译通过，5份参考检查为源码/参数表核验＋oracle夹具编译，未执行数值差分。PC已处理反馈字段、跨read累计及短写续发，但尚无规划进程/链路失联停车、反馈时效检测或串口重连；不要把当前版本当成已验证安全的可直接运行固件。

执行入口统一为互斥`category`＋`submode`：`catch`只四腿，`serve_way1`只双臂＋盘，`test_way0`只选丝杆/盘/双臂之一。run上升沿向主控发当前模式对应复位事务；双臂ARM1实际复位后ARM2置零，不额外检查ARM2复位回报（用户确认的原流程）。当前模式复位准备和真实配置/使能反馈齐后才逐5ms推进。

帧及特殊ARM2状态语义见 `PROTOCOL.md`；各模式和参数示例见 `src/action_planner/README.md`。发球唯一光电触发`ps_num[3]`保留，小电脑接收原值。合并硬件已确定改到UART7 PE8 TX / PE7 RX，不再沿用原发球USART3 PD8/PD9；接线与9字节协议见主控项目 `PS3_UART7_README.md`。
