# 舵轮接入与腿 PVT：改造方案（先规划，后实施）

本文只描述要改什么、怎么改，不包含代码改动。下一轮照此执行。

**用户已确认的决策（不要再问）：**

1. 舵轮是挂在 **CAN1** 上的**同款普通驱动器**，驱动号 **1..4**。
2. 舵轮**运行动作同步**：四个轮子的位置目标相同，来自一个虚拟主轮。但**复位不同步**，四轮各自独立。
3. 舵轮**进复位事务**，用同一套事务、同一个事务号，只是多四个复位位。（我先前说"风险高"是错的，这套代码本来就兼容。）
4. 每个舵轮的增益**独立**：4×位置 kp、4×位置 kd、4×速度 kp、4×速度 ki。前馈电流也是每轮独立 4 个。
5. 腿 PVT **不新设上限**，保持现有 ±1.05 圈/秒临时值。

---

## 一、两类电机的控制路径不同（这是最容易搞混的地方）

### 腿：PVT 混合控制，一条命令发完

`PVTCtrl` 一帧携带五个量：

| 量 | 含义 | 协议槽 |
| --- | --- | --- |
| p | 期望位置 | `motor_pos` |
| v | 期望速度 | `motor_vel` |
| kp | 位置刚度 | `motor_pos_kp` |
| kd | 速度阻尼 | `motor_pos_kd` |
| t | 前馈力矩 | `motor_cur` |

**PVT 是位置-速度-力矩的混合律，驱动器内部没有独立的"速度环 kp/ki"可调。**
`SetVelKpKi` 是速度模式（速度环）用的，**与 PVT 无关，腿的 PVT 路径不调用它。**

`motor_vel_kp` / `motor_vel_ki` 两槽在腿 PVT 下**不使用**。

### 舵轮：位置环，两条命令

| 命令 | 发什么 |
| --- | --- |
| `PosCtrl` | 位置 + 位置 kp + 位置 kd |
| `SetVelKpKi` | 速度 kp + 速度 ki |

舵轮用位置模式，驱动器内部是串级结构，速度内环参数必须单独下发，所以是**两条命令**。

| 量 | 协议槽 |
| --- | --- |
| 位置期望 | `motor_pos` |
| 位置 kp | `motor_pos_kp` |
| 位置 kd | `motor_pos_kd` |
| 速度 kp | `motor_vel_kp` |
| 速度 ki | `motor_vel_ki` |
| 前馈电流 | `motor_cur` |

`motor_vel`（速度期望）在位置环下不使用。

---

## 二、已核实的现状

| 项 | 结论 |
| --- | --- |
| CAN1 硬件 | **已初始化并启动**：`MX_FDCAN1_Init`、`HAL_FDCAN_Start`、RX FIFO0 中断、独立发送队列、BUS-OFF 自恢复都已存在 |
| CAN1 接收 | 已接入统一回调 `User_FDCAN_RxTaskParse` → `MergedControl_OnCanFrame` |
| 主控对 CAN1 | `MergedControl_OnCanFrame` 开头 `if (bus != &hfdcan3) return;` **直接丢弃 CAN1 帧** |
| 帧首字节 | 命令 `0x300 + driver`，回报 `0x280 + driver`，两条总线协议相同 |
| 舵轮在协议 | `VB_AXIS_COUNT = 8`，注释写明"四舵轮ROS预留，不占本帧" |
| 舵轮在 ROS 消息 | `McuTransmit/McuReceive` **已有 `wheel1..4` 字段**，不用加 |
| 舵轮在 mcuNode | `Send2MCU` 只组 8 轴数组，`ReadFromMCU` 只解 8 轴，**舵轮完全不编码** |
| 舵轮在动作节点 | 只在"全轴禁用"时把 wheel 置 `kDisabled`，从不主动控制 |
| 腿 PVT 门控 | `MERGED_ALLOW_UNVALIDATED_PVT` 默认 0；`MergedRequestedMode` 要求 `axis >= VB_LEG1` |
| 现有 PVT 映射 | `pos→p`、`vel→v`、`pos_kp→kp`、`pos_kd→kd`、`cur→t`，**已经是完整的，不需要补任何东西** |
| 复位事务 | `MergedRunResetTransaction` 每轴走同一套流程，唯一特例是硬编码的 `VB_ARM2` 配对 |

---

## 三、协议改动（两端共用，必须同时改）

### 3.1 拆开两个轴数概念

现在 `VB_AXIS_COUNT = 8` 同时表示"帧里几台电机"和"几台进复位事务"。加舵轮后必须分开：

| 宏 | 值 | 用途 |
| --- | --- | --- |
| `VB_MECH_AXIS_COUNT` | 8 | 机构电机（双臂、盘、丝杆、四腿），**复位事务遍历这个** |
| `VB_AXIS_COUNT` | **12** | 帧内全部电机 = 机构 8 + 舵轮 4 |

所有 `for (axis = 0; axis < VB_AXIS_COUNT; ++axis)` 都要逐个判断该用哪个：

- 拆帧、收 CAN、编码反馈、ready 计算 → **12**
- 复位事务的失能/复位遍历 → **12**（舵轮也进事务），但 `MergedValidResetMask` 逻辑按机构分组

### 3.2 帧尺寸

| | 现在 | 改后 |
| --- | --- | --- |
| 下发槽 | 1 + 8×7 = 57 | 1 + 12×7 = **85** |
| 下发帧 | 261 B | **373 B** |
| 反馈槽 | 1 + 8×3 = 25 | 1 + 12×3 = **37** |
| 反馈帧 | 133 B | **181 B** |
| 控制区 | 29 B | **29 B 不变** |

### 3.3 控制区字节分配（不改长度，只用空位）

下发控制区现用 2..10，11..28 空着：

| 字节 | 用途 |
| --- | --- |
| 11 | 舵轮模式：4 轮 × 2 bit |
| 12 | 舵轮 clean 位：4 bit |
| 13 | 舵轮复位掩码：4 bit |

反馈控制区现用 2..18，19..28 空着：

| 字节 | 用途 |
| --- | --- |
| 19..22 | 舵轮 4 轴 error |
| 23 | 舵轮 enable 位：4 bit |
| 24 | 舵轮 reset 准备位：4 bit |
| 25 | 舵轮 ready 位：4 bit |

### 3.4 轴槽枚举

```c
VB_WHEEL1 = 8, VB_WHEEL2, VB_WHEEL3, VB_WHEEL4
```

### 3.5 必须同步的断言

| 文件 | 内容 |
| --- | --- |
| `protocol/usb_protocol.h` | 帧长宏由轴数推导，改轴数即生效 |
| `protocol/tests/codec_test.c` | `261/133` → `373/181` |
| `pc/src/mcu/include/mcu/mcuNode.hpp` | `static_assert(261/133)` 同上 |
| `merged_control.c` | `_Static_assert(VB_COMMAND_BYTES == 261U)` 等三条 |

**帧长两端必须同时更新，否则主控会拒收所有命令。**

---

## 四、主控改动（merged_control.c / .h）

### 4.1 总线按轴选择

现在所有发送硬写 `&hfdcan3`。改为：

```c
/* 中文名：取本轴所在 CAN 总线。
 * 作用：机构轴 0..7 用 CAN3，舵轮 8..11 用 CAN1。 */
static FDCAN_HandleTypeDef *MergedAxisBus(uint8_t axis);

/* 中文名：取本轴驱动器号。
 * 作用：机构轴返回 1..8，舵轮返回 1..4。 */
static uint8_t MergedAxisDriver(uint8_t axis);
```

所有 `SetDriverState`、`SetDriverMode`、`SetVelKpKi`、`PosCtrl`、`PVTCtrl`、`ResetDriver`、`SetDriverZero`、`IfResetOkFlag` 的 `&hfdcan3` 和 `driver` 都要换成这两个函数。

### 4.2 收 CAN1

`MergedControl_OnCanFrame` 现在：

```c
if (bus != &hfdcan3 || ...) return;
axis = identifier - 0x281;
```

改为：

- `bus == &hfdcan3`：ID 0x281..0x288 → 轴 0..7
- `bus == &hfdcan1`：ID 0x281..0x284 → 轴 8..11
- 其它总线或 ID 仍丢弃

### 4.3 腿 PVT 打开

只改门控，PVT 的发送逻辑本来就已经完整：

1. `merged_control.h`：`MERGED_ALLOW_UNVALIDATED_PVT` 默认 **0 → 1**
2. `MergedRequestedMode`：确认只允许腿用 PVT（`axis >= VB_LEG1 && axis < VB_WHEEL1`），舵轮不允许 PVT 请求
3. `MergedSendTarget` 的 PVT 分支：**不用改**，现有映射已经发齐 p/v/kp/kd/t

### 4.4 舵轮执行路径

舵轮走位置模式，复用 `MergedExecuteAxis`，但要处理：

| 点 | 现在 | 需要 |
| --- | --- | --- |
| 复位判据 | `MergedResetReady(axis, ...)` 用 `status.reset_pos_ok_flag` | 舵轮同样用真实复位位，不改 |
| 位置限幅 | `MergedPosition()` 里 `axis >= VB_LEG1` 给腿夹 ±1.05 圈 | **改成 `axis >= VB_LEG1 && axis < VB_WHEEL1`**，否则舵轮被误夹 |
| 增益逆缩放 | 已删除（上一轮清理"小电脑预乘→主控除回"的往返） | 舵轮不参与，保持现状 |
| 速度环增益 | `runtime->vel_kp/vel_ki/gains_known` 缓存 | 舵轮复用同一套缓存 |

**`axis >= VB_LEG1` 是隐蔽陷阱**：加舵轮后 `VB_WHEEL1 = 8` 也满足该条件。

### 4.5 复位事务支持舵轮

现有代码基本不用改，只需：

1. `MergedValidResetMask`：增加"舵轮四位任意子集合法"的分支。现有函数按机构分组（腿/臂/盘丝），加一组舵轮判断即可，**不是重写**。
2. `MergedRunResetTransaction`：遍历到轴 8..11 时，`ResetDriver` 用 `MergedAxisBus()` 返回的 hfdcan1，驱动号用 `1..4`。
3. ARM2 特例条件 `s_reset_axis == VB_ARM2 && (mask & MCU_MASK_ARMS) == MCU_MASK_ARMS` 保持不动，舵轮不会命中。

**结论：舵轮进复位事务只改两处判断，现有流程完全兼容。**

### 4.6 复位后重新配置

复位事务开始时会把所有轴的 `sent_state` / `sent_mode` / `mode_confirmed` 清零等待重配。舵轮纳入后自然一起清零，不需要额外处理。

### 4.7 反馈

`MergedSendFeedback`：

- ready 计算循环扩到 12 轴
- 舵轮的 ready 判据包含真实复位 + 模式确认 + 使能确认（与机构轴相同）
- error / enable / reset / ready 各扩到覆盖舵轮 4 位（见 3.3）

---

## 五、小电脑改动

### 5.1 消息定义

`McuTransmit.msg` / `McuReceive.msg` **字段不动**，只更新注释（说明 wheel1..4 现在真的编码了）。

### 5.2 mcuNode 发送编码

`Send2MCU`：

```cpp
// 现在
const std::array<base_interfaces::msg::Motor, 8> motors{ arm1, arm2, plate, slide, leg1..4 };
// 改成
const std::array<base_interfaces::msg::Motor, 12> motors{ ... , wheel1..4 };
```

循环里逐轴组帧的逻辑（填 pos/vel/cur/kp/kd/vel_kp/vel_ki、算 mode 字节、算 clean 位）对 12 轴通用，但：

| 项 | 现在 | 需要 |
| --- | --- | --- |
| mode 字节 | `3 + i/4`，两字节装 8 轴 | 舵轮另写 `txDataMcu_[11]`，模式值取 `(wheel.controlmode & 3) << ((i-8)*2)` |
| clean 字节 | `5 + i/8` | 舵轮另写 `txDataMcu_[12]`，第 `i-8` 位 |
| 复位掩码 | `txDataMcu_[6]` 装机构 8 位 | 舵轮掩码另写 `txDataMcu_[13]` 低 4 位 |

### 5.3 mcuNode 接收解码

`ReadFromMCU`：

- 槽数从 25 扩到 37，循环到 12 轴
- 舵轮反馈写回 `receivedData_.wheel1..4` 的 `motor_pos / motor_vel / motor_cur / error / enablestate`
- 舵轮 enable / reset / ready 各从新增控制字节取 4 位

### 5.4 虚拟主轮分发

在 `action_planner_node.cpp` 加一个独立函数，把虚拟主轮位置映射到四轮：

```cpp
/* 中文名：把虚拟主轮位置分发到四个舵轮。
 * 作用：把上层给的一个虚拟主轮位置（圈）换算成四个舵轮各自的位置目标（圈）。
 * 当前四个轮子运动同步，返回四个相同值；若将来改为差速或转向角几何，只改本函数。
 * 参数：virtual_turns 虚拟主轮位置（圈）。返回：四轮位置目标（圈）数组。 */
std::array<float, 4> DispatchVirtualWheel(float virtual_turns) const;
```

### 5.5 舵轮参数（每轮独立）

新增 ROS 参数，16 个增益 + 4 个前馈电流：

| 参数 | 含义 |
| --- | --- |
| `wheel1_pos_kp` … `wheel4_pos_kp` | 各轮位置比例增益 |
| `wheel1_pos_kd` … `wheel4_pos_kd` | 各轮位置微分增益 |
| `wheel1_vel_kp` … `wheel4_vel_kp` | 各轮速度环比例增益 |
| `wheel1_vel_ki` … `wheel4_vel_ki` | 各轮速度环积分增益 |
| `wheel1_ff_current` … `wheel4_ff_current` | 各轮前馈电流（A） |
| `wheel_virtual_turns` | 虚拟主轮位置（圈），或由模式计算 |

全部默认 NaN，必须显式配置，与现有 `plate_previous_kp` / `slide_pos_kp` 的做法一致。

### 5.6 模式入口

`kModes` 表加一个舵轮测试子模式（例如 `test_way0` 下的 `wheel_virtual`），参与轴掩码加舵轮位 `0x0F00`（bit8..11）。

`validateWheelTest()` 检查：16 个增益和 4 个电流必须全部有限，否则拒绝输出并报错。

---

## 六、实施顺序（每步可独立验证）

| 步 | 内容 | 验证方式 |
| --- | --- | --- |
| 1 | 拆 `VB_MECH_AXIS_COUNT` / `VB_AXIS_COUNT`，帧长 373/181，两端断言同步 | `codec_test.c` 通过；主控编译链接通过；舵轮字段全 0 时行为不变 |
| 2 | 主控 `OnCanFrame` 接受 CAN1 | 0x281..0x284 落到轴 8..11 |
| 3 | 主控总线参数化 `MergedAxisBus` / `MergedAxisDriver` | 机构轴行为不变（回归） |
| 4 | 舵轮执行路径（位置模式） | 先只做"失能 → 使能"，确认驱动器有反应 |
| 5 | 复位事务支持舵轮 | 单轮复位、四轮一起复位 |
| 6 | PC 编码/解码 12 轴 | 回环验证 |
| 7 | 虚拟主轮 + 16 个参数 | 先发固定位置验证四轮同向同速 |
| 8 | 腿 PVT 门控打开 | 低速小幅度先试 |

**腿 PVT 放最后**，因为原注释写明"尚未完成电驱验证"。

---

## 七、风险清单

| 风险 | 说明 | 对策 |
| --- | --- | --- |
| `axis >= VB_LEG1` 语义污染 | 舵轮满足该条件，会被误夹 ±1.05 圈 | 改成 `>= VB_LEG1 && < VB_WHEEL1` |
| 帧长变化 | 两端版本不一致则主控拒收所有命令 | 同时更新，加静态断言 |
| 5ms 内帧变长 | 373 B @921600 约 4ms，加 CAN 配置帧可能挤爆周期 | 实测；必要时拆分发送 |
| CAN1 与 CAN3 发送任务互不影响 | 两条总线各有独立队列和任务 | 已核实，无需改 |
| 腿 PVT 未验证 | 原注释写明尚未完成电驱验证 | 保持 ±1.05 圈临时限幅，低速小幅度先试 |
| 舵轮复位语义 | 进复位事务后，四轮独立复位，与"运动同步"不冲突 | 已确认，按此实现 |

---

## 八、需要修改的文件清单

| 文件 | 改什么 |
| --- | --- |
| `protocol/usb_protocol.h` | 拆轴数宏、加舵轮槽、加控制区字节定义 |
| `protocol/tests/codec_test.c` | 帧长断言 |
| `pc/src/mcu/include/mcu/mcuNode.hpp` | 帧长断言 |
| `pc/src/mcu/src/mcuNode.cpp` | 12 轴编解码、舵轮控制字节 |
| `pc/src/base_interfaces/msg/McuTransmit.msg` | 仅注释 |
| `pc/src/base_interfaces/msg/McuReceive.msg` | 仅注释 |
| `pc/src/action_planner/src/action_planner_node.cpp` | 虚拟主轮分发、16 参数、模式表 |
| `pc/src/action_planner/launch/*.py` | 新参数转发（可选） |
| `2027_RC_.../User_Ctrl/Src/merged_control.c` | 总线参数化、CAN1 接收、位置限幅条件、舵轮复位分支、反馈扩 12 轴 |
| `2027_RC_.../User_Ctrl/Inc/merged_control.h` | `MERGED_ALLOW_UNVALIDATED_PVT` 默认值 |
| `2027_RC_.../User_Ctrl/Src/user_task.c` | 可能需要注册 CAN1 回调（已注册，待核实是否真的分派） |
