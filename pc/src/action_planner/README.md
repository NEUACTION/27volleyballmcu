# action_planner

C++17 ROS2 节点 `actionPlannerNode`，依赖 `planner`、`rclcpp`、`base_interfaces`。
参考 PC 工程只沿用节点和话题命名；当前消息以本仓库定义为准。
发布 `/mcu/transmit`，订阅 `/mcu/receive`，5ms `wall_timer` 每次回调在线计算并发布一帧。
保存的是规划系数、实例时间、上一周期增益和最新反馈，没有预生成缓存点列，也不上传轨迹。
5ms 是调度请求而非实时保证；调度延迟时每次回调仍仅推进一个5ms逻辑周期，不补算丢失周期。
目标发布和反馈订阅均使用QoS depth=1，只保留最新一帧，避免旧目标积压。
规划与MCU串口节点的两个5ms定时器相位独立，理想情况下也可能有至多一个周期的节点相位等待；
ROS传输、系统调度和串口实际延迟需实测，不能据此宣称严格硬实时5ms或总延迟≤5ms。

## 构建与默认启动

在配置好 ROS2 的 Linux 环境、工作区 `pc` 中运行：

```sh
colcon build --packages-up-to action_planner mcu
. install/setup.sh
ros2 launch action_planner action_planner.launch.py
```

launch 同时启动既有 `mcu` 可执行程序；可传 `mcu_serial:=/dev/ttyUSB0 baudrate:=921600`。
默认 `category=idle`、`submode=""`、`run_successed=false`；idle 全轴禁用，运行标志始终为false。
任何模式在 `run_successed=false` 时均只发布全轴禁用帧，不调用规划Step、不推进采样时钟。
观察到 `false→true` 时按当前模式重置实例，以该次启动作为离散时间采样原点；没有预生成或预测缓冲。
只有显式设置 `run_successed=true` 才开始在线采样并向MCU声明运行；这不代表硬件使能成功或动作完成。

## 真实复位与配置握手

所有动作都读取MCU反馈的 `reset_mask` 与 `ready_mask`，只等待当前模式实际使用的轴。
位0/1为ARM1/ARM2、位2为PLATE、位3为SLIDE、位4–7为LEG1–LEG4；双臂模式mask=3，
盘mask=4，丝杆mask=8，接球mask=240，组合mask=7，不要求无关轴复位。
PC发独立`reset_request_mask`/`reset_request_id`，先等首条反馈分配非旧号，再同时等待本次`reset_request_ack`和所选`reset_mask`。其余机构不参与本次复位。ARM1、盘、丝杆、腿使用真实复位完成状态；ARM2按原流程在ARM1完成后SetDriverZero成功入队，**不额外检查ARM2复位回报**，mask bit1只表示该配对准备流程完成，不代表ARM2机械到位或ACK。`ready_mask`另要求各轴新模式/使能回报，不由PC延迟或周期计数伪造。
没有反馈或所选轴reset未齐时，运行标志false、全轴禁用，规划不推进。

reset已齐但ready未齐时，PC给选定模式发真实第一步目标及对应控制模式/增益，运行标志true供MCU完成配置；
每周期生成前和发出后重置实例，重复同一首步，不累计时间、位移、准备就位或组合计数。
这不是数学t=0：一般row、准备角和丝杆等原首帧含离散步长，不能把机械0臆定为安全准备角。
配置过程中驱动可能先到这一真实首步，ready齐后仍从同一首步开始；这是等待真实复位/配置的握手，
没有轨迹预上传或缓存未来点列。组合首步额外给所选PLATE零速及显式前次增益配置，避免盘尚未被序列选中而永远无法ready。
初始参数无效仍拒绝输出，PVT实验另受双门控限制。

中途丢失reset时禁用；未完成轨迹等待配置时回到首步，不继续消耗旧时间。
已按用户批准锁存结束失能的双臂，不因其enable/ready位清除而重新配置或起挥；
组合结束后只继续检查盘的ready以求值盘目标，双臂仍需保留真实复位确认。
结束锁存只有mode变化或显式运行门控重新开启才清除。反馈目前无超时检测，需实测链路时效。

## 类别互斥与参数

通过`category`选择唯一机构类别，并用`submode`选择该类别算法；非法组合拒绝输出。运行期改变类别、子模式或`run_successed`上升沿都会重置规划状态。`idle`不运动；`catch`仅接受`catch_t`、`catch_five1`、`catch_five2`及默认关闭的PVT实验；`serve_way1`仅接受八个`legacy_way1_*`组合身份；`test_way0`开放独立历史算法测试，但不接受catch或serve组合身份。`ps_num3`仅由serve组合消费。所有指令仍在统一5ms Tick出口逐帧生成，不预上传轨迹。

示例：`ros2 launch action_planner action_planner.launch.py category:=catch submode:=catch_t run_successed:=false`。发球例：`category:=serve_way1 submode:=legacy_way1_high`；单独历史臂测试例：`category:=test_way0 submode:=row_legacy_way0_five`。

| mode | 在线算法与输出 |
| --- | --- |
| `idle` | 全轴禁用 |
| `plate_time` | `PlateTimePlanner::Step()`，盘速度模式 |
| `plate_distance_way0_high` | `ServeDistanceProfile::Way0High` |
| `plate_distance_way0_low` | `ServeDistanceProfile::Way0Low` |
| `plate_distance_way0_try` | `ServeDistanceProfile::Way0Try` |
| `plate_distance_way1_high` | `ServeDistanceProfile::Way1High` |
| `plate_distance_way1_low` | `ServeDistanceProfile::Way1Low` |
| `plate_distance_way1_high_down` | `ServeDistanceProfile::Way1HighDown` |
| `plate_distance_way1_high_down117` | `ServeDistanceProfile::Way1HighDown117` |
| `plate_distance_way1_high_down1110` | `ServeDistanceProfile::Way1HighDown1110` |
| `plate_distance_way1_low118a` | `ServeDistanceProfile::Way1Low118A` |
| `plate_distance_way1_low118b` | `ServeDistanceProfile::Way1Low118B` |
| `plate_distance_way1_low119` | `ServeDistanceProfile::Way1Low119` |
| `catch_t` | 高度梯形轨迹，四腿位置模式 |
| `catch_five1` | 高度五次轨迹，四腿位置模式 |
| `catch_five2` | 角度五次轨迹，四腿位置模式 |
| `catch_five2_pvt_experimental` | FIVE2位置/速度PVT实验；另需allow_unvalidated_pvt=true，默认拒绝 |
| `row_general` | 一般挥臂状态机，双臂位置模式 |
| `row_legacy_way0_five` | WAY0 FIVE，显式输入，五次起挥+恒速FOLLOW+提前失能 |
| `row_legacy_way1_five1_high_down1110` | WAY1原数字12 FIVE1实验，调用完整外层策略 |
| `row_legacy_way1_high` | WAY1 HIGH独立轨迹，结束保持失能 |
| `row_legacy_way1_high_down` | WAY1 HIGHDOWN独立轨迹，结束保持失能 |
| `row_legacy_way1_high_down117` | WAY1 HIGHDOWN117独立轨迹，结束保持失能 |
| `row_legacy_way1_high_down1110` | WAY1 HIGHDOWN1110独立轨迹，结束保持失能 |
| `row_legacy_way1_low118a` | WAY1 LOW118A独立轨迹，结束保持失能 |
| `row_legacy_way1_low118b` | WAY1 LOW118B独立轨迹，结束保持失能 |
| `row_legacy_way1_low119` | WAY1 LOW119独立轨迹，结束保持失能 |
| `slide_legacy_way0_high` | 历史WAY0 High，软件位置每周期增加35/200圈，目标241.16圈 |
| `slide_legacy_way0_low` | WAY0 Low，软件位置每周期增加40/200圈，目标11.42圈（当前用户调整值） |
| `slide_legacy_way1_high` | 历史WAY1 High，软件位置每周期增加35/200圈，目标241.16圈 |
| `slide_legacy_way1_low` | 历史WAY1 Low，软件位置每周期增加40/200圈，目标711.42圈 |
| `slide_legacy_way1_high_down` | 历史WAY1 HighDown，软件位置每周期增加35/200圈，目标241.17圈 |

每次观察到 `mode` 变化，重置所有规划实例与接球时间，重新读取前次盘增益和显式丝杆位置增益。
同名模式重复赋值不会重启；可先切回idle再切入目标模式，或关闭运行门控后重新开启。
未知或未支持字符串（包括除显式实验入口外的PVT模式及其它未列出的历史模式）会明确报错，保留原字符串参数，
拒绝运动并每周期输出全轴禁用消息，不会静默当作idle继续。

`plate_previous_kp`、`plate_previous_ki` 默认NaN，必须在进入路程模式前显式填写有限值，
取值应来自调用前真实驱动增益；本包不猜测初始增益。每帧将库返回增益保留为下一帧前次值，
以保持原函数未赋值字段的历史行为。运行中修改这两个参数后需重新进入mode或重新开启运行门控才生效。
路程模式未收到反馈、位置非有限或前次增益未配置时只输出禁用命令并周期报错。
输入是反馈 `plate.motor_pos` 的原始有符号圈数，直接沿用原 `plateMsg.pos/4096` 的回零坐标，
不减进入模式时的位置、不取绝对值、不额外除4096；MCU/驱动回零由现有系统负责。
无反馈超时管理：收到过一帧后会继续使用最新值，部署前应另行确认反馈链路策略。
盘速度为圈/秒。盘PID直接填写驱动内部乘之前的原值（如1.8/0.4），驱动 SetVelKpKi 会再乘1000；小电脑与主控都不再做预乘/逆缩放。

例如先启动默认launch，再配置一种路程模式并观察禁用输出：

```sh
# 示例增益须根据实际驱动前次状态调整；此处仍保持run_successed=false。
ros2 param set /actionPlannerNode plate_previous_kp 1.8
ros2 param set /actionPlannerNode plate_previous_ki 0.4
ros2 param set /actionPlannerNode category test_way0
ros2 param set /actionPlannerNode submode plate_distance_way0_high
ros2 topic echo /mcu/transmit
# 切回空闲，丢弃动作实例状态。
ros2 param set /actionPlannerNode category idle
```

接球按当前接球 `User_Ctrl/Src/user_task.c:712–785`：先采样t位置，若t未到总时长则t+=0.005，
再按推进后的时间判断增益。四轴位置符号为 `[-,+,-,+]`，kd均为0.0005，kp逐轴独立：
加速 `[12,11.8,10.8,11.5]`、匀速 `[10.5,9.8,10,10.5]`、减速/保持 `[9,8.8,9,9.5]`。
总时长后保持终点位置和最后阶段增益；这三个普通catch模式仅发送位置目标。
普通catch位置保持原规划数学值，由MCU最终保留±1.05圈安全限幅，不把驱动限幅混入规划曲线。

一般row每帧调用 `GeneralRowPlanner::Step()`，通过 `MapGeneralRowTurns` 展开双臂，
ARMSTART=1.75圈已核对发球工程 `User_Ctrl/Inc/user_task.h:28`。
保持两个输出目标的独立kp/kd，STOPPING失能意图及Complete阶段均输出禁用帧。
单逻辑轴没有拆成预存轨迹或合并左右臂增益。
PC本地控制模式使用0禁用、1速度、2位置、3实验PVT；未选中轴每帧清零禁用。

## FIVE2 PVT实验入口

`catch_five2_pvt_experimental` 需同时满足 `run_successed=true` 和
`allow_unvalidated_pvt=true`；两项均默认false，launch也默认关闭实验许可。
缺少任一许可时全轴禁用且不推进采样时间。mode变化、运行门控上升沿、以及本实验许可重新开启时从t=0重启。
集中在同一个Tick出口发布，无额外发布路径，也没有预生成轨迹点。

用户已明确批准修复原 `getHitPosPVT` 第822–825行将速度误写回`.pos`的笔误：
本入口保留位置到 `motor_pos`，把同时刻的速度写入 `motor_vel`。
每5ms先用FIVE2数学求t时刻位置和速度，再按原条件推进t；四腿位置和速度均采用 `[-,+,-,+]`。
ROS统一接口位置单位圈、速度单位圈/秒；小电脑不乘2π，后续主控桥接到PVT时转换为rad和rad/s。
在串口每轴七float布局中，`motor_pos_kp/motor_pos_kd` 两槽对PVT承载原 `mit_kp/mit_kd`，
四轴原值均为1.0，不乘普通PosCtrl所用的增益换算，也不冒称普通位置环增益。
原`tor`没有有效赋值，本节点 `motor_cur=0` 表示没有额外前馈电流，不猜测历史力矩/电流目标。

源 `VEL_LIMIT=1.05` 带“随便设”的试验性质，不能作为已实测速度限幅或新轨迹参数；
本节点不据此裁剪FIVE2速度，不在PVT数学接口里加入猜测限幅。
原普通位置模式的MCU±1.05圈安全边界与该速度试验常量用途不同，仍由主控最终位置控制路径负责。
PVT只提供默认关闭的显式实验入口，尚未完成机构、反馈闭环、增益与最终2π桥接的实机验证。
开启该模式不代表已验证，也不代表主控或驱动一定支持当前实验命令。

## 历史挥臂与独立准备模式

`row_legacy_way0_five` 需要以下七项显式有限参数，默认全部NaN，mode变化或运行门控上升沿读取：

| 参数 | 单位及用途 |
| --- | --- |
| `row_five_home_rad` | 准备角，rad |
| `row_five_hit_rad` | 击球角，rad |
| `row_five_end_rad` | 恒速收杆终点，rad |
| `row_five_hit_velocity_rad_s` | 非零击球角速度，rad/s，与两段位移方向一致 |
| `row_five_max_speed_rad_s` | 正角速度上限，rad/s |
| `row_five_max_acceleration_rad_s2` | 正角加速度上限，rad/s² |
| `row_five_minimum_lead_s` | 非负起挥预留时间，s |

参考553–655行有效语句的历史几何是 `home=-1.75*2π`、`hit=home+4.3*2π`、
`end=hit+1.8*2π`，lead=0.035s、角加速度上限9000rad/s²；节点不把这些变成隐式默认值。
击球时间按原式 `now+2*abs(hit-home)/abs(hit_velocity)+lead+0.005` 推导，时间每5ms递增后采样。
只调用 `PlanRowPositionTrajectory` 保存规划参数、`GetRowPositionReference` 现算当前点；
库的首段101点限幅检查不是缓存或发送轨迹，也不保证整段/实机限制。
FOLLOW阶段覆盖为 `hit+hit_velocity*(now-hitTime)`，保持恒速，
并在 `hitTime+(end-hit)/hit_velocity` 提前失能，不等待数学核心第二段五次减速结束。
左右臂位置由 `MapLogicalRowTurns` 展开，保留-0.009圈偏置及独立原始增益。
完成后持续发带原终点位置的禁用模式帧，直到模式或运行门控重置；运行标志仅表示已提交有效指令，不表示到位。
规划或求值失败锁存，本次不继续动作；修正参数后需重新启动。

`row_legacy_way1_five1_high_down1110` 调用 `RowVariants::ControlArmPosition2(12)`，
保留库内原历史实验参数、分段增益、方向零偏、恒速follow和提前失能，不套WAY0的显式输入。
七个 `row_legacy_way1_*` 一般轨迹在库中分别独立保存时间/位置状态，但原控制外层在停止周期
同时产生“先失能、再PosCtrl”的顺序意图。当前单帧ROS命令只能表达一个控制模式，不能忠实保留这两条事件的先后。
用户已明确批准“结束保持失能”：本节点在该停止周期选择失能并锁存，不再发送随后的PosCtrl，
后续每帧继续禁用双臂，直到mode变化或运行门控重新开启才允许重启动作。
这是经用户批准改变原停止周期“失能+位置”旧顺序的控制策略；库内两种意图仍原样保留。
各轨迹的方向、起始角、-0.009偏置是否适用以及每轴增益直接取库结果，不统一套一组映射。

准备模式均为显式独立测试入口，需要有限的MCU `arm1.motor_pos` 反馈（圈），未收到则禁用且不推进准备状态。
支持以下八个完整mode名称：

- `row_prepare_legacy_way1_high`
- `row_prepare_legacy_way1_high_down`
- `row_prepare_legacy_way1_high_down117`
- `row_prepare_legacy_way1_high_down1110`
- `row_prepare_legacy_way1_low118a`
- `row_prepare_legacy_way1_low118b`
- `row_prepare_legacy_way1_low119`
- `row_prepare_legacy_way1_five1_high_down1110`

每周期调用 `ControlArmPosition1(mode, measuredTurns)`，直接发送两轴各自位置/增益，
不套执行阶段的统一映射：准备阶段arm2=-arm1，没有-0.009圈偏置。
准备位置从软件0圈每周期下降0.05圈至库内对应准备角，不从反馈位置重新规划。
误差相对固定准备角，满足0.02圈误差的次数累计至200后置就位；中间超差不会清零，
这是累计计数而非连续200周期到位。首次就位打印计数日志，保持准备控制，不自动切换到挥击。
mode变化或运行门控重启会清零准备实例；无反馈时效检查，复用旧反馈的限制同盘路程模式。
这些准备模式和独立挥击模式不自动串联；只有显式选择下面组合模式时才使用唯一ps_num[3]触发字节执行原组合试验。

## WAY1光电触发的原组合实验

以下为八个内部规划身份。启动时固定 `category:=serve_way1`，`submode`填下表名称去掉`serve_combined_`前缀（例如`legacy_way1_high`）；不再使用旧`mode`参数。仅透传原 `ps_num[3]`，不扩展其它光电接口，不猜测传感器安装含义：

| mode | 原执行身份 / 盘路程分支 |
| --- | --- |
| `serve_combined_legacy_way1_high` | HIGH / Way1High |
| `serve_combined_legacy_way1_high_down` | HIGHDOWN / Way1HighDown |
| `serve_combined_legacy_way1_high_down117` | HIGHDOWN117 / Way1HighDown117 |
| `serve_combined_legacy_way1_high_down1110` | HIGHDOWN1110 / Way1HighDown1110 |
| `serve_combined_legacy_way1_low118a` | LOW118A / Way1Low118A |
| `serve_combined_legacy_way1_low118b` | LOW118B / Way1Low118B |
| `serve_combined_legacy_way1_low119` | LOW119 / Way1Low119 |
| `serve_combined_legacy_way1_five1_high_down1110` | FHIGHDOWN1110(12) / Way1HighDown1110 |

前置条件是所选ARM1/ARM2/PLATE的真实复位与配置就绪、有限的arm1/plate圈位置、显式有限的
`plate_previous_kp/ki`。唯一触发输入为 `McuReceive.ps_num3`；reset/ready只是硬件状态反馈，
不扩展新的传感器。未复位不执行；配置等待只重复首步并重置原序列，配置/复位不依赖PC计数猜测。

ready之后每5ms先读取准备前positionOk；其为false时调用 `ControlArmPosition1`，
取得本周期准备后positionOk，再将before/after与原字节传给 `ServeSequence::step`。
准备计数保持原累计非连续规则；本周期刚到位即可参与启动字节计数。
就位后至真正起挥前，重发最后已下发的准备目标，保持双臂位置，不提前禁用；
保存的是一个已发送保持目标，不是预生成未来点列。

原固定计数不变：positionOk为true后，原字节为0连续1000周期才锁存start，非零清该连续计数；
再等1400周期，下一周期才首次求盘；首个盘周期开始计22周期间隔，第23个盘周期才起挥。
若从首个已就绪执行周期起positionOk一直true且字节始终0，首次盘意图2401、首次臂意图2423。
所有原模式实际都使用gap22；同周期盘先于臂求值，二者填入同一消息，不使用新上手/下手分类。
FHIGHDOWN1110明确配HighDown1110盘曲线，LOW118A明确配118A，不猜测其它分支。
臂结束按用户批准保持失能，盘继续按原持续盘意图求值，不自动重复、重启或擅自停止盘。
类别/子模式或显式运行门控重启会清零组合状态；所选轴驱动复位事务由主控处理，ARM2按上述原置零流程。未移植原发球蜂鸣/PWM或其它传感器控制，也未加入机械限位传感器闭环。

## 历史丝杆模式的输入与边界

五个 `slide_legacy_way*` 名称仅对应原WAY分支和函数名，不声明上手/下手分类。
内部使用 `HistoricalSlidePlanner`，每次进入模式或重新开启运行门控，按对应预设创建独立实例并从软件0圈开始。
每个有效5ms周期只调用一次 `Step()`；首次输出即为一步增量，到目标后输出目标保持位置。
各模式统一在累计位置超过目标时返回目标；恰好等于目标时直接返回相等的累计位置，因此无需区分`>=`和`>`。内部累计量仍增长，不预存轨迹点列。
输入是 `mode`、`run_successed` 和必填 `slide_pos_kp`、`slide_pos_kd`，不使用盘路程或MCU滑轨位置作为积分输入。
输出到 `slide.motor_pos`，单位沿用原电机圈数，不乘4096、不猜传动比或毫米换算。

两项位置增益默认NaN，必须显式配置为有限的原驱动数值；未配置时只发禁用帧，且不推进丝杆实例。
增益在mode变化或运行门控上升沿读取，因此修改后需重新进入模式或重新开启运行门控。
当前读取参考发球源码的WAY0有效调用（461–466行），实际为
`PosCtrl(..., slideTar.pos * 4096, 1.0 * 1000 * 60, 0.05 * 100)`，原程序传入 kp=60000、kd=5；
当前通道改由驱动 PosCtrl 内部乘 ×60000/×1000，因此这里应填驱动乘之前的原值 kp=1.0、kd=0.005。
这里只记录历史源码证据，不把它自动设置为当前机构默认值。
WAY1的滑轨目标和PosCtrl调用（1954–1956行）均被注释，kd=-1只是该注释中的历史哨兵，
不代表当前WAY1有已运行或已实测的增益。两类模式均要求操作者确认实际驱动配置后显式提供增益。

默认禁用状态下可配置和核对模式参数，例如：

```sh
# 仅当当前硬件已核实采用上述WAY0历史增益时填写以下值；本示例不启动运动。
ros2 param set /actionPlannerNode slide_pos_kp 1.0
ros2 param set /actionPlannerNode slide_pos_kd 0.005
ros2 param set /actionPlannerNode category test_way0
ros2 param set /actionPlannerNode submode slide_legacy_way0_high
ros2 topic echo /mcu/transmit
```

软件累计位置从0起步不等于驱动已回零。当前没有丝杆回零、真实机械限位、位置跟随检查、到位判定、
传动比、速度/加速度约束或WAY1实机验证；需在实际系统确认原点及行程后才显式开启运行门控。
通用 `ScrewPositionPlanner` 不是这些历史预设的实际配置来源，本节点没有混用二者。

## 尚未覆盖

- 已接入十一种明确命名的历史盘路程分支；未把它们归类为新的球种参数，历史自动选择器未接入。
- 丝杆已提供五个历史目标/步长预设的显式实验入口，真实机构回零、行程、传动和增益配置仍未确认，默认不运动。
- 未实现视觉、复杂模式仲裁、硬件完成判断、自动重复发球、反馈时效保护或完整MCU状态机。
- PVT仅有默认关闭的FIVE2实验入口；组合模式仅覆盖原ps_num[3]触发及固定计数协调，不覆盖其它传感器、完整硬件动作生命周期及接球同名冲突函数的其它历史版本。
- 已有规划库使用固定参考几何和动作参数，本包没有新增球种/视觉调参入口。
- 需要安装ROS2后完成真实colcon构建及串口/机构验证；Windows非ROS环境的静态检查不等于整机验收。
