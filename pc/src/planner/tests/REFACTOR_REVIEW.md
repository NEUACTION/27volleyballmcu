# Planner 参数与重复实现精简审查记录

> 目录已按机构整理；本文保留迁移时的范围和验证记录，当前接口、构建接入、调参和模式选择请看 [planner 使用指南](../README.md)。通用 ScrewPositionPlanner 现在与历史丝杆接口同在 slide/position 中。

本轮只修改 planner 包。盘、滑轨和 row 的公开函数、枚举值及结果字段保持，RowVariants 私有 stepGeneral 参数收敛为状态槽索引。用户追加明确要求去除 Catch 时长 getter 后，经 lead 协调 action 调用同步改造，删除其六个时长 getter 与 Height Mode()，改为直接公开共享 CatchTiming timing 和 Height 的 mode 字段。发送仍由调用层 action Tick 负责，planner 不引入 ROS 或总线发送。

## 实质变更与规模

|实现|本轮前行数|本轮后行数|变化|
|---|---:|---:|---|
|serve/plate_distance.cpp|899|129|移除11个内部 GetPlateVelocity 函数和 LegacySqrt；11行模式表由公开 EvaluateServeDistance 共用求值|
|slide/position.cpp|138|47|移除5个内部 GetSlidePosition 函数；预设查询与 Step 共用同一张表|
|serve/plate_time.cpp|81|64|移除3个仅单公式的 AccelerationFromDistance / DurationFromSpeedChange / DurationFromDistance|
|serve/arm_legacy.cpp|384|386|原先已有共用状态机；本轮把七模式速度、两段行程、准备角、方向和偏置集中一表，替代三处 switch 与分散的包装实参|
|catch/angle_five2.cpp|237|219|删除3个只读时长getter，将时长字段移至共用CatchTiming|
|catch/height_profile.cpp|247|224|删除3个时长getter与Mode()，共用CatchTiming；总时长按原求和式初始化|
|改动实现合计|1986|1069|减少917行（46.2%）；整个planner八个cpp由2301降至1384行|

函数净减少27个（盘12、滑轨5、时间盘3、Catch7）。row 的14个单行公开兼容入口仍保留，以免调用方编译冲突；它们只传状态槽索引，已不重复存储模式参数。没有为了内部旧名字保留盘/滑轨包装层。

## 保持的历史细节

- 盘速 float 被开方量先按原 `2 * distance * acceleration`、`target * target - 2 * deceleration * (distance - 1.70f)` 求值，再转 double 调用 sqrt；相同表达式重复 sqrt 合并一次，不变换乘除结合次序。
- Try 首段 <=1.0、第二段 <=1.50、首段饱和 `(target * 4 / 3)`；减速仍减1.70。重复PID同值写入及不可达分支不保留额外副本。
- Low 恒速段不写 ki，停止段不写 kp；Way1Low 起段不写 ki，低于1.2也不写 kp。所有未写字段从 previous_gains 继承。
- 滑轨每次先加步长，仅夹紧输出；内部累计值继续增长。当前统一使用 `>`：等于目标时直接返回同值累计量，删除无输出区别的比较开关。WAY0 Low 当前目标按用户修改保留为11.42圈；下方验证输出为当时历史记录。
- row 仍先更新速度后积分、旧累计量判停止、下一周期才失能；七组状态独立，双臂增益独立，执行阶段偏置及准备阶段无偏置保持。
- F 模式与1110共用准备位置，但第八组到位计数独立；误差超限不清计数/标记。
- 时间盘函数直接展开原公式，没有合并常量改变表达式结合顺序；仍每5ms求当前帧。

## 验证证据与边界

2026-11-23 使用指定 Python 与 ARM g++ 执行：

```text
verify_distance_reference.py --compiler <ARM g++> --compile-only
PASS: all 11 table rows match source speed/acceleration/deceleration and complete PID write sequences
PASS: distance core and differential fixture compile; numerical execution NOT RUN
verify_slide_reference.py --compiler <ARM g++> --compile-only
PASS: all 5 table rows match source target/increment/comparison data
PASS: slide core and differential fixture compile; numerical execution NOT RUN
verify_row_variants_reference.py --compiler <ARM g++> --compile-only
PASS: all 7 WAY1 table rows and 3 FIVE1 math bodies match reference
PASS: WAY1 core and original-function oracle compile; numerical execution NOT RUN
verify_row_reference.py --compiler <ARM g++> --compile-only
PASS: FIVE 3 function bodies and general trajectory statements match reference
PASS: core and runtime differential fixture compile; numerical execution NOT RUN
```

追加执行 `verify_catch_reference.py --compiler <ARM g++> --compile-only`：两份 CatchTiming 的全部时长表达式核对 PASS，三种 Catch 原源码 oracle 差分夹具 ARM 编译 PASS，数值执行仍未运行。夹具直接抽取只读 zlr_math.c 与其头中结构体，分别按 T/FIVE1/FIVE2 编译，覆盖分段时刻及相邻 double、三模式共30066次位置/速度对照与时长/重初始化断言。

另将全部8个 planner 实现逐一执行 `-std=c++17 -Wall -Wextra -Werror -Ipc/src/planner/include -c`，全部成功生成临时 ARM 对象。没有宿主编译器，没有执行数值差分程序，没有宣称上机通过。

旧盘/滑轨测试的逐函数语句比较已替换为完整模式表对照，原函数 oracle 仍从只读历史源直接抽取；未删除差分样本。盘夹具保留11模式边界/相邻float、密集逆序、增益继承及非法输入检查；滑轨保留5×10000步。新增 WAY1 row 夹具对原七个函数交错采样7×150帧，核对累计位置、双臂独立增益、位置映射与失能意图，并保留数学三函数逐句核对、重置和独立实例断言。未执行的夹具只算编译证据。

## 仍未化简项

Catch 三种数学公式保持现状，时长与模式读取已删除微小 getter 包装。两类直接公开同一 CatchTiming timing，字段为 acceleration_seconds / constant_seconds / deceleration_seconds / total_seconds（单位秒），原私有时长字段同步移除，无重复副本；Height 另公开 const CatchHeightMode mode。高度正逆解、雅可比、边界扫描、FIVE2系数求解有实际复用价值；两份 Catch 实现的几何重复尚未跨类抽取。WAY0/FIVE 与 WAY1/FIVE1 也仍有相似数学代码，暂保留其公开类型与源路径，避免本轮扩大更改范围。通用 ScrewPositionPlanner、序列调度以及双臂公开映射未更改。没有修改其它包或主控、协议、MCU目录。
