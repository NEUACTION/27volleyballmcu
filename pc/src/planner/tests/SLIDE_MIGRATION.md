# 真实GetSlidePosition历史实验预设迁移

> 目录已按机构整理；本文保留迁移时的范围和验证记录，当前接口、构建接入、调参和模式选择请看 [planner 使用指南](../README.md)。通用 ScrewPositionPlanner 现在与历史丝杆接口同在 slide/position 中。

> 当前补充：比较开关已删除，所有模式统一用 `>` 限制返回值；相等时累计值和目标数值一致，输出不变。WAY0 Low 当前目标保留用户调整的11.42圈；以下表格的711.42圈是原始参考值。查询接口只返回目标和增量，参考夹具对该用户调整显式应用同值后核对。
>
> 以下为初次迁移历史记录。本轮已删除5份内部同名函数，预设查询和 Step 共用同一张参数表。当前结构、数据对照和原函数 oracle 编译证据见 [精简审查记录](REFACTOR_REVIEW.md)。旧函数名/逐句验证描述仅记录初版。

本次澄清：原任务所指丝杆函数真实名称为GetSlidePosition，并非GetPlatePosition。新建slide/position.hpp/.cpp专门承载真实历史分支，未改写此前独立配置接口或现有文件；没有把这些历史数字指定为新的上手/下手业务参数。

## 实际源码与完整覆盖

只读参考：`D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c`。
条件编译WAY=0从51行起，WAY=1从1590行起。全文件GetSlidePosition搜索共找到5个函数体，全部迁移：

| 显式历史身份 | 真实函数/源行号 | 历史目标（转） | 每次增量（转） | 原目标判断 |
| --- | --- | --- | --- | --- |
| Way0High | GetSlidePositionHigh 1045–1066 | 241.16f | 35.0f / 200.0f | > |
| Way0Low | GetSlidePositionLow 1071–1082 | 711.42f | 40.0f / 200.0f | >= |
| Way1High | GetSlidePositionHigh 3319–3332 | 241.16f | 35.0f / 200.0f | > |
| Way1Low | GetSlidePositionLow 3337–3347 | 711.42f | 40.0f / 200.0f | >= |
| Way1HighDown | GetSlidePositionHighDown 3352–3362 | 241.17f | 35.0f / 200.0f | > |

Way0/Way1 High和Low即使当前数学式相同，也保持不同显式身份。未移植注释掉的其它候选目标。Low 1074行写“1秒70转”、3340行写“1秒80转”，实际有效代码均为40.0f/200.0f；迁移有效运算式，不根据注释改成70或80。

## 明确区分历史预设与新业务参数

HistoricalSlideProfile只表示旧WAY与函数身份，必须明确传入，没有默认选择。GetHistoricalSlidePreset提供单独的历史目标、每次增量和比较边界查询，不将其填入新的上手/下手参数表。

HistoricalSlidePlanner一个实例对应一个源函数的累计位置。Reset恢复源静态初值0；Step由调用方每5ms执行一次，首次调用即增加一步，然后根据原比较返回累计值或目标值。内部累计位置超过目标后仍继续增加，仅返回值限制在目标，这是源代码行为，未改成内部钳位。每个实例状态独立，不因调用别的预设而互相影响。应为需交替调用的不同历史函数各保留一个实例，而不是重新构造来隐式复位。

内部GetSlidePositionHigh/Low/HighDown函数名保留，原静态currentPosition改为实例引用参数，Low的static targetPosition和其余原有效语句、常量、比较符及顺序保持不变。

输出为原滑轨电机位置转数，未增加毫米换算、传动比、编码器计数、回零、机械限位或驱动指令。这里是每5ms现算一份位置，没有预生成或上传点列；调用方负责周期调度和实际发送。

## 验证证据

verify_slide_reference.py按原行号读取5个函数体，在仅移出currentPosition静态初值后，核对全部有效语句、常量、比较符和顺序；结果PASS。非法枚举返回nullopt，没有回退到任意历史预设。

使用`C:\Users\city\.eide\tools\gcc_arm\bin\arm-none-eabi-g++.exe`，`-std=c++17 -Wall -Wextra -Werror`成功把slide/position.cpp编译成临时对象，原函数oracle差分夹具也通过语法检查。ARM产物没有在Windows运行，未声称运行时数值验证通过。

```text
python tests/verify_slide_reference.py
python tests/verify_slide_reference.py --compiler <本机g++或clang++>
python tests/verify_slide_reference.py --compiler <ARM g++> --compile-only
```

使用本机可执行编译器时，测试5种来源各10000次共50000次采样，覆盖首步、整个推进过程、目标前后输出、长期保持，以及Reset、实例独立、历史预设查询和非法模式。不会把交叉产物作为Windows程序执行。

## 范围与未完成项

本文件中真实GetSlidePosition函数体已全覆盖。尚未接入主控周期、发球模式选择、回零流程、驱动反馈或ROS/CAN；历史High/Low/HighDown到新业务动作类别的映射也未指定。未修改CMake，因此实现尚未自动加入现有库。未修改现有serve/plate_time.cpp、MCU或主控。
