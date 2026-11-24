// 文件作用：发球盘路程规划，保存历史增益写入规则。 声明公共类型与接口。
#pragma once

#include <optional>

namespace planner
{

    // 显式选择参考源码的条件编译分支；不隐式指定工程实际使用的 WAY。
    enum class ServeDistanceProfile
    {
        Way0High,  // WAY=0 的 GetPlateVelocityHigh，源 1186–1285 行。
        Way0Low,   // WAY=0 的 GetPlateVelocityLow，源 1402–1464 行。
        Way0Try,   // WAY=0 的 GetPlateVelocityTry，源 1290–1397 行。
        Way1High,  // WAY=1 的 GetPlateVelocityHigh，源 3367–3438 行。
        Way1Low,   // WAY=1 的 GetPlateVelocityLow，源 3867–3926 行。
        Way1HighDown,      // WAY=1 GetPlateVelocityHighDown，源 3443–3513 行。
        Way1HighDown117,   // WAY=1 GetPlateVelocityHighDown117，源 3518–3591 行。
        Way1HighDown1110,  // WAY=1 GetPlateVelocityHighDown1110，源 3596–3663 行。
        Way1Low118A,       // WAY=1 GetPlateVelocityLow118A，源 3668–3729 行。
        Way1Low118B,       // WAY=1 GetPlateVelocityLow118B，源 3734–3797 行。
        Way1Low119         // WAY=1 GetPlateVelocityLow119，源 3802–3862 行。
    }
    ;

    // 驱动原始增益单位；原函数没有赋值的字段必须沿用上次值，不能补零。
    struct DistanceVelocityGains
    {
        float kp;  // 速度环比例增益，保留源代码的 *1000 量纲。
        float ki;  // 速度环积分增益，保留源代码的 *1000 量纲。
    }
    ;

    // 单次路程采样结果；输入路程为盘转数，速度为盘转/秒。
    struct ServeDistanceTarget
    {
        float velocity_turns_per_second;  // 源函数返回的盘速。
        DistanceVelocityGains gains;      // 执行源分支后应使用的增益。
    }
    ;

    // 按实际路程求盘速和 PID；previous_gains 必填，以保留原函数未写字段。
    // 非有限路程/增益、非法枚举返回 nullopt（参考代码对此没有完整定义）。
    // 按路程求盘目标：profile 选择历史模式，travel_distance_turns 为累计盘转数，previous_gains 为上次驱动增益；返回本次速度及增益，输入无效返回 nullopt。
    std::optional<ServeDistanceTarget> EvaluateServeDistance(
        ServeDistanceProfile profile, float travel_distance_turns,
        DistanceVelocityGains previous_gains);

}
// namespace planner
