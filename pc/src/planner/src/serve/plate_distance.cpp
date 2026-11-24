// 文件作用：发球盘路程规划，保存历史增益写入规则。 实现轨迹计算与状态更新。
#include "planner/serve/plate_distance.hpp"

#include <cmath>
#include <iterator>

namespace planner
{
    namespace
    {
        // 仅表示增益写入规则；未写字段沿用调用方传入值。
        // High0/High1 区分两个 WAY 的高速增益，Low/Low1 区分低速增益，Try 使用试验分段。
        enum class GainRule
        {
            High0, High1, Low, Low1, Try
        }
        ;
        // 盘路程模式预设：保存目标速度、加减速度和增益切换规则。
        struct DistancePreset
        {
            float target_velocity;  // 盘速，转/s。
            float acceleration, deceleration;  // 转/s²，保留原左右乘法顺序。
            // 该预设采用的历史增益写入规则。
            GainRule gains;
        }
        ;
        // 顺序与公开 ServeDistanceProfile 一致，所有历史模式集中于此。
        // 按公开模式枚举顺序排列的盘速、加减速度及增益规则预设。
        constexpr DistancePreset kDistancePresets[] =
        {
            {10.0f, 900.0f, 900.0f, GainRule::High0}, {11.2f, 200.0f, 200.0f, GainRule::Low},
            {30.0f, 1500.0f, 1500.0f, GainRule::Try}, {32.5f, 1500.0f, 1500.0f, GainRule::High1},
            {20.0f, 400.0f, 400.0f, GainRule::Low1}, {35.80f, 1500.0f, 1500.0f, GainRule::High1},
            {33.5f, 1500.0f, 1500.0f, GainRule::High1}, {58.0f, 1500.0f, 1500.0f, GainRule::High1},
            {11.2f, 200.0f, 200.0f, GainRule::Low}, {7.6f, 200.0f, 200.0f, GainRule::Low},
            {9.0f, 200.0f, 200.0f, GainRule::Low},
        }
        ;
    }
    // namespace

    // 按路程求盘目标：profile 选择历史模式，travel_distance_turns 为累计盘转数，previous_gains 为上次驱动增益；返回本次速度及增益，输入无效返回 nullopt。
    std::optional<ServeDistanceTarget> EvaluateServeDistance(
        ServeDistanceProfile profile, float travel_distance_turns,
        DistanceVelocityGains previous_gains)
    {
        // 当前模式对应的预设或状态槽索引，用于选择相应参数和独立状态。
        const auto index = static_cast<unsigned>(profile);
        if (index >= std::size(kDistancePresets) || !std::isfinite(travel_distance_turns) ||
            !std::isfinite(previous_gains.kp) || !std::isfinite(previous_gains.ki))
        {
            return std::nullopt;
        }
        // 当前模式对应的只读预设，决定速度、行程或增益规则。
        const auto& preset = kDistancePresets[index];
        // 当前预设的目标盘速，转/s。
        const float targetVelocity = preset.target_velocity;
        // 当前预设加速度幅值，转/s²。
        const float acceleration = preset.acceleration;
        // 当前预设减速度幅值，转/s²。
        const float deceleration = preset.deceleration;
        // 是否使用 Try 历史分段和增益规则。
        const bool isTry = preset.gains == GainRule::Try;
        // 是否为两种低速增益规则之一。
        const bool low = preset.gains == GainRule::Low || preset.gains == GainRule::Low1;
        // 本次返回的盘速度与增益；初值先保留调用者增益。
        ServeDistanceTarget target = {0.0f, previous_gains};
        // 返回目标中增益的可写引用；未写入的字段沿用调用者传值。
        auto& gains = target.gains;
        if (travel_distance_turns <= 0.0f) travel_distance_turns = 0.0f;
        if (travel_distance_turns <= (isTry ? 1.50f : 1.60f))
        {
            if (preset.gains == GainRule::Low1)
            {
                if (travel_distance_turns >= 1.2f) gains.kp = 2.5f;
            }
            else if (preset.gains == GainRule::Low)
            {
                gains.kp = 1.8f;
                gains.ki = 0.5f;
                if (travel_distance_turns >= 0.6f) gains.ki = 0.2f;
            }
            else if (isTry && travel_distance_turns > 1.0f)
            {
                gains.kp = 0.8f;
                gains.ki = 0.2f;
            }
            else
            {
                gains.kp = 1.8f;
                gains.ki = 0.4f;
                if (travel_distance_turns >= 1.2f && travel_distance_turns < 1.4f)
                {
                    gains.kp = (preset.gains == GainRule::High1 ? 1.6f : 10.0f);
                    gains.ki = (preset.gains == GainRule::High1 ? 0.4f : 1.0f);
                }
                if (travel_distance_turns >= 1.4f && travel_distance_turns <= 1.6f)
                {
                    gains.kp = (preset.gains == GainRule::High1 ? 0.9f : 0.8f);
                    gains.ki = 0.2f;
                }
            }
            // C 的 sqrt 接收 double；被开方量必须先完成原 float 连乘。
            // 由路程公式开方得到的盘速，转/s，随后按历史条件限幅。
            const double speed = std::sqrt(static_cast<double>(2 * travel_distance_turns * acceleration));
            if (speed < 0.3f) target.velocity_turns_per_second = 0.3f;
            else if (speed < targetVelocity) target.velocity_turns_per_second = speed;
            else target.velocity_turns_per_second =
                isTry && travel_distance_turns <= 1.0f ? (targetVelocity * 4 / 3) : targetVelocity;
        }
        else if (!isTry && travel_distance_turns <= 1.70f)
        {
            gains.kp = (low ? 1.8f : 1.65f);
            if (!low) gains.ki = 0.4f;
            target.velocity_turns_per_second = targetVelocity;
        }
        else if (travel_distance_turns <= 2.95f)
        {
            gains.kp = (low ? 2.5f : 2.0f);
            gains.ki = (preset.gains == GainRule::High1 ? 0.2f : 0.0f);
            // Try 从 >1.5 开始减速，仍减去 1.70；不修正历史分段。
            // 减速段速度平方，单位 (转/s)²；负值时不执行开方。
            const float radicand = targetVelocity * targetVelocity -
            2 * deceleration * (travel_distance_turns - 1.70f);
            if (radicand >= 0.0f)
            {
                // 由路程公式开方得到的盘速，转/s，随后按历史条件限幅。
                const double speed = std::sqrt(static_cast<double>(radicand));
                if (speed >= 0.0001f) target.velocity_turns_per_second = speed;
            }
        }
        else
        {
            if (low) gains.ki = 1.0f;
            else
            {
                gains.kp = 0.6f;
                gains.ki = 0.4f;
            }
        }
        return target;
    }

}
// namespace planner
