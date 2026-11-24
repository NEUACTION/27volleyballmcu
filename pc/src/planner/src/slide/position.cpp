/* 文件作用：实现历史丝杆预设位置累加，以及调用者显式配置的正向定步长位置接口。
 * 历史五模式共用参数表并按5ms累加；ScrewPositionPlanner使用调用者指定的单位和步长，不自行假定周期。
 * 本文件只返回当前目标，不操作硬件；Reset仅重置软件状态，不是电机回零。
 */
#include "planner/slide/position.hpp"
#include <iterator>
#include <cmath>

namespace planner
{
    namespace
    {
        // 各模式目标位置（圈）和每5ms增量（圈）；到达或超过目标时输出均不再增长。
        constexpr HistoricalSlidePreset kSlidePresets[] =
        {
            {241.16f, 35.0f / 200.0f}, {11.42f, 40.0f / 200.0f}, {241.16f, 35.0f / 200.0f},
            {711.42f, 40.0f / 200.0f}, {241.17f, 35.0f / 200.0f},
        }
        ;
    }
    /* 中文名：配置丝杆正向步进。
     * 作用：config给出同一位置单位下的目标与每次正向增量，initial_position为软件初始位置；
     * 返回配置是否有效。每次调用先清除旧配置，非法输入使Step返回空，不读取或复位电驱。
     */
    bool ScrewPositionPlanner::Configure(ScrewPositionConfig config, float initial_position)
    {
        config_.reset();
        if (!std::isfinite(config.target_position) || !std::isfinite(config.increment_per_call) ||
            !std::isfinite(initial_position) || config.increment_per_call <= 0.0f ||
            initial_position > config.target_position)
        {
            return false;
        }
        config_ = config;
        current_position_ = initial_position;
        return true;
    }

    /* 中文名：计算配置丝杆的下一步位置。
     * 作用：按已配置步长推进一次并把内部位置限制在目标，返回同配置单位的位置；
     * 未配置返回nullopt。本接口不计时、不发送指令，也不使用历史五模式的参数表。
     */
    std::optional<float> ScrewPositionPlanner::Step()
    {
        if (!config_) return std::nullopt;
        if (config_->target_position - current_position_ <= config_->increment_per_call)
        {
            current_position_ = config_->target_position;
        }
        else
        {
            current_position_ += config_->increment_per_call;
        }
        return current_position_;
    }

    /* 中文名：查询历史丝杆参数。
     * 作用：以profile的枚举序号查询预设表；返回目标和每周期增量，
     * 非法枚举返回nullopt，不替调用方选择其它模式。
     */
    std::optional<HistoricalSlidePreset> GetHistoricalSlidePreset(HistoricalSlideProfile profile)
    {
        const auto index = static_cast<unsigned>(profile);  // 模式对应的参数表下标，不是物理电机ID。
        if (index >= std::size(kSlidePresets)) return std::nullopt;
        return kSlidePresets[index];
    }

    /* 中文名：创建指定模式的丝杆规划器。
     * 作用：保存profile（所选历史模式）；current_position_由头文件成员初值设为0圈，
     * 创建对象只建立软件规划状态，不操作电驱或开始机械运动。
     */
    HistoricalSlidePlanner::HistoricalSlidePlanner(HistoricalSlideProfile profile)
        : profile_(profile)
    {
    }

    /* 中文名：重置丝杆软件累计位置。
     * 作用：让已经使用的实例从软件0圈重新规划，不发电机复位命令；无参数和返回值。
     */
    void HistoricalSlidePlanner::Reset()
    {
        current_position_ = 0.0f;  // 每个原函数静态累计位置的实际初值。
    }

    /* 中文名：计算丝杆本周期位置。
     * 作用：先给累计量加一次模式增量，超过目标时返回目标，否则返回累计位置。
     * 只计算本次调用的目标（圈），累计量超目标后仍增长；非法模式返回nullopt。
     */
    std::optional<float> HistoricalSlidePlanner::Step()
    {
        const auto preset = GetHistoricalSlidePreset(profile_);  // 当前模式参数，可为空；不缓存未来轨迹。
        if (!preset) return std::nullopt;
        // 每5ms现算一次；只夹紧输出，原内部累计量超过目标后仍增长。
        current_position_ = current_position_ + preset->increment_per_call_turns;
        if (current_position_ > preset->target_position_turns)
        {
            return preset->target_position_turns;
        }
        return current_position_;
    }
}
// namespace planner
