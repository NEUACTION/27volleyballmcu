// 文件作用：发球盘时间规划：按五个时间边界切换速度与驱动增益，每次查询后推进 5 ms。 实现轨迹计算与状态更新。
#include "planner/serve/plate_time.hpp"

namespace planner
{
    namespace
    {
        constexpr double kPeriodSeconds = 0.005;  // 每次 Step 推进5ms。
        constexpr double kTargetSpeed = 34.0;  // 盘目标速度，转/s。
        constexpr double kS1 = 0.7;  // 加速/减速段行程，转。
        constexpr double kS2 = 2.1;  // 减速开始的累计行程，转。
        constexpr double kS3 = 1.2;  // 第一恒速段末端累计行程，转。
        constexpr double kS4 = 1.8;  // 第二恒速段末端累计行程，转。

    }
    // namespace

    // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
    void PlateTimePlanner::Reset()
    {
        elapsed_seconds_ = 0.0;
        motion_ = Motion{};
    }

    // 计算时间域盘速：elapsed_seconds 为轨迹时间（s）；更新五个时间边界，返回盘速（转/s）。
    double PlateTimePlanner::Evaluate(double elapsed_seconds)
    {
        // 时间轨迹加速段加速度，转/s²。
        const double accel = (kTargetSpeed * kTargetSpeed - 0.0 * 0.0) / (2.0 * kS1);
        // 时间轨迹减速段带符号加速度，转/s²。
        const double decel = (0.0 * 0.0 - kTargetSpeed * kTargetSpeed) / (2.0 * kS1);
        // 由目标盘速与加速度求出的加速时长，s。
        const double accel_time = (kTargetSpeed - 0.0) / accel;
        // 由目标盘速与减速度求出的减速时长，s。
        const double decel_time = (0.0 - kTargetSpeed) / decel;
        // 第一恒速区间时长，s，用于第二个增益边界。
        const double t2_span = (kS3 - kS1) / kTargetSpeed;
        // 第二个增益区间的历史时长公式，s。
        const double t3_span = (kS4 - kS3) / (kTargetSpeed / 4.0 * 3.0);
        // 减速前最后一个恒速区间时长，s。
        const double t4_span = (kS2 - kS4) / kTargetSpeed;
        motion_.t1 = accel_time;
        motion_.t2 = t2_span + accel_time;
        motion_.t3 = t2_span + accel_time + t3_span;
        motion_.t4 = t2_span + accel_time + t3_span + t4_span;
        motion_.t5 = motion_.t4 + decel_time;

        if (elapsed_seconds <= motion_.t1) return accel * elapsed_seconds;
        if (elapsed_seconds <= motion_.t4) return kTargetSpeed;
        if (elapsed_seconds <= motion_.t5) return kTargetSpeed + decel * (elapsed_seconds - motion_.t4);
        return 0.0;
    }

    // 推进盘时间轨迹：无参数；先返回当前时间的速度及增益，再累计 5 ms。
    PlateTarget PlateTimePlanner::Step()
    {
        // 本周期盘目标，接收当前时间求得的盘速和分段驱动增益。
        PlateTarget target;
        target.velocity_turns_per_second = Evaluate(elapsed_seconds_);
        if (elapsed_seconds_ <= motion_.t2)
        {
            target.velocity_kp = 1800.0;
            target.velocity_ki = 400.0;
        }
        else if (elapsed_seconds_ <= motion_.t3)
        {
            target.velocity_kp = 5800.0;
            target.velocity_ki = 4500.0;
        }
        else if (elapsed_seconds_ <= motion_.t4)
        {
            target.velocity_kp = 3500.0;
            target.velocity_ki = 3000.0;
        }
        else
        {
            target.velocity_kp = 1800.0;
            target.velocity_ki = 400.0;
        }
        elapsed_seconds_ += kPeriodSeconds;
        return target;
    }

}
// namespace planner
