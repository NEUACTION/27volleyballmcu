// 文件作用：发球盘时间规划：按五个时间边界切换速度与驱动增益，每次查询后推进 5 ms。 声明公共类型与接口。
#pragma once

namespace planner
{

    // 历史盘时间轨迹的五个分段边界，单位 s。
    // 盘时间轨迹边界：t1 加速结束，t2/t3 为增益分界，t4 减速开始，t5 轨迹结束，均为 s。
    struct Motion
    {
        // 加速段结束时刻，s。
        double t1 = 0.0;
        // 第一组增益结束时刻，s。
        double t2 = 0.0;
        // 第二组增益结束时刻，s。
        double t3 = 0.0;
        // 恒速结束、减速开始时刻，s。
        double t4 = 0.0;
        // 减速结束、速度回零时刻，s。
        double t5 = 0.0;
    }
    ;

    // 单次 5 ms 周期使用的盘速度与驱动增益。
    // 单周期盘目标：速度为转/s，增益使用驱动配置数值。
    struct PlateTarget
    {
        // 本周期目标盘速，转/s。
        double velocity_turns_per_second = 0.0;
        // 本周期驱动速度环比例增益。
        double velocity_kp = 0.0;
        // 本周期驱动速度环积分增益。
        double velocity_ki = 0.0;
    }
    ;

    // 盘时间规划器：按历史速度时间分段公式逐周期求值。
    class PlateTimePlanner
    {
        public:
        // 选择该模式时重置，从 t=0 重新开始。
        // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
        void Reset();
        // 返回当前周期速度和增益，随后推进一个参考周期。
        // 推进盘时间轨迹：无参数；先返回当前时间的速度及增益，再累计 5 ms。
        PlateTarget Step();

        private:
        // 按累计秒数求盘速，同时刷新分段时间边界。
        // 计算时间域盘速：elapsed_seconds 为轨迹时间（s）；更新五个时间边界，返回盘速（转/s）。
        double Evaluate(double elapsed_seconds);
        // 下一次 Step 求值使用的累计轨迹时间，s。
        double elapsed_seconds_ = 0.0;
        // 最近一次求值更新的五个时间边界，供速度和增益分段使用。
        Motion motion_;
    }
    ;

}
// namespace planner
