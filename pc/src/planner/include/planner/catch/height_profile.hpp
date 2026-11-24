// 文件作用：接球高度域规划：提供梯形及五次高度轨迹，并换算成原定义的电机圈数和速度。 声明公共类型与接口。
#pragma once
#include "planner/catch/timing.hpp"

namespace planner
{

    // 高度域接球规划模式；运行时选择，不使用源代码的 T/FIVE1 宏切换。
    enum class CatchHeightMode
    {
        Trapezoidal,  // 源 T：梯形高度轨迹。
        Quintic       // 源 FIVE1：高度五次多项式轨迹。
    }
    ;

    // 接球高度域规划器；高度单位 mm，时间单位 s，输出电机圈数与圈数/s。
    class CatchHeightPlanner
    {
        public:
        const CatchHeightMode mode;  // 本实例固定的 T 或 FIVE1 模式。
        CatchTiming timing;  // 初始化后的分段时间，供调用方直接读取。
        // 以指定运行时模式构造并初始化一条完整轨迹。
        // 构造高度域规划器：selected_mode 指定梯形或五次模式；初始化对应轨迹，无返回值。
        explicit CatchHeightPlanner(CatchHeightMode selected_mode);

        // 按当前模式重算几何边界、三段时间和五次系数。
        // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
        void InitThetaProfile();
        // 查询 t 秒时的平台高度，单位 mm；对应源 get_profile_height。
        // 查询平台高度：t 为轨迹起点后的时间（s）；返回高度（mm），段外保持端点。
        double GetProfileHeight(double t) const;
        // 查询 t 秒时的平台线速度，单位 mm/s；对应源 get_plate_velocity。
        // 查询平台速度：t 为轨迹起点后的时间（s）；返回线速度（mm/s）。
        double GetPlateVelocity(double t) const;
        // 查询 t 秒时的电机目标圈数，沿用源 get_revs_by_h 的 theta/(PI/3) 定义。
        // 查询电机位置：t 为轨迹起点后的时间（s）；返回原 theta/(PI/3) 定义的圈数。
        double GetMotorRevsThetaPlan(double t) const;
        // 查询 t 秒时的电机目标速度，单位为源定义的圈数/s。
        // 查询电机速度：t 为轨迹起点后的时间（s）；返回原定义的圈数/s。
        double GetMotorVelAtTime(double t) const;
        // 求高度对机构角度 theta（rad）的雅可比绝对值，单位 mm/rad。
        // 计算高度雅可比：theta 为机构角度（rad）；返回高度对角度导数的绝对值（mm/rad）。
        static double GetJ(double theta);

        private:
        // T/FIVE1 共用的高度域轨迹参数；系数只在 Quintic 模式使用。
        struct MotionProfile
        {
            double a_acc = 0.0;          // T 加速段加速度，mm/s^2。
            double a_dec = 0.0;          // T 减速段减速度，mm/s^2。
            double h_start_const = 0.0;  // 匀速起始高度，mm。
            double h_end_const = 0.0;    // 匀速结束高度，mm。
            double acc_c3 = 0.0;         // FIVE1 加速段三次项系数，mm/s^3。
            double acc_c4 = 0.0;         // FIVE1 加速段四次项系数，mm/s^4。
            double acc_c5 = 0.0;         // FIVE1 加速段五次项系数，mm/s^5。
            double dec_c3 = 0.0;         // FIVE1 减速段三次项系数，mm/s^3。
            double dec_c4 = 0.0;         // FIVE1 减速段四次项系数，mm/s^4。
            double dec_c5 = 0.0;         // FIVE1 减速段五次项系数，mm/s^5。
        }
        ;

        MotionProfile profile_;  // 当前实例的高度轨迹参数，替代源全局 profile。
    }
    ;

}
// namespace planner
