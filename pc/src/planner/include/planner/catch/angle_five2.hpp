// 文件作用：接球角度域 FIVE2 规划：由机构几何确定分段边界，以五次角度轨迹衔接高度匀速段。 声明公共类型与接口。
#pragma once
#include "planner/catch/timing.hpp"

namespace planner
{

    // 接球 FIVE2 规划：高度为 mm，时间为 s，角度为 rad；不依赖 MCU 或其它轨迹模式。
    // 迁移 zlr_math.c:14–222、296–408 的几何和 FIVE2 分支。
    class CatchFive2Planner
    {
        public:
        CatchTiming timing;  // 初始化后的分段时间，供调用方直接读取。
        // 构造时初始化固定参数的轨迹，可立即按时间查询。
        // 构造角度域规划器：无参数；初始化几何轨迹及分段时间，无返回值。
        CatchFive2Planner();

        // 重算匀速边界、分段时长和角度五次系数，对应 init_theta_profile。
        // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
        void InitThetaProfile();
        // 查询 t 秒时的目标位置，保留原“电机圈数”定义 theta/(PI/3)。
        // 查询电机位置：t 为轨迹起点后的时间（s）；返回原 theta/(PI/3) 定义的圈数。
        double GetMotorRevsThetaPlan(double t) const;
        // 查询 t 秒时的目标速度，单位为上述原始圈数/s。
        // 查询电机速度：t 为轨迹起点后的时间（s）；返回原定义的圈数/s。
        double GetMotorVelAtTime(double t) const;
        // 求高度对 theta（rad）的雅可比绝对值，单位 mm/rad。
        // 计算高度雅可比：theta 为机构角度（rad）；返回高度对角度导数的绝对值（mm/rad）。
        static double GetJ(double theta);

        private:
        // FIVE2 分段参数；系数 cN 的单位为 rad/s^N。
        struct ThetaProfile
        {
            double h_start_const = 0.0;  // 匀速起点高度，mm。
            double acc_c0 = 0.0;         // 加速段角度常数项，rad。
            double acc_c1 = 0.0;         // 加速段一次项，rad/s。
            double acc_c2 = 0.0;         // 加速段二次项，rad/s^2。
            double acc_c3 = 0.0;         // 加速段三次项，rad/s^3。
            double acc_c4 = 0.0;         // 加速段四次项，rad/s^4。
            double acc_c5 = 0.0;         // 加速段五次项，rad/s^5。
            double dec_c0 = 0.0;         // 减速段角度常数项，rad。
            double dec_c1 = 0.0;         // 减速段一次项，rad/s。
            double dec_c2 = 0.0;         // 减速段二次项，rad/s^2。
            double dec_c3 = 0.0;         // 减速段三次项，rad/s^3。
            double dec_c4 = 0.0;         // 减速段四次项，rad/s^4。
            double dec_c5 = 0.0;         // 减速段五次项，rad/s^5。
        }
        ;

        ThetaProfile tp_;  // 当前实例的固定几何轨迹，替代原全局 tp。
    }
    ;

}
// namespace planner
