// 文件作用：接球高度域规划：提供梯形及五次高度轨迹，并换算成原定义的电机圈数和速度。 实现轨迹计算与状态更新。
#include "planner/catch/height_profile.hpp"

#include <cmath>

namespace planner
{
    namespace
    {
        // 以下几何、扫描和限幅保留 zlr_math.c:9–113 原值，未引用或改动 FIVE2 模块。
        constexpr double PI = 3.14159265358979323846;  // 圆周率，用于弧度换算。
        constexpr double PI_3 = PI / 3.0;             // 原电机圈数的机构角度比例，rad/圈。
        constexpr double LINK_LENGTH = 210.0;        // 连杆长度，mm。
        constexpr double OFFSET_RADIUS = 20.0;       // 几何偏置半径，mm。
        constexpr double HEIGHT_OFFSET = 59.46128;   // 高度零点偏移，mm。
        constexpr double TOTAL_H = 277.13;           // 轨迹终点高度，mm。
        constexpr double V_CONST = 1000.0;           // 匀速段平台速度，mm/s。

        // 由机构角度 theta（rad）正解平台高度，单位 mm。
        // 角度转高度：theta 为机构角度（rad）；返回平台高度（mm）。
        double GetHeightByTheta(double theta)
        {
            double phi = 5.0 * PI / 9.0 - theta;  // 连杆夹角，rad。
            double root = std::sqrt(44100.0 - std::pow(210.0 * std::sin(phi) - 20.0, 2.0));  // 闭环根项，mm。
            return 210.0 * std::cos(phi) + root - HEIGHT_OFFSET;
        }

        // 由高度 h（mm）反解原电机圈数，保留源 acos 输入限幅。
        // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
        double GetRevsByHeight(double h)
        {
            double d = h + HEIGHT_OFFSET;  // 零点偏移后的高度，mm。
            double root = std::sqrt(d * d + OFFSET_RADIUS * OFFSET_RADIUS);  // 逆解斜边长度，mm。
            double acos_arg = root / (2.0 * LINK_LENGTH);  // 反余弦无量纲输入。
            if (acos_arg > 1.0)
            {
                acos_arg = 1.0;
            }
            if (acos_arg < -1.0)
            {
                acos_arg = -1.0;
            }
            double phi = std::atan2(OFFSET_RADIUS, d) + std::acos(acos_arg);  // 逆解夹角，rad。
            double theta = 5.0 * PI / 9.0 - phi;  // 机构角度，rad。
            return theta / PI_3;
        }

        // 扫描雅可比最大值的 90% 区域，将起止高度写入 h_start/h_end（mm）。
        // 寻找匀速区高度：h_start、h_end 为起止高度输出指针（mm）；无返回值。
        void FindConstHeights(double* h_start, double* h_end)
        {
            const double step = 0.0001;  // 角度扫描步长，rad。
            double max_j = 0.0;  // 扫描最大雅可比，mm/rad。
            double theta;  // 当前扫描角度，rad。
            for (theta = 0.0; theta <= PI_3 + 1e-6; theta += step)
            {
                double j = CatchHeightPlanner::GetJ(theta);  // 当前雅可比，mm/rad。
                if (j > max_j)
                {
                    max_j = j;
                }
            }
            double threshold = 0.9 * max_j;  // 匀速边界阈值，mm/rad。
            double theta_start = 0.0;  // 匀速起点角度，rad。
            double theta_end = PI_3;  // 匀速终点角度，rad。
            for (theta = 0.0; theta <= PI_3; theta += step)
            {
                if (CatchHeightPlanner::GetJ(theta) >= threshold)
                {
                    theta_start = theta;
                    break;
                }
            }
            for (theta = PI_3; theta >= 0.0; theta -= step)
            {
                if (CatchHeightPlanner::GetJ(theta) >= threshold)
                {
                    theta_end = theta;
                    break;
                }
            }
            *h_start = GetHeightByTheta(theta_start);
            *h_end = GetHeightByTheta(theta_end);
        }
    }
    // namespace

    // 记录调用方显式选择的 mode，并执行对应源初始化公式。
    // 构造高度域规划器：selected_mode 指定梯形或五次模式；初始化对应轨迹，无返回值。
    CatchHeightPlanner::CatchHeightPlanner(CatchHeightMode selected_mode) : mode(selected_mode)
    {
        // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
        InitThetaProfile();
    }

    // 原 get_J：保留几何导数和绝对值，theta 单位 rad，结果 mm/rad。
    // 计算高度雅可比：theta 为机构角度（rad）；返回高度对角度导数的绝对值（mm/rad）。
    double CatchHeightPlanner::GetJ(double theta)
    {
        double phi = 5.0 * PI / 9.0 - theta;  // 连杆夹角，rad。
        double sin_phi = std::sin(phi);  // 夹角正弦。
        double cos_phi = std::cos(phi);  // 夹角余弦。
        double term = 210.0 * sin_phi - 20.0;  // 闭环水平分量，mm。
        double root = std::sqrt(44100.0 - term * term);  // 闭环竖直根项，mm。
        double dh_dphi = -210.0 * sin_phi - (term * 210.0 * cos_phi) / root;  // 高度对 phi 的导数，mm/rad。
        return std::fabs(dh_dphi);
    }

    // 源 140–178 行的 T/FIVE1 初始化；仅将 FIVE1 条件编译改为运行时判断。
    // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
    void CatchHeightPlanner::InitThetaProfile()
    {
        double h_start;  // 匀速起点高度，mm。
        double h_end;  // 匀速终点高度，mm。
        // 寻找匀速区高度：h_start、h_end 为起止高度输出指针（mm）；无返回值。
        FindConstHeights(&h_start, &h_end);
        double s_acc = h_start;  // 加速段位移，mm。
        double s_const = h_end - h_start;  // 匀速段位移，mm。
        double s_dec = TOTAL_H - h_end;  // 减速段位移，mm。
        profile_.h_start_const = h_start;
        profile_.h_end_const = h_end;
        timing.acceleration_seconds = 2.0 * s_acc / V_CONST;
        timing.constant_seconds = s_const / V_CONST;
        timing.deceleration_seconds = 2.0 * s_dec / V_CONST;
        timing.total_seconds = timing.acceleration_seconds + timing.constant_seconds + timing.deceleration_seconds;
        profile_.a_acc = V_CONST / timing.acceleration_seconds;
        profile_.a_dec = V_CONST / timing.deceleration_seconds;
        if (mode == CatchHeightMode::Quintic)
        {
            double t_acc = timing.acceleration_seconds;  // FIVE1 加速段时长，s。
            profile_.acc_c3 = 10.0 * s_acc / std::pow(t_acc, 3.0)
            - 4.0 * V_CONST / std::pow(t_acc, 2.0);
            profile_.acc_c4 = -15.0 * s_acc / std::pow(t_acc, 4.0)
            + 7.0 * V_CONST / std::pow(t_acc, 3.0);
            profile_.acc_c5 = 6.0 * s_acc / std::pow(t_acc, 5.0)
            - 3.0 * V_CONST / std::pow(t_acc, 4.0);
            double t_dec = timing.deceleration_seconds;  // FIVE1 减速段时长，s。
            profile_.dec_c3 = 10.0 * s_dec / std::pow(t_dec, 3.0)
            - 6.0 * V_CONST / std::pow(t_dec, 2.0);
            profile_.dec_c4 = -15.0 * s_dec / std::pow(t_dec, 4.0)
            + 8.0 * V_CONST / std::pow(t_dec, 3.0);
            profile_.dec_c5 = 6.0 * s_dec / std::pow(t_dec, 5.0)
            - 3.0 * V_CONST / std::pow(t_dec, 4.0);
        }
    }

    // 源 226–260 行；t 为秒，保留 t<=0、分段右端包含及段外终点保持。
    // 查询平台高度：t 为轨迹起点后的时间（s）；返回高度（mm），段外保持端点。
    double CatchHeightPlanner::GetProfileHeight(double t) const
    {
        if (t <= 0.0)
        {
            return 0.0;
        }
        if (t <= timing.acceleration_seconds)
        {
            if (mode == CatchHeightMode::Trapezoidal)
            {
                return 0.5 * profile_.a_acc * t * t;
            }
            else
            {
                return profile_.acc_c3 * std::pow(t, 3.0)
                + profile_.acc_c4 * std::pow(t, 4.0)
                + profile_.acc_c5 * std::pow(t, 5.0);
            }
        }
        if (t <= timing.acceleration_seconds + timing.constant_seconds)
        {
            return profile_.h_start_const + V_CONST * (t - timing.acceleration_seconds);
        }
        if (t <= timing.acceleration_seconds + timing.constant_seconds + timing.deceleration_seconds)
        {
            double tau = t - timing.acceleration_seconds - timing.constant_seconds;  // 减速段局部时间，s。
            if (mode == CatchHeightMode::Trapezoidal)
            {
                return profile_.h_end_const + V_CONST * tau
                - 0.5 * profile_.a_dec * tau * tau;
            }
            else
            {
                return profile_.h_end_const + V_CONST * tau
                + profile_.dec_c3 * std::pow(tau, 3.0)
                + profile_.dec_c4 * std::pow(tau, 4.0)
                + profile_.dec_c5 * std::pow(tau, 5.0);
            }
        }
        return TOTAL_H;
    }

    // 源 263–293 行；t 為秒，保留 t<0 而非 t<=0 的源速度边界。
    // 查询平台速度：t 为轨迹起点后的时间（s）；返回线速度（mm/s）。
    double CatchHeightPlanner::GetPlateVelocity(double t) const
    {
        if ((t < 0.0) || (t > timing.acceleration_seconds + timing.constant_seconds + timing.deceleration_seconds))
        {
            return 0.0;
        }
        if (t <= timing.acceleration_seconds)
        {
            if (mode == CatchHeightMode::Trapezoidal)
            {
                return profile_.a_acc * t;
            }
            else
            {
                return 3.0 * profile_.acc_c3 * t * t
                + 4.0 * profile_.acc_c4 * t * t * t
                + 5.0 * profile_.acc_c5 * t * t * t * t;
            }
        }
        if (t <= timing.acceleration_seconds + timing.constant_seconds)
        {
            return V_CONST;
        }
        double tau = t - timing.acceleration_seconds - timing.constant_seconds;  // 减速段局部时间，s。
        if (mode == CatchHeightMode::Trapezoidal)
        {
            return V_CONST - profile_.a_dec * tau;
        }
        else
        {
            return V_CONST
            + 3.0 * profile_.dec_c3 * tau * tau
            + 4.0 * profile_.dec_c4 * tau * tau * tau
            + 5.0 * profile_.dec_c5 * tau * tau * tau * tau;
        }
    }

    // 源 299–307 行；时间 t 为秒，高度限幅到 [0,TOTAL_H] 后反解原电机圈数。
    // 查询电机位置：t 为轨迹起点后的时间（s）；返回原 theta/(PI/3) 定义的圈数。
    double CatchHeightPlanner::GetMotorRevsThetaPlan(double t) const
    {
        double h = GetProfileHeight(t);  // 高度轨迹值，mm。
        if (h < 0.0)
        {
            h = 0.0;
        }
        else if (h > TOTAL_H)
        {
            h = TOTAL_H;
        }
        // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
        return GetRevsByHeight(h);
    }

    // 源 356–377 行；时间 t 为秒，保留非正平台速度返回零和 J>=1e-6 限制。
    // 查询电机速度：t 为轨迹起点后的时间（s）；返回原定义的圈数/s。
    double CatchHeightPlanner::GetMotorVelAtTime(double t) const
    {
        double h;  // 限幅后高度，mm。
        double plate_velocity = GetPlateVelocity(t);  // 当前平台速度，mm/s。
        if (plate_velocity <= 0.0)
        {
            return 0.0;
        }
        h = GetProfileHeight(t);
        if (h < 0.0)
        {
            h = 0.0;
        }
        else if (h > TOTAL_H)
        {
            h = TOTAL_H;
        }
        double theta = GetRevsByHeight(h) * PI_3;  // 当前机构角度，rad。
        double j = GetJ(theta);  // 当前雅可比，mm/rad。
        if (j < 1e-6)
        {
            j = 1e-6;
        }
        return plate_velocity / (j * PI_3);
    }

}
// namespace planner
