// 文件作用：接球角度域 FIVE2 规划：由机构几何确定分段边界，以五次角度轨迹衔接高度匀速段。 实现轨迹计算与状态更新。
#include "planner/catch/angle_five2.hpp"

#include <cmath>

namespace planner
{
    namespace
    {
        // 原 zlr_math.c:9–23 的数值与单位原样保留。
        constexpr double PI = 3.14159265358979323846;  // 圆周率，用于几何角度换算。
        constexpr double PI_3 = PI / 3.0;             // 原电机一圈对应的机构角度，rad；不替换为 2PI。
        constexpr double LINK_LENGTH = 210.0;        // 连杆长度，mm。
        constexpr double OFFSET_RADIUS = 20.0;       // 几何偏置半径，mm。
        constexpr double HEIGHT_OFFSET = 59.46128;   // 高度零点偏移，mm。
        constexpr double TOTAL_H = 277.13;           // 轨迹终点高度，mm。
        constexpr double V_CONST = 1000.0;           // 匀速段平台速度，mm/s。

        // 由机构角度 theta（rad）正解平台高度（mm），保留源几何常数写法。
        // 角度转高度：theta 为机构角度（rad）；返回平台高度（mm）。
        double GetHeightByTheta(double theta)
        {
            double phi = 5.0 * PI / 9.0 - theta;  // 几何闭环中的连杆夹角，rad。
            double root = std::sqrt(44100.0 - std::pow(210.0 * std::sin(phi) - 20.0, 2.0));  // 闭环根项，mm。
            return 210.0 * std::cos(phi) + root - HEIGHT_OFFSET;
        }

        // 由高度 h（mm）反解原电机圈数；仅对 acos 输入按源代码限幅。
        // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
        double GetRevsByHeight(double h)
        {
            double d = h + HEIGHT_OFFSET;  // 加入零点偏移后的高度，mm。
            double root = std::sqrt(d * d + OFFSET_RADIUS * OFFSET_RADIUS);  // 逆解斜边长度，mm。
            double acos_arg = root / (2.0 * LINK_LENGTH);  // 反余弦的无量纲输入。
            if (acos_arg > 1.0)
            {
                acos_arg = 1.0;
            }
            if (acos_arg < -1.0)
            {
                acos_arg = -1.0;
            }
            double phi = std::atan2(OFFSET_RADIUS, d) + std::acos(acos_arg);  // 逆解连杆夹角，rad。
            double theta = 5.0 * PI / 9.0 - phi;  // 机构角度，rad。
            return theta / PI_3;
        }

        // 扫描 theta∈[0,PI/3]，将最大雅可比的 90% 对应两端高度写入输出指针（mm）。
        // 寻找匀速区高度：h_start、h_end 为起止高度输出指针（mm）；无返回值。
        void FindConstHeights(double* h_start, double* h_end)
        {
            const double step = 0.0001;  // 角度扫描步长，rad。
            double max_j = 0.0;  // 扫描得到的最大雅可比，mm/rad。
            double theta;  // 当前扫描角度，rad。
            for (theta = 0.0; theta <= PI_3 + 1e-6; theta += step)
            {
                double j = CatchFive2Planner::GetJ(theta);  // 当前雅可比，mm/rad。
                if (j > max_j)
                {
                    max_j = j;
                }
            }
            double threshold = 0.9 * max_j;  // 匀速区的雅可比阈值，mm/rad。
            double theta_start = 0.0;  // 匀速起点机构角度，rad。
            double theta_end = PI_3;  // 匀速终点机构角度，rad。
            for (theta = 0.0; theta <= PI_3; theta += step)
            {
                if (CatchFive2Planner::GetJ(theta) >= threshold)
                {
                    theta_start = theta;
                    break;
                }
            }
            for (theta = PI_3; theta >= 0.0; theta -= step)
            {
                if (CatchFive2Planner::GetJ(theta) >= threshold)
                {
                    theta_end = theta;
                    break;
                }
            }
            *h_start = GetHeightByTheta(theta_start);
            *h_end = GetHeightByTheta(theta_end);
        }

        // 对输入角度 theta（rad）中心差分，估算雅可比导数（mm/rad^2）。
        // 计算雅可比导数：theta 为机构角度（rad）；中心差分返回 mm/rad²。
        double GetJPrime(double theta)
        {
            const double step = 1e-6;  // 原中心差分步长，rad。
            return (CatchFive2Planner::GetJ(theta + step) - CatchFive2Planner::GetJ(theta - step)) / (2.0 * step);
        }

        // 根据角位移 d_theta（rad）、末速度 v1（rad/s）、末加速度 a1（rad/s^2）和时长 duration（s），
        // 求零初始状态的五次高阶系数，输出 c3/c4/c5 分别为 rad/s^3、rad/s^4、rad/s^5。
        // 求五次高阶系数：d_theta 为角位移（rad），v1、a1 为末端角速度与角加速度，duration 为时长（s）；c3/c4/c5 输出对应高阶系数，无返回值。
        void Solve5thCoeffs(double d_theta, double v1, double a1, double duration,
            double* c3, double* c4, double* c5)
        {
            double x = 10.0 * d_theta - 4.0 * v1 * duration
            + 0.5 * a1 * duration * duration;  // 三次系数的归一化分子，rad。
            double y = -15.0 * d_theta + 7.0 * v1 * duration
            - a1 * duration * duration;  // 四次系数的归一化分子，rad。
            double z = 6.0 * d_theta - 3.0 * v1 * duration
            + 0.5 * a1 * duration * duration;  // 五次系数的归一化分子，rad。
            *c3 = x / std::pow(duration, 3.0);
            *c4 = y / std::pow(duration, 4.0);
            *c5 = z / std::pow(duration, 5.0);
        }
    }
    // namespace

    // 构造实例并建立原 FIVE2 轨迹，使查询无需依赖外部全局初始化顺序。
    // 构造角度域规划器：无参数；初始化几何轨迹及分段时间，无返回值。
    CatchFive2Planner::CatchFive2Planner()
    {
        // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
        InitThetaProfile();
    }

    // 原 get_J：theta 为机构角度（rad），返回高度雅可比的绝对值（mm/rad）。
    // 计算高度雅可比：theta 为机构角度（rad）；返回高度对角度导数的绝对值（mm/rad）。
    double CatchFive2Planner::GetJ(double theta)
    {
        double phi = 5.0 * PI / 9.0 - theta;  // 连杆夹角，rad。
        double sin_phi = std::sin(phi);  // 连杆夹角正弦。
        double cos_phi = std::cos(phi);  // 连杆夹角余弦。
        double term = 210.0 * sin_phi - 20.0;  // 闭环水平分量，mm。
        double root = std::sqrt(44100.0 - term * term);  // 闭环竖直根项，mm。
        double dh_dphi = -210.0 * sin_phi - (term * 210.0 * cos_phi) / root;  // 高度对 phi 的导数，mm/rad。
        return std::fabs(dh_dphi);
    }

    // 原 init_theta_profile 的 FIVE2 分支；保留时长与端点速度、加速度公式。
    // 初始化轨迹：无参数；更新本实例的边界、时间及系数，无返回值。
    void CatchFive2Planner::InitThetaProfile()
    {
        double h_start;  // 匀速起点高度，mm。
        double h_end;  // 匀速终点高度，mm。
        // 寻找匀速区高度：h_start、h_end 为起止高度输出指针（mm）；无返回值。
        FindConstHeights(&h_start, &h_end);
        tp_.h_start_const = h_start;
        timing.acceleration_seconds = 2.0 * h_start / V_CONST;
        timing.constant_seconds = (h_end - h_start) / V_CONST;
        timing.deceleration_seconds = 2.0 * (TOTAL_H - h_end) / V_CONST;
        timing.total_seconds = timing.acceleration_seconds + timing.constant_seconds + timing.deceleration_seconds;

        double theta0 = GetRevsByHeight(0.0) * PI_3;  // 起点角度，rad。
        double theta1 = GetRevsByHeight(h_start) * PI_3;  // 匀速起点角度，rad。
        double theta2 = GetRevsByHeight(h_end) * PI_3;  // 匀速终点角度，rad。
        double theta3 = GetRevsByHeight(TOTAL_H) * PI_3;  // 轨迹终点角度，rad。
        double t_acc = timing.acceleration_seconds;  // 加速段时长，s。
        double j1 = GetJ(theta1);  // 匀速起点雅可比，mm/rad。
        double j1_prime = GetJPrime(theta1);  // 匀速起点雅可比导数，mm/rad^2。
        double v1 = V_CONST / j1;  // 加速段末角速度，rad/s。
        double a1 = -V_CONST * V_CONST * j1_prime / (j1 * j1 * j1);  // 加速段末角加速度，rad/s^2。
        double d_theta = theta1 - theta0;  // 待求高阶项所承担的角位移，rad。
        tp_.acc_c0 = theta0;
        tp_.acc_c1 = 0.0;
        tp_.acc_c2 = 0.0;
        // 求五次高阶系数：d_theta 为角位移（rad），v1、a1 为末端角速度与角加速度，duration 为时长（s）；c3/c4/c5 输出对应高阶系数，无返回值。
        Solve5thCoeffs(d_theta, v1, a1, t_acc, &tp_.acc_c3, &tp_.acc_c4, &tp_.acc_c5);

        double t_dec = timing.deceleration_seconds;  // 减速段时长，s。
        double j2 = GetJ(theta2);  // 匀速终点雅可比，mm/rad。
        double j2_prime = GetJPrime(theta2);  // 匀速终点雅可比导数，mm/rad^2。
        double v2 = V_CONST / j2;  // 减速段初角速度，rad/s。
        double a2 = -V_CONST * V_CONST * j2_prime / (j2 * j2 * j2);  // 减速段初角加速度，rad/s^2。
        tp_.dec_c0 = theta2;
        tp_.dec_c1 = v2;
        tp_.dec_c2 = 0.5 * a2;
        d_theta = theta3 - (theta2 + v2 * t_dec + 0.5 * a2 * t_dec * t_dec);
        double v1_prime = -(v2 + a2 * t_dec);  // 扣除低阶项后的末速度约束，rad/s。
        double a1_prime = -a2;  // 扣除低阶项后的末加速度约束，rad/s^2。
        // 求五次高阶系数：d_theta 为角位移（rad），v1、a1 为末端角速度与角加速度，duration 为时长（s）；c3/c4/c5 输出对应高阶系数，无返回值。
        Solve5thCoeffs(d_theta, v1_prime, a1_prime, t_dec, &tp_.dec_c3, &tp_.dec_c4, &tp_.dec_c5);
    }

    // 原位置查询 FIVE2 分支；t 为起始后的秒数，段外保持起点或终点。
    // 查询电机位置：t 为轨迹起点后的时间（s）；返回原 theta/(PI/3) 定义的圈数。
    double CatchFive2Planner::GetMotorRevsThetaPlan(double t) const
    {
        if (t <= 0.0)
        {
            // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
            return GetRevsByHeight(0.0);
        }
        double theta_rad;  // 加速或减速段角度多项式的结果，rad。
        if (t <= timing.acceleration_seconds)
        {
            double tau = t;  // 加速段局部时间，s。
            theta_rad = tp_.acc_c0
            + tp_.acc_c1 * tau
            + tp_.acc_c2 * tau * tau
            + tp_.acc_c3 * tau * tau * tau
            + tp_.acc_c4 * std::pow(tau, 4.0)
            + tp_.acc_c5 * std::pow(tau, 5.0);
        }
        else if (t <= timing.acceleration_seconds + timing.constant_seconds)
        {
            double h = tp_.h_start_const + V_CONST * (t - timing.acceleration_seconds);  // 匀速段高度，mm。
            if (h > TOTAL_H)
            {
                h = TOTAL_H;
            }
            // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
            return GetRevsByHeight(h);
        }
        else if (t <= timing.total_seconds)
        {
            double tau = t - timing.acceleration_seconds - timing.constant_seconds;  // 减速段局部时间，s。
            theta_rad = tp_.dec_c0
            + tp_.dec_c1 * tau
            + tp_.dec_c2 * tau * tau
            + tp_.dec_c3 * tau * tau * tau
            + tp_.dec_c4 * std::pow(tau, 4.0)
            + tp_.dec_c5 * std::pow(tau, 5.0);
        }
        else
        {
            // 高度转圈数：h 为平台高度（mm）；返回原 theta/(PI/3) 定义的圈数。
            return GetRevsByHeight(TOTAL_H);
        }
        return theta_rad / PI_3;
    }

    // 原速度查询 FIVE2 分支；t 为秒，保持原 t==总时长仍求多项式的边界条件。
    // 查询电机速度：t 为轨迹起点后的时间（s）；返回原定义的圈数/s。
    double CatchFive2Planner::GetMotorVelAtTime(double t) const
    {
        if ((t <= 0.0) || (t > timing.total_seconds))
        {
            return 0.0;
        }
        if (t <= timing.acceleration_seconds)
        {
            double tau = t;  // 加速段局部时间，s。
            return (tp_.acc_c1
                + 2.0 * tp_.acc_c2 * tau
                + 3.0 * tp_.acc_c3 * tau * tau
                + 4.0 * tp_.acc_c4 * std::pow(tau, 3.0)
                + 5.0 * tp_.acc_c5 * std::pow(tau, 4.0)) / PI_3;
        }
        if (t <= timing.acceleration_seconds + timing.constant_seconds)
        {
            double h = tp_.h_start_const + V_CONST * (t - timing.acceleration_seconds);  // 匀速段高度，mm。
            double theta = GetRevsByHeight(h) * PI_3;  // 匀速段机构角度，rad。
            return V_CONST / (GetJ(theta) * PI_3);
        }
        double tau = t - timing.acceleration_seconds - timing.constant_seconds;  // 减速段局部时间，s。
        return (tp_.dec_c1
            + 2.0 * tp_.dec_c2 * tau
            + 3.0 * tp_.dec_c3 * tau * tau
            + 4.0 * tp_.dec_c4 * std::pow(tau, 3.0)
            + 5.0 * tp_.dec_c5 * std::pow(tau, 4.0)) / PI_3;
    }

}
// namespace planner
