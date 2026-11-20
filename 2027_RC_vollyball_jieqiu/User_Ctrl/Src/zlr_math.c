/*数学计算模块
*目前分三种模式，梯形规划，五次多项式1,2
*有π的宏定义
*/

#include <math.h>
#include "zlr_math.h"

/* 宏：圆周率常量，供连杆几何和角度换算使用；当前计算路径启用。 */
#define PI       3.14159265358979323846
/* 宏：六十度对应的弧度值，用于电机圈数与角度互换；当前计算路径启用。 */
#define PI_3     (PI / 3.0)

/* 静态常量：机构连杆长度，参与正逆运动学；当前启用。 */
static const double LINK_LENGTH = 210.0;
/* 静态常量：几何偏置半径，参与正逆运动学；当前启用。 */
static const double OFFSET_RADIUS = 20.0;
/* 静态常量：机构高度零点偏移，参与高度换算；当前启用。 */
static const double HEIGHT_OFFSET = 59.46128;
/* 静态常量：规划终点总行程高度，供轨迹边界限幅；当前启用。 */
static const double TOTAL_H = 277.13;
/* 静态常量：匀速段平台速度，供各轨迹模式计算时间和速度；当前启用。 */
static const double V_CONST = 1000.0;

/* 全局变量：梯形/FIVE1 轨迹参数；仅 T 或 FIVE1 模式读取，其他模式不使用。 */
MotionProfile profile;
/* 全局变量：FIVE2 角度域轨迹参数；仅 FIVE2 模式读取，其他模式不使用。 */
ThetaProfile tp;

/* 函数：由机构角度计算平台高度；供轨迹边界搜索使用，当前启用。 */
static double get_h_by_theta(double theta)
{
    /* phi 为连杆夹角，root 为几何闭环平方根项；均为本次正运动学计算的临时量。 */
    double phi = 5.0 * PI / 9.0 - theta;
    double root = sqrt(44100.0 - pow(210.0 * sin(phi) - 20.0, 2.0));
    return 210.0 * cos(phi) + root - HEIGHT_OFFSET;
}

/* 函数：由平台高度反解电机圈数；被位置、速度规划接口调用，当前启用。 */
static double get_revs_by_h(double h)
{
    /* d/root/acos_arg 用于逆几何求角；对 acos_arg 限幅以维持反余弦定义域。 */
    double d = h + HEIGHT_OFFSET;
    double root = sqrt(d * d + OFFSET_RADIUS * OFFSET_RADIUS);
    double acos_arg = root / (2.0 * LINK_LENGTH);

    if (acos_arg > 1.0) {
        acos_arg = 1.0;
    }

    if (acos_arg < -1.0) {
        acos_arg = -1.0;
    }

    double phi = atan2(OFFSET_RADIUS, d) + acos(acos_arg);
    double theta = 5.0 * PI / 9.0 - phi;
    return theta / PI_3;
}

/* 函数：计算高度对角度的雅可比绝对值；供轨迹速度换算和边界搜索使用。 */
double get_J(double theta)
{
    /* term/root 和导数项组成机构雅可比，返回高度对角度变化率的绝对值。 */
    double phi = 5.0 * PI / 9.0 - theta;
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double term = 210.0 * sin_phi - 20.0;
    double root = sqrt(44100.0 - term * term);
    double dh_dphi = -210.0 * sin_phi - (term * 210.0 * cos_phi) / root;

    return fabs(dh_dphi);
}

/* 函数：扫描雅可比并求匀速段起止高度；初始化轨迹时调用。 */
static void find_const_heights(double *h_start, double *h_end)
{
    /* step/max_j 用于扫描最大雅可比，threshold 取其 90% 作为匀速段边界判据。 */
    const double step = 0.0001;
    double max_j = 0.0;
    double theta;

    for (theta = 0.0; theta <= PI_3 + 1e-6; theta += step) {
        double j = get_J(theta);
        if (j > max_j) 
        {
            max_j = j;
        }
    }

    double threshold = 0.9 * max_j;
    double theta_start = 0.0;
    double theta_end = PI_3;

    for (theta = 0.0; theta <= PI_3; theta += step) 
    {
        if (get_J(theta) >= threshold) 
        {
            theta_start = theta;
            break;
        }
    }

    for (theta = PI_3; theta >= 0.0; theta -= step) 
    {
        if (get_J(theta) >= threshold) {
            theta_end = theta;
            break;
        }
    }

    *h_start = get_h_by_theta(theta_start);
    *h_end = get_h_by_theta(theta_end);
}

#if defined(FIVE2)
/* 函数：用中心差分估算雅可比导数；仅 FIVE2 模式编译并使用。 */
static double get_J_prime(double theta)
{
    const double step = 1e-6;
    return (get_J(theta + step) - get_J(theta - step)) / (2.0 * step);
}

/* 函数：依据端点位移、速度、加速度和时长求五次多项式高阶系数；仅 FIVE2 模式使用。 */
static void solve_5th_coeffs(double d_theta, double v1, double a1, double duration,
                             double *c3, double *c4, double *c5)
{
    double x = 10.0 * d_theta - 4.0 * v1 * duration
             + 0.5 * a1 * duration * duration;
    double y = -15.0 * d_theta + 7.0 * v1 * duration
             - a1 * duration * duration;
    double z = 6.0 * d_theta - 3.0 * v1 * duration
             + 0.5 * a1 * duration * duration;

    *c3 = x / pow(duration, 3.0);
    *c4 = y / pow(duration, 4.0);
    *c5 = z / pow(duration, 5.0);
}
#endif

/* 函数：初始化所选运动规划模式的时间、边界及多项式系数；需在查询轨迹前调用。 */
void init_theta_profile(void)
{
    double h_start;
    double h_end;

    find_const_heights(&h_start, &h_end);

#if defined(T) || defined(FIVE1)
/* 初始化局部行程量分别表示加速、匀速、减速位移；仅当前选中的 T/FIVE1 轨迹模式使用。 */
    double s_acc = h_start;
    double s_const = h_end - h_start;
    double s_dec = TOTAL_H - h_end;

    profile.h_start_const = h_start;
    profile.h_end_const = h_end;
    profile.t_acc = 2.0 * s_acc / V_CONST;
    profile.t_const = s_const / V_CONST;
    profile.t_dec = 2.0 * s_dec / V_CONST;
    profile.a_acc = V_CONST / profile.t_acc;
    profile.a_dec = V_CONST / profile.t_dec;

#if defined(FIVE1)
    double t_acc = profile.t_acc;
    profile.acc_c3 = 10.0 * s_acc / pow(t_acc, 3.0)
                   - 4.0 * V_CONST / pow(t_acc, 2.0);
    profile.acc_c4 = -15.0 * s_acc / pow(t_acc, 4.0)
                   + 7.0 * V_CONST / pow(t_acc, 3.0);
    profile.acc_c5 = 6.0 * s_acc / pow(t_acc, 5.0)
                   - 3.0 * V_CONST / pow(t_acc, 4.0);

    double t_dec = profile.t_dec;
    profile.dec_c3 = 10.0 * s_dec / pow(t_dec, 3.0)
                   - 6.0 * V_CONST / pow(t_dec, 2.0);
    profile.dec_c4 = -15.0 * s_dec / pow(t_dec, 4.0)
                   + 8.0 * V_CONST / pow(t_dec, 3.0);
    profile.dec_c5 = 6.0 * s_dec / pow(t_dec, 5.0)
                   - 3.0 * V_CONST / pow(t_dec, 4.0);
#endif

#elif defined(FIVE2)
    /* FIVE2 的角度端点和局部边界状态只在该编译模式下构造。 */
    tp.h_start_const = h_start;
    tp.t_acc = 2.0 * h_start / V_CONST;
    tp.t_const = (h_end - h_start) / V_CONST;
    tp.t_dec = 2.0 * (TOTAL_H - h_end) / V_CONST;
    tp.t_total = tp.t_acc + tp.t_const + tp.t_dec;

    double theta0 = get_revs_by_h(0.0) * PI_3;
    double theta1 = get_revs_by_h(h_start) * PI_3;
    double theta2 = get_revs_by_h(h_end) * PI_3;
    double theta3 = get_revs_by_h(TOTAL_H) * PI_3;

    double t_acc = tp.t_acc;
    double j1 = get_J(theta1);
    double j1_prime = get_J_prime(theta1);
    double v1 = V_CONST / j1;
    double a1 = -V_CONST * V_CONST * j1_prime / (j1 * j1 * j1);
    double d_theta = theta1 - theta0;

    tp.acc_c0 = theta0;
    tp.acc_c1 = 0.0;
    tp.acc_c2 = 0.0;
    solve_5th_coeffs(d_theta, v1, a1, t_acc,
                     &tp.acc_c3, &tp.acc_c4, &tp.acc_c5);

    double t_dec = tp.t_dec;
    double j2 = get_J(theta2);
    double j2_prime = get_J_prime(theta2);
    double v2 = V_CONST / j2;
    double a2 = -V_CONST * V_CONST * j2_prime / (j2 * j2 * j2);

    tp.dec_c0 = theta2;
    tp.dec_c1 = v2;
    tp.dec_c2 = 0.5 * a2;

    d_theta = theta3 - (theta2 + v2 * t_dec + 0.5 * a2 * t_dec * t_dec);
    double v1_prime = -(v2 + a2 * t_dec);
    double a1_prime = -a2;
    solve_5th_coeffs(d_theta, v1_prime, a1_prime, t_dec,
                     &tp.dec_c3, &tp.dec_c4, &tp.dec_c5);
#endif
}

#if defined(T) || defined(FIVE1)
/* 函数：按 T/FIVE1 模式计算指定时刻的平台高度；其他模式下不编译。 */
static double get_profile_height(double t)
{
    if (t <= 0.0) {
        return 0.0;
    }

    if (t <= profile.t_acc) {
#if defined(T)
        return 0.5 * profile.a_acc * t * t;
#elif defined(FIVE1)
        return profile.acc_c3 * pow(t, 3.0)
             + profile.acc_c4 * pow(t, 4.0)
             + profile.acc_c5 * pow(t, 5.0);
#endif
    }

    if (t <= profile.t_acc + profile.t_const) {
        return profile.h_start_const + V_CONST * (t - profile.t_acc);
    }

    if (t <= profile.t_acc + profile.t_const + profile.t_dec) {
        double tau = t - profile.t_acc - profile.t_const;
#if defined(T)
        return profile.h_end_const + V_CONST * tau
             - 0.5 * profile.a_dec * tau * tau;
#elif defined(FIVE1)
        return profile.h_end_const + V_CONST * tau
             + profile.dec_c3 * pow(tau, 3.0)
             + profile.dec_c4 * pow(tau, 4.0)
             + profile.dec_c5 * pow(tau, 5.0);
#endif
    }

    return TOTAL_H;
}

/* 函数：按 T/FIVE1 模式计算指定时刻的平台速度；其他模式下不编译。 */
static double get_plate_velocity(double t)
{
    if ((t < 0.0) ||
        (t > profile.t_acc + profile.t_const + profile.t_dec)) {
        return 0.0;
    }

    if (t <= profile.t_acc) {
#if defined(T)
        return profile.a_acc * t;
#elif defined(FIVE1)
        return 3.0 * profile.acc_c3 * t * t
             + 4.0 * profile.acc_c4 * t * t * t
             + 5.0 * profile.acc_c5 * t * t * t * t;
#endif
    }

    if (t <= profile.t_acc + profile.t_const) {
        return V_CONST;
    }

    double tau = t - profile.t_acc - profile.t_const;
#if defined(T)
    return V_CONST - profile.a_dec * tau;
#elif defined(FIVE1)
    return V_CONST
         + 3.0 * profile.dec_c3 * tau * tau
         + 4.0 * profile.dec_c4 * tau * tau * tau
         + 5.0 * profile.dec_c5 * tau * tau * tau * tau;
#endif
}
#endif

/* 函数：返回指定时刻的电机目标圈数；按当前编译模式选择轨迹分段。 */
double get_motor_revs_theta_plan(double t)
{
#if defined(T) || defined(FIVE1)
    /* h 是高度域轨迹值，限幅到机构有效总行程后再换算为电机圈数。 */
    double h = get_profile_height(t);
    if (h < 0.0) {
        h = 0.0;
    } else if (h > TOTAL_H) {
        h = TOTAL_H;
    }
    return get_revs_by_h(h);
#elif defined(FIVE2)
    if (t <= 0.0) {
        return get_revs_by_h(0.0);
    }

    /* theta_rad 暂存加速/减速段多项式角度；匀速段直接由高度反算并提前返回。 */
    double theta_rad;
    if (t <= tp.t_acc) 
    {
        double tau = t;
        theta_rad = tp.acc_c0
                  + tp.acc_c1 * tau
                  + tp.acc_c2 * tau * tau
                  + tp.acc_c3 * tau * tau * tau
                  + tp.acc_c4 * pow(tau, 4.0)
                  + tp.acc_c5 * pow(tau, 5.0);
    } 
    else if (t <= tp.t_acc + tp.t_const) 
    {
        double h = tp.h_start_const + V_CONST * (t - tp.t_acc);
        if (h > TOTAL_H) 
        {
            h = TOTAL_H;
        }
        return get_revs_by_h(h);
    } 
    else if 
    (t <= tp.t_total) 
    {
        double tau = t - tp.t_acc - tp.t_const;
        theta_rad = tp.dec_c0
                  + tp.dec_c1 * tau
                  + tp.dec_c2 * tau * tau
                  + tp.dec_c3 * tau * tau * tau
                  + tp.dec_c4 * pow(tau, 4.0)
                  + tp.dec_c5 * pow(tau, 5.0);
    } else 
    {
        return get_revs_by_h(TOTAL_H);
    }

    return theta_rad / PI_3;
#endif
}

/* 函数：返回指定时刻的电机目标速度；T/FIVE1/FIVE2 分别走各自计算分支。 */
double get_motor_vel_at_time(double t)
{
#if defined(T) || defined(FIVE1)
    /* plate_velocity 为平台线速度，先检查方向/边界，再用雅可比换算电机角速度。 */
    double h;
    double plate_velocity = get_plate_velocity(t);

    if (plate_velocity <= 0.0) {
        return 0.0;
    }

    h = get_profile_height(t);
    if (h < 0.0) {
        h = 0.0;
    } else if (h > TOTAL_H) {
        h = TOTAL_H;
    }

    double theta = get_revs_by_h(h) * PI_3;
    double j = get_J(theta);
    if (j < 1e-6) {
        j = 1e-6;
    }
    return plate_velocity / (j * PI_3);
#elif defined(FIVE2)
    if ((t <= 0.0) || (t > tp.t_total)) 
    {
        return 0.0;
    }

    if (t <= tp.t_acc) 
    {
        double tau = t;
        return (tp.acc_c1
              + 2.0 * tp.acc_c2 * tau
              + 3.0 * tp.acc_c3 * tau * tau
              + 4.0 * tp.acc_c4 * pow(tau, 3.0)
              + 5.0 * tp.acc_c5 * pow(tau, 4.0)) / PI_3;
    }

    if (t <= tp.t_acc + tp.t_const) 
    {
        double h = tp.h_start_const + V_CONST * (t - tp.t_acc);
        double theta = get_revs_by_h(h) * PI_3;
        return V_CONST / (get_J(theta) * PI_3);
    }

    double tau = t - tp.t_acc - tp.t_const;
    return (tp.dec_c1
          + 2.0 * tp.dec_c2 * tau
          + 3.0 * tp.dec_c3 * tau * tau
          + 4.0 * tp.dec_c4 * pow(tau, 3.0)
          + 5.0 * tp.dec_c5 * pow(tau, 4.0)) / PI_3;
#endif
}
