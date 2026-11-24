/*数学计算模块
*目前分三种模式，梯形规划，五次多项式1,2
*/
/* 头文件保护宏：避免运动规划接口重复定义。 */
#ifndef ZLR_MATH_H
#define ZLR_MATH_H

/* T: trapezoidal height; FIVE1: quintic height; FIVE2: quintic motor angle. */
/* 宏：选择五次电机角度规划模式；当前唯一启用的运动规划模式。 */
#define FIVE2 
/* 宏：五次高度规划模式选项；当前被注释屏蔽，不能与启用模式并用。 */
// #define FIVE1 
/* 宏：梯形高度规划模式选项；当前被注释屏蔽，不能与启用模式并用。 */
// #define T

/* 编译期检查：要求 T、FIVE1、FIVE2 中恰好启用一个规划模式。 */
#if (defined(T) + defined(FIVE1) + defined(FIVE2)) != 1
#error "Define exactly one motion mode: T, FIVE1, or FIVE2"
#endif

#ifdef __cplusplus

extern "C" {
#endif


// 规划参数结构体
/* 结构体：梯形/高度五次规划参数；在 T 或 FIVE1 模式下由 profile 使用。 */
typedef struct {
    double t_acc, /* 加速段持续时间，单位秒。 */
           t_const, /* 匀速段持续时间，单位秒。 */
           t_dec; /* 减速段持续时间，单位秒。 */ // 三段时间 (s)
    double a_acc, /* 加速段加速度参数，单位按高度规划约定。 */
           a_dec; /* 减速段减速度参数，单位按高度规划约定。 */ // 加减速度 (mm/s^2)
    double h_start_const, /* 匀速段起始高度。 */
           h_end_const; /* 匀速段结束高度。 */ // 匀速段起止高度 (mm)
    // 加速段系数
    double acc_c3, /* 加速段三次项系数。 */
           acc_c4, /* 加速段四次项系数。 */
           acc_c5; /* 加速段五次项系数。 */
    // 减速段系数
    double dec_c3, /* 减速段三次项系数。 */
           dec_c4, /* 减速段四次项系数。 */
           dec_c5; /* 减速段五次项系数。 */
} MotionProfile;

/* 结构体：电机角度域分段五次轨迹参数；当前 FIVE2 模式使用 tp。 */
typedef struct {
    double t_acc, /* 加速段持续时间，单位秒。 */
           t_const, /* 匀速段持续时间，单位秒。 */
           t_dec, /* 减速段持续时间，单位秒。 */
           t_total; /* 完整轨迹总时长，单位秒。 */
    /* 匀速段开始时对应的机构高度。 */
    double h_start_const;

    /* 加速段：θ(τ) = c0 + c1τ + c2τ² + c3τ³ + c4τ⁴ + c5τ⁵ */
    double acc_c0, /* 加速段角度多项式常数项系数。 */
           acc_c1, /* 加速段角度多项式一次项系数。 */
           acc_c2, /* 加速段角度多项式二次项系数。 */
           acc_c3, /* 加速段角度多项式三次项系数。 */
           acc_c4, /* 加速段角度多项式四次项系数。 */
           acc_c5; /* 加速段角度多项式五次项系数。 */

    /* 减速段：θ(τ) = c0 + c1τ + c2τ² + c3τ³ + c4τ⁴ + c5τ⁵ */
    double dec_c0, /* 减速段角度多项式常数项系数。 */
           dec_c1, /* 减速段角度多项式一次项系数。 */
           dec_c2, /* 减速段角度多项式二次项系数。 */
           dec_c3, /* 减速段角度多项式三次项系数。 */
           dec_c4, /* 减速段角度多项式四次项系数。 */
           dec_c5; /* 减速段角度多项式五次项系数。 */
} ThetaProfile;

// 全局变量声明（在 .c 文件中定义）
/* 外部变量：梯形或高度域规划参数；仅 T/FIVE1 分支使用，当前 FIVE2 模式不使用。 */
extern MotionProfile profile;
/* 外部变量：角度域规划参数；当前 FIVE2 模式启用并由规划函数读取。 */
extern ThetaProfile tp;

// ==========================================================
// 接口函数声明
// ==========================================================

/* Selects the active planner at compile time via T, FIVE1, or FIVE2. */
/* 函数声明：按编译时选定模式初始化全局运动规划参数；系统启动规划前调用。 */
void init_theta_profile(void);

/* Position in revolutions and velocity in revolutions per second. */
/* 函数声明：计算时刻 t 的电机转速；当前启用模式均提供对应规划分支。 */
double get_motor_vel_at_time(double t);
/* 函数声明：由电机角度计算机构雅可比绝对值；当前供规划函数内部调用。 */
double get_J(double theta);
/* 函数声明：计算时刻 t 的规划电机转数；当前启用模式下可供控制任务调用。 */
double get_motor_revs_theta_plan(double t);




#ifdef __cplusplus
}
#endif

#endif // ZLR_MATH_H