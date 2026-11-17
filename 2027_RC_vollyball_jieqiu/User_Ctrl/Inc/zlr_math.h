#ifndef ZLR_MATH_H
#define ZLR_MATH_H

/* T: trapezoidal height; FIVE1: quintic height; FIVE2: quintic motor angle. */
#define FIVE2

#if (defined(T) + defined(FIVE1) + defined(FIVE2)) != 1
#error "Define exactly one motion mode: T, FIVE1, or FIVE2"
#endif

#ifdef __cplusplus

extern "C" {
#endif


// 规划参数结构体
typedef struct {
    double t_acc, t_const, t_dec;   // 三段时间 (s)
    double a_acc, a_dec;            // 加减速度 (mm/s^2)
    double h_start_const, h_end_const; // 匀速段起止高度 (mm)
    // 加速段系数
    double acc_c3, acc_c4, acc_c5;
    // 减速段系数
    double dec_c3, dec_c4, dec_c5;
} MotionProfile;

typedef struct {
    double t_acc, t_const, t_dec, t_total;
    double h_start_const;

    /* 加速段：θ(τ) = c0 + c1τ + c2τ² + c3τ³ + c4τ⁴ + c5τ⁵ */
    double acc_c0, acc_c1, acc_c2, acc_c3, acc_c4, acc_c5;

    /* 减速段：θ(τ) = c0 + c1τ + c2τ² + c3τ³ + c4τ⁴ + c5τ⁵ */
    double dec_c0, dec_c1, dec_c2, dec_c3, dec_c4, dec_c5;
} ThetaProfile;

// 全局变量声明（在 .c 文件中定义）
extern MotionProfile profile;
extern ThetaProfile tp;

// ==========================================================
// 接口函数声明
// ==========================================================

/* Selects the active planner at compile time via T, FIVE1, or FIVE2. */
void init_theta_profile(void);

/* Position in revolutions and velocity in revolutions per second. */
double get_motor_vel_at_time(double t);
double get_J(double theta);
double get_motor_revs_theta_plan(double t);




#ifdef __cplusplus
}
#endif

#endif // ZLR_MATH_H