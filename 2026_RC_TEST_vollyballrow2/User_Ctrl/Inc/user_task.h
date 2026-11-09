#ifndef USER_TASK_H

#define USER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "user_communication.h"
#include "user_FDCAN.h"

extern osThreadId_t user_task;
extern TaskHandle_t user_task_handle;

extern bool camp_flag;//false红场，true蓝场
extern bool star_receive_flag;
typedef struct {
    int32_t *buffer;   // 数据缓冲区指针（支持动态/静态内存）
    int32_t sum;       // 窗口内数据累加和（防溢出用int32_t）
    uint16_t index;    // 当前写入位置索引
    uint16_t size;     // 窗口大小N
    uint16_t count;    // 当前有效数据量（未填满时< size）
} MAFilter;

#define TEST (1)    //0滑轨1盘2臂
#define WAY (1)    //0分测1合测
#define ARMSTART 1.75f

#if WAY == 0
    void platecrl(void);
    void arm_pos1(void);
    void arm_pos2(void);
    float pos_row(void);
    void get_rowpos();
#elif WAY == 1

    void platecrl(int mode);
    void arm_pos1(int mode , bool *pos_ok);
    void arm_pos2(int mode);
    float pos_row(int mode);
    void get_rowpos_high();

    float get_platevel_highdown117(float r);
    void get_rowpos_highdown117(void);
    float pos_row_highdown117(void);

    float get_platevel_low118b(float r);
    void get_rowpos_low118b(void);
    float pos_row_low118b(void);

    float get_platevel_low118a(float r);
    void get_rowpos_low118a(void);
    float pos_row_low118a(void);
    
#endif



void User_Task_Init(void);
void User_Task(void *argument);
void GetADCMsg();
void valve_ctrl();

void get_rowpos_highdown();
void test(void);
void MAFilter_Init(MAFilter *f, int32_t *buf, uint16_t window_size);

float pos_row_high(void);
float pos_row_highdown(void);
float get_platevel_high(float r);
float get_platevel_highdown(float r);
float get_platevel_low(float r);
float user_abs(float x);
float get_slidepos_low(void);
float get_slidepos_high(void);
float get_slidepos_highdown(void);



int32_t MAFilter_Put(MAFilter *f, int32_t new_value);


#ifndef USE_NEW_POS_ROW
#define USE_NEW_POS_ROW 1
#endif

#ifdef FIVE
    #ifndef POS_ROW_H
    #define POS_ROW_H

    #include <stdbool.h>

    /*
    * 击球臂参考轨迹接口。
    *
    * 功能：指定击球时刻、击球角度和击球角速度，生成两段平滑轨迹：
    *   准备角静止 -> 击球角且保留指定速度 -> 结束角静止。
    *
    * 使用顺序：
    *   1. 上层先控制击球臂回到准备角，并确认实际角速度接近零。
    *   2. 填写 PosRowPlan 的五个输入字段，调用 pos_row_plan() 规划一次。
    *   3. 规划成功后，每个控制周期调用 pos_row()，取得位置和速度参考。
    *   4. 将参考值交给各侧电机的闭环控制；下一次击球重新规划。
    *
    * 统一单位：角度 rad，角速度 rad/s，角加速度 rad/s²，时间 s。
    * 角度换算：rad = 角度数 * pi / 180；圈数 = rad / (2*pi)。
    * 速度换算：rad/s = rpm * 2*pi / 60。
    * 所有角度应属于同一关节坐标系；驱动的零点、方向和传动比由外部换算。
    *
    * 本模块只计算参考值，不读取编码器、不执行 PD，也不发送 CAN 指令。
    * 参考轨迹到达目标，不等于已经检测到球接触或实际电机准确到位。
    */

    /*
    * 当前时间所在的参考轨迹阶段，由 pos_row() 返回。
    * 阶段直接根据时间计算，不依赖上一次调用；调用延迟时可能跨过某个阶段。
    */
    typedef enum {
        POS_ROW_ERROR = -1, /* 输入或计划无效；本次不改写输出，上层处理故障。 */
        POS_ROW_WAIT,       /* 尚未到起挥时刻：输出准备角和零角速度。 */
        POS_ROW_SWING,      /* 起挥到预计击球时刻：执行第一段轨迹。 */
        POS_ROW_FOLLOW,     /* 预计击球时刻到收杆结束：执行第二段轨迹。 */
        POS_ROW_HOLD        /* 参考轨迹已结束：持续输出结束角和零角速度。 */
    } PosRowStage;

    /*
    * 一次击球的规划数据；建议声明时使用 {0} 初始化。
    *
    * 输入字段：q_home_rad、q_hit_rad、w_hit_rad_s、q_end_rad、t_hit_s。
    * 输出字段：t_start_s、t_end_s、valid，由 pos_row_plan() 写入。
    *
    * 采用连续累计角度，例如 -270° 到 -30° 表示沿正方向转动 240°。
    * 不要把这些角度归一化到 [-pi, pi]，否则会改变所需转动方向和行程。
    * 规划与周期求值不要并发修改同一个对象；规划成功后保持字段不变。
    */
    typedef struct {
        float q_home_rad;  /* 输入：起挥前准备角，rad；该处参考角速度为零。 */
        float q_hit_rad;   /* 输入：预计击球时刻要求达到的角度，rad。 */
        float w_hit_rad_s; /* 输入：预计击球时刻的角速度，rad/s，带方向，不能为零。 */
        float q_end_rad;   /* 输入：击球后收杆的结束角，rad；该处参考角速度为零。 */
        float t_hit_s;     /* 输入：预计击球时刻，s；不是挥击持续时间。 */
        float t_start_s;   /* 输出：反推得到的起挥时刻，s。 */
        float t_end_s;     /* 输出：第二段收杆轨迹结束的时刻，s。 */
        bool valid;       /* 输出：只有完整规划和检查成功后才为 true。 */
    } PosRowPlan;

    /*
    * 规划一次击球；在收到上层动作参数和预计击球时刻后调用。
    *
    * 参数：
    *   plan                      输入/输出：待规划对象，不能为 NULL。
    *                             调用前填写上面的五个输入字段。
    *   now_s                     输入：规划时的当前时间，s，必须有限且 >= 0。
    *   max_speed_rad_s           输入：允许的参考角速度绝对值上限，rad/s，> 0。
    *                             当前项目 750 rpm 对应约 78.539816 rad/s。
    *   max_acceleration_rad_s2   输入：允许的参考角加速度绝对值上限，rad/s²，> 0。
    *                             当前项目配置为 6000 rad/s²。
    *   minimum_lead_s            输入：从本次传入的当前时间到起挥时刻，
    *                             至少应预留的时间，s，>= 0；项目示例为 0.035 s。
    *
    * 时间约定：now_s、t_hit_s 和后续 pos_row() 的 now_s 必须共用时间零点，
    * 通常取“本次开始抛球的时刻”为零点。
    * 例如 now_s=0.10、t_hit_s=0.30，表示现在距预计击球还有 0.20 秒，
    * 但其中可能包含等待起挥的时间，并非全部用于挥动。
    *
    * 计算规则，与项目 ServeSimulation._plan_strike() 一致：
    *   第一段时长 T1 = 2*|q_hit-q_home| / |w_hit|。
    *   第二段时长 T2 = 2*|q_end-q_hit| / |w_hit|。
    *   t_start = t_hit - T1；t_end = t_hit + T2。
    *
    * 返回值：
    *   true ：计划有效，可周期调用 pos_row()。
    *   false：输入非法、转动方向不一致、剩余时间不足，或参考轨迹超限。
    *          对于非 NULL 的 plan，valid 会先清为 false；旧计划也随之失效。
    *          失败时不要使用 t_start_s、t_end_s 或继续发送旧参考值。
    *
    * 检查范围：每段取 101 个点检查速度和加速度，不计算所需电机力矩，
    * 也不检查机械碰撞和关节位置限位；通过检查不能保证实际电机完全跟踪。
    */
    bool pos_row_plan(PosRowPlan *plan, float now_s, float max_speed_rad_s,
                    float max_acceleration_rad_s2, float minimum_lead_s);

    /*
    * 按当前时间求参考角度和参考角速度；每个控制周期调用一次。
    *
    * 参数：
    *   plan           输入：已经由 pos_row_plan() 成功生成的计划，只读。
    *   now_s          输入：实际计时得到的当前时间，s，必须有限且 >= 0。
    *                  与计划使用同一零点，调用者应保证时间单调递增。
    *                  建议使用定时器实测时间，避免每次固定加 0.005 的累计误差。
    *   q_ref_rad      输出指针：当前参考角度，rad，不能为 NULL。
    *   w_ref_rad_s    输出指针：当前参考角速度，rad/s，不能为 NULL。
    *                  两个输出应使用不同的变量接收。
    *
    * 返回值：PosRowStage 枚举；ERROR 时不改写两个输出变量。
    * 进入 HOLD 只表示参考轨迹结束，不代表实际击球臂已经停稳。
    *
    * 在 now_s == t_hit_s 时返回 FOLLOW，并输出 q_hit_rad、w_hit_rad_s。
    * 这是两段轨迹的连接点：角度、角速度连续，连接处参考角加速度为零。
    * 函数不会在击球时刻把速度清零，也不会在收杆结束后自动失能。
    *
    * 调用周期可以为 5 ms，但若 t_hit_s 落在两个周期之间，就不会刚好在
    * 该时刻发送一帧；时间同步、驱动插值和实际跟踪误差仍由外部控制处理。
    */
    PosRowStage pos_row(const PosRowPlan *plan, float now_s,
                        float *q_ref_rad, float *w_ref_rad_s);

    #endif
#endif


#ifdef __cplusplus
}
#endif

#endif /* USER_TASK_H */



