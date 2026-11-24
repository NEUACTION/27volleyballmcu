// 文件作用：双臂划杆规划：提供两段五次位置轨迹、固定周期一般轨迹及双臂位置映射。 声明公共类型与接口。
#pragma once

namespace planner
{

    // 原头 user_task.h:162–168；阶段由当前时间决定，无须保存点列。
    enum PosRowStage
    {
        POS_ROW_ERROR = -1,  // 计划或输入错误，求值函数不改写输出。
        POS_ROW_WAIT,        // 起挥前保持准备角。
        POS_ROW_SWING,       // 第一段五次轨迹。
        POS_ROW_FOLLOW,      // 第二段五次轨迹。
        POS_ROW_HOLD         // 结束后保持结束角；不表示硬件已停稳。
    }
    ;

    // 原头 user_task.h:177–187；五个输入必须由调用方明确填写，不猜测目标/限幅。
    // 调用方应使用聚合初始化；其余字段由 PlanRowPositionTrajectory 写入。
    struct PosRowPlan_t
    {
        float homePositionRadians;  // 输入：准备角，rad。
        float hitPositionRadians;  // 输入：击球角，rad。
        float hitAngularVelocityRadPerSecond;  // 输入：非零击球角速度，rad/s。
        float endPositionRadians;  // 输入：收杆角，rad。
        float hitTimeSeconds;  // 输入：与调用者时钟同一原点的击球时刻，s。
        float startTimeSeconds;  // 输出：起挥时刻，s；规划成功后可用。
        float endTimeSeconds;  // 输出：结束时刻，s；规划成功后可用。
        bool valid;  // 输出：规划是否通过；求值前必须先规划。
    }
    ;

    // 原 user_task.c:812–892，保留仅对第一段101点检查限速/限加速度的行为。
    // nowSeconds及击球时间用同一时钟；各限幅与预留时间由调用方显式提供。
    // 规划双段划杆：plan 输入端点并接收起止时间及有效标记，nowSeconds 为当前时刻（s），maxSpeedRadPerSecond/maxAccelerationRadPerSecondSquared 为速度/加速度上限，minimumLeadSeconds 为最少起挥预留时间（s）；返回校验及首段采样是否通过。
    bool PlanRowPositionTrajectory(PosRowPlan_t *plan, float nowSeconds,
        float maxSpeedRadPerSecond, float maxAccelerationRadPerSecondSquared,
        float minimumLeadSeconds);

    // 原 user_task.c:897–939；每5ms调用一次，用当下时间现算一份位置/速度。
    // 原核心的FOLLOW是第二段五次轨迹，未包含外层624–632行的恒速覆盖。
    // 查询双段划杆：plan 为已规划数据，nowSeconds 为同一时钟下的时刻（s）；referencePositionRadians/referenceAngularVelocityRadPerSecond 输出 rad 与 rad/s，返回当前阶段，错误时不改输出。
    PosRowStage GetRowPositionReference(const PosRowPlan_t *plan, float nowSeconds,
        float *referencePositionRadians, float *referenceAngularVelocityRadPerSecond);

    // 双臂增益独立保存，单位保持原驱动pos_kp/pos_kd，不合并左右数值。
    struct RowArmGains
    {
        float kp;  // 位置比例增益。
        float kd;  // 位置微分增益。
    }
    ;

    // 一般轨迹原user_task.c:46–49的状态机阶段。
    // Accelerating 加速，ConstantSpeed 恒速，Stopping 执行停止，Complete 已进入完成状态。
    enum class RowMotionStage
    {
        Accelerating = 0, ConstantSpeed = 1, Stopping = 2, Complete = 3
    }
    ;

    // 一次5ms更新的逻辑row轴参考；逻辑累计行程尚未叠加-ARMSTART零点。
    struct GeneralRowReference
    {
        float position_turns;  // 从起挥起累计位置，圈。
        float velocity_turns_per_second;  // 本周期积分使用速度，圈/秒。
        RowArmGains arm1_gains;  // ARM1本周期独立增益。
        RowArmGains arm2_gains;  // ARM2本周期独立增益。
        RowMotionStage stage;  // 更新后的状态机阶段。
        bool disable_requested;  // 原991–992行失能意图，仅STOPPING执行周期为true。
    }
    ;

    // 一般轨迹944–1006：每次Step严格推进5ms，位置先更新速度后积分。
    // 不发送总线指令；上层读取disable_requested后执行硬件动作。
    class GeneralRowPlanner
    {
        public:
        // 构造一般划杆规划器：无参数；清零累计位置、速度并初始化阶段与双臂增益，无返回值。
        GeneralRowPlanner();  // 原静态状态改为实例状态，首次从0圈、0速度起步。
        // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
        void Reset();  // 重置一次发球；复用源初始状态，不保留上一轮累计行程。
        // 推进一般划杆：无参数；按固定 5 ms 更新速度、积分位置，返回本周期参考和失能意图。
        GeneralRowReference Step();  // 现算当前一帧，不生成或预上传点列。

        private:
        float reference_position_;  // 累计逻辑行程，圈，原956行。
        float reference_velocity_;  // 当前逻辑速度，圈/秒，原957行。
        RowMotionStage trajectory_stage_;  // 原958–959行阶段。
        RowArmGains arm1_gains_;  // 原armTar1独立位置增益状态。
        RowArmGains arm2_gains_;  // 原armTar2独立位置增益状态。
    }
    ;

    // 映射后的双臂参考，仍使用圈，不承担4096编码计数或传动比换算。
    struct RowArmTargets
    {
        float arm1_position_turns;  // ARM1绝对逻辑圈位置。
        float arm2_position_turns;  // ARM2反向位置，保留-0.009圈偏置。
        RowArmGains arm1_gains;  // 原样保留ARM1增益。
        RowArmGains arm2_gains;  // 原样保留ARM2增益。
    }
    ;

    // 原646–647行：arm1=q，arm2=-q-0.009；FIVE弧度需调用方显式换算为圈。
    // 映射逻辑划杆位置：logical_position_turns 为逻辑圈数，arm1_gains/arm2_gains 为各臂独立增益；返回两臂反向位置及原样增益。
    RowArmTargets MapLogicalRowTurns(float logical_position_turns,
        RowArmGains arm1_gains, RowArmGains arm2_gains);

    // 原658–665行：arm1=-ARMSTART+positionSum，arm2=ARMSTART-positionSum-0.009。
    // arm_start_turns必须显式传入，避免猜测ARMSTART宏值。
    // 映射累计行程：position_sum_turns 为累计圈数，arm_start_turns 为准备零点（圈），arm1_gains/arm2_gains 为独立增益；返回带准备零点和双臂偏置的目标。
    RowArmTargets MapGeneralRowTurns(float position_sum_turns, float arm_start_turns,
        RowArmGains arm1_gains, RowArmGains arm2_gains);

}
// namespace planner
