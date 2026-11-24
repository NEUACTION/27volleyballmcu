// 文件作用：双臂划杆规划：提供两段五次位置轨迹、固定周期一般轨迹及双臂位置映射。 实现轨迹计算与状态更新。
#include "planner/serve/arm_trajectory.hpp"

#include <cmath>
#include <cstddef>

namespace planner
{
    using std::isfinite;

    // 以下FIVE数学核心逐句迁自user_task.c:767–939。
    // 源767–807行：端点位置/速度约束的五次多项式，端点加速度为零。
    // normalizedTime为[0,1]时间，positionDelta为位移，coefficient1/3/4/5为归一化系数。
    // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出角度、角速度、角加速度，无返回值。
    static void EvaluateQuinticTrajectory(float startPosition, float startVelocity,
        float targetPosition,
        float targetVelocity,
        float durationSeconds,
        float elapsedSeconds, float *position,
        float *velocity, float *acceleration)
    {
        // 段内时间除以段长并限制到 [0,1] 的无量纲进度。
        const float normalizedTime =
        fminf(1.0f, fmaxf(0.0f, elapsedSeconds / durationSeconds));
        // 终点减起点的角位移，rad。
        const float positionDelta = targetPosition - startPosition;
        // 归一化五次式的一次项系数，由起点角速度和段长确定，rad。
        const float coefficient1 = startVelocity * durationSeconds;
        // 归一化五次式的三次项系数，用于满足端点位置与速度约束，rad。
        const float coefficient3 =
        10.0f * positionDelta -
        (6.0f * startVelocity + 4.0f * targetVelocity) * durationSeconds;
        // 归一化五次式的四次项系数，用于满足端点位置与速度约束，rad。
        const float coefficient4 =
        -15.0f * positionDelta +
        (8.0f * startVelocity + 7.0f * targetVelocity) * durationSeconds;
        // 归一化五次式的五次项系数，用于满足端点位置与速度约束，rad。
        const float coefficient5 =
        6.0f * positionDelta -
        3.0f * (startVelocity + targetVelocity) * durationSeconds;

        *position = startPosition +
        normalizedTime *
        (coefficient1 +
            normalizedTime * normalizedTime *
            (coefficient3 +
            normalizedTime *
            (coefficient4 + normalizedTime * coefficient5)));
        *velocity =
        (coefficient1 +
            normalizedTime * normalizedTime *
            (3.0f * coefficient3 +
            normalizedTime * (4.0f * coefficient4 +
            5.0f * normalizedTime * coefficient5))) /
        durationSeconds;
        *acceleration = normalizedTime *
        (6.0f * coefficient3 +
            normalizedTime * (12.0f * coefficient4 +
            20.0f * normalizedTime * coefficient5)) /
        (durationSeconds * durationSeconds);
    }

    /**
     * @brief 校验计划并检查轨迹速度、加速度限制。
     */
    // 规划双段划杆：plan 输入端点并接收起止时间及有效标记，nowSeconds 为当前时刻（s），maxSpeedRadPerSecond/maxAccelerationRadPerSecondSquared 为速度/加速度上限，minimumLeadSeconds 为最少起挥预留时间（s）；返回校验及首段采样是否通过。
    bool PlanRowPositionTrajectory(PosRowPlan_t *plan, float nowSeconds,
        float maxSpeedRadPerSecond,
        float maxAccelerationRadPerSecondSquared,
        float minimumLeadSeconds)
    {
        if (plan == NULL)
        {
            return false;
        }
        plan->valid = false;

        if (!isfinite(nowSeconds) || nowSeconds < 0.0f ||
            !isfinite(plan->hitTimeSeconds) ||
            !isfinite(plan->homePositionRadians) ||
            !isfinite(plan->hitPositionRadians) ||
            !isfinite(plan->endPositionRadians) ||
            !isfinite(plan->hitAngularVelocityRadPerSecond) ||
            !isfinite(maxSpeedRadPerSecond) || maxSpeedRadPerSecond <= 0.0f ||
            !isfinite(maxAccelerationRadPerSecondSquared) ||
            maxAccelerationRadPerSecondSquared <= 0.0f ||
            !isfinite(minimumLeadSeconds) || minimumLeadSeconds < 0.0f ||
            plan->hitAngularVelocityRadPerSecond == 0.0f ||
            fabsf(plan->hitAngularVelocityRadPerSecond) > maxSpeedRadPerSecond)
        {
            return false;
        }

        // 两段位移、时长保留原式；只对首段进行101点速度/加速度抽样检查。
        // 准备位置到击球位置的角位移，rad。
        const float firstPositionDelta =
        plan->hitPositionRadians - plan->homePositionRadians;
        // 击球位置到结束位置的角位移，rad。
        const float secondPositionDelta =
        plan->endPositionRadians - plan->hitPositionRadians;
        if (!isfinite(firstPositionDelta) || !isfinite(secondPositionDelta) ||
            firstPositionDelta == 0.0f || secondPositionDelta == 0.0f ||
            (firstPositionDelta > 0.0f) !=
            (plan->hitAngularVelocityRadPerSecond > 0.0f) ||
            (secondPositionDelta > 0.0f) !=
            (plan->hitAngularVelocityRadPerSecond > 0.0f))
        {
            return false;
        }

        // 击球角速度的绝对值，rad/s。
        const float hitSpeed = fabsf(plan->hitAngularVelocityRadPerSecond);
        // 由首段位移及击球速度确定的起挥段时长，s。
        const float firstSegmentDurationSeconds =
        2.0f * fabsf(firstPositionDelta) / hitSpeed;
        // 由第二段位移及击球速度确定的跟随段时长，s。
        const float secondSegmentDurationSeconds =
        2.0f * fabsf(secondPositionDelta) / hitSpeed;
        plan->startTimeSeconds = plan->hitTimeSeconds - firstSegmentDurationSeconds;
        plan->endTimeSeconds = plan->hitTimeSeconds + secondSegmentDurationSeconds;
        if (!isfinite(plan->startTimeSeconds) || !isfinite(plan->endTimeSeconds) ||
            !(plan->startTimeSeconds < plan->hitTimeSeconds &&
            plan->hitTimeSeconds < plan->endTimeSeconds) ||
            plan->startTimeSeconds - nowSeconds < minimumLeadSeconds)
        {
            return false;
        }

        // speedTolerance为浮点限速容差，durationSeconds为第一段时长。
        // 首段采样比较时允许的浮点速度误差，rad/s。
        const float speedTolerance =
        fmaxf(1.0f, maxSpeedRadPerSecond) * 1e-5f + 1e-4f;
        // 首段起挥时长，s，用于采样计算。
        const float durationSeconds = plan->hitTimeSeconds - plan->startTimeSeconds;
        // i 为首段 101 个等间隔采样点的索引，0 和 100 分别包含两端。
        for (int i = 0; i <= 100; ++i)
        {
            // 采样点角度，rad，用于有限性检查。
            float position = 0.0f;
            // 采样点角速度，rad/s，用于限速检查。
            float velocity = 0.0f;
            // 五次求值的角加速度输出，rad/s²；查询函数仅接收该辅助结果。
            float acceleration = 0.0f;
            // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出角度、角速度、角加速度，无返回值。
            EvaluateQuinticTrajectory(
                plan->homePositionRadians, 0.0f, plan->hitPositionRadians,
                plan->hitAngularVelocityRadPerSecond, durationSeconds,
                durationSeconds * ((float)i / 100.0f), &position, &velocity,
                &acceleration);
            if (!isfinite(position) || !isfinite(velocity) ||
                !isfinite(acceleration) ||
                fabsf(velocity) > maxSpeedRadPerSecond + speedTolerance ||
                fabsf(acceleration) > maxAccelerationRadPerSecondSquared + 1e-3f)
            {
                return false;
            }
        }

        plan->valid = true;
        return true;
    }

    /**
     * @brief 按当前时刻计算轨迹阶段和位置参考。
     */
    // 查询双段划杆：plan 为已规划数据，nowSeconds 为同一时钟下的时刻（s）；referencePositionRadians/referenceAngularVelocityRadPerSecond 输出 rad 与 rad/s，返回当前阶段，错误时不改输出。
    PosRowStage GetRowPositionReference(const PosRowPlan_t *plan, float nowSeconds,
        float *referencePositionRadians,
        float *referenceAngularVelocityRadPerSecond)
    {
        if (plan == NULL || !plan->valid || referencePositionRadians == NULL ||
            referenceAngularVelocityRadPerSecond == NULL || !isfinite(nowSeconds) ||
            nowSeconds < 0.0f)
        {
            return POS_ROW_ERROR;
        }

        // 五次求值的角加速度输出，rad/s²；查询函数仅接收该辅助结果。
        float acceleration = 0.0f;
        if (nowSeconds < plan->startTimeSeconds)
        {
            *referencePositionRadians = plan->homePositionRadians;
            *referenceAngularVelocityRadPerSecond = 0.0f;
            return POS_ROW_WAIT;
        }
        if (nowSeconds < plan->hitTimeSeconds)
        {
            // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出角度、角速度、角加速度，无返回值。
            EvaluateQuinticTrajectory(
                plan->homePositionRadians, 0.0f, plan->hitPositionRadians,
                plan->hitAngularVelocityRadPerSecond,
                plan->hitTimeSeconds - plan->startTimeSeconds,
                nowSeconds - plan->startTimeSeconds, referencePositionRadians,
                referenceAngularVelocityRadPerSecond, &acceleration);
            return POS_ROW_SWING;
        }
        if (nowSeconds < plan->endTimeSeconds)
        {
            // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出角度、角速度、角加速度，无返回值。
            EvaluateQuinticTrajectory(
                plan->hitPositionRadians, plan->hitAngularVelocityRadPerSecond,
                plan->endPositionRadians, 0.0f,
                plan->endTimeSeconds - plan->hitTimeSeconds,
                nowSeconds - plan->hitTimeSeconds, referencePositionRadians,
                referenceAngularVelocityRadPerSecond, &acceleration);
            return POS_ROW_FOLLOW;
        }

        *referencePositionRadians = plan->endPositionRadians;
        *referenceAngularVelocityRadPerSecond = 0.0f;
        return POS_ROW_HOLD;
    }

    // 构造一般划杆规划器：无参数；清零累计位置、速度并初始化阶段与双臂增益，无返回值。
    GeneralRowPlanner::GeneralRowPlanner()
    {
        // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
        Reset();
    }

    // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
    void GeneralRowPlanner::Reset()
    {
        reference_position_ = 0.0f;
        reference_velocity_ = 0.0f;
        trajectory_stage_ = RowMotionStage::Accelerating;
        // 源首次Step必然写加速段增益；这里使用同一组数值初始化实例状态。
        arm1_gains_.kp = 3.327241f;  // ARM1位置比例增益，沿用原数值尺度。
        arm1_gains_.kd = 0.00005f;  // ARM1位置微分增益。
        arm2_gains_.kp = 3.5f;  // ARM2位置比例增益，与ARM1独立。
        arm2_gains_.kd = 0.00005f;  // ARM2位置微分增益。
    }

    // 推进一般划杆：无参数；按固定 5 ms 更新速度、积分位置，返回本周期参考和失能意图。
    GeneralRowReference GeneralRowPlanner::Step()
    {
        const float targetVelocity = 68.0f;  // 原946行：目标速度，圈/秒。
        const float accelerationDistance = 4.0f;  // 原947行：加速段行程，圈。
        const float constantSpeedDistance = 1.5f;  // 原948行：恒速段行程，圈。
        const float timeStepSeconds = 0.005f;  // 原949行：每次推进5ms。
        const float accelerationTimeSeconds =
        2.0f * accelerationDistance / targetVelocity;  // 原950–951行：加速时长。
        const float acceleration = targetVelocity / accelerationTimeSeconds;  // 原952–954行。
        bool disable_requested = false;  // 仅本次原STOPPING分支产生失能意图。
        switch (trajectory_stage_)
        {
            case RowMotionStage::Accelerating:
            reference_velocity_ += acceleration * timeStepSeconds;
            arm1_gains_.kp = 3.327241f;
            arm1_gains_.kd = 0.00005f;
            arm2_gains_.kp = 3.5f;
            arm2_gains_.kd = 0.00005f;
            if (reference_velocity_ >= targetVelocity)
            {
                reference_velocity_ = targetVelocity;
                trajectory_stage_ = RowMotionStage::ConstantSpeed;
            }
            break;
            case RowMotionStage::ConstantSpeed:
            arm1_gains_.kp = 3.6814f;
            arm1_gains_.kd = 0.001f;
            arm2_gains_.kp = 3.5f;
            arm2_gains_.kd = 0.001f;
            if (reference_position_ >= accelerationDistance + constantSpeedDistance)
            {
                trajectory_stage_ = RowMotionStage::Stopping;
            }
            break;
            case RowMotionStage::Stopping:
            reference_velocity_ = 0.f;
            disable_requested = true;  // 原991–992行CAN失能由调用方消费，不在数学核心发送。
            // 原993–996行：-1表示不发送对应增益，不是负的实际控制增益。
            arm1_gains_.kp = -1;
            arm1_gains_.kd = -1;
            arm2_gains_.kp = -1;
            arm2_gains_.kd = -1;
            trajectory_stage_ = RowMotionStage::Complete;
            break;
            default:
            break;
        }
        // 原1004行：即使恒速段本次刚切入STOPPING，仍按非零速度积分一帧。
        reference_position_ += reference_velocity_ * timeStepSeconds;
        return
        {
            reference_position_, reference_velocity_, arm1_gains_, arm2_gains_,
            trajectory_stage_, disable_requested
        }
        ;
    }

    // 映射逻辑划杆位置：logical_position_turns 为逻辑圈数，arm1_gains/arm2_gains 为各臂独立增益；返回两臂反向位置及原样增益。
    RowArmTargets MapLogicalRowTurns(float logical_position_turns,
        RowArmGains arm1_gains, RowArmGains arm2_gains)
    {
        return
        {
            logical_position_turns, -logical_position_turns - 0.009f,
            arm1_gains, arm2_gains
        }
        ;
    }

    // 映射累计行程：position_sum_turns 为累计圈数，arm_start_turns 为准备零点（圈），arm1_gains/arm2_gains 为独立增益；返回带准备零点和双臂偏置的目标。
    RowArmTargets MapGeneralRowTurns(float position_sum_turns, float arm_start_turns,
        RowArmGains arm1_gains, RowArmGains arm2_gains)
    {
        // 保留原表达式顺序；ARMSTART来自调用方而非本模块补数。
        return
        {
            -arm_start_turns + position_sum_turns,
            arm_start_turns - position_sum_turns - 0.009f, arm1_gains, arm2_gains
        }
        ;
    }

}
// namespace planner
