// 文件作用：WAY1 历史划杆变体：保留各模式独立状态、准备角推进和 FIVE1 调度，输出软件控制意图。 实现轨迹计算与状态更新。
#include "planner/serve/arm_legacy.hpp"
#include <cmath>
#include <cstddef>
#include <climits>
namespace planner
{
    namespace legacy_way1
    {
        using std::isfinite;
        namespace
        {
            // 一般划杆模式预设：保存模式编号、速度、行程与两臂映射参数。
            struct GeneralPreset
            {
                int mode; // 原模式值；同时确定独立状态槽。
                float targetVelocity, s1, s2; // 圈/s、加速行程/匀速行程（圈）。
                float start, direction, offset; // 准备角（圈）、方向、arm2额外零偏（圈）。
            }
            ;
            // 七种一般划杆预设；数组顺序同时决定各模式的独立状态槽。
            constexpr GeneralPreset kGeneralPresets[] =
            {
                {HIGH,50.f,4.f,1.50f,STARTHIGH,1.f,0.f}, {HIGHDOWN,57.f,4.3f,1.70f,STARTHIGHDOWN,1.f,0.f},
                {HIGHDOWN117,60.f,4.3f,1.80f,STARTHIGHDOWN,1.f,0.009f},
                {HIGHDOWN1110,45.f,4.3f,1.80f,STARTHIGHDOWN,1.f,0.009f},
                {LOW118A,45.f,4.3f,1.80f,STARTLOW118A,-1.f,0.f},
                {LOW118B,57.f,4.3f,1.80f,STARTLOW118B,-1.f,0.f},
                {LOW119,64.f,4.3f,1.80f,STARTLOW119,-1.f,0.f},
            }
            ;
        }
        // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
        void RowVariants::reset()
        {
            *this = RowVariants{};
        }
        // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
        RowOutput RowVariants::stepGeneral(int index)
        {
            // 当前模式对应的只读预设，决定速度、行程或增益规则。
            const auto& preset = kGeneralPresets[index];
            // 当前预设目标速度，圈/s；同声明的 s1、s2 为加速和匀速行程（圈）。
            const float targetVelocity = preset.targetVelocity, s1 = preset.s1, s2 = preset.s2;
            // 本模式独立的一般划杆状态引用，本次直接更新该槽。
            GeneralState& state = general_[index];
            const float dt = 0.005f; // s；保持原先先更新速度、后积分位置的次序
            // 一般划杆加速段时长，s。
            const float t1 = 2.0f*s1/targetVelocity;
            const float a1 = targetVelocity/t1; // 圈/s²
            // 本周期是否经过停止分支并要求失能。
            bool disable = false;
            switch(state.stage)
            {
                case ROW_MOTION_STAGE_ACCELERATING:
                state.vel += a1*dt;
                arm1_.pos_kp = 3.327241f; arm1_.pos_kd = 0.00005f;
                arm2_.pos_kp = 3.1202f; arm2_.pos_kd = 0.00005f;
                if(state.vel >= targetVelocity)
                {
                    state.vel = targetVelocity; state.stage = ROW_MOTION_STAGE_CONSTANT_SPEED;
                }
                break;
                case ROW_MOTION_STAGE_CONSTANT_SPEED:
                arm1_.pos_kp = 3.6814f; arm1_.pos_kd = 0.00005f;
                arm2_.pos_kp = 3.1202f; arm2_.pos_kd = 0.00005f;
                if(state.referencePosition >= s1+s2) state.stage = ROW_MOTION_STAGE_STOPPING;
                break;
                case ROW_MOTION_STAGE_STOPPING:
                state.vel = 0.f; disable = true;
                arm1_.pos_kp = arm1_.pos_kd = arm2_.pos_kp = arm2_.pos_kd = -1;
                state.stage = ROW_MOTION_STAGE_COMPLETE;
                break;
                default: break;
            }
            state.referencePosition += state.vel*dt;
            // start 为准备位置（圈），direction 为正反方向乘数，offset 为 ARM2 额外位置偏置（圈）。
            const float start = preset.start, direction = preset.direction, offset = preset.offset;
            arm1_.positionRevolutions = start+direction*state.referencePosition;
            arm2_.positionRevolutions = -arm1_.positionRevolutions-offset;
            arm1_.velocityRps = direction*state.vel; arm2_.velocityRps = -arm1_.velocityRps;
            // 本周期返回值，汇集双臂参考、状态和控制意图。
            RowOutput out;
            out.arm1 = arm1_; out.arm2 = arm2_; out.referencePosition = state.referencePosition;
            out.vel = state.vel; out.stage = state.stage; out.disableIntent = disable;
            // 对应原 ControlArmPosition2：停止分支发失能后，仍会执行下方 PosCtrl。
            out.positionIntent = true;
            return out;
        }
        // 推进 HIGH 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionHighTrajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(0);
        }
        // 推进 HIGHDOWN 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionHighDownTrajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(1);
        }
        // 推进 HIGHDOWN117 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionHighDown117Trajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(2);
        }
        // 推进 HIGHDOWN1110 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionHighDown1110Trajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(3);
        }
        // 推进 LOW118A 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionLow118ATrajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(4);
        }
        // 推进 LOW118B 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionLow118BTrajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(5);
        }
        // 推进 LOW119 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
        RowOutput RowVariants::GetRowPositionLow119Trajectory()
        {
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            return stepGeneral(6);
        }
        // 一般划杆分派：mode 选择历史模式；返回该模式推进一周期后的参考，未知模式返回无位置意图的默认结果。
        RowOutput RowVariants::GetRowPositionReference(int mode)
        {
            // index 遍历七种一般划杆预设以匹配模式编号。
            for(int index = 0; index<7; ++index)
            {
                if(kGeneralPresets[index].mode == mode) return stepGeneral(index);
            }
            return RowOutput{}; // 原未知模式不发送位置意图。
        }
        // 划杆模式分派：mode 选择一般轨迹或 FIVE1；返回本周期双臂参考和控制意图。
        RowOutput RowVariants::ControlArmPosition2(int mode)
        {
            if(mode == FHIGHDOWN1110) return ControlArmPosition2HighDown1110();
            // 一般划杆分派：mode 选择历史模式；返回该模式推进一周期后的参考，未知模式返回无位置意图的默认结果。
            return GetRowPositionReference(mode);
        }

        // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
        RowOutput RowVariants::stepPreparation(int index, float startPosition)
        {
            float& referencePosition = preparationPosition_[index]; // 圈，原 static 初值 0
            // 推进前准备参考位置，圈；用于差分求速度。
            const float previousPosition = referencePosition;
            // 保留原多个 if 的先后次序，不改成 else if 或基于实测位置的重新规划。
            if(referencePosition>startPosition)
            {
                referencePosition = referencePosition-0.05f;
                if(referencePosition <= startPosition) referencePosition = startPosition;
            }
            if(referencePosition <= startPosition) referencePosition = startPosition;
            arm1_.positionRevolutions = referencePosition;
            arm2_.positionRevolutions = -referencePosition; // 准备阶段原实现没有 -0.009 圈零偏
            arm1_.velocityRps = (referencePosition-previousPosition)/ROW_DT_SECONDS;
            arm2_.velocityRps = -arm1_.velocityRps; // 仅补充参考差分速度，原 PosCtrl 不发送速度字段
            // 本周期返回值，汇集双臂参考、状态和控制意图。
            RowOutput out;
            out.arm1 = arm1_; out.arm2 = arm2_;
            out.referencePosition = referencePosition; out.vel = arm1_.velocityRps;
            out.positionOk = positionOk_;
            return out; // UpdateRowPosition* 原来只更新目标，不发位置/使能/失能
        }
        // 更新 HIGH 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionHigh()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(0,kGeneralPresets[0].start);
        }
        // 更新 HIGHDOWN 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionHighDown()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(1,kGeneralPresets[1].start);
        }
        // 更新 HIGHDOWN117 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionHighDown117()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(2,kGeneralPresets[2].start);
        }
        // 更新 HIGHDOWN1110 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionHighDown1110()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(3,kGeneralPresets[3].start);
        }
        // 更新 LOW118A 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionLow118A()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(4,kGeneralPresets[4].start);
        }
        // 更新 LOW118B 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionLow118B()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(5,kGeneralPresets[5].start);
        }
        // 更新 LOW119 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
        RowOutput RowVariants::UpdateRowPositionLow119()
        {
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            return stepPreparation(6,kGeneralPresets[6].start);
        }
        // 准备角控制：mode 选择历史模式，arm1MeasuredRevolutions 为 ARM1 实测位置（圈）；累计到位次数，返回准备目标和本周期控制意图。
        RowOutput RowVariants::ControlArmPosition1(int mode, float arm1MeasuredRevolutions)
        {
            // 本周期返回值，汇集双臂参考、状态和控制意图。
            RowOutput out;
            // 当前模式对应的预设或状态槽索引，用于选择相应参数和独立状态。
            int index = -1;
            // candidate 为准备角模式查找的候选预设索引，范围 0 至 6。
            for(int candidate = 0; candidate<7; ++candidate)
            {
                if(kGeneralPresets[candidate].mode == mode) index = candidate;
            }
            // F 模式共享1110准备位置，但使用第八个独立到位计数。
            if(mode == FHIGHDOWN1110) index = 3;
            if(index<0)
            {
                positionOk_ = false;
                out.arm1 = arm1_; out.arm2 = arm2_;
                return out;
            }
            // 当前模式的准备目标位置，圈。
            const float startPosition = kGeneralPresets[index].start;
            out = stepPreparation(index,startPosition);
            if(mode == FHIGHDOWN1110) index = 7;
            // ARM1 实测位置相对准备目标的偏差，圈；随后转为绝对值供到位计数。
            float error = arm1MeasuredRevolutions-startPosition;
            if(error<0.f) error = -error; // 原 AbsoluteValue 顺序；不新增非有限输入安全规则
            if(error <= 0.02f)
            {
                ++readyCount_[index];
                if(readyCount_[index] >= 200)
                {
                    readyCount_[index] = 201;
                    positionOk_ = true;
                }
            }
            // 原误差超限没有 else：不会清 zCount，也不会清已置位 positionOk。
            out.positionOk = positionOk_; out.readyCount = readyCount_[index];
            out.positionIntent = true;
            // 原 PosCtrl 直接传常量，不写 armTar 的共享增益；因此只覆盖本次输出。
            out.arm1.pos_kp = out.arm2.pos_kp = 1.2f;
            out.arm1.pos_kd = out.arm2.pos_kd = -1;
            return out;
        }

        // 以下三个数学函数保留原表达式及校验顺序。
        /**
         * @brief 计算五次轨迹位置(rad)、速度(rad/s)、加速度(rad/s²)，时间单位 s。
         */
        // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出 rad、rad/s、rad/s²，无返回值。
        void EvaluateFive1QuinticState(float startPosition, float startVelocity,
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
         * @brief 校验计划并采样检查起挥段；最大速度 rad/s、最大加速度 rad/s²、预留时间 s。
         */
        // 规划 FIVE1：plan 输入端点并接收起止时间及有效标记，nowSeconds 为当前时刻（s），maxSpeed/maxAcceleration 为 rad/s 与 rad/s² 上限，minimumLead 为最少预留时间（s）；返回校验及首段采样是否通过。
        bool PlanFive1Trajectory(Five1Plan_t *plan, float nowSeconds,
            float maxSpeed, float maxAcceleration,
            float minimumLead)
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
                !isfinite(maxSpeed) || maxSpeed <= 0.0f || !isfinite(maxAcceleration) ||
                maxAcceleration <= 0.0f || !isfinite(minimumLead) ||
                minimumLead < 0.0f || plan->hitAngularVelocityRadPerSecond == 0.0f ||
                fabsf(plan->hitAngularVelocityRadPerSecond) > maxSpeed)
            {
                return false;
            }

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
                plan->startTimeSeconds - nowSeconds < minimumLead)
            {
                return false;
            }

            // 首段采样比较时允许的浮点速度误差，rad/s。
            const float speedTolerance = fmaxf(1.0f, maxSpeed) * 1e-5f + 1e-4f;
            // 首段起挥时长，s，用于采样计算。
            const float durationSeconds = plan->hitTimeSeconds - plan->startTimeSeconds;
            // i 为首段 101 个等间隔采样点的索引，0 和 100 分别包含两端。
            for (int i = 0; i <= 100; ++i)
            {
                // 采样点角度，rad，用于有限性检查。
                float position = 0.0f;
                // 采样点角速度，rad/s，用于限速检查。
                float velocity = 0.0f;
                // 采样点角加速度，rad/s²，用于限加速度检查。
                float acceleration = 0.0f;
                // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出 rad、rad/s、rad/s²，无返回值。
                EvaluateFive1QuinticState(
                    plan->homePositionRadians, 0.0f, plan->hitPositionRadians,
                    plan->hitAngularVelocityRadPerSecond, durationSeconds,
                    durationSeconds * ((float)i / 100.0f), &position, &velocity,
                    &acceleration);
                if (!isfinite(position) || !isfinite(velocity) ||
                    !isfinite(acceleration) ||
                    fabsf(velocity) > maxSpeed + speedTolerance ||
                    fabsf(acceleration) > maxAcceleration + 1e-3f)
                {
                    return false;
                }
            }

            plan->valid = true;
            return true;
        }

        // 查询 FIVE1 阶段：plan 为已规划数据，nowSeconds 为同一时钟下的时刻（s）；referencePositionRadians/referenceAngularVelocityRadPerSecond 输出 rad 与 rad/s，返回阶段，错误时不改输出。
        Five1Stage EvaluateFive1Trajectory(
            const Five1Plan_t *plan, float nowSeconds, float *referencePositionRadians,
            float *referenceAngularVelocityRadPerSecond)
        {
            if (plan == NULL || !plan->valid || referencePositionRadians == NULL ||
                referenceAngularVelocityRadPerSecond == NULL || !isfinite(nowSeconds) ||
                nowSeconds < 0.0f)
            {
                return FIVE1_ERROR;
            }

            // 五次求值的角加速度输出，rad/s²；查询函数仅接收该辅助结果。
            float acceleration = 0.0f;
            if (nowSeconds < plan->startTimeSeconds)
            {
                *referencePositionRadians = plan->homePositionRadians;
                *referenceAngularVelocityRadPerSecond = 0.0f;
                return FIVE1_WAIT;
            }
            if (nowSeconds < plan->hitTimeSeconds)
            {
                // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出 rad、rad/s、rad/s²，无返回值。
                EvaluateFive1QuinticState(
                    plan->homePositionRadians, 0.0f, plan->hitPositionRadians,
                    plan->hitAngularVelocityRadPerSecond,
                    plan->hitTimeSeconds - plan->startTimeSeconds,
                    nowSeconds - plan->startTimeSeconds, referencePositionRadians,
                    referenceAngularVelocityRadPerSecond, &acceleration);
                return FIVE1_SWING;
            }
            if (nowSeconds < plan->endTimeSeconds)
            {
                // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出 rad、rad/s、rad/s²，无返回值。
                EvaluateFive1QuinticState(
                    plan->hitPositionRadians, plan->hitAngularVelocityRadPerSecond,
                    plan->endPositionRadians, 0.0f,
                    plan->endTimeSeconds - plan->hitTimeSeconds,
                    nowSeconds - plan->hitTimeSeconds, referencePositionRadians,
                    referenceAngularVelocityRadPerSecond, &acceleration);
                return FIVE1_FOLLOW;
            }

            *referencePositionRadians = plan->endPositionRadians;
            *referenceAngularVelocityRadPerSecond = 0.0f;
            return FIVE1_HOLD;
        }

        // 推进 FIVE1 实验轨迹：无参数；每次推进 5 ms，起挥后恒速跟随，终点输出失能意图；返回本周期双臂参考。
        RowOutput RowVariants::ControlArmPosition2HighDown1110()
        {
            const float twoPi = 6.28318530718f; // rad/圈
            const float targetSpeedRps = 62.5f; // 圈/s
            const float minimumLeadSeconds = 0.035f; // s
            // 本周期返回值，汇集双臂参考、状态和控制意图。
            RowOutput out;
            out.arm1 = arm1_; out.arm2 = arm2_;
            out.referencePosition = arm1_.positionRevolutions;
            out.stage = five1_.complete ? FIVE1_HOLD : FIVE1_WAIT;
            if(five1_.planFailed || five1_.complete)
            {
                out.failed = five1_.planFailed;
                if(out.failed) out.stage = FIVE1_ERROR;
                return out; // 原实现完成/失败后不再发送任何命令
            }
            if(five1_.loopCount<UINT32_MAX) ++five1_.loopCount;
            // 由本实例累计调用次数换算的轨迹时刻，s。
            const float nowSeconds = static_cast<float>(five1_.loopCount)*0.005f;
            // 准备位置换算后的角度，rad。
            const float homePositionRadians = STARTHIGHDOWN*twoPi;
            // 准备位置加起挥行程得到的击球角度，rad。
            const float hitPositionRadians = homePositionRadians+4.4f*twoPi;
            // 击球位置加跟随行程得到的结束角度，rad。
            const float endPositionRadians = hitPositionRadians+1.8f*twoPi;
            // 目标圈速换算后的击球角速度，rad/s。
            const float hitAngularVelocityRadPerSecond = targetSpeedRps*twoPi;
            // 由起挥角位移及击球角速度计算的首段时长，s。
            const float timeToHitSeconds = 2.0f*fabsf(hitPositionRadians-homePositionRadians)/fabsf(hitAngularVelocityRadPerSecond);
            if(!five1_.planReady)
            {
                // 本实例保存的 FIVE1 计划引用；首次调用填写端点并规划。
                auto& plan = five1_.plan;
                plan.homePositionRadians = homePositionRadians;
                plan.hitPositionRadians = hitPositionRadians;
                plan.hitAngularVelocityRadPerSecond = hitAngularVelocityRadPerSecond;
                plan.endPositionRadians = endPositionRadians;
                plan.hitTimeSeconds = nowSeconds+timeToHitSeconds+minimumLeadSeconds+0.005f;
                if(PlanFive1Trajectory(&plan,nowSeconds,targetSpeedRps*twoPi,9000.0f,minimumLeadSeconds)) five1_.planReady = true;
                else
                {
                    five1_.planFailed = true; out.failed = true; out.stage = FIVE1_ERROR;
                    return out;
                }
            }
            // 击球后按固定击球速度走完跟随行程的时间，s。
            const float cruiseDuration = (endPositionRadians-hitPositionRadians)/hitAngularVelocityRadPerSecond;
            if(nowSeconds >= five1_.plan.hitTimeSeconds+cruiseDuration)
            {
                arm1_.positionRevolutions = endPositionRadians/twoPi;
                arm2_.positionRevolutions = -arm1_.positionRevolutions-0.009f;
                arm1_.velocityRps = arm2_.velocityRps = 0.f;
                out.arm1 = arm1_; out.arm2 = arm2_;
                out.referencePosition = arm1_.positionRevolutions;
                out.disableIntent = true;
                // 原来转 int32_t 脉冲后下发；此处显式保留截断后对应的圈值。
                out.disablePosition1 = static_cast<std::int32_t>(arm1_.positionRevolutions*4096.0f)/4096.0f;
                out.disablePosition2 = static_cast<std::int32_t>(arm2_.positionRevolutions*4096.0f)/4096.0f;
                out.stage = FIVE1_HOLD;
                five1_.complete = true;
                return out;
            }
            // 当前求值角度（rad）与角速度（rad/s）；FOLLOW 分支会覆盖为恒速参考。
            float referencePositionRadians = 0.f, referenceAngularVelocityRadPerSecond = 0.f;
            const Five1Stage stage = EvaluateFive1Trajectory(&five1_.plan,nowSeconds,&referencePositionRadians,&referenceAngularVelocityRadPerSecond);
            out.stage = stage;
            if(stage == FIVE1_ERROR)
            {
                five1_.planFailed = true; out.failed = true;
                return out;
            }
            if(stage == FIVE1_FOLLOW)
            {
                // 原控制器有意覆盖数学求值器第二段：保持击球速度，不执行五次减速。
                referencePositionRadians = hitPositionRadians+hitAngularVelocityRadPerSecond*(nowSeconds-five1_.plan.hitTimeSeconds);
                referenceAngularVelocityRadPerSecond = hitAngularVelocityRadPerSecond;
                arm1_.pos_kp = 3.6814f; arm1_.pos_kd = 0.001f;
                arm2_.pos_kp = 3.5f; arm2_.pos_kd = 0.001f;
            }
            else
            {
                arm1_.pos_kp = 3.327241f; arm1_.pos_kd = 0.00005f;
                arm2_.pos_kp = 3.5f; arm2_.pos_kd = 0.00005f;
            }
            arm1_.positionRevolutions = referencePositionRadians/twoPi;
            arm2_.positionRevolutions = -arm1_.positionRevolutions-0.009f;
            arm1_.velocityRps = referenceAngularVelocityRadPerSecond/twoPi;
            arm2_.velocityRps = -arm1_.velocityRps;
            out.arm1 = arm1_; out.arm2 = arm2_;
            out.referencePosition = arm1_.positionRevolutions; out.vel = arm1_.velocityRps;
            out.positionIntent = true;
            return out;
        }
    }

}
// namespace planner::legacy_way1
