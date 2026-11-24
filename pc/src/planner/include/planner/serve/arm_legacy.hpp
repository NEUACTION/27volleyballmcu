// 文件作用：WAY1 历史划杆变体：保留各模式独立状态、准备角推进和 FIVE1 调度，输出软件控制意图。 声明公共类型与接口。
#pragma once
#include <cstdint>

// WAY1 原实验轨迹；不是新上手/下手配置，不包含 CAN 或硬件依赖。
namespace planner
{
    namespace legacy_way1
    {
        constexpr float ROW_DT_SECONDS = 0.005f; // row 逻辑轴周期，s
        // 历史模式编号：HIGH、HIGHDOWN、HIGHDOWN117、LOW118A/B、LOW119、HIGHDOWN1110、FHIGHDOWN1110 分别用于分派对应轨迹。
        constexpr int HIGH = 1, HIGHDOWN = 3, HIGHDOWN117 = 7, LOW118A = 8,
        LOW118B = 9, LOW119 = 10, HIGHDOWN1110 = 11, FHIGHDOWN1110 = 12;
        // 各历史模式准备位置（圈）：STARTHIGH 用于 HIGH，STARTHIGHDOWN 用于高位下打系列，STARTLOW118A/B/119 用于对应低位模式。
        constexpr float STARTHIGH = -1.75f, STARTHIGHDOWN = -2.0f,
        STARTLOW118A = -1.6f, STARTLOW118B = -1.6f, STARTLOW119 = -1.6f; // 圈
        // 原一般轨迹 stage 数值须与源 ROW_MOTION_STAGE 定义一致。
        // ACCELERATING 逐周期加速；CONSTANT_SPEED 保持速度；STOPPING 产生一次失能；COMPLETE 保持停止状态。
        enum RowMotionStage
        {
            ROW_MOTION_STAGE_ACCELERATING = 0,
            ROW_MOTION_STAGE_CONSTANT_SPEED, ROW_MOTION_STAGE_STOPPING, ROW_MOTION_STAGE_COMPLETE
        }
        ;
        // 单臂软件参考：保存圈位置、圈速度与独立驱动增益。
        struct MotorReference
        {
            float positionRevolutions = 0.0f; // 电机位置，圈；发送层自行换算 4096 脉冲/圈
            float velocityRps = 0.0f; // 电机速度参考，圈/s（普通轨迹为离散积分所用速度）
            float pos_kp = -1.0f, pos_kd = -1.0f; // 位置比例/微分增益，单位由原驱动协议定义；-1 不发对应参数。
        }
        ;
        // WAY1 本周期输出：两臂目标及调用意图；意图不代表硬件实际动作已完成。
        struct RowOutput
        {
            // ARM1 与 ARM2 的本周期参考，各自保存位置、速度和增益。
            MotorReference arm1, arm2;
            float referencePosition = 0.0f; // 一般轨迹累计位移，圈；FIVE1 为 arm1 绝对圈数
            float vel = 0.0f; // 一般轨迹正向位移速度/FIVE1 arm1 速度，圈/s
            int stage = ROW_MOTION_STAGE_ACCELERATING; // 一般轨迹 RowMotionStage 或 FIVE1 Five1Stage
            bool positionIntent = false; // 本周期原实现会发位置控制；不代表实际到位
            bool disableIntent = false; // 本周期双电机失能意图；一般轨迹失能位置原值均为 0
            float disablePosition1 = 0.0f, disablePosition2 = 0.0f; // 失能命令位置，圈
            bool failed = false; // FIVE1 原规划/求值失败；不隐式增加失能命令
            bool enableIntent = false; // 原函数本周期是否使能双臂；准备角函数实际未发使能/失能，均为 false
            bool positionOk = false; // ControlArmPosition1 原调用者共享到位标记；不是实时误差判定
            int readyCount = 0; // 当前模式累计满足 0.02 圈误差的调用次数，达到 200 后置为 201
        }
        ;
        // 单个一般划杆模式的独立累计位置、速度和阶段。
        struct GeneralState
        {
            float referencePosition = 0.0f; // 累计位移，圈
            float vel = 0.0f; // 位移速度，圈/s
            // 当前一般划杆阶段；初始为加速。
            int stage = ROW_MOTION_STAGE_ACCELERATING;
        }
        ;
        // FIVE1 两段五次计划：输入端点条件，规划后保存起止时间与有效标记。
        struct Five1Plan_t
        {
            float homePositionRadians = 0, hitPositionRadians = 0; // 输入：准备角、击球角，rad。
            float hitAngularVelocityRadPerSecond = 0; // 输入：击球瞬间角速度，rad/s。
            float endPositionRadians = 0; // 输入：跟随段结束角，rad。
            float hitTimeSeconds = 0, startTimeSeconds = 0, endTimeSeconds = 0; // hitTimeSeconds 为输入击球时刻，startTimeSeconds/endTimeSeconds 为规划输出起止时刻，s，同一时间零点。
            // 计划是否通过校验；未规划时为 false。
            bool valid = false;
        }
        ;
        // FIVE1 求值阶段：错误、等待、起挥、跟随及终点保持。
        // ERROR 表示计划或求值失败，WAIT 为起挥前，SWING 为起挥段，FOLLOW 为击球后段，HOLD 为结束保持。
        enum Five1Stage
        {
            FIVE1_ERROR = -1, FIVE1_WAIT, FIVE1_SWING, FIVE1_FOLLOW, FIVE1_HOLD
        }
        ;
        // 原数学接口；仅起挥段按 101 点检查，不增加原来不存在的安全保证。
        // 五次轨迹求值：startPosition/targetPosition 为端点角度（rad），startVelocity/targetVelocity 为端点角速度（rad/s）；durationSeconds 为段长、elapsedSeconds 为段内时间（s）；position/velocity/acceleration 输出 rad、rad/s、rad/s²，无返回值。
        void EvaluateFive1QuinticState(float startPosition, float startVelocity,
            float targetPosition, float targetVelocity, float durationSeconds,
            float elapsedSeconds, float* position, float* velocity, float* acceleration);
        // 规划 FIVE1：plan 输入端点并接收起止时间及有效标记，nowSeconds 为当前时刻（s），maxSpeed/maxAcceleration 为 rad/s 与 rad/s² 上限，minimumLead 为最少预留时间（s）；返回校验及首段采样是否通过。
        bool PlanFive1Trajectory(Five1Plan_t* plan, float nowSeconds, float maxSpeed,
            float maxAcceleration, float minimumLead);
        // 查询 FIVE1 阶段：plan 为已规划数据，nowSeconds 为同一时钟下的时刻（s）；referencePositionRadians/referenceAngularVelocityRadPerSecond 输出 rad 与 rad/s，返回阶段，错误时不改输出。
        Five1Stage EvaluateFive1Trajectory(const Five1Plan_t* plan, float nowSeconds,
            float* referencePositionRadians, float* referenceAngularVelocityRadPerSecond);
        // FIVE1 实例运行状态：保存调用时钟、计划和失败/结束锁存。
        struct Five1State
        {
            std::uint32_t loopCount = 0; // 5 ms 调用计数，饱和至 UINT32_MAX
            // 本实例五次轨迹计划，规划后供逐周期求值。
            Five1Plan_t plan;
            // planReady 表示计划已建成，planFailed 锁存失败，complete 锁存执行结束。
            bool planReady = false, planFailed = false, complete = false;
        }
        ;
        // 每个对象独立保存原函数 static 和 armTar 全局增益；同一对象不可并发调用。
        class RowVariants
        {
            public:
            // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
            void reset(); // 全实例重置；不发送命令
            // 一般划杆分派：mode 选择历史模式；返回该模式推进一周期后的参考，未知模式返回无位置意图的默认结果。
            RowOutput GetRowPositionReference(int mode); // 一般轨迹分派，单次推进 5 ms
            // 推进 HIGH 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionHighTrajectory();
            // 推进 HIGHDOWN 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionHighDownTrajectory();
            // 推进 HIGHDOWN117 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionHighDown117Trajectory();
            // 推进 HIGHDOWN1110 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionHighDown1110Trajectory();
            // 推进 LOW118A 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionLow118ATrajectory();
            // 推进 LOW118B 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionLow118BTrajectory();
            // 推进 LOW119 一般划杆：无参数；调用对应独立状态槽，返回本周期双臂参考及控制意图。
            RowOutput GetRowPositionLow119Trajectory();
            // 划杆模式分派：mode 选择一般轨迹或 FIVE1；返回本周期双臂参考和控制意图。
            RowOutput ControlArmPosition2(int mode); // 包含 FIVE1 分派及双电机方向/零偏
            // 推进 FIVE1 实验轨迹：无参数；每次推进 5 ms，起挥后恒速跟随，终点输出失能意图；返回本周期双臂参考。
            RowOutput ControlArmPosition2HighDown1110(); // FIVE1 原实验：五次起挥+匀速 follow+失能
            // 读取 FIVE1 状态：无参数；返回实例内部状态的只读引用，不推进轨迹。
            const Five1State& five1State() const
            {
                return five1_;
            }
            // 每次调用准备角推进一次 5 ms；输入实测 arm1 位置（圈），不在此读取传感器。
            // 准备角控制：mode 选择历史模式，arm1MeasuredRevolutions 为 ARM1 实测位置（圈）；累计到位次数，返回准备目标和本周期控制意图。
            RowOutput ControlArmPosition1(int mode, float arm1MeasuredRevolutions);
            // 原 UpdateRowPosition* 只改目标，不发送控制。返回 positionIntent=false。
            // 更新 HIGH 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionHigh();
            // 更新 HIGHDOWN 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionHighDown();
            // 更新 HIGHDOWN117 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionHighDown117();
            // 更新 HIGHDOWN1110 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionHighDown1110();
            // 更新 LOW118A 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionLow118A();
            // 更新 LOW118B 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionLow118B();
            // 更新 LOW119 准备目标：无参数；推进对应准备位置槽，返回目标与差分速度，positionIntent 保持 false。
            RowOutput UpdateRowPositionLow119();
            private:
            // 推进准备角：index 为准备位置槽索引，startPosition 为目标准备位置（圈）；返回一次准备目标及差分速度，位置控制意图由外层决定。
            RowOutput stepPreparation(int index, float startPosition);
            // 推进一般划杆：index 为七种预设的状态槽索引；按 5 ms 更新对应实例状态，返回两臂参考及控制意图。
            RowOutput stepGeneral(int index);
            float preparationPosition_[7] = {}; // 各 UpdateRowPosition* 原 static 参考位置，初值 0 圈
            int readyCount_[8] = {}; // 原各 case 的 zCount；F 模式单独计数，但共享 1110 准备位置
            bool positionOk_ = false; // 原调用者 positionOk 的实例化标记；有效模式误差超限不清零
            GeneralState general_[7] = {}; // 各原函数各自的累计位移、速度、阶段
            // 本实例 FIVE1 的调用时钟、计划及终止状态。
            Five1State five1_;
            MotorReference arm1_, arm2_; // 原全局 armTar 保留的增益与位置
        }
        ;
    }

}
// namespace planner::legacy_way1
