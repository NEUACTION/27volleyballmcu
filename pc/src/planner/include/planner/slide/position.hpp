/* 文件作用：声明历史丝杆预设规划器及显式目标/步长配置规划器，两种独立API。
 * 历史预设按5ms取目标，单位圈；显式配置接口由调用方统一位置单位、步长和调用周期。
 * 两种接口都只计算当前目标，不操作串口、CAN或电机，不存整段轨迹，不推算毫米行程。
 */
#pragma once

#include <optional>

namespace planner
{

    // 仅标识原GetSlidePosition历史实验预设，不等同于新的上手/下手业务分类。
    enum class HistoricalSlideProfile
    {
        Way0High,      // WAY=0 GetSlidePositionHigh，user_task.c:1045–1066。
        Way0Low,       // WAY=0 GetSlidePositionLow，user_task.c:1071–1082。
        Way1High,      // WAY=1 GetSlidePositionHigh，user_task.c:3319–3332。
        Way1Low,       // WAY=1 GetSlidePositionLow，user_task.c:3337–3347。
        Way1HighDown   // WAY=1 GetSlidePositionHighDown，user_task.c:3352–3362。
    }
    ;

    // 历史实验数据只通过本显式接口查询，不自动成为其它运动分类的默认参数。
    struct HistoricalSlidePreset
    {
        float target_position_turns;  // 原目标位置（转）；不推算成毫米或机械限位。
        float increment_per_call_turns;  // 每次5ms调用的增量，原35/200或40/200。
    }
    ;

    // 显式正向定步长配置接口；独立于历史预设，不宣称迁移不存在的源函数。
    struct ScrewPositionConfig
    {
        float target_position;  // 待确认的丝杆目标位置，禁止从其它函数猜测。
        float increment_per_call;  // 每次 Step 的位置增量，不隐含周期或传动换算。
    }
    ;

    // 显式目标的正向位置规划器，与五种历史丝杆模式分开保存状态。
    class ScrewPositionPlanner
    {
        public:
        /* 中文名：配置丝杆正向步进。
         * 作用：config指定目标和正向步长，initial_position指定软件初始位置，单位由调用者统一；
         * 返回是否配置成功。输入须有限、步长>0、初始位置不超过目标；失败会清除旧配置。
         */
        bool Configure(ScrewPositionConfig config, float initial_position);
        /* 中文名：计算配置丝杆的下一步位置。
         * 作用：无参数，按步长增加内部位置并在目标处保持，返回同配置单位的位置；
         * 未配置返回nullopt，不读取硬件反馈，也不自行假定调用周期。
         */
        std::optional<float> Step();

        private:
        std::optional<ScrewPositionConfig> config_;  // 当前有效目标与步长；空表示不可输出。
        float current_position_ = 0.0f;  // 软件累计位置，单位同config；每次Configure明确设置。
    }
    ;

    /* 中文名：查询历史丝杆参数。
     * 作用：profile选择当前预设表中的模式，返回该模式目标圈数和每周期增量；
     * 非法枚举返回nullopt。查询不改变任何规划实例，也不向电机发指令。
     */
    std::optional<HistoricalSlidePreset> GetHistoricalSlidePreset(HistoricalSlideProfile profile);

    // 一个实例对应一个历史函数的独立静态累计位置；切换预设应使用另一实例。
    // 调用方每5ms现算并发送Step返回的一帧，本类不存点列、不调度或发送总线。
    class HistoricalSlidePlanner
    {
        public:
        /* 中文名：创建指定模式的丝杆规划器。
         * 作用：将profile（历史模式）保存到本实例；累计位置由成员初值设为0圈。
         * 创建对象不会复位电驱、发送目标或使丝杆运动。
         */
        explicit HistoricalSlidePlanner(HistoricalSlideProfile profile);
        /* 中文名：重置丝杆软件累计位置。
         * 作用：将本实例的累计位置清回0圈，使后续Step重新计算一次行程；不改变模式，
         * 不读取真实位置、不向驱动发送复位命令。无参数、无返回值。
         */
        void Reset();
        /* 中文名：计算丝杆本周期位置。
         * 作用：按所选模式增加一次位移，返回本周期目标（圈）；超目标时只钳制输出。
         * 非法模式返回nullopt；每次调用代表一个5ms逻辑周期，不自行计时或发帧。
         */
        std::optional<float> Step();

        private:
        HistoricalSlideProfile profile_;  // 固定的源WAY和函数身份，不解释成新动作类型。
        float current_position_ = 0.0f;  // 软件累计位置（圈），新对象从0开始；超目标后内部仍累计。
    }
    ;

}
// namespace planner
