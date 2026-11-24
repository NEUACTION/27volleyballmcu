// 文件作用：WAY1 发球顺序调度：按调用计数锁存准备、盘和双臂动作意图，不直接操作硬件。 声明公共类型与接口。
#pragma once
#include <cstdint>

namespace planner
{
    namespace legacy_way1
    {
        // WAY1 UserTask 原计数调度；不定义传感器意义，不操作 CAN/复位/使能。
        constexpr int SEQUENCE_START_COUNT = 1000; // ps_num[3]==0 连续调用次数；原周期5 ms
        constexpr int SEQUENCE_WAITING_COUNT2 = 1400; // start 之后等待调用次数
        constexpr int GAP_TIME118A = 22; // 原 UserTask 对所有模式实际均使用此常量，不按模式替换

        // 发球顺序实例状态：保存条件累计次数及各阶段锁存。
        struct ServeSequenceState
        {
            bool start = false; // 原 start：连续字节条件达到后锁存
            int startCount = 0; // 次数；达到阈值后原赋值1001
            bool startArm = false; // 原 startArm：第一次 plate 调用后锁存
            bool start2 = false; // 原 start2：1400等待到达后锁存
            int waitingCount2 = 0; // 次数；达到阈值后原赋值1401
            bool start1 = false; // 原 start1：22次间隔之后锁存
            int waitingCount = 0; // 次数；间隔到达后原赋值23
        }
        ;
        // 本周期调度结果：透传模式并指示上层应调用的控制步骤。
        struct ServeSequenceOutput
        {
            int mode; // 仅透传调用者必选原模式；本模块不写死 LOW118A，不重新解释模式
            bool prepareIntent = false; // 原 if(!positionOk) 调用 ControlArmPosition1
            bool plateIntent = false; // 原本周期调用 PlateCtrl(mode)
            bool rowIntent = false; // 原本周期调用 ControlArmPosition2(mode)
            bool continueIntent = false; // 原 else continue，跳过本周期后续 startArm/row 判断
        }
        ;

        // 每次调用推进一个原 UserTask 5 ms 逻辑周期；实例计数，不使用实际秒数定时。
        // 独立对象对应独立序列；没有自动重启，也不修改传入到位标记或实测字节。
        class ServeSequence
        {
            public:
            // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
            void reset(); // 全部计数/锁存恢复原 static 初值，不输出动作
            // 读取顺序状态：无参数；返回内部计数与锁存状态的只读引用。
            const ServeSequenceState& state() const
            {
                return state_;
            }
            // 精确组合版本：原 ControlArmPosition1 可能在本周期将 positionOk 置真。
            // before 用于prepare意图，after用于随后if/else if；调用方先算准备控制结果，
            // 再调此接口即可重现同周期“调用prepare且立即累计startCount”。
            // 推进发球顺序：mode 透传模式，position_ok_before_prepare/position_ok_after_prepare 为准备控制前后到位标记，ps_num3 为原始检测字节；返回本周期准备、盘、划杆及跳过后续步骤的意图。
            ServeSequenceOutput step(int mode, bool position_ok_before_prepare,
                bool position_ok_after_prepare, std::uint8_t ps_num3);
            private:
            // 本实例发球顺序的计数和锁存，不与其他实例共享。
            ServeSequenceState state_;
        }
        ;
    }

}
// namespace planner::legacy_way1
