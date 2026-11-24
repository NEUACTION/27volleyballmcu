// 文件作用：WAY1 发球顺序调度：按调用计数锁存准备、盘和双臂动作意图，不直接操作硬件。 实现轨迹计算与状态更新。
#include "planner/serve/sequence.hpp"

namespace planner
{
    namespace legacy_way1
    {
        // 重置实例：无参数；恢复累计时间、位置或调度锁存的初始状态，无返回值。
        void ServeSequence::reset()
        {
            state_ = ServeSequenceState{};
        }

        // 推进发球顺序：mode 透传模式，position_ok_before_prepare/position_ok_after_prepare 为准备控制前后到位标记，ps_num3 为原始检测字节；返回本周期准备、盘、划杆及跳过后续步骤的意图。
        ServeSequenceOutput ServeSequence::step(int mode, bool position_ok_before_prepare,
            bool position_ok_after_prepare, std::uint8_t ps_num3)
        {
            // 本周期调度结果，汇集模式和准备、盘、划杆控制意图。
            ServeSequenceOutput out;
            out.mode = mode;
            out.prepareIntent = !position_ok_before_prepare;
            // 源 UserTask 1889 默认 mode=LOW118A，仅为历史预设；此处必须由调用方选择mode。
            // 源1958–1963先执行准备角，后续1969起读取准备角函数更新后的positionOk。
            if (!state_.start && position_ok_after_prepare)
            {
                if (ps_num3 == 0) ++state_.startCount;
                else state_.startCount = 0;
                if (state_.startCount >= SEQUENCE_START_COUNT)
                {
                    state_.startCount = SEQUENCE_START_COUNT + 1;
                    state_.start = true;
                }
                // 必须保留else if：本周期即使刚置start，也不会进入1400等待分支。
            }
            else if (state_.start && position_ok_after_prepare)
            {
                if (!state_.start2)
                {
                    ++state_.waitingCount2;
                    if (state_.waitingCount2 >= SEQUENCE_WAITING_COUNT2)
                    {
                        state_.start2 = true;
                        state_.waitingCount2 = SEQUENCE_WAITING_COUNT2 + 1;
                    }
                    // 本周期刚置start2仍不执行plate；下一周期才进入下面else。
                }
                else
                {
                    out.plateIntent = true;
                    state_.startArm = true;
                }
            }
            else
            {
                // 对应原2008–2010的continue；不清计数/锁存，不执行后面的row判断。
                // 即使此前startArm/start1已为真，positionOk=false仍会跳过本周期row。
                out.continueIntent = true;
                return out;
            }

            if (state_.startArm)
            {
                // 保留先比较、后++：首个plate周期waitingCount从0变1，不立刻row。
                if (state_.waitingCount < GAP_TIME118A && !state_.start1)
                {
                    ++state_.waitingCount;
                }
                else
                {
                    state_.waitingCount = GAP_TIME118A + 1;
                    state_.start1 = true;
                }
                if (state_.start1) out.rowIntent = true;
            }
            // 准备、plate、row按源顺序执行；锁存后每周期持续输出相应意图，不是单次脉冲。
            return out;
        }
    }

}
// namespace planner::legacy_way1
