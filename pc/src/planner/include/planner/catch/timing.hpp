// 文件作用：接球轨迹公共时间数据：保存加速、匀速、减速及总时长，单位为秒。 声明公共类型与接口。
#pragma once

namespace planner
{
    // 三种 Catch 轨迹共用的分段时间数据；InitThetaProfile 写入，周期调用方直接读取。
    struct CatchTiming
    {
        double acceleration_seconds = 0.0;  // 加速段时长，s。
        double constant_seconds = 0.0;      // 高度匀速段时长，s。
        double deceleration_seconds = 0.0;  // 减速段时长，s。
        double total_seconds = 0.0;         // 三段总时长，按原从左到右的求和顺序。
    }
    ;
}
// namespace planner
