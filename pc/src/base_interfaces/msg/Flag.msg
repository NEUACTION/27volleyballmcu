# 文件作用：保留参考工程的上下位机标志类型；字段是否参与当前协议以下列说明为准。
bool get_mcu_error_flag  # 兼容保留；当前桥接不编码/更新，不能代替各轴error。
bool run_successed_flag  # 下行运行请求：true允许执行目标；false失能，仍可请求显式复位。
bool pc_error_flag  # 兼容保留；当前不编码PC故障字段。
bool only_need_start_flag  # 兼容保留；当前不作为自动启动条件。
bool read_ok_flag  # 兼容保留；当前不作为USB完整帧确认。
bool reset_flag  # 上行仅表示reset_mask==0xFF；实际当前机构复位必须核对掩码及事务ACK。
bool upr1_flag  # 兼容保留；当前不参与动作选择或USB编码。
