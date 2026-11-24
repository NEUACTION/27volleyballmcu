# 文件作用：声明参考工程心跳容器；当前只有mcu_to_pc_hb由USB反馈更新。
uint8 mcu_hb  # 兼容保留，当前不更新。
uint8 mcu_to_pc_hb  # 主控5ms执行心跳，8位循环计数；不代表所有电机在线。
uint8 arm1_hb  # ARM1独立心跳预留，当前不更新。
uint8 arm2_hb  # ARM2独立心跳预留，当前不更新。
uint8 plate_hb  # 发球盘独立心跳预留，当前不更新。
uint8 slide_hb  # 丝杆独立心跳预留，当前不更新。
uint8 leg1_hb  # 接球腿1心跳预留，当前不更新。
uint8 leg2_hb  # 接球腿2心跳预留，当前不更新。
uint8 leg3_hb  # 接球腿3心跳预留，当前不更新。
uint8 leg4_hb  # 接球腿4心跳预留，当前不更新。
uint8 wheel1_hb  # 后续CAN1舵轮1预留，当前不更新。
uint8 wheel2_hb  # 后续CAN1舵轮2预留，当前不更新。
uint8 wheel3_hb  # 后续CAN1舵轮3预留，当前不更新。
uint8 wheel4_hb  # 后续CAN1舵轮4预留，当前不更新。
