# 文件作用：定义小电脑到主控的当前周期目标，桥接节点按SP协议编码；不含整段轨迹数组。
Flag pcflag  # 当前仅run_successed_flag参与USB运行门控。
int32 period_cnt_pc  # ROS保留的周期字段；桥接实际编码时使用自己的31位递增计数。
uint8 reset_request_mask  # 机构轴复位位：bit0/1双臂、bit2盘、bit3丝杆、bit4..7四腿。
uint8 wheel_reset_mask  # 舵轮复位位：bit0..3对应舵轮1..4，可任意子集，与机构掩码共用事务号。
uint32 reset_request_id  # 非0事务号；同号重传不重复复位，需和上行ACK一起核对，机构轴和舵轮共用。
Motor arm1  # USB轴槽0，CAN3物理电机1：主臂。
Motor arm2  # USB轴槽1，CAN3物理电机2：从臂。
Motor plate  # USB轴槽2，CAN3物理电机3：发球盘。
Motor slide  # USB轴槽3，CAN3物理电机4：丝杆。
Motor leg1  # USB轴槽4，CAN3物理电机5：接球腿1，PVT模式下pos/vel/pos_kp/pos_kd/cur为位置/速度/kp/kd/力矩。
Motor leg2  # USB轴槽5，CAN3物理电机6：接球腿2，字段同上。
Motor leg3  # USB轴槽6，CAN3物理电机7：接球腿3，字段同上。
Motor leg4  # USB轴槽7，CAN3物理电机8：接球腿4，字段同上。
Motor wheel1  # USB轴槽8，CAN1物理电机1：舵轮1，位置环用pos/pos_kp/pos_kd/vel_kp/vel_ki/cur。
Motor wheel2  # USB轴槽9，CAN1物理电机2：舵轮2，字段同上。
Motor wheel3  # USB轴槽10，CAN1物理电机3：舵轮3，字段同上。
Motor wheel4  # USB轴槽11，CAN1物理电机4：舵轮4，字段同上。
