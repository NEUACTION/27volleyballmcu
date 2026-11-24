# 文件作用：声明单轴目标/反馈公用类型；下行传七个数值槽和模式/clean，上行只更新实测量、故障与使能。
float32 motor_pos  # 位置：下行目标、上行实测，单位圈；PVT仅在显式实验许可下使用。
float32 motor_vel  # 速度：下行目标、上行实测，单位圈/秒。
float32 motor_pos_kp  # 下行位置比例增益，PC沿用原机构尺度，MCU按具体轴适配。
float32 motor_pos_kd  # 下行位置微分增益，PC沿用原机构尺度，MCU按具体轴适配。
float32 motor_vel_kp  # 下行速度比例增益；上行不回传该值。
float32 motor_vel_ki  # 下行速度积分增益；上行不回传该值。
float32 motor_cur  # 下行电流目标/上行实测电流，A。
float32 motor_ex_cur  # 兼容保留附加电流字段，当前USB不编码或更新。
uint8 controlmode  # 下行模式：0失能、1速度、2位置、3实验PVT；上行不更新。
uint8 error  # 上行驱动故障字节；0表示当前反馈未报告故障，不等同在线确认。
uint8 enablestate  # 上行使能回报：0失能、1使能；不是下行运行开关。
float32 motor_vel_limit  # 兼容保留速度限制字段，当前USB不编码或更新。
bool clean  # 下行清积分请求；位置/PVT按请求上升沿处理，速度模式沿用驱动接口语义。
