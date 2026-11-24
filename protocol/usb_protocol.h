/* 上下位机共享帧定义：沿用 SP/QR + CRLF，不在此头文件执行任何规划。
 * 下发位置=圈、速度=圈/秒、电流=A；增益量纲由各轴适配层保持原实测编码。
 * 状态字节索引以整个帧的绝对起点0计算；数据槽均为4字节。
 */
#ifndef VOLLEYBALL_USB_PROTOCOL_H
#define VOLLEYBALL_USB_PROTOCOL_H

#define VB_MECH_AXIS_COUNT 8U /* 机构电机：双臂、盘、丝杆、四腿；复位事务按这个分组。 */
#define VB_WHEEL_COUNT 4U /* 四个舵轮：CAN1、驱动号1..4，运动同步、复位各自独立。 */
#define VB_AXIS_COUNT (VB_MECH_AXIS_COUNT + VB_WHEEL_COUNT) /* 帧内全部电机12台。 */
#define VB_TX_CONTROL_BYTES 29U /* PC->MCU控制/预留字节数。 */
#define VB_RX_CONTROL_BYTES 29U /* MCU->PC控制/预留字节数。 */
#define VB_COMMAND_FIELDS 7U /* 每轴位置、速度、电流、位置kp/kd、速度kp/ki。 */
#define VB_FEEDBACK_FIELDS 3U /* 每轴实际位置、速度、电流。 */
#define VB_COMMAND_SLOTS (1U + VB_AXIS_COUNT * VB_COMMAND_FIELDS) /* 下发槽数（个），首槽为uint32周期计数。 */
#define VB_FEEDBACK_SLOTS (1U + VB_AXIS_COUNT * VB_FEEDBACK_FIELDS) /* 反馈槽数（个），首槽为uint32周期计数。 */
#define VB_COMMAND_BYTES (4U + VB_TX_CONTROL_BYTES + 4U * VB_COMMAND_SLOTS) /* 含SP及CRLF的下发整帧字节数。 */
#define VB_FEEDBACK_BYTES (4U + VB_RX_CONTROL_BYTES + 4U * VB_FEEDBACK_SLOTS) /* 含QR及CRLF的反馈整帧字节数。 */
#define VB_COMMAND_DATA_OFFSET (2U + VB_TX_CONTROL_BYTES) /* 下发数据区相对帧首的字节偏移。 */
#define VB_FEEDBACK_DATA_OFFSET (2U + VB_RX_CONTROL_BYTES) /* 反馈数据区相对帧首的字节偏移。 */

#define VB_RUN_BYTE 2U /* bit0:PC运行请求，其余保留。 */
#define VB_MODE_BYTE 3U /* 两字节，每字节四轴、每轴两位；覆盖机构轴0..7。 */
#define VB_CLEAN_BYTE 5U /* 一字节，每轴一位清积分请求；覆盖机构轴0..7。 */
#define VB_RESET_REQUEST_BYTE 6U /* 所选子模式请求的机构轴位掩码；MCU将双臂请求成对处理。 */
#define VB_RESET_ID_BYTE 7U /* 7..10字节，小端uint32复位事务号，同号不重复复位。 */
#define VB_WHEEL_MODE_BYTE 11U /* 舵轮模式：四轮各两位，轮1占bit0-1。 */
#define VB_WHEEL_CLEAN_BYTE 12U /* 舵轮清积分位：四轮各一位。 */
#define VB_WHEEL_RESET_REQUEST_BYTE 13U /* 舵轮复位请求位：四轮各一位，与事务号共用。 */
#define VB_HEARTBEAT_BYTE 2U /* MCU反馈心跳计数。 */
#define VB_ERROR_BYTE 3U /* 接下来的八字节分别为机构八轴错误。 */
#define VB_ENABLE_BYTE 11U /* 机构八轴使能状态位。 */
#define VB_RESET_BYTE 12U /* 机构复位准备位；bit1特例=ARM1真实复位+ARM2置零入队，非ARM2回报。 */
#define VB_PS3_BYTE 13U /* 仅发球使用的ps_num[3]原值，不翻转电平。 */
#define VB_READY_BYTE 14U /* 机构复位准备+真实新模式/使能确认；ARM2复位准备使用bit1特殊语义。 */
#define VB_RESET_ACK_BYTE 15U /* 15..18字节，小端uint32，所选复位准备流程完成才回显；ARM2见bit1。 */
#define VB_WHEEL_ERROR_BYTE 19U /* 19..22字节，四个舵轮各自的错误码。 */
#define VB_WHEEL_ENABLE_BYTE 23U /* 舵轮使能状态位：四轮各一位。 */
#define VB_WHEEL_RESET_BYTE 24U /* 舵轮真实复位准备位：四轮各一位。 */
#define VB_WHEEL_READY_BYTE 25U /* 舵轮复位+模式+使能确认位：四轮各一位。 */

/* VbAxisIndex为无量纲协议槽序号，不是CAN节点ID或电机电气地址。
 * VB_ARM1/VB_ARM2分别标识双臂第一/第二反馈槽，VB_PLATE为盘槽，VB_SLIDE为丝杆槽；
 * VB_LEG1/VB_LEG2/VB_LEG3/VB_LEG4分别为四条腿的独立槽，不推断其现场方位；
 * VB_WHEEL1..VB_WHEEL4为四个舵轮槽，挂在CAN1、驱动号1..4，运动同步但复位独立。
 * 槽序号同时对应逐轴状态掩码的位序，不能合并双臂或四腿实测反馈。 */
enum VbAxisIndex
{
    VB_ARM1 = 0, VB_ARM2, VB_PLATE, VB_SLIDE,
    VB_LEG1, VB_LEG2, VB_LEG3, VB_LEG4,
    VB_WHEEL1, VB_WHEEL2, VB_WHEEL3, VB_WHEEL4
}
;

/* 编译期确认轴数拆分与帧长一致；两端改轴数时必须同时更新本头和固件断言。 */
#if defined(__cplusplus)
static_assert(VB_AXIS_COUNT == 12U, "protocol now carries 8 mechanism axes plus 4 wheels");
static_assert(VB_COMMAND_BYTES == 373U, "command frame must be 373 bytes for 12 axes");
static_assert(VB_FEEDBACK_BYTES == 181U, "feedback frame must be 181 bytes for 12 axes");
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(VB_AXIS_COUNT == 12U, "protocol now carries 8 mechanism axes plus 4 wheels");
_Static_assert(VB_COMMAND_BYTES == 373U, "command frame must be 373 bytes for 12 axes");
_Static_assert(VB_FEEDBACK_BYTES == 181U, "feedback frame must be 181 bytes for 12 axes");
#endif

#endif
