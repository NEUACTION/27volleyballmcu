/* 头文件保护宏：避免电机驱动接口重复定义。 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_FDCAN.h"
#include "string.h"

/*控制标识符*/
/* 宏：设置驱动器使能状态命令标识。 */
#define IDENTIFIER_SET_DRIVER_STATE             (0x01)
/* 宏：设置驱动器控制模式命令标识。 */
#define IDENTIFIER_SET_CTRL_MODE                (0x02)
/* 宏：设置速度环比例增益命令标识。 */
#define IDENTIFIER_SET_SPD_KP                   (0x03)
/* 宏：设置速度环积分增益命令标识。 */
#define IDENTIFIER_SET_SPD_KI                   (0x04)
/* 宏：设置位置环比例增益命令标识。 */
#define IDENTIFIER_SET_POS_KP                   (0x05)
/* 宏：设置位置环微分增益命令标识。 */
#define IDENTIFIER_SET_POS_KD                   (0x06)
/* 宏：设置目标扭矩命令标识。 */
#define IDENTIFIER_SET_TORQUE                   (0x07)
/* 宏：设置目标速度命令标识。 */
#define IDENTIFIER_SET_VEL                      (0x08)
/* 宏：设置绝对目标位置命令标识。 */
#define IDENTIFIER_SET_POS_ABS                  (0x09)
/* 宏：设置相对目标位置命令标识。 */
#define IDENTIFIER_SET_POS_RE                   (0x0A)
/* 宏：设置速度加速度命令标识。 */
#define IDENTIFIER_SET_VEL_ACC                  (0x0B)
/* 宏：设置速度减速度命令标识。 */
#define IDENTIFIER_SET_VEL_DEC                  (0x0C)
/* 宏：设置扭矩限制命令标识。 */
#define IDENTIFIER_SET_TORQUE_LIMIT             (0x0D)
/* 宏：设置速度限制命令标识。 */
#define IDENTIFIER_SET_VEL_LIMIT                (0x0E)
/* 宏：设置积分清除状态的回报/设置标识。 */
#define IDENTIFIER_SET_INTEGRAL_CLEARED         (0x0F)
/* 宏：设置位置加速度命令标识。 */
#define IDENTIFIER_SET_POS_ACC                  (0x10)
/* 宏：设置位置减速度命令标识。 */
#define IDENTIFIER_SET_POS_DEC                  (0x11)
/* 宏：请求清除积分命令标识。 */
#define IDENTIFIER_SET_CLEAR_INTEGRAL           (0x12)
/* 宏：设置 Q 轴电流限制命令标识。 */
#define IDENTIFIER_SET_CURRENTQ_LIMIT           (0x13)
/* 宏：驱动器复位命令标识。 */
#define IDENTIFIER_SET_RESET                    (0x14)
/* 宏：消除空行程命令标识；名称按现有驱动器协议保留。 */
#define IDENTIFIER_ELIMINATE_EMPTY_DIATANCE     (0x15)
/* 宏：设置前馈电流命令标识。 */
#define IDENTIFIER_SET_FEEDFORWARD_CURR         (0x16)
/* 宏：设置当前位置为零命令标识。 */
#define IDENTIFIER_SET_ZERO                     (0x17)
/*读取标识符*/
/* 宏：读取类命令的标识符偏移量；与具体读取项组合。 */
#define IDENTIFIER_READ_OFFSET                  (0x40)
/* 宏：读取加速度反馈项。 */
#define IDENTIFIER_READ_ACC                     (0x01)
/* 宏：读取扭矩反馈项。 */
#define IDENTIFIER_READ_TORQUE                  (0x02)
/* 宏：读取速度反馈项。 */
#define IDENTIFIER_READ_VEL                     (0x03)
/* 宏：读取位置反馈项。 */
#define IDENTIFIER_READ_POS                     (0x04)
/* 宏：读取编码器位置反馈项。 */
#define IDENTIFIER_READ_ENCODER_POS             (0x05)
/* 宏：读取 D 轴电压反馈项。 */
#define IDENTIFIER_READ_VOL_D                   (0x06)
/* 宏：读取 D 轴电流反馈项。 */
#define IDENTIFIER_READ_CURR_D                  (0x07)
/* 宏：读取 Q 轴电压反馈项。 */
#define IDENTIFIER_READ_VOL_Q                   (0x08)
/* 宏：读取 Q 轴电流反馈项。 */
#define IDENTIFIER_READ_CURR_Q                  (0x09)
/* 宏：读取速度环输出反馈项。 */
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT         (0x0A)
/* 宏：读取位置环输出反馈项。 */
#define IDENTIFIER_READ_POS_LOOP_OUTPUT         (0x0B)
/* 宏：读取位置指令脉冲数反馈项。 */
#define IDENTIFIER_READ_POS_EXP_PULSE           (0x0C)
/* 宏：读取负载观测器反馈项。 */
#define IDENTIFIER_READ_LOAD_OBSERVER           (0x0D)
/* 宏：俯仰速率发送数据项标识。 */
#define IDENTIFIER_PITCH_RATE_SEND              (0x0E)
/* 宏：读取风扇模式反馈项。 */
#define IDENTIFIER_READ_FAN_MODE                (0x0F)
/* 宏：读取 G 轴电流反馈项。 */
#define IDENTIFIER_READ_CURR_G                  (0x10)
/* 宏：读取复位状态反馈项。 */
#define IDENTIFIER_READ_RESET                   (0x11)
/* 宏：驱动器读请求返回消息标识。 */
#define IDENTIFIER_RETURN_MSG                   (0x12)
/* 宏：驱动器错误上报标识。 */
#define IDENTIFIER_REPORT_ERROR                 (0xEE)

/*驱动器控制模式*/
/* 宏：速度控制模式标识；普通驱动器接口使用。 */
#define VEL_MODE                                (0x01)
/* 宏：位置控制模式标识；普通驱动器接口使用。 */
#define POS_MODE                                (0x02)
/* 宏：PVT 控制模式标识；普通驱动器接口使用。 */
#define PVT_MODE                                (0x04)

/*DAMIAO电机控制标识符*/
/* 宏：达妙位置控制模式 ID；当前用于达妙位置控制接口。 */
#define DM_POS_MODE 0x100
/* 宏：达妙速度控制模式 ID；当前定义供达妙控制接口选择。 */
#define DM_VEL_MODE 0x200

// pvt控制限值 ，需与电驱方面比对数据是否相同
// PVT 位置输入最小值；原有协议注释提示需与电驱端核对，宏当前启用。
#define P_K_MIN  (-100.0f * 1000.f)  // 单位rad
// PVT 位置输入最大值；宏当前启用。
#define P_K_MAX  (100.0f * 1000.f)
// PVT 速度输入最小值；宏当前启用，单位 rad/s。
#define V_K_MIN  (-400.0f * 1000.f)  // 单位rad/s
// PVT 速度输入最大值；宏当前启用。
#define V_K_MAX  (400.0f * 1000.f)     
// PVT 位置比例增益最小值；宏当前启用。
#define KP_K_MIN (0.0f * 1000.f)     // 记得*1000
// PVT 位置比例增益最大值；宏当前启用。
#define KP_K_MAX (100.0f * 1000.f)
// PVT 位置微分增益最小值；宏当前启用。
#define KD_K_MIN (0.0f * 1000.f)
// PVT 位置微分增益最大值；宏当前启用。
#define KD_K_MAX (100.0f * 1000.f)
// PVT 扭矩输入最小值；宏当前启用，单位 A。
#define T_K_MIN  (-150.0f * 1000.f)  // 单位 A
// PVT 扭矩输入最大值；宏当前启用。
#define T_K_MAX  (150.0f * 1000.f)

/* 宏：达妙电机位置输入最小值；当前范围按达妙协议定义。 */
#define P_DM_MIN -27.0f
/* 宏：达妙电机位置输入最大值。 */
#define P_DM_MAX 27.0f
/* 宏：达妙电机速度输入最小值。 */
#define V_DM_MIN -30.0f
/* 宏：达妙电机速度输入最大值。 */
#define V_DM_MAX 30.0f
/* 宏：达妙电机扭矩输入最小值。 */
#define T_DM_MIN -10.0f
/* 宏：达妙电机扭矩输入最大值。 */
#define T_DM_MAX 10.0f

/*主控接收驱动器消息内容结构体*/
/* 结构体：驱动器反馈数据快照；由 GetDriverMsg/DMGetMotorMsg 按协议更新。 */
typedef struct
{
    /* 驱动器位置复位成功标志；复位轮询逻辑读取。 */
    uint8_t reset_pos_ok_flag;
    /* 空行程消除完成标志；消空行程流程读取。 */
    uint8_t empty_distance_ok_flag;
    /* 反馈速度；单位和缩放由驱动器协议决定。 */
    float vel;
    /* 反馈位置；普通驱动器解析时写入。 */
    float pos;
    /* 反馈加速度；对应驱动器状态数据项。 */
    float acc;
    /* 机构高度或经上层换算的高度值。 */
    float height;
    /* 负载或载荷观测值；采用有符号 32 位存储。 */
    int32_t loaded;
    /* 电机角度反馈；按对应协议解码。 */
    float angle;	
    /* 反馈扭矩。 */
    float torque;
    /* Q 轴电压反馈值。 */
    int32_t volQ;
    /* Q 轴电流反馈值。 */
    float currQ;
    /* G 轴电流反馈值。 */
    int32_t currG;
    /* D 轴电压反馈值。 */
    int32_t volD;
    /* D 轴电流反馈值。 */
    int32_t currD;
    /* 编码器位置原始反馈值。 */
    int32_t encoder;
    /* 速度环输出值。 */
    int32_t velLoopOut;
    /* 位置环输出值。 */
    int32_t posLoopOut;
    /* 驱动器使能状态反馈。 */
    uint8_t enableState;
    /* 驱动器当前控制模式反馈。 */
    uint8_t driverMode;
    /* 编码器错误状态反馈。 */
    uint8_t encoderErr;
    /* 硬件故障状态反馈。 */
    uint8_t hardFault;
    /* 使能错误状态/计数；控制逻辑检测使能异常时可能更新。 */
    uint8_t enable_error;
    /* 通用错误标志或错误码。 */
    uint8_t error;
}driverMsg_t;

/* 函数声明：解析普通驱动器反馈载荷并更新消息结构；data8 指向载荷字节。 */
void GetDriverMsg(driverMsg_t *driverMsg, const uint8_t *data8);
/* 函数声明：解析达妙电机反馈载荷并更新消息结构；data 指向载荷字节。 */
void DMGetMotorMsg(driverMsg_t *driverMsg, const uint8_t *data);

/*----------------------------------------------------------------------*/

/* 函数声明：发送达妙位置/速度控制目标；返回 HAL 发送状态。 */
HAL_StatusTypeDef DMPosCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel);
/* 函数声明：使能达妙电机并选择控制模式；返回 HAL 发送状态。 */
HAL_StatusTypeDef DMEnable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
/* 函数声明：失能达妙电机；返回 HAL 发送状态。 */
HAL_StatusTypeDef DMDisable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
/* 函数声明：将达妙电机当前位置设为零；返回 HAL 发送状态。 */
HAL_StatusTypeDef DMSetPosZero(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
/* 函数声明：请求普通驱动器复位；返回 HAL 发送状态。 */
HAL_StatusTypeDef ResetDriver(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/* 函数声明：读取/检查驱动器复位完成标志；返回 HAL 发送状态。 */
HAL_StatusTypeDef IfResetOkFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/* 函数声明：请求驱动器消除空行程；返回 HAL 发送状态。 */
HAL_StatusTypeDef EmptyDriverDistance(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/* 函数声明：设置普通驱动器使能状态；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetDriverState(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state);
/* 函数声明：设置驱动器使能状态并同步位置控制目标；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetStateAndPosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state, int32_t pos);
/* 函数声明：设置驱动器控制模式及关联位置目标；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetDriverMode(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t CtlMode, int32_t pos);
/* 函数声明：设置速度环 KP/KI 增益及积分清除标志；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetVelKpKi(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, float kp, float ki, bool i);
/* 函数声明：下发速度、电流前馈及积分清除控制量；返回 HAL 发送状态。 */
HAL_StatusTypeDef VelCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t vel, float curr, bool clean);
/* 函数声明：下发位置目标及位置环 KP/KD；返回 HAL 发送状态。 */
HAL_StatusTypeDef PosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t pos, float kp, float kd);
/* 函数声明：设置速度、扭矩和电流限制；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetLimits(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t VelLimit, int32_t TrqLimit, float CurLimit);
/* 函数声明：设置速度加速和减速限制；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetVelAcc(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t AccLimit, int32_t DecLimit);
/* 函数声明：下发达妙电机 PVT/MIT 风格控制目标；返回 HAL 发送状态。 */
HAL_StatusTypeDef PVTCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel, float kp, float kd, float t);
/* 函数声明：将普通驱动器当前位置设为零；返回 HAL 发送状态。 */
HAL_StatusTypeDef SetDriverZero(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/*----------------------------------------------------------------------*/

/* 函数声明：请求读取驱动器 PVT/位置相关反馈项；返回 HAL 发送状态。 */
HAL_StatusTypeDef ReadPVT(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/* 函数声明：请求读取驱动器状态反馈；返回 HAL 发送状态。 */
HAL_StatusTypeDef ReadDriverStatus(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);


#ifdef __cplusplus
}
#endif

#endif //  MOTOR_DRIVER_H