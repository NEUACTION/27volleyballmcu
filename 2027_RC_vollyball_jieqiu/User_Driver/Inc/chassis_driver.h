/* 头文件保护宏：避免底盘驱动接口重复定义。 */
#ifndef CHASSIS_DRIVER_H
#define CHASSIS_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_FDCAN.h"

/*常发标识符*/
/* 宏：舵轮位置速度控制命令标识；SteeringWheelCtrl 当前使用。 */
#define IDENTIFIER_POS_SPD_CTRL        (0x01)
/* 宏：舵轮位置速度读取标识；接收解析中用于区分反馈类型。 */
#define IDENTIFIER_POS_SPD_READ        (0x02)
/* 宏：电机 1 位置读取标识；GetSteeringWheelMsg 当前识别。 */
#define IDENTIFIER_MT1_POS_READ        (0x03)
/* 宏：电机 2 速度读取标识；GetSteeringWheelMsg 当前识别。 */
#define IDENTIFIER_MT2_SPD_READ        (0x04)
/* 宏：开关设置命令标识；底盘状态命令打包时使用。 */
#define IDENTIFIER_SW_SET              (0x06)

/* 宏：驱动器模式回报标识；当前供协议解析使用。 */
#define IDENTIFIER_DRIVER_MODE         (0x0A)
/* 宏：驱动器状态回报标识；GetSteeringWheelMsg 当前识别。 */
#define IDENTIFIER_DRIVER_STATE        (0x0B)

/*回复标识符*/
/* 宏：位置速度读取回复标识；当前声明供底盘协议解析使用。 */
#define IDENTIFIER_POS_SPD_READ_BCK    (0x0F)

/*限流挡位设置*/
typedef enum{
    /* 测试限流模式；用于测试场景，具体限值由驱动器固件定义。 */
    TEST_MODE    = 0x01,
    /* 常规限流模式；当前正常运行模式。 */
    NORMAL_MODE  = 0x02,
    /* 极限限流模式；高限流运行，使用前需确认硬件限制。 */
    EXTREME_MODE = 0x03,
    /* 原始限流档；协议值为 0，当前可选模式之一。 */
    ORIGINAL_L   = 0x00, 
} CurrQLim_Mode;	

/*轮子加速度设置*/
typedef enum{
    /* 半加速度档；用于设置较缓的轮子加速度。 */
    HALFACC = 0x01,
    /* 全加速度档；用于设置较大的轮子加速度。 */
    FULLACC = 0x02,
    /* 轻加速度档；用于设置较小的轮子加速度。 */
    LITACC  = 0x03,
} ACC_MODE;				

typedef enum{
    /* 驱动器使能状态；发送该值表示使能全部相关通道。 */
    ENABLE_ALL  = 0x00,
    /* 驱动器失能状态；发送该值表示关闭全部相关通道。 */
    DISABLE_ALL = 0x01,
} DriverState_Mode;

/* 结构体：HSV 灯光颜色指令；各分量依协议范围编码。 */
typedef struct{
    uint8_t h, /* 色相 H，协议范围 0～180。 */
            s, /* 饱和度 S，协议范围 0～255。 */
            v; /* 明度 V，协议范围 0～255。 */
} HSV_Cmd;

/*主控接收舵轮消息内容结构体*/
/* 结构体：舵轮反馈快照，包含速度、位置及驱动使能状态。 */
typedef struct
{
	/* 舵轮反馈速度；单位按舵轮协议定义。 */
	float vel;
	/* 舵轮反馈位置；当前解析以 0.1 单位还原。 */
	float pos;	
    /* 舵轮驱动器当前状态；由状态反馈标识更新。 */
    DriverState_Mode status;
}SteeringWheelMsg_t;

/* 结构体：舵轮控制目标，包含运动指令、状态和灯光颜色。 */
typedef struct
{
    /* 目标速度；发送时按协议缩放为整数。 */
    float vel;
    /* 目标位置；发送时按协议缩放为整数。 */
    float pos;
    /* 目标使能状态；用于底盘状态控制。 */
    DriverState_Mode status;
    /* 灯板 HSV 颜色目标；控制命令发送时使用。 */
    HSV_Cmd hsv; 
}SteeringWheelTar_t;


/* 函数声明：解析舵轮位置、速度或状态反馈；实现存在，目标目录内未检索到调用。 */
void GetSteeringWheelMsg(SteeringWheelMsg_t *driverMsg, const uint8_t *data8);
/* 函数声明：发送舵轮位置、速度及灯光指令；实现存在，目标目录内未检索到调用。 */
void SteeringWheelCtrl(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, float velAim, float PosAim, HSV_Cmd hsv_cmd);
/* 函数声明：发送底盘驱动状态、加速度、限流和清积分配置；实现存在，目标目录内未检索到调用。 */
void DriverStateChassis(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, ACC_MODE acc_mode,     \
                CurrQLim_Mode ILIMmode, DriverState_Mode wheelstate, DriverState_Mode turnstate, uint8_t clearintergral);


#ifdef __cplusplus
}
#endif

#endif //  CHASSIS_DRIVER_H
