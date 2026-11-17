#ifndef CHASSIS_DRIVER_H
#define CHASSIS_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_FDCAN.h"

/*常发标识符*/
#define IDENTIFIER_POS_SPD_CTRL        (0x01)
#define IDENTIFIER_POS_SPD_READ        (0x02)
#define IDENTIFIER_MT1_POS_READ        (0x03)
#define IDENTIFIER_MT2_SPD_READ        (0x04)
#define IDENTIFIER_SW_SET              (0x06)

#define IDENTIFIER_DRIVER_MODE         (0x0A)
#define IDENTIFIER_DRIVER_STATE        (0x0B)

/*回复标识符*/
#define IDENTIFIER_POS_SPD_READ_BCK    (0x0F)

/*限流挡位设置*/
typedef enum{
    TEST_MODE    = 0x01,
    NORMAL_MODE  = 0x02,
    EXTREME_MODE = 0x03,
    ORIGINAL_L   = 0x00, 
} CurrQLim_Mode;	

/*轮子加速度设置*/
typedef enum{
    HALFACC = 0x01,
    FULLACC = 0x02,
    LITACC  = 0x03,
} ACC_MODE;				

typedef enum{
    ENABLE_ALL  = 0x00,
    DISABLE_ALL = 0x01,
} DriverState_Mode;

typedef struct{
    uint8_t h,s,v;
} HSV_Cmd;

/*主控接收舵轮消息内容结构体*/
typedef struct
{
	float vel;
	float pos;	
    DriverState_Mode status;
}SteeringWheelMsg_t;

typedef struct
{
    float vel;
    float pos;
    DriverState_Mode status;
    HSV_Cmd hsv; 
}SteeringWheelTar_t;


void GetSteeringWheelMsg(SteeringWheelMsg_t *driverMsg, const uint8_t *data8);
void SteeringWheelCtrl(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, float velAim, float PosAim, HSV_Cmd hsv_cmd);
void DriverStateChassis(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, ACC_MODE acc_mode,     \
                CurrQLim_Mode ILIMmode, DriverState_Mode wheelstate, DriverState_Mode turnstate, uint8_t clearintergral);


#ifdef __cplusplus
}
#endif

#endif //  CHASSIS_DRIVER_H
