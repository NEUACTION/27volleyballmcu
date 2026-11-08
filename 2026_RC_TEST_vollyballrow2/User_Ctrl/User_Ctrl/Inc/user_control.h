#ifndef USER_CONTROL_H
#define USER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_FDCAN.h"
#include "user_communication.h"

//各电机ID宏定义
#define ARM1          (1)
#define ARM2          (2)
#define PLATE          (3)
#define SLIDE       (4)
#define PLATFORMMOTOR       (5)
#define CONNECTOR          (6)
#define CONNECTORBLUE      (7)
#define RAWBLUE           (8)

#define TONGUE1MOTOR            (1)
#define TONGUE2MOTOR            (2)
#define TONGUE1TURN             (9)
#define TONGUE2TURN             (10)


#define HIGH (1)        //高手球
#define HIGHUP  (2)     //高球上旋
#define HIGHDOWN (3)	//高球下旋
#define LOW (4)			//低手球
#define LOWUP (5)		//低球上旋
#define LOWDOWN (6)	    //
#define HIGHDOWN117 (7)	//高球下旋
#define LOW118A (8)		//低球下旋
#define LOW118B (9)		//低球下旋


//上下层对应的can号
#define FDCANWHEEL1 (hfdcan3)
#define FDCANWHEEL2 (hfdcan3)
#define FDCANWHEEL3 (hfdcan3)
#define FDCANPLATFORMMOTOR (hfdcan3)
#define FDCANTONGUEMOTOR1 (hfdcan1)
#define FDCANTONGUEMOTOR2 (hfdcan1)
#define FDCANRAW (hfdcan3)
#define FDCANCONNECTOR (hfdcan3)
#define FDCANTONGUE1TURN (hfdcan1)
#define FDCANTONGUE2TURN (hfdcan1)
#define FDCANCONNECTORBLUE (hfdcan3)
#define FDCANRAWBLUE (hfdcan3)

// #define GAP_TIME (43)//highdown
#define GAP_TIME (30)//highdown117
#define GAP_TIME2 (30)//low118A
#define GAP_TIME118B (15)//low118B

#define STARTHIGH (-1.75f)
#define STARTHIGHDOWN (-2.0f)//2.0
#define STARTLOW (1.5f)
#define STARTLOWUP (1.75f)
#define STARTLOW118A (-1.5f)
#define STARTLOW118B (-1.6f)



// 电机期望
typedef struct{
    uint8_t status;
    uint8_t mode;
	float vel;
	float pos;
	bool set_intergrial_zero_flag;
	float pos_kp;
	float pos_kd;
	float vel_kp;
	float vel_ki;
	float limit_vel;
	float last_limit_vel;
	float last_vel_kp;
	float last_vel_ki;
	float last_pos_kp;
	float last_pos_kd;
	float mit_kp;
	float mit_kd;
	float tor;
	float last_tor;
	int ErrCnt;
	uint8_t last_mode;
}motorTarget_t;

// 心跳包
typedef struct{
	uint8_t arm1;
	uint8_t arm2;
	uint8_t plate;
	uint8_t slide;
}Heart_t;

extern Heart_t HB;

extern SteeringWheelMsg_t wheel1Msg, wheel2Msg, wheel3Msg;
extern driverMsg_t armMsg1 , armMsg2, plateMsg, slideMsg;

extern SteeringWheelTar_t wheel1Tar, wheel2Tar, wheel3Tar;
extern motorTarget_t armTar1, armTar2 , plateTar, slideTar;

extern int buffer_valve_flag;

extern int Esc1;
extern int Esc2;
extern int LaserDis;
extern int LaserDis_front;
extern int LaserDis_right;

extern int LightwheelH;
extern int LightwheelS;
extern int LightwheelV;

extern char valve_number;
extern char valve2_number;

/* ------------------------------------------------------------
 * Driver message snapshot
 * - 在任务上下文调用
 * - 内部用临界区一次性拷贝，避免读到“半更新”的结构体
 * ------------------------------------------------------------ */
typedef struct
{
	SteeringWheelMsg_t oneWheel;
	SteeringWheelMsg_t twoWheel;
	SteeringWheelMsg_t thrWheel;
	driverMsg_t armMsg1;
	driverMsg_t armMsg2;
	driverMsg_t plateMsg;
	driverMsg_t slideMsg;
	uint32_t tick;
} DriverMsgsSnapshot_t;
extern DriverMsgsSnapshot_t driverMsgs;

void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out);

// 常用控制函数
void RobotInit(void);
void GPIOControl(void);
void DMControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget);
void DriverControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget, driverMsg_t *driverMsg);
void MotorControl(void);
void ChassisControl(void);
void ErrorStop(void);
void DisableAll(void);
void EscControl(void);
bool reset_control(void);
void DriverStateControl(void);
void RobotControl(void);

#ifdef __cplusplus
}
#endif

#endif //  USER_CONTROL_H