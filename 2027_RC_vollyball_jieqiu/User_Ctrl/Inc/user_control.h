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
// #define WHEEL1          (8)
// #define WHEEL2          (1)
// #define WHEEL3          (2)
// #define WHEEL4          (3)

// #define LEG1			 (4)
// #define LEG2			 (5)
// #define LEG3			 (6)
// #define LEG4			 (7)
#define LEG1			 (5)
#define LEG2			 (6)
#define LEG3			 (7)
#define LEG4			 (8)

#define SHOULDER		 (9)
#define ELBOW			 (0x0A)
#define WRIST			 (0x0B)

//上下层对应的can号
#define FDCAN_WL        (hfdcan3)
#define FDCAN_TK     	(hfdcan1)

// 电机期望
typedef struct{
    uint8_t status;
    uint8_t mode;
	uint8_t last_mode;
	bool clean;
	float last_vel_kp;
	float last_vel_ki;
	float vel;
	float pos;
	float tor;
	float mit_kp;
	float mit_kd;
    float pos_kp;
    float pos_kd;
	float vel_kp;
	float vel_ki;
}motorTarget_t;

// 心跳包
typedef struct{
	uint8_t wheel1;
	uint8_t wheel2;
	uint8_t wheel3;
	uint8_t wheel4;
	uint8_t leg1;
	uint8_t leg2;
	uint8_t leg3;
	uint8_t leg4;
	uint8_t shoulder;
	uint8_t elbow;
	uint8_t wrist;
	uint8_t PC;
	uint8_t op;
}Heart_t;

typedef struct
{
    bool runSucceedFlag;
	uint8_t periodCntPC;
	bool reset_flag;

}	gRobot_t;

extern Heart_t HB;

extern gRobot_t gRobot;

extern driverMsg_t leg1Msg, leg2Msg, leg3Msg, leg4Msg;

extern motorTarget_t leg1Tar, leg2Tar, leg3Tar, leg4Tar;

extern int esc;

/* ------------------------------------------------------------
 * Driver message snapshot
 * - 在任务上下文调用
 * - 内部用临界区一次性拷贝，避免读到“半更新”的结构体
 * ------------------------------------------------------------ */
typedef struct
{
	driverMsg_t leg1Msg;
	driverMsg_t leg2Msg;
	driverMsg_t leg3Msg;
	driverMsg_t leg4Msg;
	driverMsg_t wheel1Msg;
	driverMsg_t wheel2Msg;
	driverMsg_t wheel3Msg;
	driverMsg_t wheel4Msg;
	driverMsg_t shoulderMsg;
	driverMsg_t elbowMsg;
	driverMsg_t wristMsg;
	
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