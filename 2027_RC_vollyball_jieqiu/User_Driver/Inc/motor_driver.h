#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_FDCAN.h"
#include "string.h"

/*控制标识符*/
#define IDENTIFIER_SET_DRIVER_STATE             (0x01)
#define IDENTIFIER_SET_CTRL_MODE                (0x02)
#define IDENTIFIER_SET_SPD_KP                   (0x03)
#define IDENTIFIER_SET_SPD_KI                   (0x04)
#define IDENTIFIER_SET_POS_KP                   (0x05)
#define IDENTIFIER_SET_POS_KD                   (0x06)
#define IDENTIFIER_SET_TORQUE                   (0x07)
#define IDENTIFIER_SET_VEL                      (0x08)
#define IDENTIFIER_SET_POS_ABS                  (0x09)
#define IDENTIFIER_SET_POS_RE                   (0x0A)
#define IDENTIFIER_SET_VEL_ACC                  (0x0B)
#define IDENTIFIER_SET_VEL_DEC                  (0x0C)
#define IDENTIFIER_SET_TORQUE_LIMIT             (0x0D)
#define IDENTIFIER_SET_VEL_LIMIT                (0x0E)
#define IDENTIFIER_SET_INTEGRAL_CLEARED         (0x0F)
#define IDENTIFIER_SET_POS_ACC                  (0x10)
#define IDENTIFIER_SET_POS_DEC                  (0x11)
#define IDENTIFIER_SET_CLEAR_INTEGRAL           (0x12)
#define IDENTIFIER_SET_CURRENTQ_LIMIT           (0x13)
#define IDENTIFIER_SET_RESET                    (0x14)
#define IDENTIFIER_ELIMINATE_EMPTY_DIATANCE     (0x15)
#define IDENTIFIER_SET_FEEDFORWARD_CURR         (0x16)
#define IDENTIFIER_SET_ZERO                     (0x17)
/*读取标识符*/
#define IDENTIFIER_READ_OFFSET                  (0x40)
#define IDENTIFIER_READ_ACC                     (0x01)
#define IDENTIFIER_READ_TORQUE                  (0x02)
#define IDENTIFIER_READ_VEL                     (0x03)
#define IDENTIFIER_READ_POS                     (0x04)
#define IDENTIFIER_READ_ENCODER_POS             (0x05)
#define IDENTIFIER_READ_VOL_D                   (0x06)
#define IDENTIFIER_READ_CURR_D                  (0x07)
#define IDENTIFIER_READ_VOL_Q                   (0x08)
#define IDENTIFIER_READ_CURR_Q                  (0x09)
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT         (0x0A)
#define IDENTIFIER_READ_POS_LOOP_OUTPUT         (0x0B)
#define IDENTIFIER_READ_POS_EXP_PULSE           (0x0C)
#define IDENTIFIER_READ_LOAD_OBSERVER           (0x0D)
#define IDENTIFIER_PITCH_RATE_SEND              (0x0E)
#define IDENTIFIER_READ_FAN_MODE                (0x0F)
#define IDENTIFIER_READ_CURR_G                  (0x10)
#define IDENTIFIER_READ_RESET                   (0x11)
#define IDENTIFIER_RETURN_MSG                   (0x12)
#define IDENTIFIER_REPORT_ERROR                 (0xEE)

/*驱动器控制模式*/
#define VEL_MODE                                (0x01)
#define POS_MODE                                (0x02)
#define PVT_MODE                                (0x04)

/*DAMIAO电机控制标识符*/
#define DM_POS_MODE 0x100
#define DM_VEL_MODE 0x200

// pvt控制限值 ，需与电驱方面比对数据是否相同
#define P_K_MIN  (-100.0f * 1000.f)  // 单位rad
#define P_K_MAX  (100.0f * 1000.f)
#define V_K_MIN  (-400.0f * 1000.f)  // 单位rad/s
#define V_K_MAX  (400.0f * 1000.f)     
#define KP_K_MIN (0.0f * 1000.f)     // 记得*1000
#define KP_K_MAX (100.0f * 1000.f)
#define KD_K_MIN (0.0f * 1000.f)
#define KD_K_MAX (100.0f * 1000.f)
#define T_K_MIN  (-150.0f * 1000.f)  // 单位 A
#define T_K_MAX  (150.0f * 1000.f)

#define P_DM_MIN -27.0f
#define P_DM_MAX 27.0f
#define V_DM_MIN -30.0f
#define V_DM_MAX 30.0f
#define T_DM_MIN -10.0f
#define T_DM_MAX 10.0f

/*主控接收驱动器消息内容结构体*/
typedef struct
{
    uint8_t reset_pos_ok_flag;
    uint8_t empty_distance_ok_flag;
    float vel;
    float pos;
    float acc;
    float height;
    int32_t loaded;
    float angle;	
    float torque;
    int32_t volQ;
    float currQ;
    int32_t currG;
    int32_t volD;
    int32_t currD;
    int32_t encoder;
    int32_t velLoopOut;
    int32_t posLoopOut;
    uint8_t enableState;
    uint8_t driverMode;
    uint8_t encoderErr;
    uint8_t hardFault;
    uint8_t enable_error;
    uint8_t error;
}driverMsg_t;

void GetDriverMsg(driverMsg_t *driverMsg, const uint8_t *data8);
void DMGetMotorMsg(driverMsg_t *driverMsg, const uint8_t *data);

/*----------------------------------------------------------------------*/

HAL_StatusTypeDef DMPosCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel);
HAL_StatusTypeDef DMEnable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
HAL_StatusTypeDef DMDisable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
HAL_StatusTypeDef DMSetPosZero(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode);
HAL_StatusTypeDef ResetDriver(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
HAL_StatusTypeDef IfResetOkFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
HAL_StatusTypeDef EmptyDriverDistance(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
HAL_StatusTypeDef SetDriverState(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state);
HAL_StatusTypeDef SetStateAndPosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state, int32_t pos);
HAL_StatusTypeDef SetDriverMode(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t CtlMode, int32_t pos);
HAL_StatusTypeDef SetVelKpKi(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, float kp, float ki, bool i);
HAL_StatusTypeDef VelCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t vel, float curr, bool clean);
HAL_StatusTypeDef PosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t pos, float kp, float kd);
HAL_StatusTypeDef SetLimits(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t VelLimit, int32_t TrqLimit, float CurLimit);
HAL_StatusTypeDef SetVelAcc(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t AccLimit, int32_t DecLimit);
HAL_StatusTypeDef PVTCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel, float kp, float kd, float t);
HAL_StatusTypeDef SetDriverZero(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
/*----------------------------------------------------------------------*/

HAL_StatusTypeDef ReadPVT(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
HAL_StatusTypeDef ReadDriverStatus(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);


#ifdef __cplusplus
}
#endif

#endif //  MOTOR_DRIVER_H