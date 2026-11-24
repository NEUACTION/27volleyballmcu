/* 头文件保护宏：避免机器人控制接口重复定义。 */
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
// 轮子 1 的备选驱动器 ID；当前被注释屏蔽，未生效。
// #define WHEEL2          (1)
// 轮子 2 的备选驱动器 ID；当前被注释屏蔽，未生效。
// #define WHEEL3          (2)
// 轮子 3 的备选驱动器 ID；当前被注释屏蔽，未生效。
// #define WHEEL4          (3)
// 轮子 4 的备选驱动器 ID；当前被注释屏蔽，未生效。

// #define LEG1			 (4)
// 腿部电机 1 的备选驱动器 ID；当前被注释屏蔽，实际启用值由下方定义。
// #define LEG2			 (5)
// 腿部电机 2 的备选驱动器 ID；当前被注释屏蔽，实际启用值由下方定义。
// #define LEG3			 (6)
// 腿部电机 3 的备选驱动器 ID；当前被注释屏蔽，实际启用值由下方定义。
// #define LEG4			 (7)
// 腿部电机 4 的备选驱动器 ID；当前被注释屏蔽，实际启用值由下方定义。
/* 宏：腿部电机 1 的驱动器 ID；当前启用并用于上层 CAN 的控制与解析。 */
#define LEG1			 (5)
/* 宏：腿部电机 2 的驱动器 ID；当前启用并用于上层 CAN 的控制与解析。 */
#define LEG2			 (6)
/* 宏：腿部电机 3 的驱动器 ID；当前启用并用于上层 CAN 的控制与解析。 */
#define LEG3			 (7)
/* 宏：腿部电机 4 的驱动器 ID；当前启用并用于上层 CAN 的控制与解析。 */
#define LEG4			 (8)

/* 宏：肩部电机驱动器 ID；当前启用，实际控制总线映射需结合任务代码确认。 */
#define SHOULDER		 (9)
/* 宏：肘部达妙电机驱动器 ID；当前启用。 */
#define ELBOW			 (0x0A)
/* 宏：腕部达妙电机驱动器 ID；当前启用。 */
#define WRIST			 (0x0B)

//上下层对应的can号
/* 宏：上层腿部 CAN 控制器句柄别名；当前映射到 FDCAN3。 */
#define FDCAN_WL        (hfdcan3)
/* 宏：另一 CAN 控制器句柄别名；当前映射到 FDCAN1。 */
#define FDCAN_TK     	(hfdcan1)

// 电机期望
/* 结构体：单个电机目标参数；mode/目标值由控制逻辑写入，控制函数读取。 */
typedef struct{
    /* 目标驱动使能状态；0 通常表示失能，非 0 表示使能。 */
    uint8_t status;
    /* 当前期望控制模式；驱动控制按该值选择速度、位置或 PVT 等模式。 */
    uint8_t mode;
	/* 上次已下发的控制模式；用于检测模式切换并避免重复配置。 */
	uint8_t last_mode;
	/* 速度环积分清除请求标志；由控制层传入驱动器接口。 */
	bool clean;
	/* 最近一次使用的速度比例增益；用于判断是否需要更新驱动器增益。 */
	float last_vel_kp;
	/* 最近一次使用的速度积分增益；用于判断是否需要更新驱动器增益。 */
	float last_vel_ki;
	/* 期望速度；具体量纲随对应电机控制接口约定。 */
	float vel;
	/* 期望位置；具体量纲随对应电机控制接口约定。 */
	float pos;
	/* 扭矩或前馈项；由模式对应控制接口解释。 */
	float tor;
	/* MIT 模式位置比例增益；达妙电机控制时使用。 */
	float mit_kp;
	/* MIT 模式速度微分增益；达妙电机控制时使用。 */
	float mit_kd;
    /* 普通位置模式比例增益；位置控制接口使用。 */
    float pos_kp;
    /* 普通位置模式微分增益；位置控制接口使用。 */
    float pos_kd;
	/* 速度环比例增益；控制模式相关时使用。 */
	float vel_kp;
	/* 速度环积分增益；控制模式相关时使用。 */
	float vel_ki;
}motorTarget_t;

// 心跳包
/* 结构体：各电机、PC 和操作端的心跳计数；收到对应数据后由接收处理清零。 */
typedef struct{
	/* 轮子 1 心跳超时计数；当前轮子控制映射可能屏蔽，按接收路径判断是否更新。 */
	uint8_t wheel1;
	/* 轮子 2 心跳超时计数；当前轮子控制映射可能屏蔽，按接收路径判断是否更新。 */
	uint8_t wheel2;
	/* 轮子 3 心跳超时计数；当前轮子控制映射可能屏蔽，按接收路径判断是否更新。 */
	uint8_t wheel3;
	/* 轮子 4 心跳超时计数；当前轮子控制映射可能屏蔽，按接收路径判断是否更新。 */
	uint8_t wheel4;
	/* 腿部电机 1 心跳计数；对应 CAN 接收分支当前启用。 */
	uint8_t leg1;
	/* 腿部电机 2 心跳计数；对应 CAN 接收分支当前启用。 */
	uint8_t leg2;
	/* 腿部电机 3 心跳计数；对应 CAN 接收分支当前启用。 */
	uint8_t leg3;
	/* 腿部电机 4 心跳计数；对应 CAN 接收分支当前启用。 */
	uint8_t leg4;
	/* 肩部电机心跳计数；当前接收分支可能被屏蔽。 */
	uint8_t shoulder;
	/* 肘部电机心跳计数；当前接收分支可能被屏蔽。 */
	uint8_t elbow;
	/* 腕部电机心跳计数；当前接收分支可能被屏蔽。 */
	uint8_t wrist;
	/* PC 通信心跳计数；由通信接收逻辑更新。 */
	uint8_t PC;
	/* 操作端心跳计数；由外部输入接收逻辑更新。 */
	uint8_t op;
}Heart_t;

/* 结构体：机器人级运行状态与复位标志；由任务和控制流程共享。 */
typedef struct
{
    /* 最近一次运行控制是否成功的标志；由机器人控制流程维护。 */
    bool runSucceedFlag;
	/* PC 心跳周期计数；由系统任务或通信流程维护。 */
	uint8_t periodCntPC;
	/* 机器人复位流程标志；当前复位控制流程使用相关状态字段。 */
	bool reset_flag;

}	gRobot_t;

/* 外部变量：各设备心跳计数；由接收解析和控制任务共同更新。 */
extern Heart_t HB;

/* 外部变量：机器人运行状态；由任务和控制模块共享。 */
extern gRobot_t gRobot;

/* 外部变量：腿部四个驱动器的最新反馈消息；由 CAN 接收解析更新。 */
extern driverMsg_t leg1Msg, /* 腿部电机 1 反馈；CAN 接收解析更新。 */
                   leg2Msg, /* 腿部电机 2 反馈；CAN 接收解析更新。 */
                   leg3Msg, /* 腿部电机 3 反馈；CAN 接收解析更新。 */
                   leg4Msg; /* 腿部电机 4 反馈；CAN 接收解析更新。 */

/* 外部变量：腿部四个电机的目标参数；由任务赋值、控制函数读取。 */
extern motorTarget_t leg1Tar, /* 腿部电机 1 的控制目标；任务写入，控制函数读取。 */
                     leg2Tar, /* 腿部电机 2 的控制目标；任务写入，控制函数读取。 */
                     leg3Tar, /* 腿部电机 3 的控制目标；任务写入，控制函数读取。 */
                     leg4Tar; /* 腿部电机 4 的控制目标；任务写入，控制函数读取。 */

/* 外部变量：电调比较值；由 EscControl 输出到定时器通道。 */
extern int esc;

/* ------------------------------------------------------------
 * Driver message snapshot
 * - 在任务上下文调用
 * - 内部用临界区一次性拷贝，避免读到“半更新”的结构体
 * ------------------------------------------------------------ */
typedef struct
{
	/* 腿部驱动器 1 的反馈消息快照；由快照函数在临界区复制。 */
	driverMsg_t leg1Msg;
	/* 腿部驱动器 2 的反馈消息快照。 */
	driverMsg_t leg2Msg;
	/* 腿部驱动器 3 的反馈消息快照。 */
	driverMsg_t leg3Msg;
	/* 腿部驱动器 4 的反馈消息快照。 */
	driverMsg_t leg4Msg;
	/* 轮子驱动器 1 的反馈消息快照；当前轮子驱动路径可能屏蔽。 */
	driverMsg_t wheel1Msg;
	/* 轮子驱动器 2 的反馈消息快照；当前轮子驱动路径可能屏蔽。 */
	driverMsg_t wheel2Msg;
	/* 轮子驱动器 3 的反馈消息快照；当前轮子驱动路径可能屏蔽。 */
	driverMsg_t wheel3Msg;
	/* 轮子驱动器 4 的反馈消息快照；当前轮子驱动路径可能屏蔽。 */
	driverMsg_t wheel4Msg;
	/* 肩部驱动器的反馈消息快照；当前 CAN 接收分支可能屏蔽。 */
	driverMsg_t shoulderMsg;
	/* 肘部驱动器的反馈消息快照；当前 CAN 接收分支可能屏蔽。 */
	driverMsg_t elbowMsg;
	/* 腕部驱动器的反馈消息快照；当前 CAN 接收分支可能屏蔽。 */
	driverMsg_t wristMsg;
	
	/* 快照复制时的 RTOS Tick 时间戳，用于判断数据新旧。 */
	uint32_t tick;
} DriverMsgsSnapshot_t;
/* 外部变量：最近一次驱动反馈快照存储；由快照读取接口写入或供调用方访问。 */
extern DriverMsgsSnapshot_t driverMsgs;

/* 函数声明：在临界区复制所有驱动反馈及 Tick；out 为空时不执行。 */
void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out);

// 常用控制函数
/* 函数声明：设置机器人各电机初始使能和模式状态；控制启动时调用。 */
void RobotInit(void);
/* 函数声明：执行 GPIO 控制；实现为空，当前仅由未启用的 RobotControl 路径调用。 */
void GPIOControl(void);
/* 函数声明：向达妙电机下发使能/失能或位置速度控制；实现存在，但当前有效调用仅在已屏蔽的 MotorControl 中。 */
void DMControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget);
/* 函数声明：按目标模式控制普通驱动器；实现存在，但当前引用仅在已屏蔽的 MotorControl 中。 */
void DriverControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget, driverMsg_t *driverMsg);
/* 函数声明：更新各电机控制输出；本文件实现被注释屏蔽，RobotControl 中仍有调用，需确认工程是否另有实现。 */
void MotorControl(void);
/* 函数声明：底盘控制输出接口；当前工作区目标代码未找到实现或调用。 */
void ChassisControl(void);
/* 函数声明：执行错误停车处理；当前本文件实现为空，目标目录内未检索到调用。 */
void ErrorStop(void);
/* 函数声明：对所有电机执行紧急失能；当前本文件实现为空，目标目录内未检索到调用。 */
void DisableAll(void);
/* 函数声明：输出电调控制量到定时器；由 RobotControl 调用，但该控制入口当前未由主任务启用。 */
void EscControl(void);
/* 函数声明：轮询腿部复位完成状态；全部复位标志完成时返回 true。 */
bool reset_control(void);
/* 函数声明：轮询并处理驱动器状态；当前工作区目标代码未找到实现或调用。 */
void DriverStateControl(void);
/* 函数声明：执行机器人控制入口并调用气阀、GPIO、电机及电调控制；主任务中的调用当前被注释屏蔽。 */
void RobotControl(void);

#ifdef __cplusplus
}
#endif

#endif //  USER_CONTROL_H