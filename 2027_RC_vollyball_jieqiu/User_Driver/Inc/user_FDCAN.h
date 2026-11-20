/* 头文件保护宏：避免用户 FDCAN 接口重复定义。 */
#ifndef USER_FDCAN_H
#define USER_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

#ifdef data_len
/* 预处理处理：移除外部同名 data_len 宏，避免与消息结构字段冲突；此处为条件性屏蔽。 */
#undef data_len
#endif

/* Driver ID base (StdId) */
/* 宏：驱动器回报标准帧 ID 基值；接收 ID 按基值加驱动器编号解析。 */
#define DRIVER_CLIENT_BASE_ID	                (0x280) /* 驱动器回报：0x280 + DriverNum */
/* 宏：主控下发标准帧 ID 基值；命令帧 ID 按基值及驱动器编号/模式生成。 */
#define DRIVER_SERVER_BASE_ID	                (0x300) /* 主控下发：0x300 + DriverNum (+ CMDmode) */

/* 错误标识符 */
/* 宏：编码器错误回报标识；驱动器协议解析时识别。 */
#define IDENTIFIER_ENCODER_ERROR                (0xEE)
/* 宏：驱动器硬件故障回报标识；驱动器协议解析时识别。 */
#define IDENTIFIER_HARD_FAULT                   (0xFF)
/* 宏：静默/无数据标识；当前用于构造底盘控制帧填充字段。 */
#define IDENTIFIER_SILENCE                      (0xFE)

/* 驱动器指令模式 */
typedef enum{
	/* 点对点指令模式；针对单个驱动器发送。 */
	PTP_MODE = 0x00, 
	/* 广播指令模式；用于向一组驱动器发送。 */
	BROADCAST_MODE = 0x40
} CommandMode;

/* 联合体：FDCAN 4 字节载荷的整数、浮点或字节视图；按驱动器协议解释。 */
typedef union
{
	/* 有符号整数视图。 */
	int32_t  datai;
	/* 无符号整数视图。 */
	uint32_t datau;
	/* 单精度浮点视图。 */
	float    dataf;
	/* 原始 4 字节视图。 */
	uint8_t  datac[4];
}FDCAN_Trans;

/* 结构体：待分发的 FDCAN 接收消息；接收任务从 FIFO 搬运后提供给回调。 */
typedef struct
{
	/* 收到该帧的 FDCAN 总线句柄。 */
	FDCAN_HandleTypeDef *bus;
	/* HAL 接收帧头，包含标准 ID、DLC 等元数据。 */
	FDCAN_RxHeaderTypeDef header;
	/* 接收数据缓存，最多保存 64 字节。 */
	uint8_t data[64];
	/* 实际数据长度，单位字节。 */
	uint8_t data_len;
} UserFDCAN_RxMsg_t;

/* 结构体：待发送的 FDCAN 消息及超时设置；由发送任务处理。 */
typedef struct
{
	/* 目标 FDCAN 总线句柄。 */
	FDCAN_HandleTypeDef *bus;
	/* HAL 发送帧头，包含标准 ID、DLC 等元数据。 */
	FDCAN_TxHeaderTypeDef header;
	/* 待发送数据缓存，最多保存 64 字节。 */
	uint8_t data[64];
	/* 待发送数据长度，单位字节。 */
	uint8_t data_len;
    /* 发送入队或执行等待时使用的超时毫秒数。 */
    uint32_t timeout_ms;
} UserFDCAN_TxMsg_t;

/* RX 回调（任务上下文触发：由 rxTask 从 FIFO 搬运后调用） */
/* 回调类型：收到 FDCAN 消息时接收消息快照指针；由用户注册，任务上下文调用。 */
typedef void (*UserFDCAN_RxCallback_t)(const UserFDCAN_RxMsg_t *msg);

/* 函数声明：将字节长度换算为 HAL FDCAN DLC 编码；由发送封装使用。 */
uint32_t Fdcan_BytesToDlc(uint8_t len);
/* 函数声明：将 HAL FDCAN DLC 编码换算为字节数；由接收处理使用。 */
uint8_t Fdcan_DlcToBytes(uint32_t dlc);

/* 函数声明：处理指定 FDCAN 控制器的错误状态；当前由 HAL 错误流程调用。 */
void CAN_Error_Handle(FDCAN_HandleTypeDef *hfdcan);
/* 函数声明：创建收发资源并启动 FDCAN 服务；系统初始化时调用。 */
void User_FDCAN_Init(void);
/* 函数声明：启用工程配置的 FDCAN 控制器；启动流程调用。 */
void FDCAN_ENABLE(void);

/* 函数声明：向指定总线发送标准 ID 数据；timeout_ms 控制等待时间，返回发送状态。 */
int User_FDCAN_SendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint8_t data_len, uint32_t stdId, uint32_t timeout_ms);
/* 函数声明：注册接收消息回调；传 NULL 可取消回调。 */
void User_FDCAN_RegisterRxCallback(UserFDCAN_RxCallback_t cb);

#ifdef __cplusplus
}
#endif

#endif  // USER_FDCAN_H