#ifndef USER_FDCAN_H
#define USER_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

#ifdef data_len
#undef data_len
#endif

/* Driver ID base (StdId) */
#define DRIVER_CLIENT_BASE_ID	                (0x280) /* 驱动器回报：0x280 + DriverNum */
#define DRIVER_SERVER_BASE_ID	                (0x300) /* 主控下发：0x300 + DriverNum (+ CMDmode) */

/* 错误标识符 */
#define IDENTIFIER_ENCODER_ERROR                (0xEE)
#define IDENTIFIER_HARD_FAULT                   (0xFF)
#define IDENTIFIER_SILENCE                      (0xFE)

/* 驱动器指令模式 */
typedef enum{
	PTP_MODE = 0x00, 
	BROADCAST_MODE = 0x40
} CommandMode;

typedef union
{
	int32_t  datai;
	uint32_t datau;
	float    dataf;
	uint8_t  datac[4];
}FDCAN_Trans;

typedef struct
{
	FDCAN_HandleTypeDef *bus;
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[64];
	uint8_t data_len;
} UserFDCAN_RxMsg_t;

typedef struct
{
	FDCAN_HandleTypeDef *bus;
	FDCAN_TxHeaderTypeDef header;
	uint8_t data[64];
	uint8_t data_len;
    uint32_t timeout_ms;
} UserFDCAN_TxMsg_t;

/* RX 回调（任务上下文触发：由 rxTask 从 FIFO 搬运后调用） */
typedef void (*UserFDCAN_RxCallback_t)(const UserFDCAN_RxMsg_t *msg);

uint32_t Fdcan_BytesToDlc(uint8_t len);
uint8_t Fdcan_DlcToBytes(uint32_t dlc);

void CAN_Error_Handle(FDCAN_HandleTypeDef *hfdcan);
void User_FDCAN_Init(void);
void FDCAN_ENABLE(void);

int User_FDCAN_SendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint8_t data_len, uint32_t stdId, uint32_t timeout_ms);
void User_FDCAN_RegisterRxCallback(UserFDCAN_RxCallback_t cb);

#ifdef __cplusplus
}
#endif

#endif  // USER_FDCAN_H