#ifndef USER_COMMUNICATION_H
#define USER_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

#define MAX_GET_LEN (1024)

#define USB_RX_UINT8_NUM (13)
#define USB_RX_DATA_NUM  (68)
#define USB_TX_UINT8_NUM (26)
#define USB_TX_DATA_NUM  (41)

#define USB_RX_LEN (USB_RX_DATA_NUM*4 + USB_RX_UINT8_NUM+4)
#define USB_TX_LEN (USB_TX_DATA_NUM*4 + USB_TX_UINT8_NUM+4)

typedef union 
{
	int		 dataint[USB_RX_DATA_NUM];
	uint32_t dataint32[USB_RX_DATA_NUM];
	float    dataf[USB_RX_DATA_NUM];
	char     datac[USB_RX_DATA_NUM*4];
}trans_r;

typedef union
{
	int		 dataint[USB_TX_DATA_NUM];
	uint32_t dataint32[USB_TX_DATA_NUM];
	float    data_float[USB_TX_DATA_NUM];
	char     datac[USB_TX_DATA_NUM*4];
}trans_t;

extern trans_t TX_Data;		// 发送联合体
extern trans_r RX_Data;		// 接收联合体
extern int PC_OK_Flag;
extern uint32_t periodCntMCU;
extern int reset_ok_flag;

void User_Communication_Init(void);
bool UsbCdc_IsConfigured(void);

/* ---------- USB TX ---------- */

void UsbTx_OnCdcTxCpltFromISR(void);
bool UsbTx_IsReady(void);
void UsbTx_GetStats(uint32_t *enqueued_bytes, uint32_t *dropped_bytes, uint32_t *tx_ok_pkts, uint32_t *tx_busy_cnt);
int UsbTx_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int UsbTx_SendString(const char *str, uint32_t timeout_ms);
int DEBUG_USB(const char *fmt, ...);
void HB_Add(void);

typedef struct
{
	uint32_t tx_enqueued_bytes;
	uint32_t tx_dropped_bytes;
	uint32_t tx_ok_pkts;
	uint32_t tx_busy_cnt;
	uint32_t tx_fail_pkts;
	uint32_t tx_fail_bytes;
	uint32_t rx_isr_enqueued_bytes;
	uint32_t rx_isr_dropped_bytes;
	uint32_t app_rx_forwarded_bytes;
	uint32_t app_rx_dropped_bytes;
} UsbStats_t;



void Usb_GetStats(UsbStats_t *out);

/* ---------- USB RX ---------- */
void UsbRx_OnCdcRxFromISR(const uint8_t *data, uint32_t len);
uint16_t UsbRx_Read(uint8_t *out, uint16_t out_len, uint32_t timeout_ms);
size_t UsbRx_Available(void);
void UsbRx_GetStats(uint32_t *rx_enqueued_bytes, uint32_t *rx_dropped_bytes);

typedef void (*UsbRx_Callback_t)(const uint8_t *data, uint16_t len);
void UsbRx_RegisterCallback(UsbRx_Callback_t cb);

/* ---------- Process ---------- */

/*
 * 在 UsbRxTask 上下文触发的接收回调：把来自 PC 的字节流拼帧并解析。
 * 说明：USB CDC 是字节流，回调 data/len 不保证对齐到帧边界。
 */
void GetPCMsg(const uint8_t *data, uint16_t len);
void SendPCMsg(void);

#ifdef __cplusplus
}
#endif

#endif // USER_COMMUNICATION_H