/* 头文件保护宏：避免 USB 通信接口重复展开。 */
#ifndef USER_COMMUNICATION_H
#define USER_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

/* 宏：PC 接收帧缓存上限，单位字节；GetPCMsg 实现当前屏蔽，活动流程未使用。 */
#define MAX_GET_LEN (1024)

/* 宏：USB 接收帧中 uint8_t 项的数量；当前协议解析使用。 */
#define USB_RX_UINT8_NUM (10)//先留着吧
/* 宏：USB 接收帧中 32 位数据项数量；当前协议接收布局。 */
#define USB_RX_DATA_NUM  (54)
/* 宏：USB 发送帧中 uint8_t 项的数量；当前协议发送布局。 */
#define USB_TX_UINT8_NUM (37)
/* 宏：USB 发送帧中 32 位数据项数量；当前协议发送布局。 */
#define USB_TX_DATA_NUM  (41)//先认为有多少写多少

/* 宏：按接收协议字段数量计算完整接收帧字节长度。 */
#define USB_RX_LEN (USB_RX_DATA_NUM*4 + USB_RX_UINT8_NUM+4)
/* 宏：按发送协议字段数量计算完整发送帧字节长度。 */
#define USB_TX_LEN (USB_TX_DATA_NUM*4 + USB_TX_UINT8_NUM+4)

typedef union 
{
	/* 接收数据的有符号整数视图；与联合体其他视图共享同一存储空间。 */
	int		 dataint[USB_RX_DATA_NUM];
	/* 接收数据的无符号 32 位视图；与浮点和字符视图共用缓冲区。 */
	uint32_t dataint32[USB_RX_DATA_NUM];
	/* 接收数据的浮点视图；当前用于解析 PC 下发的浮点参数。 */
	float    dataf[USB_RX_DATA_NUM];
	/* 接收数据的原始字节视图；容量为数据项数乘 4。 */
	char     datac[USB_RX_DATA_NUM*4];
}trans_r;

typedef union
{
	/* 发送数据的有符号整数视图；与联合体其他视图共用存储空间。 */
	int		 dataint[USB_TX_DATA_NUM];
	/* 发送数据的无符号 32 位视图；当前用于协议数据打包。 */
	uint32_t dataint32[USB_TX_DATA_NUM];
	/* 发送数据的浮点视图；当前用于打包浮点遥测值。 */
	float    data_float[USB_TX_DATA_NUM];
	/* 发送数据的原始字节视图；容量为数据项数乘 4。 */
	char     datac[USB_TX_DATA_NUM*4];
}trans_t;

/* 外部变量：USB 发送协议联合体；仅被已屏蔽的 SendPCMsg 逻辑引用，当前活动发送未使用。 */
extern trans_t TX_Data;		// 发送联合体
/* 外部变量：USB 接收协议联合体；仅被已屏蔽的 GetPCMsg 逻辑引用，当前活动接收未使用。 */
extern trans_r RX_Data;		// 接收联合体


/* 函数声明：初始化 USB 通信缓冲区和收发任务；系统启动时调用。 */
void User_Communication_Init(void);
/* 函数声明：查询 USB CDC 是否配置完成；收发前用于确认设备可用。 */
bool UsbCdc_IsConfigured(void);

/* ---------- USB TX ---------- */

/* 函数声明：CDC 发送完成 ISR 通知入口；USB HAL 完成回调中调用。 */
void UsbTx_OnCdcTxCpltFromISR(void);
/* 函数声明：查询 USB 发送通道是否就绪；接口已实现，目标目录内未检索到直接调用。 */
bool UsbTx_IsReady(void);
/* 函数声明：读取 USB 发送统计计数；接口已实现，目标目录内未检索到调用。 */
void UsbTx_GetStats(uint32_t *enqueued_bytes, uint32_t *dropped_bytes, uint32_t *tx_ok_pkts, uint32_t *tx_busy_cnt);
/* 函数声明：将指定字节写入 USB 发送队列；timeout_ms 控制等待时间。 */
int UsbTx_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
/* 函数声明：将以零结尾字符串写入 USB 发送队列；timeout_ms 控制等待时间。 */
int UsbTx_SendString(const char *str, uint32_t timeout_ms);
/* 函数声明：格式化并通过 USB 输出调试信息；接口已实现，目标目录内未检索到调用。 */
int DEBUG_USB(const char *fmt, ...);
/* 函数声明：递增各设备心跳计数；实现保留，但当前周期任务中的调用被注释屏蔽。 */
void HB_Add(void);

typedef struct
{
	/* 累计进入 USB 发送队列的字节数。 */
	uint32_t tx_enqueued_bytes;
	/* 累计未能进入 USB 发送队列而丢弃的字节数。 */
	uint32_t tx_dropped_bytes;
	/* 累计成功发送的数据包数。 */
	uint32_t tx_ok_pkts;
	/* CDC 发送接口返回忙状态的累计次数。 */
	uint32_t tx_busy_cnt;
	/* 发送失败的数据包累计数。 */
	uint32_t tx_fail_pkts;
	/* 发送失败涉及的累计字节数。 */
	uint32_t tx_fail_bytes;
	/* USB 接收 ISR 成功写入原始接收流的累计字节数。 */
	uint32_t rx_isr_enqueued_bytes;
	/* USB 接收 ISR 未能写入原始接收流而丢弃的累计字节数。 */
	uint32_t rx_isr_dropped_bytes;
	/* 从原始接收流转发到应用接收流的累计字节数。 */
	uint32_t app_rx_forwarded_bytes;
	/* 应用接收流空间不足而丢弃的累计字节数。 */
	uint32_t app_rx_dropped_bytes;
} UsbStats_t;



/* 函数声明：获取 USB 收发统计快照；out 为空时不操作，目标目录内未检索到调用。 */
void Usb_GetStats(UsbStats_t *out);

/* ---------- USB RX ---------- */
/* 函数声明：USB CDC 接收 ISR 入口；将收到的数据提交给后台接收处理。 */
void UsbRx_OnCdcRxFromISR(const uint8_t *data, uint32_t len);
/* 函数声明：从 USB 应用接收缓冲区读取数据；接口已实现，目标目录内未检索到调用。 */
uint16_t UsbRx_Read(uint8_t *out, uint16_t out_len, uint32_t timeout_ms);
/* 函数声明：查询 USB 应用接收缓冲区当前可读字节数；目标目录内未检索到调用。 */
size_t UsbRx_Available(void);
/* 函数声明：读取 USB 接收队列统计计数；各输出参数可独立传 NULL，目标目录内未检索到调用。 */
void UsbRx_GetStats(uint32_t *rx_enqueued_bytes, uint32_t *rx_dropped_bytes);

/* 回调类型：收到 USB 应用数据时传递数据指针和长度；回调在接收任务上下文执行。 */
typedef void (*UsbRx_Callback_t)(const uint8_t *data, uint16_t len);
/* 函数声明：注册 USB 应用层接收回调；接口已实现，当前工程中的注册调用被注释屏蔽。 */
void UsbRx_RegisterCallback(UsbRx_Callback_t cb);

/* ---------- Process ---------- */

/*
 * 在 UsbRxTask 上下文触发的接收回调：把来自 PC 的字节流拼帧并解析。
 * 说明：USB CDC 是字节流，回调 data/len 不保证对齐到帧边界。
 */
/* 函数声明：解析 PC 下发字节流；当前实现及注册调用均被注释屏蔽，未启用。 */
void GetPCMsg(const uint8_t *data, uint16_t len);
/* 函数声明：组装并发送 PC 协议数据帧；当前函数实现及调用点均被注释屏蔽。 */
void SendPCMsg(void);

#ifdef __cplusplus
}
#endif

#endif // USER_COMMUNICATION_H