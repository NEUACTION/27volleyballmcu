#include "user_communication.h"
#include "user_control.h"
#include "user_usart.h"

#include "usbd_cdc_if.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

/*
 * ------------------------------------------------------------
 * USB CDC 通信层（user_communication）
 * ------------------------------------------------------------
 * 目标：
 *  - 为上层提供统一的 USB CDC 发送/接收接口，屏蔽 USBD_BUSY/时序细节。
 *
 * 线程模型：
 *  - usbTxTask：唯一的 CDC IN 发送线程。所有发送请求先入队到 s_usb_tx_stream，
 *              再由该线程调用 CDC_Transmit_HS() 发出；发送完成依赖
 *              CDC_TransmitCplt_HS 回调通过 UsbTx_OnCdcTxCpltFromISR() 通知。
 *  - usbRxTask：CDC OUT 接收处理线程。CDC_Receive_HS() 在 USB 协议栈上下文把
 *              Buf/Len 投递到 s_usb_rx_stream（通过 UsbRx_OnCdcRxFromISR()），
 *              再由该线程取出并：
 *                1) 转发到“应用层缓冲” s_usb_app_rx_stream（供 User_Task 轮询读取）
 *                2) 回调 s_usb_rx_cb（如注册，用于事件式处理）
 *
 * 注意：
 *  - CDC 是“字节流”，并不自带消息边界；上层需自行定义协议（帧头帧尾）。
 *  - ISR 上下文（*FromISR 函数）必须短小，不做格式化/阻塞操作。
 *  - UsbCdc_IsConfigured() 用于确保枚举完成后再收发，避免未配置时卡住/无回调。
 */

#define USB_TX_TASK_STACK_SIZE        (512U * 4U)
#define USB_TX_TASK_PRIORITY          ((osPriority_t)osPriorityAboveNormal)

#define USB_TX_STREAM_SIZE_BYTES      (2048U)
#define USB_TX_TRIGGER_LEVEL          (1U)

#define USB_RX_TASK_STACK_SIZE        (512U * 4U)
#define USB_RX_TASK_PRIORITY          ((osPriority_t)osPriorityAboveNormal)

#define USB_RX_STREAM_SIZE_BYTES      (2048U)
#define USB_RX_TRIGGER_LEVEL          (1U)

#define USB_APP_RX_STREAM_SIZE_BYTES  (1024U)
#define USB_APP_RX_TRIGGER_LEVEL      (1U)

//接收帧帧头帧尾
const char frame_header_read1  = 'S';
const char frame_header_read2  = 'P';
const char frame_tail_read1    = '\r';
const char frame_tail_read2    = '\n';

//发送帧帧头帧尾
const char frame_header_write1 = 'Q';
const char frame_header_write2 = 'R'; 
const char frame_tail_write1   = '\r';
const char frame_tail_write2   = '\n';

//发送缓冲区和接收缓冲区
trans_t TX_Data;		
trans_r RX_Data;		

/*
 * s_usb_tx_stream      : 发送队列（任意线程 -> usbTxTask）
 * s_usb_rx_stream      : 原始接收队列（USB ISR/协议栈 -> usbRxTask）
 */

static StreamBufferHandle_t s_usb_tx_stream = NULL;
static osThreadId_t s_usb_tx_task = NULL;
static TaskHandle_t s_usb_tx_task_handle = NULL;

static StreamBufferHandle_t s_usb_rx_stream = NULL;
static osThreadId_t s_usb_rx_task = NULL;
static UsbRx_Callback_t s_usb_rx_cb = NULL;

/* 发送统计：用于判断是否有丢包/拥塞（调试用途） */
static volatile uint32_t s_usb_enqueued_bytes = 0;
static volatile uint32_t s_usb_dropped_bytes = 0;
static volatile uint32_t s_usb_tx_ok_pkts = 0;
static volatile uint32_t s_usb_tx_busy_cnt = 0;
static volatile uint32_t s_usb_tx_fail_pkts = 0;
static volatile uint32_t s_usb_tx_fail_bytes = 0;

/* 接收统计：记录进入 RX 缓冲的字节数与丢弃字节数（调试用途） */
static volatile uint32_t s_usb_rx_enqueued_bytes = 0;
static volatile uint32_t s_usb_rx_dropped_bytes = 0;
static volatile uint32_t s_usb_app_rx_forwarded_bytes = 0;
static volatile uint32_t s_usb_app_rx_dropped_bytes = 0;

static void UsbTxTask(void *argument);
static void UsbRxTask(void *argument);

/**
 * @brief  初始化 USB 通信相关资源（任务、缓冲区等）
 * @note   记得调用
 */
void User_Communication_Init(void)
{
	/* 创建 stream buffer（只创建一次）；失败则保持 NULL，上层可据此做降级处理 */
	if (s_usb_tx_stream == NULL)
	{
		s_usb_tx_stream = xStreamBufferCreate(USB_TX_STREAM_SIZE_BYTES, USB_TX_TRIGGER_LEVEL);
	}
	if (s_usb_rx_stream == NULL)
	{
		s_usb_rx_stream = xStreamBufferCreate(USB_RX_STREAM_SIZE_BYTES, USB_RX_TRIGGER_LEVEL);
	}

	if ((s_usb_tx_task == NULL) && (s_usb_tx_stream != NULL))
	{
		/* 启动 USB 发送线程（唯一发送者，避免并发 CDC_Transmit 导致 USBD_BUSY 混乱） */
		const osThreadAttr_t usbTxTask_attributes = {
			.name = "usbTxTask",
			.stack_size = USB_TX_TASK_STACK_SIZE,
			.priority = USB_TX_TASK_PRIORITY,
		};
		s_usb_tx_task = osThreadNew(UsbTxTask, NULL, &usbTxTask_attributes);
	}

	if ((s_usb_rx_task == NULL) && (s_usb_rx_stream != NULL))
	{
		/* 启动 USB 接收线程（把 ISR 中的字节流搬运到任务上下文处理） */
		const osThreadAttr_t usbRxTask_attributes = {
			.name = "usbRxTask",
			.stack_size = USB_RX_TASK_STACK_SIZE,
			.priority = USB_RX_TASK_PRIORITY,
		};
		s_usb_rx_task = osThreadNew(UsbRxTask, NULL, &usbRxTask_attributes);
	}
}

/**
 * @brief  判断 CDC 是否已配置完成
 * @note   dev_state == CONFIGURED 且 class data 就绪，才允许开始收发。
 */
bool UsbCdc_IsConfigured(void)
{
	return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) && (hUsbDeviceHS.pClassData != NULL);
}

/**
 * @brief  判断 USB CDC 发送是否准备好
 */
bool UsbTx_IsReady(void)
{
	return UsbCdc_IsConfigured();
}

/**
 * @brief  获取 USB 发送统计信息
 * @param  enqueued_bytes 已入队列的字节数
 * @param  dropped_bytes 丢弃的字节数
 * @param  tx_ok_pkts 成功发送的包数
 * @param  tx_busy_cnt 发送忙碌计数
 */
void UsbTx_GetStats(uint32_t *enqueued_bytes, uint32_t *dropped_bytes, uint32_t *tx_ok_pkts, uint32_t *tx_busy_cnt)
{
	if (enqueued_bytes != NULL)
	{
		*enqueued_bytes = s_usb_enqueued_bytes;
	}
	if (dropped_bytes != NULL)
	{
		*dropped_bytes = s_usb_dropped_bytes;
	}
	if (tx_ok_pkts != NULL)
	{
		*tx_ok_pkts = s_usb_tx_ok_pkts;
	}
	if (tx_busy_cnt != NULL)
	{
		*tx_busy_cnt = s_usb_tx_busy_cnt;
	}
}

/**
 * @brief 获取USB通信统计信息
 * @param out 指向UsbStats_t结构体的指针，用于存储获取到的统计信息
 * @note 如果传入的指针为NULL，则函数直接返回，不进行任何操作
 */
void Usb_GetStats(UsbStats_t *out)
{
	// 检查输出指针是否有效
	if (out == NULL)
	{
		return;
	}



	// 填充发送相关的统计信息
	out->tx_enqueued_bytes = s_usb_enqueued_bytes;      // 已入队的发送字节数
	out->tx_dropped_bytes = s_usb_dropped_bytes;        // 已丢弃的发送字节数
	out->tx_ok_pkts = s_usb_tx_ok_pkts;                 // 成功发送的数据包数量
	out->tx_busy_cnt = s_usb_tx_busy_cnt;               // 发送忙碌计数
	out->tx_fail_pkts = s_usb_tx_fail_pkts;             // 发送失败的数据包数量
	out->tx_fail_bytes = s_usb_tx_fail_bytes;           // 发送失败的字节数


	// 填充接收相关的统计信息
	out->rx_isr_enqueued_bytes = s_usb_rx_enqueued_bytes;      // 中断服务例程中入队的接收字节数
	out->rx_isr_dropped_bytes = s_usb_rx_dropped_bytes;        // 中断服务例程中丢弃的接收字节数
	out->app_rx_forwarded_bytes = s_usb_app_rx_forwarded_bytes; // 应用层转发的接收字节数
	out->app_rx_dropped_bytes = s_usb_app_rx_dropped_bytes;     // 应用层丢弃的接收字节数
}

/**
 * @brief  CDC IN 传输完成回调触发（ISR上下文调用）
 */
/**
 * @brief USB发送完成中断服务函数
 * 当CDC USB发送完成时，此函数从中断服务程序(ISR)中被调用
 * 它会通知USB发送任务，可能触发任务切换
 */
void UsbTx_OnCdcTxCpltFromISR(void)
{
    // 创建一个标志变量，用于判断是否需要进行任务切换
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 检查USB发送任务句柄是否有效
	if (s_usb_tx_task_handle != NULL)
	{
        // 从中断服务程序中向任务发送通知
        // 这会使任务从阻塞状态变为就绪状态
		vTaskNotifyGiveFromISR(s_usb_tx_task_handle, &xHigherPriorityTaskWoken);
        // 如果需要，进行任务切换
        // 只有当xHigherPriorityTaskWoken为pdTRUE时才会切换
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * @brief  异步投递数据到USB发送任务
 * @retval 0:成功 负数:失败
 */
int UsbTx_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
	/*
	 * 将数据投递到发送队列，由 usbTxTask 统一发送。
	 * timeout_ms == 0：不阻塞尝试一次入队，失败立即返回。
	 */
	if ((s_usb_tx_stream == NULL) || (data == NULL) || (len == 0U))
	{
		return -1;
	}

	const TickType_t start_tick = xTaskGetTickCount();
	const TickType_t timeout_ticks = ms_to_ticks(timeout_ms);
	uint32_t offset = 0U;

	while (offset < (uint32_t)len)
	{
		TickType_t remain;
		if (timeout_ticks == 0U)
		{
			/* 不阻塞：允许尝试一次发送，失败再返回 */
			remain = 0U;
		}
		else if (timeout_ticks == portMAX_DELAY)
		{
			remain = portMAX_DELAY;
		}
		else
		{
			const TickType_t elapsed = xTaskGetTickCount() - start_tick;
			if (elapsed >= timeout_ticks)
			{
				return -2;
			}
			remain = timeout_ticks - elapsed;
		}

		const size_t sent = xStreamBufferSend(
			s_usb_tx_stream,
			(const void *)&data[offset],
			(size_t)((uint32_t)len - offset),
			remain);

		if (sent == 0U)
		{
			s_usb_dropped_bytes += (uint32_t)len - offset;
			return -2;
		}
		s_usb_enqueued_bytes += (uint32_t)sent;
		offset += (uint32_t)sent;
	}

	return 0;
}

/**
 * @brief  发送字符串（不自动追加\r\n）
 */
int UsbTx_SendString(const char *str, uint32_t timeout_ms)
{
	if (str == NULL)
	{
		return -1;
	}
	return UsbTx_Send((const uint8_t *)str, (uint16_t)strlen(str), timeout_ms);
}

int DEBUG_USB(const char *fmt, ...)
{
	/* printf 风格调试输出：格式化到栈上缓冲并异步投递（默认不阻塞） */
	if (fmt == NULL)
	{
		return -1;
	}
	if (!UsbCdc_IsConfigured())
	{
		return -3;
	}

	char out[256];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(out, sizeof(out), fmt, args);
	va_end(args);

	if (n < 0)
	{
		return -1;
	}

	if ((size_t)n >= sizeof(out))
	{
		/* 截断发送（vsnprintf 已确保以 '\0' 结尾） */
		n = (int)sizeof(out) - 1;
	}

	/* 0ms：不阻塞，避免影响实时任务；满了就丢 */
	return UsbTx_Send((const uint8_t *)out, (uint16_t)n, 0U);
}
/**
 * @brief  注册接收回调
 */
void UsbRx_RegisterCallback(UsbRx_Callback_t cb)
{
	/* 回调在 usbRxTask 上下文触发；可用于事件式处理（解析命令/统计等） */
	s_usb_rx_cb = cb;
}

/**
 * @brief  USB 接收回调
 * @param data 接收数据
 * @param len 数据长度
 */
void UsbRx_OnCdcRxFromISR(const uint8_t *data, uint32_t len)
{
	/*
	 * 在 CDC_Receive_HS() 中调用（USB 协议栈上下文/近似 ISR 约束）：
	 * 只做“快速入队 + 统计”，不要做耗时处理。
	 */
	if ((s_usb_rx_stream == NULL) || (data == NULL) || (len == 0U))
	{
		return;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	const size_t written = xStreamBufferSendFromISR(s_usb_rx_stream, data, (size_t)len, &xHigherPriorityTaskWoken);
	s_usb_rx_enqueued_bytes += (uint32_t)written;
	if (written < (size_t)len)
	{
		s_usb_rx_dropped_bytes += (uint32_t)((size_t)len - written);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  供应用层读取 USB 接收数据
 * @param  out 输出缓冲区
 * @param  out_len 输出缓冲区长度
 * @param  timeout_ms 超时时间，单位毫秒
 * @retval uint16_t 接收字节数
 */
uint16_t UsbRx_Read(uint8_t *out, uint16_t out_len, uint32_t timeout_ms)
{
	if ((s_usb_rx_stream == NULL) || (out == NULL) || (out_len == 0U))
	{
		return 0;
	}

	const TickType_t t = ms_to_ticks(timeout_ms);
	const size_t n = xStreamBufferReceive(s_usb_rx_stream, out, (size_t)out_len, t);
	return (uint16_t)n;
}

/**
 * @brief  获取应用层 USB 接收缓冲区中可用的数据字节数
 * @retval size_t 可用字节数
 */
size_t UsbRx_Available(void)
{
	if (s_usb_rx_stream == NULL)
	{
		return 0;
	}
	return xStreamBufferBytesAvailable(s_usb_rx_stream);
}

/**
 * @brief  获取 USB 接收统计数据
 * @param rx_enqueued_bytes 已入队字节数
 * @param rx_dropped_bytes 丢弃字节数
 */
void UsbRx_GetStats(uint32_t *rx_enqueued_bytes, uint32_t *rx_dropped_bytes)
{
	if (rx_enqueued_bytes != NULL)
	{
		*rx_enqueued_bytes = s_usb_rx_enqueued_bytes;
	}
	if (rx_dropped_bytes != NULL)
	{
		*rx_dropped_bytes = s_usb_rx_dropped_bytes;
	}
}

/**
 * @brief  处理来自 PC 的消息（UsbRxTask 回调触发）
 */
// void GetPCMsg(const uint8_t *data, uint16_t len)
// {
// 	if ((data == NULL) || (len == 0U))
// 	{
// 		return;
// 	}

// 	/*
// 	 * CDC OUT 可能把一帧拆成多次回调，也可能多帧粘连；
// 	 * 这里用一个滑动缓存拼接字节流，并保持“只解析最新完整帧”的行为。
// 	 */
// 	static uint8_t s_rx_cache[MAX_GET_LEN];
// 	static uint16_t s_rx_cache_len = 0U;

// 	/* 追加到缓存：超出则丢弃最旧数据 */
// 	uint16_t copy_len = len;
// 	if (copy_len > (uint16_t)sizeof(s_rx_cache))
// 	{
// 		/* 过长：只保留最后 MAX_GET_LEN 字节 */
// 		data += (uint16_t)(copy_len - (uint16_t)sizeof(s_rx_cache));
// 		copy_len = (uint16_t)sizeof(s_rx_cache);
// 		s_rx_cache_len = 0U;
// 	}

// 	if ((uint32_t)s_rx_cache_len + (uint32_t)copy_len > (uint32_t)sizeof(s_rx_cache))
// 	{
// 		const uint16_t overflow = (uint16_t)(((uint32_t)s_rx_cache_len + (uint32_t)copy_len) - (uint32_t)sizeof(s_rx_cache));
// 		if (overflow >= s_rx_cache_len)
// 		{
// 			s_rx_cache_len = 0U;
// 		}
// 		else
// 		{
// 			memmove(&s_rx_cache[0], &s_rx_cache[overflow], (size_t)(s_rx_cache_len - overflow));
// 			s_rx_cache_len = (uint16_t)(s_rx_cache_len - overflow);
// 		}
// 	}

// 	memcpy(&s_rx_cache[s_rx_cache_len], data, (size_t)copy_len);
// 	s_rx_cache_len = (uint16_t)(s_rx_cache_len + copy_len);

// 	if (s_rx_cache_len < (uint16_t)USB_RX_LEN)
// 	{
// 		return;
// 	}

// 	uint8_t pcdata[USB_RX_LEN] = {};
// 	bool found = false;
// 	int found_end = -1;

// 	/* 只解析最新帧：从末尾往前找最后一个完整帧（固定长度 USB_RX_LEN） */
// 	for (int end = (int)s_rx_cache_len - 1; end >= ((int)USB_RX_LEN - 1); end--)
// 	{
// 		const int start = end - ((int)USB_RX_LEN - 1);
// 		if ((s_rx_cache[start] == (uint8_t)frame_header_read1) && (s_rx_cache[start + 1] == (uint8_t)frame_header_read2) &&
// 			(s_rx_cache[end - 1] == (uint8_t)frame_tail_read1) && (s_rx_cache[end] == (uint8_t)frame_tail_read2))
// 		{
// 			memcpy(pcdata, &s_rx_cache[start], (size_t)USB_RX_LEN);
// 			found = true;
// 			found_end = end;
// 			break;
// 		}
// 	}

// 	if (!found)
// 	{
// 		return;
// 	}

// 	/* 丢弃到 found_end（含）之前的数据，避免重复处理同一帧 */
// 	{
// 		const uint16_t consume = (uint16_t)(found_end + 1);
// 		if (consume >= s_rx_cache_len)
// 		{
// 			s_rx_cache_len = 0U;
// 		}
// 		else
// 		{
// 			memmove(&s_rx_cache[0], &s_rx_cache[consume], (size_t)(s_rx_cache_len - consume));
// 			s_rx_cache_len = (uint16_t)(s_rx_cache_len - consume);
// 		}
// 	}

// 	// TODO 在这里处理 pcdata
// 	(void)pcdata;

// 	/* 帧格式：2 字节帧头 + USB_RX_UINT8_NUM 字节控制 + payload + 2 字节帧尾 */
// 	const uint32_t ctrlBase = 2U;
// 	HB.PC = 0;//心跳包归零表示收到

// 	gRobot.runSucceedFlag = (pcdata[ctrlBase] >> 0) & 0x01;
	
// 	leg1Tar.mode = (pcdata[ctrlBase + 1] >> 0) & 0x03;
// 	leg2Tar.mode = (pcdata[ctrlBase + 1] >> 2) & 0x03;
// 	leg3Tar.mode = (pcdata[ctrlBase + 1] >> 4) & 0x03;
// 	leg4Tar.mode = (pcdata[ctrlBase + 1] >> 6) & 0x03;

// 	wheel1Tar.mode = (pcdata[ctrlBase + 2] >> 0) & 0x03;
// 	wheel2Tar.mode = (pcdata[ctrlBase + 2] >> 2) & 0x03;
// 	wheel3Tar.mode = (pcdata[ctrlBase + 2] >> 4) & 0x03;
// 	wheel4Tar.mode = (pcdata[ctrlBase + 2] >> 6) & 0x03;
	
// 	valve_num[4]  = (pcdata[ctrlBase + 3] >> 0) & 0x01;// 倒
// 	valve_num[7]  = (pcdata[ctrlBase + 3] >> 1) & 0x01;// 伸缩
// 	valve_num[9] = (pcdata[ctrlBase + 3] >> 2) & 0x01;// 夹
// 	valve_num[8]  = (pcdata[ctrlBase + 3] >> 3) & 0x01;// 压块
// 	valve_num[14]  = (pcdata[ctrlBase + 3] >> 4) & 0x01;// 缓冲
// 	valve_num[10] = (pcdata[ctrlBase + 3] >> 5) & 0x01;//
// 	valve_num[3] = (pcdata[ctrlBase + 3] >> 6) & 0x01;// 从动
// 	valve_num[11] = (pcdata[ctrlBase + 3] >> 7) & 0x01;// 夹爪
	
// 	shoulderTar.mode       = (pcdata[ctrlBase + 4] >> 0) & 0x03;
// 	elbowTar.mode = (pcdata[ctrlBase + 4] >> 2) & 0x03;
// 	wristTar.mode      = (pcdata[ctrlBase + 4] >> 4) & 0x03;

// 	valve_num[0] = (pcdata[ctrlBase + 5] >> 0) & 0x01;//红外1
// 	valve_num[5] = (pcdata[ctrlBase + 5] >> 1) & 0x01;//红外2
// 	//联合体接收
// 	for(uint16_t i = USB_RX_UINT8_NUM + ctrlBase; i < USB_RX_LEN - 2; i++)
// 	{
// 		RX_Data.datac[i - (USB_RX_UINT8_NUM + ctrlBase)] = pcdata[i];
// 	}
	
// 	gRobot.periodCntPC = RX_Data.dataint32[0];//PC运行时间
// 	leg1Tar.pos = RX_Data.dataf[1];
// 	leg1Tar.vel = RX_Data.dataf[2];
// 	leg1Tar.tor = RX_Data.dataf[3];
// 	leg1Tar.pos_kp = RX_Data.dataf[4] * 60.0;
// 	leg1Tar.pos_kd = RX_Data.dataf[5] * 0.1;
// 	leg1Tar.vel_kp = RX_Data.dataf[6];
// 	leg1Tar.vel_ki = RX_Data.dataf[7];

// 	leg2Tar.pos = RX_Data.dataf[8];
// 	leg2Tar.vel = RX_Data.dataf[9];
// 	leg2Tar.tor = RX_Data.dataf[10];
// 	leg2Tar.pos_kp = RX_Data.dataf[11] * 60.0;
// 	leg2Tar.pos_kd = RX_Data.dataf[12] * 0.1;
// 	leg2Tar.vel_kp = RX_Data.dataf[13];
// 	leg2Tar.vel_ki = RX_Data.dataf[14];

// 	leg3Tar.pos = RX_Data.dataf[15];
// 	leg3Tar.vel = RX_Data.dataf[16];
// 	leg3Tar.tor = RX_Data.dataf[17];
// 	leg3Tar.pos_kp = RX_Data.dataf[18] * 60.0;
// 	leg3Tar.pos_kd = RX_Data.dataf[19] * 0.1;
// 	leg3Tar.vel_kp = RX_Data.dataf[20];
// 	leg3Tar.vel_ki = RX_Data.dataf[21];

// 	leg4Tar.pos = RX_Data.dataf[22];
// 	leg4Tar.vel = RX_Data.dataf[23];
// 	leg4Tar.tor = RX_Data.dataf[24];
// 	leg4Tar.pos_kp = RX_Data.dataf[25] * 60.0;
// 	leg4Tar.pos_kd = RX_Data.dataf[26] * 0.1;
// 	leg4Tar.vel_kp = RX_Data.dataf[27];
// 	leg4Tar.vel_ki = RX_Data.dataf[28];

// 	shoulderTar.pos = RX_Data.dataf[29];
// 	shoulderTar.vel = RX_Data.dataf[30];
// 	shoulderTar.tor = RX_Data.dataf[31];
// 	shoulderTar.pos_kp = RX_Data.dataf[32] * 60.0;
// 	shoulderTar.pos_kd = RX_Data.dataf[33] * 0.1;
// 	shoulderTar.vel_kp = RX_Data.dataf[34];
// 	shoulderTar.vel_ki = RX_Data.dataf[35];
	
// 	elbowTar.pos = RX_Data.dataf[36] * 2.0 * PI;
// 	elbowTar.vel = RX_Data.dataf[37] * 2.0 * PI;
	
// 	wristTar.pos = RX_Data.dataf[38] * 2.0 * PI;
// 	wristTar.vel = RX_Data.dataf[39] * 2.0 * PI;

// 	wheel1Tar.pos = RX_Data.dataf[40];
// 	wheel1Tar.vel = RX_Data.dataf[41];
// 	wheel1Tar.tor = RX_Data.dataf[42];
// 	wheel1Tar.pos_kp = RX_Data.dataf[43];
// 	wheel1Tar.pos_kd = RX_Data.dataf[44];
// 	wheel1Tar.vel_kp = RX_Data.dataf[45];
// 	wheel1Tar.vel_ki = RX_Data.dataf[46];

// 	wheel2Tar.pos = RX_Data.dataf[47];
// 	wheel2Tar.vel = RX_Data.dataf[48];
// 	wheel2Tar.tor = RX_Data.dataf[49];
// 	wheel2Tar.pos_kp = RX_Data.dataf[50];
// 	wheel2Tar.pos_kd = RX_Data.dataf[51];
// 	wheel2Tar.vel_kp = RX_Data.dataf[52];
// 	wheel2Tar.vel_ki = RX_Data.dataf[53];

// 	wheel3Tar.pos = RX_Data.dataf[54];
// 	wheel3Tar.vel = RX_Data.dataf[55];
// 	wheel3Tar.tor = RX_Data.dataf[56];
// 	wheel3Tar.pos_kp = RX_Data.dataf[57];
// 	wheel3Tar.pos_kd = RX_Data.dataf[58];
// 	wheel3Tar.vel_kp = RX_Data.dataf[59];
// 	wheel3Tar.vel_ki = RX_Data.dataf[60];

// 	wheel4Tar.pos = RX_Data.dataf[61];
// 	wheel4Tar.vel = RX_Data.dataf[62];
// 	wheel4Tar.tor = RX_Data.dataf[63];
// 	wheel4Tar.pos_kp = RX_Data.dataf[64];
// 	wheel4Tar.pos_kd = RX_Data.dataf[65];
// 	wheel4Tar.vel_kp = RX_Data.dataf[66];
// 	wheel4Tar.vel_ki = RX_Data.dataf[67];

// 	wheel1Tar.clean = RX_Data.dataf[68];
// 	wheel2Tar.clean = RX_Data.dataf[69];
// 	wheel3Tar.clean = RX_Data.dataf[70];
// 	wheel4Tar.clean = RX_Data.dataf[71];
	
// 	memset(pcdata, 0, sizeof(pcdata));
// }

/**
 * @brief 发送消息到 PC
 */
// void SendPCMsg(void)
// {
// 	uint8_t data[USB_TX_LEN] = {};
// 	data[0] = frame_header_write1;//帧头2位
// 	data[1] = frame_header_write2;
// 	data[USB_TX_LEN-2] = frame_tail_write1;//帧尾2位
// 	data[USB_TX_LEN-1] = frame_tail_write2;
	
// 	//心跳包
// 	data[2] = HB.PC;
// 	data[3] = HB.leg1;
// 	data[4] = HB.leg2;
// 	data[5] = HB.leg3;
// 	data[6] = HB.leg4;
// 	data[7] = HB.wheel1;
// 	data[8] = HB.wheel2;
// 	data[9] = HB.wheel3;
// 	data[10]= HB.wheel4;
// 	data[11]= HB.shoulder;
// 	data[12]= HB.elbow;
// 	data[13]= HB.wrist;
// 	data[14] = (ps_num[7] & 0x01) << 0 | (ps_num[11] & 0x01) << 1 |
// 							(ps_num[9] & 0x01) << 2 | (ps_num[3] & 0x01) << 3 |
// 							(ps_num[4] & 0x01) << 4 | (ps_num[8] & 0x01) << 5 |
// 							(ps_num[12] & 0x01) << 6 | (ps_num[13] & 0x01) << 7;
// 	data[15] = (ps_num[2] & 0x01) << 0 | (ps_num[1] & 0x01) << 1 | //  行程开关车尾
// 							(ps_num[0] & 0x01) << 2 | (ps_num[14] & 0x01) << 3 |//  左侧 右侧
// 							(ps_num[10] & 0x01) << 4 | (ps_num[5] & 0x01) << 5 |// 行程开关车头（右左）
// 							(ps_num[6] & 0x01) << 6;// 爪子
// 	data[16] = (ps_num_2[5] & 0x01) << 0 | (ps_num_2[13] & 0x01) << 1 | // 看R1(右左)
// 							(ps_num_2[0] & 0x01) << 2 | (ps_num_2[4] & 0x01) << 3; //侧边行程开关（前后）
// 	data[17] = gRobot.reset_flag;

// 	data[18] = leg1Msg.error;
// 	data[19] = leg2Msg.error;
// 	data[20] = leg3Msg.error;
// 	data[21] = leg4Msg.error;
// 	data[22] = wheel1Msg.error;
// 	data[23] = wheel2Msg.error;
// 	data[24] = wheel3Msg.error;
// 	data[25] = wheel4Msg.error;
// 	data[26] = shoulderMsg.error;

// 	data[27] = leg1Msg.enableState;
// 	data[28] = leg2Msg.enableState;
// 	data[29] = leg3Msg.enableState;
// 	data[30] = leg4Msg.enableState;

// 	TX_Data.data_float[1] = leg1Msg.pos;
// 	TX_Data.data_float[2] = leg1Msg.vel;
// 	TX_Data.data_float[3] = leg1Msg.currQ;

// 	TX_Data.data_float[4] = leg2Msg.pos;
// 	TX_Data.data_float[5] = leg2Msg.vel;
// 	TX_Data.data_float[6] = leg2Msg.currQ;

// 	TX_Data.data_float[7] = leg3Msg.pos;
// 	TX_Data.data_float[8] = leg3Msg.vel;
// 	TX_Data.data_float[9] = leg3Msg.currQ;
	
// 	TX_Data.data_float[10] = leg4Msg.pos;
// 	TX_Data.data_float[11] = leg4Msg.vel;
// 	TX_Data.data_float[12] = leg4Msg.currQ;

// 	TX_Data.data_float[13] = wheel1Msg.pos;
// 	TX_Data.data_float[14] = wheel1Msg.vel;
// 	TX_Data.data_float[15] = wheel1Msg.currQ;

// 	TX_Data.data_float[16] = wheel2Msg.pos;
// 	TX_Data.data_float[17] = wheel2Msg.vel;
// 	TX_Data.data_float[18] = wheel2Msg.currQ;

// 	TX_Data.data_float[19] = wheel3Msg.pos;
// 	TX_Data.data_float[20] = wheel3Msg.vel;
// 	TX_Data.data_float[21] = wheel3Msg.currQ;

// 	TX_Data.data_float[22] = wheel4Msg.pos;
// 	TX_Data.data_float[23] = wheel4Msg.vel;
// 	TX_Data.data_float[24] = wheel4Msg.currQ;
	
// 	TX_Data.data_float[25] = shoulderMsg.pos;
// 	TX_Data.data_float[26] = shoulderMsg.vel;
// 	TX_Data.data_float[27] = shoulderMsg.currQ;
	
// 	TX_Data.data_float[28] = elbowMsg.pos / 2.f / PI;
// 	TX_Data.data_float[29] = elbowMsg.vel / 2.f / PI;
	
// 	TX_Data.data_float[30] = wristMsg.pos / 2.f / PI;
// 	TX_Data.data_float[31] = wristMsg.vel / 2.f / PI;     

// 	//联合体发送
// 	for(uint16_t i = (uint16_t)(2 + USB_TX_UINT8_NUM); i < (uint16_t)(USB_TX_LEN - 2); i++)
// 	{
// 		data[i] = (uint8_t)TX_Data.datac[i - (uint16_t)(2 + USB_TX_UINT8_NUM)];
// 	}

// 	UsbTx_Send(data, USB_TX_LEN, 0U);
// }

/**
 * @brief  USB 发送任务
 * @param argument 任务参数
 * @note   一直运行
 */
static void UsbTxTask(void *argument)
{
	(void)argument;

	/* 记录句柄用于 ISR 通知 */
	s_usb_tx_task_handle = xTaskGetCurrentTaskHandle();

	/* 等待USB枚举到 CONFIGURED 状态，避免过早发送导致异常/无回调 */
	osDelay(200);
	while (!UsbCdc_IsConfigured())
	{
		osDelay(10);
	}

	uint8_t buf[256];

	for (;;)
	{
		if (!UsbCdc_IsConfigured())
		{
			osDelay(10);
			continue;
		}

		const size_t n = xStreamBufferReceive(s_usb_tx_stream, buf, sizeof(buf), portMAX_DELAY);
		if (n == 0U)
		{
			continue;
		}

		/* 提交一次 IN 传输，并等待发送完成回调通知；避免 1ms 轮询 */
		const TickType_t start = xTaskGetTickCount();
		bool sent_ok = false;
		for (;;)
		{
			const uint8_t ret = CDC_Transmit_HS(buf, (uint16_t)n);
			if (ret == USBD_OK)
			{
				s_usb_tx_ok_pkts++;
				sent_ok = true;
				/* 等待 CDC_TransmitCplt_HS 通知，确保 buf 生命周期覆盖本次传输 */
				(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
				break;
			}
			if (ret != USBD_BUSY)
			{
				break;
			}
			s_usb_tx_busy_cnt++;

			/* Busy：等待上一包完成通知，再重试 */
			(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
			if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(200))
			{
				break;
			}
		}
		if (!sent_ok)
		{
			s_usb_tx_fail_pkts++;
			s_usb_tx_fail_bytes += (uint32_t)n;
		}
	}
}

/**
 * @brief  USB 接收任务
 * @param argument 任务参数
 * @note   一直运行
 */
static void UsbRxTask(void *argument)
{
	(void)argument;

	/* 等待USB枚举完成再处理接收（避免没连上时空转） */
	osDelay(200);
	while (!UsbTx_IsReady())
	{
		osDelay(10);
	}

	uint8_t buf[256];
	for (;;)
	{
		if (!UsbTx_IsReady())
		{
			osDelay(10);
			continue;
		}

		const size_t n = xStreamBufferReceive(s_usb_rx_stream, buf, sizeof(buf), portMAX_DELAY);
		if (n == 0U)
		{
			continue;
		}

		/* 回调，在任务上下文触发 */
		UsbRx_Callback_t cb = s_usb_rx_cb;
		if (cb != NULL)
		{
			cb(buf, (uint16_t)n);
		}
	}
}