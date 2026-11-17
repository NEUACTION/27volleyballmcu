#include "user_FDCAN.h"

/*
 * ------------------------------------------------------------
 * User_FDCAN：FDCAN1/FDCAN3 收发任务与接口
 * ------------------------------------------------------------
 * 约束：
 *  - FDCAN1 接收：FIFO0
 *  - FDCAN3 接收：FIFO1
 *
 * 当前仅保留“任务 RX 模式”：
 *  - ISR 仅负责通知 rxTask
 *  - rxTask 负责从 FIFO 搬运并回调到上层解析（任务上下文）
 */

#define FDCAN_TX_TASK_STACK_SIZE           (512U * 4U)
#define FDCAN_TX_TASK_PRIORITY             ((osPriority_t)osPriorityAboveNormal)

#define FDCAN_RX_TASK_STACK_SIZE           (512U * 4U)
#define FDCAN_RX_TASK_PRIORITY             ((osPriority_t)osPriorityAboveNormal)

#define FDCAN_TX_QUEUE_LEN                 (32U)
// #define FDCAN1_TX_INTERFRAME_DELAY_US      (500U)
// #define FDCAN3_TX_INTERFRAME_DELAY_US      (500U) 

#define FDCAN1_TX_INTERFRAME_DELAY_US      (200U)
#define FDCAN3_TX_INTERFRAME_DELAY_US      (450U) 

// #define FDCAN1_TX_INTERFRAME_DELAY_US      (0U)
// #define FDCAN3_TX_INTERFRAME_DELAY_US      (0U)

#define FDCAN_NOTIFY_RX1_FIFO0             (1UL << 0)
#define FDCAN_NOTIFY_RX3_FIFO1             (1UL << 1)

typedef struct
{
	QueueHandle_t queue;
	FDCAN_HandleTypeDef *bus;
	uint32_t interframe_delay_us;
} FdcanTxTaskCtx_t;

static QueueHandle_t s_fdcan1_tx_queue = NULL;
static QueueHandle_t s_fdcan3_tx_queue = NULL;

static osThreadId_t s_fdcan1_tx_task = NULL;
static osThreadId_t s_fdcan3_tx_task = NULL;
static osThreadId_t s_fdcan_rx_task = NULL;
static TaskHandle_t s_fdcan_rx_task_handle = NULL;

static FdcanTxTaskCtx_t s_fdcan1_tx_ctx = {0};
static FdcanTxTaskCtx_t s_fdcan3_tx_ctx = {0};

static UserFDCAN_RxCallback_t s_fdcan_rx_cb = NULL;

static uint8_t Fdcan_CopyRxToMsg(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, UserFDCAN_RxMsg_t *out)
{
	if ((hfdcan == NULL) || (out == NULL))
	{
		return 0U;
	}

	uint8_t data[64];
	if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &out->header, data) != HAL_OK)
	{
		return 0U;
	}
	out->bus = hfdcan;
	out->data_len = Fdcan_DlcToBytes(out->header.DataLength);
	if (out->data_len > 64U)
	{
		out->data_len = 64U;
	}
	if (out->data_len > 0U)
	{
		memcpy(out->data, data, out->data_len);
	}
	return 1U;
}


static void FdcanTxTask(void *argument);
static void FdcanRxTask(void *argument);

static void Fdcan_DelayUs(uint32_t us)
{
	if (us == 0U)
	{
		return;
	}

	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	}

	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
	{
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}

	uint32_t cycles_per_us = SystemCoreClock / 1000000U;
	if (cycles_per_us == 0U)
	{
		return;
	}

	uint32_t start = DWT->CYCCNT;
	uint32_t wait_cycles = cycles_per_us * us;
	while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles)
	{
	}
}

uint32_t Fdcan_BytesToDlc(uint8_t len)
{
	if (len <= 0U)
		return FDCAN_DLC_BYTES_0;
	if (len <= 1U)
		return FDCAN_DLC_BYTES_1;
	if (len <= 2U)
		return FDCAN_DLC_BYTES_2;
	if (len <= 3U)
		return FDCAN_DLC_BYTES_3;
	if (len <= 4U)
		return FDCAN_DLC_BYTES_4;
	if (len <= 5U)
		return FDCAN_DLC_BYTES_5;
	if (len <= 6U)
		return FDCAN_DLC_BYTES_6;
	if (len <= 7U)
		return FDCAN_DLC_BYTES_7;
	if (len <= 8U)
		return FDCAN_DLC_BYTES_8;
	if (len <= 12U)
		return FDCAN_DLC_BYTES_12;
	if (len <= 16U)
		return FDCAN_DLC_BYTES_16;
	if (len <= 20U)
		return FDCAN_DLC_BYTES_20;
	if (len <= 24U)
		return FDCAN_DLC_BYTES_24;
	if (len <= 32U)
		return FDCAN_DLC_BYTES_32;
	if (len <= 48U)
		return FDCAN_DLC_BYTES_48;
	return FDCAN_DLC_BYTES_64;
}

uint8_t Fdcan_DlcToBytes(uint32_t dlc)
{
	switch (dlc)
	{
	case FDCAN_DLC_BYTES_0:
		return 0;
	case FDCAN_DLC_BYTES_1:
		return 1;
	case FDCAN_DLC_BYTES_2:
		return 2;
	case FDCAN_DLC_BYTES_3:
		return 3;
	case FDCAN_DLC_BYTES_4:
		return 4;
	case FDCAN_DLC_BYTES_5:
		return 5;
	case FDCAN_DLC_BYTES_6:
		return 6;
	case FDCAN_DLC_BYTES_7:
		return 7;
	case FDCAN_DLC_BYTES_8:
		return 8;
	case FDCAN_DLC_BYTES_12:
		return 12;
	case FDCAN_DLC_BYTES_16:
		return 16;
	case FDCAN_DLC_BYTES_20:
		return 20;
	case FDCAN_DLC_BYTES_24:
		return 24;
	case FDCAN_DLC_BYTES_32:
		return 32;
	case FDCAN_DLC_BYTES_48:
		return 48;
	case FDCAN_DLC_BYTES_64:
		return 64;
	default:
		return 0;
	}
}

void CAN_Error_Handle(FDCAN_HandleTypeDef *hfdcan)
{
	while(1)
	{
		HAL_FDCAN_DeInit(hfdcan);
        // 重新配置并启动
        if (HAL_FDCAN_Init(hfdcan) == HAL_OK)
        {
            HAL_FDCAN_Start(hfdcan);
        }
    }
}

/**
 * @brief 注册接收回调（在 rxTask 上下文触发）
 */
void User_FDCAN_RegisterRxCallback(UserFDCAN_RxCallback_t cb)
{
	s_fdcan_rx_cb = cb;
}

/**
 * @brief FDCAN 发送函数
 * @param hfdcan FDCAN 句柄
 * @param data 数据指针
 * @param data_len 数据长度
 * @param stdId 标准标识符
 * @param timeout_ms 超时时间（毫秒）
 * @retval int 返回状态
 */
int User_FDCAN_SendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint8_t data_len, uint32_t stdId, uint32_t timeout_ms)
{
	if (hfdcan == NULL || data == NULL || data_len == 0 || data_len > 64)
    {
        return -1;
    }

	QueueHandle_t tx_queue = NULL;
	if (hfdcan == &hfdcan1)
	{
		tx_queue = s_fdcan1_tx_queue;
	}
	else if (hfdcan == &hfdcan3)
	{
		tx_queue = s_fdcan3_tx_queue;
	}
	else
	{
		return -1;
	}

	if (tx_queue == NULL)
	{
		return -1;
	}

	UserFDCAN_TxMsg_t req = {0};
	req.bus = hfdcan;
	req.data_len = data_len;
	req.timeout_ms = timeout_ms;
	memcpy(req.data, data, data_len);
    req.header.Identifier = stdId;
    req.header.IdType = FDCAN_STANDARD_ID;
    req.header.TxFrameType = FDCAN_DATA_FRAME;
	req.header.DataLength = Fdcan_BytesToDlc(data_len);
    req.header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    req.header.BitRateSwitch = FDCAN_BRS_ON;
    req.header.FDFormat = FDCAN_FD_CAN;
    req.header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    req.header.MessageMarker = 0x00;

	const TickType_t t = ms_to_ticks(timeout_ms);
	if (xQueueSend(tx_queue, &req, t) != pdPASS)
	{
		return -2;
	}
	return 0;
}
/**
 * @brief FDCAN 初始化
 */
void User_FDCAN_Init(void)
{
	FDCAN_FilterTypeDef FDCAN1_RXFilter = {0};
	FDCAN_FilterTypeDef FDCAN3_RXFilter = {0};
	
	// STANDARD ID
	FDCAN1_RXFilter.IdType = FDCAN_STANDARD_ID;
	FDCAN1_RXFilter.FilterIndex = 0;
	FDCAN1_RXFilter.FilterType = FDCAN_FILTER_MASK;
	FDCAN1_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	FDCAN1_RXFilter.FilterID1 = 0x00000000;
	FDCAN1_RXFilter.FilterID2 = 0x00000000;
	FDCAN1_RXFilter.FilterIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter) != HAL_OK)
	{
		CAN_Error_Handle(&hfdcan1);
	}
	// EXTENDED ID
	FDCAN1_RXFilter.IdType = FDCAN_EXTENDED_ID;
	FDCAN1_RXFilter.FilterIndex = 0;
	FDCAN1_RXFilter.FilterType = FDCAN_FILTER_MASK;
	FDCAN1_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	FDCAN1_RXFilter.FilterID1 = 0x00000000;
	FDCAN1_RXFilter.FilterID2 = 0x00000000;
	FDCAN1_RXFilter.FilterIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter) != HAL_OK)
	{
		CAN_Error_Handle(&hfdcan1);
	}
	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);	
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	
	// STANDARD ID
	FDCAN3_RXFilter.IdType = FDCAN_STANDARD_ID;
	FDCAN3_RXFilter.FilterIndex = 0;
	FDCAN3_RXFilter.FilterType = FDCAN_FILTER_MASK;
	FDCAN3_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
	FDCAN3_RXFilter.FilterID1 = 0x00000000;
	FDCAN3_RXFilter.FilterID2 = 0x00000000;
	FDCAN3_RXFilter.FilterIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_RXFilter) != HAL_OK)
	{
		CAN_Error_Handle(&hfdcan3);
	}
	// EXTENDED ID
	FDCAN3_RXFilter.IdType = FDCAN_EXTENDED_ID;
	FDCAN3_RXFilter.FilterIndex = 0;
	FDCAN3_RXFilter.FilterType = FDCAN_FILTER_MASK;
	FDCAN3_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
	FDCAN3_RXFilter.FilterID1 = 0x00000000;
	FDCAN3_RXFilter.FilterID2 = 0x00000000;
	FDCAN3_RXFilter.FilterIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_RXFilter) != HAL_OK)
	{
		CAN_Error_Handle(&hfdcan3);
	}
	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan3, hfdcan3.Init.DataPrescaler * hfdcan3.Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan3);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);	
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
}

/**
 * @brief 配置FDCAN，开启发送和接收任务
 */
void FDCAN_ENABLE(void)
{
	User_FDCAN_Init();
    /* 创建队列/任务（只创建一次） */
	if (s_fdcan1_tx_queue == NULL)
	{
		s_fdcan1_tx_queue = xQueueCreate(FDCAN_TX_QUEUE_LEN, sizeof(UserFDCAN_TxMsg_t));
	}
	if (s_fdcan3_tx_queue == NULL)
	{
		s_fdcan3_tx_queue = xQueueCreate(FDCAN_TX_QUEUE_LEN, sizeof(UserFDCAN_TxMsg_t));
	}

	s_fdcan1_tx_ctx.queue = s_fdcan1_tx_queue;
	s_fdcan1_tx_ctx.bus = &hfdcan1;
	s_fdcan1_tx_ctx.interframe_delay_us = FDCAN1_TX_INTERFRAME_DELAY_US;

	s_fdcan3_tx_ctx.queue = s_fdcan3_tx_queue;
	s_fdcan3_tx_ctx.bus = &hfdcan3;
	s_fdcan3_tx_ctx.interframe_delay_us = FDCAN3_TX_INTERFRAME_DELAY_US;

	if ((s_fdcan1_tx_task == NULL) && (s_fdcan1_tx_queue != NULL))
	{
		const osThreadAttr_t tx1Attr = {
			.name = "fdcan1TxTask",
			.stack_size = FDCAN_TX_TASK_STACK_SIZE,
			.priority = FDCAN_TX_TASK_PRIORITY,
		};
		s_fdcan1_tx_task = osThreadNew(FdcanTxTask, &s_fdcan1_tx_ctx, &tx1Attr);
	}
	if ((s_fdcan3_tx_task == NULL) && (s_fdcan3_tx_queue != NULL))
	{
		const osThreadAttr_t tx3Attr = {
			.name = "fdcan3TxTask",
			.stack_size = FDCAN_TX_TASK_STACK_SIZE,
			.priority = FDCAN_TX_TASK_PRIORITY,
		};
		s_fdcan3_tx_task = osThreadNew(FdcanTxTask, &s_fdcan3_tx_ctx, &tx3Attr);
	}
	if (s_fdcan_rx_task == NULL)
	{
		const osThreadAttr_t rxAttr = {
			.name = "fdcanRxTask",
			.stack_size = FDCAN_RX_TASK_STACK_SIZE,
			.priority = FDCAN_RX_TASK_PRIORITY,
		};
		s_fdcan_rx_task = osThreadNew(FdcanRxTask, NULL, &rxAttr);
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((hfdcan == &hfdcan1) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U))
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if (s_fdcan_rx_task_handle != NULL)
		{
			xTaskNotifyFromISR(s_fdcan_rx_task_handle, FDCAN_NOTIFY_RX1_FIFO0, eSetBits, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((hfdcan == &hfdcan3) && ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U))
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if (s_fdcan_rx_task_handle != NULL)
		{
			xTaskNotifyFromISR(s_fdcan_rx_task_handle, FDCAN_NOTIFY_RX3_FIFO1, eSetBits, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

static void FdcanTxTask(void *argument)
{
	FdcanTxTaskCtx_t *ctx = (FdcanTxTaskCtx_t *)argument;
	if ((ctx == NULL) || (ctx->bus == NULL))
	{
		osThreadTerminate(osThreadGetId());
		return;
	}

	UserFDCAN_TxMsg_t req;
	for (;;)
	{
		if (ctx->queue == NULL)
		{
			osDelay(10);
			continue;
		}
		if (xQueueReceive(ctx->queue, &req, portMAX_DELAY) != pdPASS)
		{
			continue;
		}

		if ((req.bus == NULL) || (req.bus != ctx->bus))
		{
			continue;
		}

		FDCAN_TxHeaderTypeDef tx = req.header;

		/* timeout_ms==0：按“必须等上一帧发送完成再发下一帧”的语义，默认无限等待 */
		TickType_t waitTicks = ms_to_ticks(req.timeout_ms);
		if (waitTicks == 0U)
		{
			waitTicks = portMAX_DELAY;
		}

		const HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(req.bus, &tx, (uint8_t *)req.data);
		(void)waitTicks;
		if (st != HAL_OK)
		{
			/* HAL_BUSY: FIFO 满；HAL_ERROR: 外设未启动/BusOff 等 */
			continue;
		}

		Fdcan_DelayUs(ctx->interframe_delay_us);
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	// 单节点/无 ACK 时很容易累积错误进入 BUS-OFF，导致后续 AddMessage 一直失败。
	// 这里做一个“尽量不影响系统”的自恢复：BusOff 就重启外设。
	if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
	{
		(void)HAL_FDCAN_Stop(hfdcan);
		(void)HAL_FDCAN_Start(hfdcan);
	}
}

static void FdcanRxTask(void *argument)
{
	(void)argument;

	/* 记录句柄用于 ISR 通知 */
	s_fdcan_rx_task_handle = xTaskGetCurrentTaskHandle();

	uint32_t notifyBits = (FDCAN_NOTIFY_RX1_FIFO0 | FDCAN_NOTIFY_RX3_FIFO1);
	for (;;)
	{
		/*
		 * 如果在任务句柄建立前就来了第一帧，中断回调无法通知任务。
		 * 这里首轮先主动检查一次 FIFO，把“早到”的消息搬运走；之后再正常等待通知。
		 */
		if (notifyBits == 0U)
		{
			/* 等待任一 FIFO 有新消息 */
			(void)xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notifyBits, portMAX_DELAY);
		}

		if ((notifyBits & FDCAN_NOTIFY_RX1_FIFO0) != 0U)
		{
			while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
			{
				UserFDCAN_RxMsg_t msg = {0};
				if (!Fdcan_CopyRxToMsg(&hfdcan1, FDCAN_RX_FIFO0, &msg))
				{
					break;
				}
				UserFDCAN_RxCallback_t cb = s_fdcan_rx_cb;
				if (cb != NULL)
				{
					cb(&msg);
				}
			}
		}

		if ((notifyBits & FDCAN_NOTIFY_RX3_FIFO1) != 0U)
		{
			while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO1) > 0U)
			{
				UserFDCAN_RxMsg_t msg = {0};
				if (!Fdcan_CopyRxToMsg(&hfdcan3, FDCAN_RX_FIFO1, &msg))
				{
					break;
				}
				UserFDCAN_RxCallback_t cb = s_fdcan_rx_cb;
				if (cb != NULL)
				{
					cb(&msg);
				}
			}
		}

		/* 下一轮正常等待 */
		notifyBits = 0U;
	}
}