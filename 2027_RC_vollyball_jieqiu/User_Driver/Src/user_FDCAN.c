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

/* 宏：每条 FDCAN 发送任务的栈空间；当前启用。 */
#define FDCAN_TX_TASK_STACK_SIZE           (512U * 4U)
/* 宏：FDCAN 发送任务优先级；当前高于普通优先级。 */
#define FDCAN_TX_TASK_PRIORITY             ((osPriority_t)osPriorityAboveNormal)

/* 宏：接收任务栈空间；当前启用。 */
#define FDCAN_RX_TASK_STACK_SIZE           (512U * 4U)
/* 宏：接收任务优先级；当前高于普通优先级。 */
#define FDCAN_RX_TASK_PRIORITY             ((osPriority_t)osPriorityAboveNormal)

/* 宏：每条 CAN 总线的软件发送队列容量；当前启用。 */
#define FDCAN_TX_QUEUE_LEN                 (32U)
// #define FDCAN1_TX_INTERFRAME_DELAY_US      (500U)
// #define FDCAN3_TX_INTERFRAME_DELAY_US      (500U) 

/* 宏：FDCAN1 两帧发送间隔；当前启用值为 200 微秒，旧的 500/0 值保持屏蔽。 */
#define FDCAN1_TX_INTERFRAME_DELAY_US      (200U)
/* 宏：FDCAN3 两帧发送间隔；当前启用值为 450 微秒，旧的 500/0 值保持屏蔽。 */
#define FDCAN3_TX_INTERFRAME_DELAY_US      (450U) 

// #define FDCAN1_TX_INTERFRAME_DELAY_US      (0U)
// #define FDCAN3_TX_INTERFRAME_DELAY_US      (0U)

/* 宏：接收任务通知位，表示 FDCAN1 FIFO0 有新帧；当前启用。 */
#define FDCAN_NOTIFY_RX1_FIFO0             (1UL << 0)
/* 宏：接收任务通知位，表示 FDCAN3 FIFO1 有新帧；当前启用。 */
#define FDCAN_NOTIFY_RX3_FIFO1             (1UL << 1)

typedef struct
{
	/* 对应总线的待发送消息队列；由 FDCAN_ENABLE 初始化。 */
	QueueHandle_t queue;
	/* 本任务负责发送的 FDCAN 控制器句柄。 */
	FDCAN_HandleTypeDef *bus;
	/* 同一总线连续帧之间的微秒延时。 */
	uint32_t interframe_delay_us;
} FdcanTxTaskCtx_t;

/* 静态句柄：FDCAN1 发送队列；NULL 表示发送服务尚未建立。 */
static QueueHandle_t s_fdcan1_tx_queue = NULL;
/* 静态句柄：FDCAN3 发送队列；NULL 表示发送服务尚未建立。 */
static QueueHandle_t s_fdcan3_tx_queue = NULL;

/* 静态句柄：FDCAN1 发送任务；FDCAN_ENABLE 创建后使用。 */
static osThreadId_t s_fdcan1_tx_task = NULL;
/* 静态句柄：FDCAN3 发送任务；FDCAN_ENABLE 创建后使用。 */
static osThreadId_t s_fdcan3_tx_task = NULL;
/* 静态句柄：共用接收任务；FDCAN_ENABLE 创建后使用。 */
static osThreadId_t s_fdcan_rx_task = NULL;
/* 静态句柄：接收任务 FreeRTOS 句柄，供 ISR 通知；任务启动后有效。 */
static TaskHandle_t s_fdcan_rx_task_handle = NULL;

/* 静态上下文：FDCAN1 发送任务参数；初始化后持续使用。 */
static FdcanTxTaskCtx_t s_fdcan1_tx_ctx = {0};
/* 静态上下文：FDCAN3 发送任务参数；初始化后持续使用。 */
static FdcanTxTaskCtx_t s_fdcan3_tx_ctx = {0};

/* 静态回调：上层接收解析函数；未注册时收到的帧不向上层分发。 */
static UserFDCAN_RxCallback_t s_fdcan_rx_cb = NULL;

/* 函数：从指定硬件 FIFO 取帧并转换为统一接收消息；HAL 读取失败或参数无效返回失败。 */
static uint8_t Fdcan_CopyRxToMsg(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, UserFDCAN_RxMsg_t *out)
{
	if ((hfdcan == NULL) || (out == NULL))
	{
		return 0U;
	}

	/* data 是 HAL 读取出的临时 CAN-FD 负载，随后按 DLC 长度复制到 out。 */
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


/* 任务入口前置声明：按总线上下文发送队列消息；仅由 FDCAN_ENABLE 创建。 */
static void FdcanTxTask(void *argument);
/* 任务入口前置声明：依据 ISR 通知搬运 FIFO 并回调上层；仅由 FDCAN_ENABLE 创建。 */
static void FdcanRxTask(void *argument);

/* 函数：以 DWT 周期计数器忙等指定微秒；零延时或计数频率无效时直接返回。 */
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

/* 函数：将字节数向上匹配到 CAN-FD 支持的 DLC；发送封包路径调用。 */
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

/* 函数：将 CAN-FD DLC 转为实际字节数；接收帧转换路径调用。 */
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

/* 函数：持续重新初始化并启动发生错误的 FDCAN；错误路径进入后不会返回。 */
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
/* 函数：安装接收分发回调；传入 NULL 可停用上层解析。 */
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
/* 函数：校验并将 CAN-FD 数据封装后放入对应总线发送队列；队列满或总线无效返回错误。 */
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

	/* req 是加入发送队列的完整请求快照，避免调用方缓冲区后续变化影响待发内容。 */
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
/* 函数：配置并启动 FDCAN1/FDCAN3 接收过滤器、延迟补偿和通知；接收任务/队列另由 FDCAN_ENABLE 创建。 */
void User_FDCAN_Init(void)
{
	/* 两个过滤器配置对象分别对应 FDCAN1/FDCAN3；标准/扩展 ID 均转入各自 FIFO。 */
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
/* 函数：启动 FDCAN 控制器并创建两条发送队列、发送任务及共用接收任务；重复调用不重复建任务。 */
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

/* HAL 回调：仅对 FDCAN1 FIFO0 新消息设置接收任务通知位；ISR 不直接解析帧。 */
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

/* HAL 回调：仅对 FDCAN3 FIFO1 新消息设置接收任务通知位；ISR 不直接解析帧。 */
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

/* 任务：从本总线队列取出发送请求并提交 HAL；失败帧丢弃，成功后执行配置的帧间延时。 */
static void FdcanTxTask(void *argument)
{
	/* ctx 固定绑定单一总线/队列，req 为该任务逐条消费的发送请求。 */
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
		/* waitTicks 当前仅做超时语义换算；硬件入队失败时本实现不在任务内等待重试。 */
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

/* HAL 回调：检测 BUS-OFF 后停止并重启控制器；当前执行自动恢复分支。 */
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

/* 任务：等待通知并清空 FDCAN1 FIFO0/FDCAN3 FIFO1，调用已注册回调；当前 RX 任务模式启用。 */
static void FdcanRxTask(void *argument)
{
	(void)argument;

	/* 记录句柄用于 ISR 通知 */
	s_fdcan_rx_task_handle = xTaskGetCurrentTaskHandle();

	/* notifyBits 记录待处理总线 FIFO；每轮清零后由 ISR 通知重新置位。 */
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