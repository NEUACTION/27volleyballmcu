/*
*
*/
#include "user_usart.h"
#include "user_control.h"

#include "main.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"

/* 宏：UART8 DMA 接收缓冲区大小（字节）；1024 字节，配合 StreamBuffer 使用。 */
#define UART8_RX_DMA_BUF_SIZE          (1024U)
/* 宏：UART8 StreamBuffer 容量（字节）；当前 1024 字节，接收任务可按需读取。 */
#define UART8_RX_STREAM_BUF_SIZE       (1024U)

/* 全局变量：调试日志格式化缓冲区；位于 ram_d2，4096 字节，当前由 DEBUG 宏使用。 */
uint8_t debug_buffer[DEBUG_BUFFER_SIZE] __attribute__((section(".ram_d2"), aligned(32)));

/* 静态变量：UART8 DMA 接收原始缓冲区；位于 ram_d2、32 字节对齐；当前已启动并被 DMA 接收使用。 */
static uint8_t s_uart8_rx_dma_buf[UART8_RX_DMA_BUF_SIZE] __attribute__((section(".ram_d2"), aligned(32)));
/* 静态变量：UART8 StreamBuffer 控制结构；静态分配，尚未创建时为内部使用状态。 */
static StaticStreamBuffer_t s_uart8_stream_static;
/* 静态变量：UART8 StreamBuffer 静态存储区；在 RxDmaStart 首次初始化时使用。 */
static uint8_t s_uart8_stream_storage[UART8_RX_STREAM_BUF_SIZE];
/* 静态变量：UART8 StreamBuffer句柄；NULL 表示尚未创建，RxDmaStart 会将其启动。 */
static StreamBufferHandle_t s_uart8_rx_stream = NULL;

/* 全局变量：光电传感器 UART7 接收帧；由中断逐字节填充，当前接收和解析中使用。 */
uint8_t ps_rxbuffer[PS_RX_LEN] = {1};
/* 全局变量：光电传感器 UART7 发送帧；由 SendPs 填充帧头、帧尾并发送。 */
uint8_t ps_txbuffer[PS_RX_LEN] = {1};
/* 全局变量：光电传感器 UART7 的 16 位解析结果；由回调和 USART_ENABLE 启动的接收链路更新。 */
uint8_t ps_num[PS_RX_LEN - 4] = {0};

/* 全局变量：光电传感器 UART10 接收帧；由中断逐字节填充，当前接收和解析中使用。 */
uint8_t ps_rxbuffer_2[PS_RX_LEN] = {1};
/* 全局变量：光电传感器 UART10 发送帧；由 SendPs 填充帧头、帧尾并发送。 */
uint8_t ps_txbuffer_2[PS_RX_LEN] = {1};
/* 全局变量：光电传感器 UART10 的 16 位解析结果；由回调更新，未被当前主逻辑直接读取。 */
uint8_t ps_num_2[PS_RX_LEN - 4] = {0};

/* 全局变量：气阀 UART2 完整发送帧；由 SendValve 拼成“QF + 15 位控制量 + CRLF”。 */
uint8_t valve_txbuffer[VALVE_TX_LEN] = {};
/* 全局变量：气阀控制量数组，共 15 项；Valve 任务或控制逻辑修改后由 SendValve 发出。 */
uint8_t valve_num[VALVE_TX_LEN - 4] = {0};

/* 宏：光电传感器完整帧队列长度；UART7 和 UART10 各维护独立队列，当前各有 4 项。 */
/* PS(UART5) RX：用队列把“完整帧”从 ISR 投递到任务，避免在回调里做解析/写共享数据 */
#define PS_RX_QUEUE_LEN               (4U)
/* 静态变量：UART7 接收帧队列的控制结构；USART_ENABLE 初始化，未初始化时为未启动状态。 */
static StaticQueue_t s_ps_rx_queue_static;
/* 静态变量：UART7 接收帧队列存储区；每项为 PS_RX_LEN 字节。 */
static uint8_t s_ps_rx_queue_storage[PS_RX_QUEUE_LEN * PS_RX_LEN];
/* 静态变量：UART7 接收帧队列句柄；NULL 表示尚未初始化。 */
static QueueHandle_t s_ps_rx_queue = NULL;

/* 静态变量：UART10 接收帧队列的控制结构；USART_ENABLE 初始化。 */
static StaticQueue_t s_ps_rx_queue_static_2;
/* 静态变量：UART10 接收帧队列存储区；每项为 PS_RX_LEN 字节。 */
static uint8_t s_ps_rx_queue_storage_2[PS_RX_QUEUE_LEN * PS_RX_LEN];
/* 静态变量：UART10 接收帧队列句柄；NULL 表示尚未初始化。 */
static QueueHandle_t s_ps_rx_queue_2 = NULL;

/* 函数：将毫秒超时转换为 FreeRTOS Tick；timeout_ms=0 不等待，FFFFFFFF 表示无限等待。 */
static TickType_t UartMsToTicks(uint32_t timeout_ms)
{
	if (timeout_ms == 0U)
	{
		return 0U;
	}
	if (timeout_ms == 0xFFFFFFFFU)
	{
		return portMAX_DELAY;
	}
	return pdMS_TO_TICKS(timeout_ms);
}

/* 函数：将地址向下对齐到 32 字字节边界；用于 STM32H7 DCache 操作前的地址裁剪。 */
static uint32_t DCache_AlignDown32(uint32_t addr)
{
	return addr & ~((uint32_t)32U - 1U);
}

/* 函数：将地址向上对齐到 32 字节边界；用于计算 DCache 无效化覆盖范围。 */
static uint32_t DCache_AlignUp32(uint32_t addr)
{
	return (addr + (uint32_t)32U - 1U) & ~((uint32_t)32U - 1U);
}

/* 函数：按地址和长度无效化 H7 DCache；空指针或长度为 0 时直接返回，不修改接收逻辑。 */
static void DCache_InvalidateByAddr(const void *addr, uint32_t len)
{
	if ((addr == NULL) || (len == 0U))
	{
		return;
	}
	uint32_t start = DCache_AlignDown32((uint32_t)addr);
	uint32_t end = DCache_AlignUp32(((uint32_t)addr) + len);
	SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

/* 函数：启动 UART8 的 IDLE/DMA 接收模式；成功返回 0；流缓冲区创建失败返回 -1，HAL 启动失败返回 -2。 */
int User_UART8_RxDmaStart(void)
{
	if (s_uart8_rx_stream == NULL)
	{
		/* 用静态 StreamBuffer，避免动态内存碎片 */
		s_uart8_rx_stream = xStreamBufferCreateStatic(
					UART8_RX_STREAM_BUF_SIZE,
					1U,
					s_uart8_stream_storage,
					&s_uart8_stream_static);
		if (s_uart8_rx_stream == NULL)
		{
			return -1;
		}
	}

	/* 启动 ReceiveToIdle DMA：收到 IDLE/满缓冲会触发 HAL_UARTEx_RxEventCallback */
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart8, s_uart8_rx_dma_buf, UART8_RX_DMA_BUF_SIZE) != HAL_OK)
	{
		return -2;
	}

	/* 不需要 Half-Transfer，避免频繁回调 */
	if (huart8.hdmarx != NULL)
	{
		__HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
	}

	return 0;
}

/* 函数：从 UART8 接收 StreamBuffer 阻塞/超时读取；流未启动、输出指针空或长度为 0 时返回 0。 */
size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms)
{
	if ((s_uart8_rx_stream == NULL) || (out == NULL) || (maxLen == 0U))
	{
		return 0U;
	}
	TickType_t t = UartMsToTicks(timeout_ms);
	return xStreamBufferReceive(s_uart8_rx_stream, out, maxLen, t);
}

/* 函数：查询 UART8 StreamBuffer 当前可读取字节数；流未启动时返回 0。当前未被工程直接调用。 */
size_t User_UART8_Available(void)
{
	if (s_uart8_rx_stream == NULL)
	{
		return 0U;
	}
	return xStreamBufferBytesAvailable(s_uart8_rx_stream);
}

/* 函数：清空 UART8 StreamBuffer；流未启动时直接返回。当前未被工程直接调用，可恢复读取逻辑后使用。 */
void User_UART8_Flush(void)
{
	if (s_uart8_rx_stream == NULL)
	{
		return;
	}
	(void)xStreamBufferReset(s_uart8_rx_stream);
}

/*
 * UART8 RX HAL 中断回调：只处理 UART8，当前模式为 DMA + IDLE。
 * Size 为本次收到的数据长度；收到数据后先无效化 DCache，再写入 StreamBuffer。
 * 数据发送到流缓冲区后立即重启 ReceiveToIdle DMA，并屏蔽 Half-Transfer 中断以减少回调。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if ((huart == NULL) || (huart->Instance != UART8))
	{
		return;
	}
	/* 防止 IDLE 标志残留导致重复进入回调 */
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	if ((s_uart8_rx_stream == NULL) || (Size == 0U))
	{
		/* 尽量保持接收不断 */
		(void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart8_rx_dma_buf, UART8_RX_DMA_BUF_SIZE);
		if (huart->hdmarx != NULL)
		{
			__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
		}
		return;
	}
	if (Size > UART8_RX_DMA_BUF_SIZE)
	{
		Size = UART8_RX_DMA_BUF_SIZE;
	}

	/* H7 DCache：DMA 写入后需要 invalidate */
	DCache_InvalidateByAddr(s_uart8_rx_dma_buf, (uint32_t)Size);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	(void)xStreamBufferSendFromISR(s_uart8_rx_stream, s_uart8_rx_dma_buf, (size_t)Size, &xHigherPriorityTaskWoken);

	/* 继续接收 */
	(void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart8_rx_dma_buf, UART8_RX_DMA_BUF_SIZE);
	if (huart->hdmarx != NULL)
	{
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 函数：确保 UART7 接收帧队列已静态创建；重复调用不会重复创建。 */
static void PS_RxQueueEnsureInit(void)
{
	if (s_ps_rx_queue != NULL)
	{
		return;
	}
	s_ps_rx_queue = xQueueCreateStatic(
			PS_RX_QUEUE_LEN,
			PS_RX_LEN,
			s_ps_rx_queue_storage,
			&s_ps_rx_queue_static);
}

/* 函数：确保 UART10 接收帧队列已静态创建；重复调用不会重复创建。 */
static void PS_RxQueueEnsureInit_2(void)
{
	if (s_ps_rx_queue_2 != NULL)
	{
		return;
	}
	s_ps_rx_queue_2 = xQueueCreateStatic(
			PS_RX_QUEUE_LEN,
			PS_RX_LEN,
			s_ps_rx_queue_storage_2,
			&s_ps_rx_queue_static_2);
}

/* 函数：发送气阀控制帧到 UART2；由控制逻辑主动调用，帧尾使用 CRLF。当前已在部分控制路径启用。 */
void SendValve(void)
{
	valve_txbuffer[0] = 'Q';
	valve_txbuffer[1] = 'F';
	valve_txbuffer[VALVE_TX_LEN - 2] = '\r';
	valve_txbuffer[VALVE_TX_LEN - 1] = '\n';
	
	for (int i = 0; i < VALVE_TX_LEN - 4; i++)
	{
		valve_txbuffer[i + 2] = valve_num[i];
	}
	
	HAL_UART_Transmit_IT(&VALVE_UART, (uint8_t *)valve_txbuffer, VALVE_TX_LEN);
}

/* 函数：向 UART7 和 UART10 各发送一条“IO”查询帧；当前由任务周期性调用。 */
void SendPs(void)
{
	ps_txbuffer[0] = 'I';
	ps_txbuffer[1] = 'O';
	ps_txbuffer[14] = '\r';
	ps_txbuffer[15] = '\n';

	ps_txbuffer_2[0] = 'I';
	ps_txbuffer_2[1] = 'O';
	ps_txbuffer_2[14] = '\r';
	ps_txbuffer_2[15] = '\n';
	
	HAL_UART_Transmit_IT(&PS_UART, (uint8_t *)ps_txbuffer, PS_TX_LEN);
	HAL_UART_Transmit_IT(&PS_UART_2, (uint8_t *)ps_txbuffer_2, PS_TX_LEN);
}

/* 函数：从 UART7 队列取出完整帧并解析为 ps_num；当前已实现但工程中未直接周期调用。 */
void GetPSNum(void)
{
	if (s_ps_rx_queue == NULL)
	{
		return;
	}

	/* frame 暂存 UART7 队列完整帧，num 暂存校验帧头/帧尾后提取的数字字段。 */
	uint8_t frame[PS_RX_LEN];
	while (xQueueReceive(s_ps_rx_queue, frame, 0U) == pdPASS)
	{
		if ((frame[0] == 'I') && (frame[1] == 'O') &&
			(frame[PS_RX_LEN - 2] == '\r') && (frame[PS_RX_LEN - 1] == '\n'))
		{
			uint8_t num[PS_RX_LEN - 4];
			for (int i = 0; i < (PS_RX_LEN - 4); i++)
			{
				num[i] = frame[i + 2] - '0';
			}

			/* 一次性更新共享数据，避免读到“半更新” */
			taskENTER_CRITICAL();
			{
				memcpy(ps_num, num, (size_t)(PS_RX_LEN - 4));
			}
			taskEXIT_CRITICAL();
		}
	}
}

/* 函数：从 UART10 队列取出完整帧并解析为 ps_num_2；当前已实现但工程中未直接周期调用。 */
void GetPSNum2(void)
{
	if (s_ps_rx_queue_2 == NULL)
	{
		return;
	}

	/* frame 暂存 UART10 队列完整帧，num 暂存校验后的数字字段。 */
	uint8_t frame[PS_RX_LEN];
	while (xQueueReceive(s_ps_rx_queue_2, frame, 0U) == pdPASS)
	{
		if ((frame[0] == 'I') && (frame[1] == 'O') &&
			(frame[PS_RX_LEN - 2] == '\r') && (frame[PS_RX_LEN - 1] == '\n'))
		{
			uint8_t num[PS_RX_LEN - 4];
			for (int i = 0; i < (PS_RX_LEN - 4); i++)
			{
				num[i] = frame[i + 2] - '0';
			}

			/* 一次性更新共享数据，避免读到“半更新” */
			taskENTER_CRITICAL();
			{
				memcpy(ps_num_2, num, (size_t)(PS_RX_LEN - 4));
			}
			taskEXIT_CRITICAL();
		}
	}
}

/* HAL 回调：UART7/UART10 单字节接收完成中断入口；(现在第2轮光电没写，不管。如果想换一种模式，可以看看排球的代码）按句柄分流，向独立队列投递完整帧并续收。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart != NULL) && (huart == &PS_UART))
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;//防御性，标记是否需要切换任务
		if (s_ps_rx_queue != NULL)
		{
			/* 队列一个字节一个字节传，后面大概是错误的东西：按 item_size 拷贝整帧，避免续收覆盖 */
			(void)xQueueSendFromISR(s_ps_rx_queue, ps_rxbuffer, &xHigherPriorityTaskWoken);
		}
		/* 立刻续收：避免接收间隙造成溢出/丢包 */
		(void)HAL_UART_Receive_IT(huart, ps_rxbuffer, PS_RX_LEN);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//防御性，如果队列投递唤醒了更高优先级任务，则切换到该任务
	}
}

/* HAL 回调：串口异常入口；当前会终止接收并重新启动 UART7 接收，UART10 重启代码已被屏蔽，可按需恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ((huart == NULL))
	{
		return;
	}

	/* 出错后重启接收，避免串口卡死在错误状态 */
	(void)HAL_UART_AbortReceive_IT(huart);
	(void)HAL_UART_Receive_IT(huart, ps_rxbuffer, PS_RX_LEN);
	// (void)HAL_UART_Receive_IT(huart, ps_rxbuffer_2, PS_RX_LEN);
}

/* 函数：初始化两路光电串口接收队列，并以中断模式启动 UART7 和 UART10 接收；由任务初始化调用一次。 */
void USART_ENABLE(void)
{	
	PS_RxQueueEnsureInit();
	PS_RxQueueEnsureInit_2();
	HAL_UART_Receive_IT(&PS_UART, (uint8_t *)ps_rxbuffer, 1);//开启串口接收中断
	HAL_UART_Receive_IT(&PS_UART_2, (uint8_t *)ps_rxbuffer_2, 1);//开启串口接收中断
}
