#include "user_usart.h"
#include "user_control.h"

#include "main.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"

/* DMA RX 缓冲区：对齐 32B 方便 DCache 操作 */
#define UART8_RX_DMA_BUF_SIZE          (1024U)
#define UART8_RX_STREAM_BUF_SIZE       (1024U)

uint8_t debug_buffer[DEBUG_BUFFER_SIZE] __attribute__((section(".ram_d2"), aligned(32)));

static uint8_t s_uart8_rx_dma_buf[UART8_RX_DMA_BUF_SIZE] __attribute__((section(".ram_d2"), aligned(32)));
static StaticStreamBuffer_t s_uart8_stream_static;
static uint8_t s_uart8_stream_storage[UART8_RX_STREAM_BUF_SIZE];
static StreamBufferHandle_t s_uart8_rx_stream = NULL;

uint8_t ps_rxbuffer[PS_RX_LEN] = {1};
uint8_t ps_txbuffer[PS_RX_LEN] = {1};
uint8_t ps_num[PS_RX_LEN - 4] = {0};

uint8_t ps_rxbuffer_2[PS_RX_LEN] = {1};
uint8_t ps_txbuffer_2[PS_RX_LEN] = {1};
uint8_t ps_num_2[PS_RX_LEN - 4] = {0};

uint8_t valve_txbuffer[VALVE_TX_LEN] = {};
uint8_t valve_num[VALVE_TX_LEN - 4] = {0};

/* PS(UART5) RX：用队列把“完整帧”从 ISR 投递到任务，避免在回调里做解析/写共享数据 */
#define PS_RX_QUEUE_LEN               (4U)
static StaticQueue_t s_ps_rx_queue_static;
static uint8_t s_ps_rx_queue_storage[PS_RX_QUEUE_LEN * PS_RX_LEN];
static QueueHandle_t s_ps_rx_queue = NULL;

static StaticQueue_t s_ps_rx_queue_static_2;
static uint8_t s_ps_rx_queue_storage_2[PS_RX_QUEUE_LEN * PS_RX_LEN];
static QueueHandle_t s_ps_rx_queue_2 = NULL;

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

static uint32_t DCache_AlignDown32(uint32_t addr)
{
	return addr & ~((uint32_t)32U - 1U);
}

static uint32_t DCache_AlignUp32(uint32_t addr)
{
	return (addr + (uint32_t)32U - 1U) & ~((uint32_t)32U - 1U);
}

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

size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms)
{
	if ((s_uart8_rx_stream == NULL) || (out == NULL) || (maxLen == 0U))
	{
		return 0U;
	}
	TickType_t t = UartMsToTicks(timeout_ms);
	return xStreamBufferReceive(s_uart8_rx_stream, out, maxLen, t);
}

size_t User_UART8_Available(void)
{
	if (s_uart8_rx_stream == NULL)
	{
		return 0U;
	}
	return xStreamBufferBytesAvailable(s_uart8_rx_stream);
}

void User_UART8_Flush(void)
{
	if (s_uart8_rx_stream == NULL)
	{
		return;
	}
	(void)xStreamBufferReset(s_uart8_rx_stream);
}

/*
 * UART RX Event Callback (ISR context)
 * - Size: 本次收到的数据长度（<= UART8_RX_DMA_BUF_SIZE）
 * - 这里把数据写入 StreamBuffer，然后立刻重启 ReceiveToIdle DMA
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

void GetPSNum(void)
{
	if (s_ps_rx_queue == NULL)
	{
		return;
	}

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

void GetPSNum2(void)
{
	if (s_ps_rx_queue_2 == NULL)
	{
		return;
	}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart != NULL) && (huart == &PS_UART))
	{
		// DEBUG("okok\r\n");
		static int ps_cnt = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if (s_ps_rx_queue != NULL)
		{
			/* 队列按 item_size 拷贝整帧，避免续收覆盖 */
			(void)xQueueSendFromISR(s_ps_rx_queue, ps_rxbuffer, &xHigherPriorityTaskWoken);
		}
		/* 立刻续收：避免接收间隙造成溢出/丢包 */
		if(ps_rxbuffer[0] == 'I' && ps_rxbuffer[1] == 'O' && ps_rxbuffer[PS_RX_LEN - 2] == '\r' && ps_rxbuffer[PS_RX_LEN - 1] == '\n')
		{
			
			for (int i = 0; i < (PS_RX_LEN - 4)-1; i++)
			{
				ps_num[i] = ps_rxbuffer[i + 2] - '0';
			}
			ps_cnt = 0;
		}
		else if (ps_rxbuffer[ps_cnt] == 'I')
		{
			// DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num[0], ps_num[1], ps_num[2], ps_num[3], ps_num[4], ps_num[5], ps_num[6], ps_num[7], ps_num[8], ps_num[9], ps_num[10], ps_num[11]);
			ps_rxbuffer[0] = 'I';
			ps_cnt = 1;
		}
		else if(ps_cnt >= PS_RX_LEN - 1)
		{
			// DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num[0], ps_num[1], ps_num[2], ps_num[3], ps_num[4], ps_num[5], ps_num[6], ps_num[7], ps_num[8], ps_num[9], ps_num[10], ps_num[11]);
			ps_cnt = 0;
		}
		else
		{
			// DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num[0], ps_num[1], ps_num[2], ps_num[3], ps_num[4], ps_num[5], ps_num[6], ps_num[7], ps_num[8], ps_num[9], ps_num[10], ps_num[11]);
			ps_cnt ++;
		}
		// DEBUG("%s", ps_rxbuffer);
		(void)HAL_UART_Receive_IT(huart, ps_rxbuffer + ps_cnt, 1);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	if ((huart != NULL) && (huart == &PS_UART_2))
	{
		// DEBUG("okok\r\n");
		static int ps_cnt_2 = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if (s_ps_rx_queue_2 != NULL)
		{
			/* 队列按 item_size 拷贝整帧，避免续收覆盖 */
			(void)xQueueSendFromISR(s_ps_rx_queue_2, ps_rxbuffer_2, &xHigherPriorityTaskWoken);
		}
		/* 立刻续收：避免接收间隙造成溢出/丢包 */
		if(ps_rxbuffer_2[0] == 'I' && ps_rxbuffer_2[1] == 'O' && ps_rxbuffer_2[PS_RX_LEN - 2] == '\r' && ps_rxbuffer_2[PS_RX_LEN - 1] == '\n')
		{
			// DEBUG("%s", ps_rxbuffer);
			for (int i = 0; i < (PS_RX_LEN - 4)-1; i++)
			{
				ps_num_2[i] = ps_rxbuffer_2[i + 2] - '0';
			}
			ps_cnt_2 = 0;
		}
		else if (ps_rxbuffer_2[ps_cnt_2] == 'I')
		{
			ps_rxbuffer_2[0] = 'I';
			ps_cnt_2 = 1;
		}
		else if(ps_cnt_2 >= PS_RX_LEN - 1)
		{
			ps_cnt_2 = 0;
		}
		else ps_cnt_2 ++;
		(void)HAL_UART_Receive_IT(huart, ps_rxbuffer_2 + ps_cnt_2, 1);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

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

void USART_ENABLE(void)
{	
	PS_RxQueueEnsureInit();
	PS_RxQueueEnsureInit_2();
	HAL_UART_Receive_IT(&PS_UART, (uint8_t *)ps_rxbuffer, 1);//开启串口接收中断
	HAL_UART_Receive_IT(&PS_UART_2, (uint8_t *)ps_rxbuffer_2, 1);//开启串口接收中断
}
