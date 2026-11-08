#include "user_usart.h"
#include "user_control.h"

#include "main.h"
#include "usart.h"
#include "user_task.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"

#include <string.h>
#include <stddef.h>

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
uint8_t valve2_txbuffer[VALVE_TX_LEN] = {};
uint8_t valve_rxbuffer[VALVE_RX_LEN] = {};
uint8_t valve2_rxbuffer[VALVE_RX_LEN] = {};
uint8_t valve_num[20] = {0};
uint8_t valve2_num[20] = {0};

/* PS(USART10) RX：用队列把“完整帧”从 ISR 投递到任务，避免在回调里做解析/写共享数据 */
#define PS_RX_QUEUE_LEN               (4U)
static StaticQueue_t s_ps_rx_queue_static_2;
static uint8_t s_ps_rx_queue_storage_2[PS_RX_QUEUE_LEN * PS_RX_LEN];
static QueueHandle_t s_ps_rx_queue_2 = NULL;

int TarLaserDis = 99999;

/* 调试计数：确认 USART3 到底有没有收到数据、帧格式对不对（定位完可删） */
volatile uint32_t ps_rx_total = 0;      /* 收到的字节数 */
volatile uint32_t ps_rx_ok = 0;         /* 成功解析的帧数 */
volatile uint32_t ps_rx_bad = 0;        /* 收满一帧但校验失败次数 */
volatile uint8_t ps_last_raw[PS_RX_LEN];/* 最近一次收满的原始帧 */
volatile uint8_t ps_last_byte = 0;      /* 最近收到的那个字节 */
volatile uint32_t ps_rx_arm_fail = 0;   /* 重新开启接收失败次数（HAL 返回 != HAL_OK） */
volatile uint32_t ps_rx_err = 0;        /* USART3 出错次数 */
volatile uint32_t ps_err_code = 0;      /* 最近一次错误码 */


#include "user_usart.h"
#include <string.h>
#include <stddef.h>

// 如果 usart.h 里没有 extern 声明，请保留这行；如果有，删掉这行
extern UART_HandleTypeDef huart8; 

// 物理分配 8192 字节（2 * 4096），用于镜像回绕
static uint8_t dbg_ring[2 * DEBUG_BUFFER_SIZE] __attribute__((aligned(4)));

static volatile uint16_t dbg_head = 0;
static volatile uint16_t dbg_tail = 0;
static volatile uint16_t dbg_pending_len = 0;
static volatile uint8_t  dbg_dma_busy = 0;
static volatile uint32_t dbg_drop_count = 0;


// 开始
static inline uint32_t dbg_enter_critical(void) {
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}
static inline void dbg_exit_critical(uint32_t pm) {
    __set_PRIMASK(pm);
}

void Debug_Init(void) {
    uint32_t pm = dbg_enter_critical();
    dbg_head = 0;
    dbg_tail = 0;
    dbg_pending_len = 0;
    dbg_dma_busy = 0;
    dbg_drop_count = 0;
    dbg_exit_critical(pm);
}

uint32_t Debug_GetDropCount(void) {
    return dbg_drop_count;
}

static void Debug_Kick(void) {
    uint16_t avail;
    uint16_t local_tail;

    uint32_t pm = dbg_enter_critical();
    if (dbg_dma_busy) {
        dbg_exit_critical(pm);
        return;
    }
    
    avail = (uint16_t)(((uint32_t)dbg_head + DEBUG_BUFFER_SIZE - dbg_tail) % DEBUG_BUFFER_SIZE);
    if (avail == 0) {
        dbg_exit_critical(pm);
        return;
    }
    
    dbg_dma_busy = 1;
    dbg_pending_len = avail;
    local_tail = dbg_tail;
    dbg_exit_critical(pm);

    // 【关键】这里使用 &DEBUG_UART_HANDLER
    // 宏展开后变成 &(huart8)，完美符合 HAL 库传指针的要求
    if (HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLER, &dbg_ring[local_tail], avail) != HAL_OK) {
        pm = dbg_enter_critical();
        dbg_dma_busy = 0;
        dbg_pending_len = 0;
        dbg_exit_critical(pm);
    }
}

void Debug_Push(const uint8_t *data, uint16_t len) {
    if (len == 0 || data == NULL) return;

    uint32_t pm = dbg_enter_critical();

    uint16_t used = (uint16_t)(((uint32_t)dbg_head + DEBUG_BUFFER_SIZE - dbg_tail) % DEBUG_BUFFER_SIZE);
    uint16_t free_cnt = (uint16_t)(DEBUG_BUFFER_SIZE - 1u - used); 
    
    if (len > free_cnt) {
        dbg_drop_count += (uint32_t)(len - free_cnt);
        len = free_cnt;
    }

    uint16_t h = dbg_head;
    for (uint16_t i = 0; i < len; i++) {
        dbg_ring[h] = data[i];                     
        dbg_ring[h + DEBUG_BUFFER_SIZE] = data[i]; 
        h = (uint16_t)((h + 1u) % DEBUG_BUFFER_SIZE);
    }
    dbg_head = h;

    dbg_exit_critical(pm);
    Debug_Kick(); 
}

void Debug_OnTxComplete(void) {
    uint32_t pm = dbg_enter_critical();
    if (dbg_dma_busy) {
        dbg_tail = (uint16_t)((dbg_tail + dbg_pending_len) % DEBUG_BUFFER_SIZE);
        dbg_pending_len = 0;
        dbg_dma_busy = 0;
    }
    dbg_exit_critical(pm);

    Debug_Kick();
}

//结束

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
	valve_txbuffer[0] = valve_number;
	// valve_txbuffer[1] = 'F';
	// valve_txbuffer[VALVE_TX_LEN - 2] = '\r';
	// valve_txbuffer[VALVE_TX_LEN - 1] = '\n';

	// valve2_txbuffer[0] = 'Q';
	// valve2_txbuffer[1] = 'F';
	// valve2_txbuffer[VALVE_TX_LEN - 2] = '\r';
	// valve2_txbuffer[VALVE_TX_LEN - 1] = '\n';
	valve2_txbuffer[0] = valve2_number;
	// static int cnt = 0;
	// static bool v_flag = false;
	// cnt ++;
	// if(cnt == 200){
	// 	v_flag = v_flag ^ 1;
	// 	cnt = 0;
	// }
	// valve_num[11] = v_flag;
   	// // valve2_num[11] = v_flag;
	// for (int i = 0; i < VALVE2_TX_LEN - 4; i++)
	// {
	// 	valve_txbuffer[i + 2] = valve_num[i] + '0';
	// }
	// // for (int i = 0; i < VALVE_TX_LEN - 4; i++)
	// // {      
	// // 	valve2_txbuffer[i + 2] = valve2_num[i] + '0';
	// // }
	// // if(camp_flag){
	// // 	if(LaserDis_right > TarLaserDis)valve2_num[11] = 0;
	// // }else
	// // 	if (LaserDis > TarLaserDis)valve2_num[11] = 0;
	// for(int i = 0; i < 3; ++i){
	// 	for(int j = 0; j < 7; ++j){
	// 		// valve_txbuffer[i + 2] = valve_txbuffer[i + 2] << 1 | (valve_num[i * 7 + j] & 0x01);
	// 		valve2_txbuffer[i + 2] = valve2_txbuffer[i + 2] << 1 | (valve2_num[i * 7 + j] & 0x01);
	// 	}
	// 	valve_txbuffer[i + 2] = valve_txbuffer[i + 2] << 1 | 0x01;
	// 	// valve2_txbuffer[i + 2] = valve2_txbuffer[i + 2] << 1 | 0x01;
	// }

	HAL_UART_Transmit_IT(&VALVE_UART, (uint8_t *)valve_txbuffer, VALVE_TX_LEN);
	HAL_UART_Transmit_IT(&VALVE2_UART, (uint8_t *)valve2_txbuffer, VALVE_TX_LEN);
}


void SendPs(void)
{
	for (int i = 0; i < (PS_RX_LEN - 4); i++)
	{
		ps_txbuffer[i + 2] = '0';
		ps_txbuffer_2[i + 2] = '0';
	}
	ps_txbuffer[0] = 'I';
	ps_txbuffer[1] = 'O';
	ps_txbuffer[PS_RX_LEN - 2] = '\r';
	ps_txbuffer[PS_RX_LEN - 1] = '\n';

	ps_txbuffer_2[0] = 'I';
	ps_txbuffer_2[1] = 'O';
	ps_txbuffer_2[PS_RX_LEN - 2] = '\r';
	ps_txbuffer_2[PS_RX_LEN - 1] = '\n';
	
	HAL_UART_Transmit_IT(&PS_UART, (uint8_t *)ps_txbuffer, PS_TX_LEN);
	// HAL_UART_Transmit_IT(&PS_UART_2, (uint8_t *)ps_txbuffer_2, PS_TX_LEN);
}

/**
 * @brief 校验 "IO" + N 个 '0'/'1' + "\r\n" 帧，并解析成 0/1 数组
 * @note  只有整帧合法才写 out，避免半帧/非法字符污染结果
 */
static bool PS_ParseFrame(const uint8_t *frame, uint8_t *out)
{
	if ((frame[0] != 'I') || (frame[1] != 'O') ||
		(frame[PS_RX_LEN - 2] != '\r') || (frame[PS_RX_LEN - 1] != '\n'))
	{
		return false;
	}

	for (int i = 0; i < (PS_RX_LEN - 4); i++)
	{
		if ((frame[i + 2] != '0') && (frame[i + 2] != '1'))
		{
			return false;
		}
	}

	for (int i = 0; i < (PS_RX_LEN - 4); i++)
	{
		out[i] = frame[i + 2] - '0';
	}
	return true;
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


extern bool hook_Flag;
// 光电
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart != NULL) && (huart == &PS_UART))
	{
		// DEBUG("okok\r\n");
		static int ps_cnt = 0;
		ps_rx_total++;
		ps_last_byte = ps_rxbuffer[ps_cnt];
		/* 收满一整帧后才校验并解析，避免用上帧残留数据误判 */
		if (ps_cnt >= PS_RX_LEN - 1)
		{                                                                                                                                                                                                                                                                                                                                                                   
			memcpy((void *)ps_last_raw, ps_rxbuffer, PS_RX_LEN);
			if (PS_ParseFrame(ps_rxbuffer, ps_num))
			{
				ps_rx_ok++;
				GetADCMsg();
				// if(!ps_num[11] && TarLaserDis >= 90000 && hook_Flag)
				{
					// if(camp_flag){
					// 	TarLaserDis = 27725;
					// }else TarLaserDis = 28111;
				}
				SendValve();
			}
			else
			{
				ps_rx_bad++;
			}
			ps_cnt = 0;
		}
		else if (ps_rxbuffer[ps_cnt] == 'I')
		{
			ps_rxbuffer[0] = 'I';
			ps_cnt = 1;
		}
		else
		{
			ps_cnt ++;
		}
		if (HAL_UART_Receive_IT(huart, ps_rxbuffer + ps_cnt, 1) != HAL_OK)
		{
			ps_rx_arm_fail++;
		}
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
			for (int i = 0; i < (PS_RX_LEN - 4); i++)
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
	if(huart != NULL && huart == &VALVE_UART)
	{
		// HB.valve1 = valve_rxbuffer[2];
		// DEBUG("VALVE1: %d\r\n",HB.valve1);
		(void)HAL_UART_Receive_IT(huart, valve_rxbuffer, VALVE_RX_LEN);
	}
	if(huart != NULL && huart == &VALVE2_UART)
	{

		// HB.valve2 = valve2_rxbuffer[2];
		// DEBUG("VALVE2: %d\r\n",HB.valve2);
		(void)HAL_UART_Receive_IT(huart, valve2_rxbuffer, VALVE_RX_LEN);
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
	if (huart == &PS_UART)
	{
		ps_rx_err++;
		ps_err_code = huart->ErrorCode;
		(void)HAL_UART_Receive_IT(huart, ps_rxbuffer, 1);
	}
	else if (huart == &PS_UART_2)
	{
		(void)HAL_UART_Receive_IT(huart, ps_rxbuffer_2, 1);
	}
	else if (huart == &DEBUG_UART_HANDLER)
	{
		(void)User_UART8_RxDmaStart();
	}
}

void USART_ENABLE(void)
{	
	PS_RxQueueEnsureInit_2();
	HAL_UART_Receive_IT(&PS_UART, (uint8_t *)ps_rxbuffer, 1);//开启串口接收中断
	HAL_UART_Receive_IT(&PS_UART_2, (uint8_t *)ps_rxbuffer_2, 1);//开启串口接收中断
	HAL_UART_Receive_IT(&VALVE_UART, (uint8_t *)valve_rxbuffer, VALVE_RX_LEN);
	HAL_UART_Receive_IT(&VALVE2_UART, (uint8_t *)valve2_rxbuffer, VALVE_RX_LEN);
}


// 在你 user_usart.c 的最末尾加上：
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART8) {
        Debug_OnTxComplete();
    }
}
