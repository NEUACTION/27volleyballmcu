/* 文件作用：实现用户串口收发，包括UART8调试接收流、UART7发球PS3、保留的UART10与阀门接口。
 * UART7按原发球9字节IO协议逐字节解析，只更新ps_num[3]；PS3未收到有效帧时初值为1。
 * 各接口只处理串口数据；不在接收中断中计算轨迹，也不触发发球动作。
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

/* 接收/发送缓存当前各为 PS_RX_LEN（9）字节；UART7 使用独立九字节 PS3 解析器。 */
uint8_t ps_rxbuffer[PS_RX_LEN] =
{
    1
}
;
uint8_t ps_txbuffer[PS_RX_LEN] =
{
    1
}
;
/* ps_num（光电数值数组）：当前共五项，仅PS3默认1，防止未收到设备帧时触发PC的零电平计数。 */
uint8_t ps_num[PS_RX_LEN - 4] =
{
    [3] = 1U
}
;
/* ps3_valid（PS3有效标记）：记录启动后是否收到过合法帧，仅供本地诊断。 */
volatile uint8_t ps3_valid = 0U;
/* s_ps3_byte（当前接收字节）、s_ps3_frame（候选帧）、s_ps3_count（已收合法字节数）。 */
static uint8_t s_ps3_byte;
static uint8_t s_ps3_frame[PS3_FRAME_LEN];
static uint8_t s_ps3_count;

/* 中文名：逐字节解析PS3；作用：校验 byte 输入的 IO + 五个 ASCII 0/1 + CRLF 九字节帧。
 * 新 I 重定位帧头，非法字节清空候选计数；完整合法帧更新 ps_num[3]、ps3_valid，复位计数；无返回值。 */
static void PS3_ParseByte(uint8_t byte)
{
    if (byte == 'I')
    {
        s_ps3_frame[0] = byte;
        s_ps3_count = 1U;
        return;
    }
    if (s_ps3_count == 0U)
        return;

    /* 根据当前下标检查帧头第二字节、五个数据字节和两个帧尾字节。 */
    if ((s_ps3_count == 1U && byte != 'O') ||
        (s_ps3_count >= 2U && s_ps3_count <= 6U && byte != '0' && byte != '1') ||
        (s_ps3_count == 7U && byte != '\r') ||
        (s_ps3_count == 8U && byte != '\n'))
    {
        s_ps3_count = 0U;
        return;
    }
    s_ps3_frame[s_ps3_count++] = byte;
    if (s_ps3_count == PS3_FRAME_LEN)
    {
        /* 整帧下标5是五个数据中的第4项；转换为原始数值0/1，不翻转。 */
        ps_num[3] = (uint8_t)(s_ps3_frame[5] - '0');
        ps3_valid = 1U;
        s_ps3_count = 0U;
    }
}

/* 全局变量：光电传感器 UART10 接收帧；由中断逐字节填充，当前接收和解析中使用。 */
uint8_t ps_rxbuffer_2[PS_RX_LEN] =
{
    1
}
;
/* 全局变量：光电传感器 UART10 发送缓存，当前为九字节；SendPs 中相关历史发送代码以 #if 0 保留。 */
uint8_t ps_txbuffer_2[PS_RX_LEN] =
{
    1
}
;
/* 全局变量：光电传感器 UART10 的五项解析结果；由 GetPSNum2 读取队列后更新。 */
uint8_t ps_num_2[PS_RX_LEN - 4] =
{
    0
}
;

/* 全局变量：气阀 UART2 完整发送帧；由 SendValve 拼成“QF + 15 位控制量 + CRLF”。 */
uint8_t valve_txbuffer[VALVE_TX_LEN] = {};
/* 全局变量：气阀控制量数组，共 15 项；Valve 任务或控制逻辑修改后由 SendValve 发出。 */
uint8_t valve_num[VALVE_TX_LEN - 4] =
{
    0
}
;

/* 宏：光电传感器完整帧队列长度；UART7 和 UART10 各维护独立队列，当前各有 4 项。 */
/* PS 接收队列：UART10 回调沿用缓存投递逻辑；UART7 当前直接逐字节解析，队列结构保留。 */
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

/* 中文名：毫秒超时转节拍；作用：将 timeout_ms 毫秒转换为 FreeRTOS Tick 返回；0 不等待，0xFFFFFFFF 返回 portMAX_DELAY；无共享状态副作用。 */
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

/* 中文名：缓存地址向下对齐；作用：将输入地址 addr 向下对齐到 32 字节边界并返回，不修改内存。 */
static uint32_t DCache_AlignDown32(uint32_t addr)
{
    return addr & ~((uint32_t)32U - 1U);
}

/* 中文名：缓存地址向上对齐；作用：将输入地址 addr 向上对齐到 32 字节边界并返回，不修改内存。 */
static uint32_t DCache_AlignUp32(uint32_t addr)
{
    return (addr + (uint32_t)32U - 1U) & ~((uint32_t)32U - 1U);
}

/* 中文名：接收区缓存无效化；作用：按起始地址 addr 和字节数 len，将覆盖范围对齐到 32 字节后无效化 DCache；空地址或零长度直接返回，无返回值。 */
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

/* 中文名：启动UART8接收；作用：无参数，首次创建静态 StreamBuffer，启动 DMA + IDLE 接收并关闭半传输中断；成功返回 0，建流失败返回 -1，启动失败返回 -2。 */
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

/* 中文名：读取UART8接收流；作用：向 out 写入最多 maxLen 字节，按 timeout_ms 毫秒等待（0 不等待，0xFFFFFFFF 无限等待），返回实际读取字节数并消费流中数据；流未创建、空指针或零长度返回 0。 */
size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms)
{
    if ((s_uart8_rx_stream == NULL) || (out == NULL) || (maxLen == 0U))
    {
        return 0U;
    }
    TickType_t t = UartMsToTicks(timeout_ms);
    return xStreamBufferReceive(s_uart8_rx_stream, out, maxLen, t);
}

/* 中文名：查询UART8可读长度；作用：无参数，返回接收流内可读字节数，流未创建返回 0；不消费数据。 */
size_t User_UART8_Available(void)
{
    if (s_uart8_rx_stream == NULL)
    {
        return 0U;
    }
    return xStreamBufferBytesAvailable(s_uart8_rx_stream);
}

/* 中文名：清空UART8接收流；作用：无参数，将已创建的 StreamBuffer 复位并丢弃待读数据；流未创建直接返回，无返回值。 */
void User_UART8_Flush(void)
{
    if (s_uart8_rx_stream == NULL)
    {
        return;
    }
    (void)xStreamBufferReset(s_uart8_rx_stream);
}

/* 中文名：UART8接收事件回调；作用：huart 指定串口，Size 为本次接收字节数；仅处理非空 UART8，清除 IDLE 标志。
 * 将长度限制到 DMA 缓冲区容量，无效化 DCache 后向 StreamBuffer 投递数据，续启 DMA 并关闭半传输中断；必要时请求 ISR 任务切换，无返回值。 */
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

/* 中文名：初始化UART7帧队列；作用：无参数，首次创建每项 PS_RX_LEN 字节的静态队列并保存句柄，已有句柄直接返回；无返回值。 */
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

/* 中文名：初始化UART10帧队列；作用：无参数，首次创建每项 PS_RX_LEN 字节的静态队列并保存句柄，已有句柄直接返回；无返回值。 */
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

/* 中文名：发送气阀控制帧；作用：无参数，将 valve_num 的 15 项写入全局发送缓存，补齐 QF 帧头和 CRLF 帧尾后向 UART2 发起中断发送；无返回值。 */
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

/* 中文名：发送PS查询；作用：无参数，向 UART7 发起九字节 IO00000 加 CRLF 的中断发送。
 * 静态 query 在发送完成前保持有效；不检查 HAL 返回值，忙时不排队、不重试；无返回值，下方 UART10 历史发送代码保留。 */
void SendPs(void)
{
    static const uint8_t query[PS3_FRAME_LEN] =
    {
        'I', 'O', '0', '0', '0', '0', '0', '\r', '\n'
    }
    ;
    (void)HAL_UART_Transmit_IT(&PS_UART, query, (uint16_t)sizeof(query));

    /* 原接球发送源码保留；其长度/设备协议未确认，不启用UART10查询。 */
#if 0
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
#endif
}

/* 中文名：解析UART7队列帧；作用：无参数，非阻塞取出队列帧并校验 IO/CRLF，将当前九字节帧的五项 ASCII 数据转为数值；在临界区更新 ps_num 中除下标 3 外的项，保留 PS3 解析器结果；无返回值，队列未创建直接返回。 */
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
                /* 队列解析不得覆盖由独立九字节帧解析器更新的 PS3。 */
                for (int i = 0; i < (PS_RX_LEN - 4); i++)
                {
                    if (i != 3)
                        ps_num[i] = num[i];
                }
            }
            taskEXIT_CRITICAL();
        }
    }
}

/* 中文名：解析UART10队列帧；作用：无参数，非阻塞取出队列帧并校验 IO/CRLF，将当前九字节帧的五项 ASCII 数据转为数值，在临界区更新 ps_num_2；无返回值，队列未创建直接返回。 */
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

/* 中文名：串口接收完成回调；作用：huart 指定完成接收的串口；UART7 将收到的单字节交给 PS3 解析器并续收。
 * UART10 按现有计数逻辑投递九字节缓存到自身队列、推进接收位置并续收一字节，必要时请求 ISR 任务切换；空指针或其他串口不处理，无返回值。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == &PS_UART))
    {
        PS3_ParseByte(s_ps3_byte);
        (void)HAL_UART_Receive_IT(huart, &s_ps3_byte, 1U);
    }
    if ((huart != NULL) && (huart == &PS_UART_2))
    {
        static uint16_t ps_cnt_2 = 0U;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (ps_cnt_2 == 0U && s_ps_rx_queue_2 != NULL)
        {
            (void)xQueueSendFromISR(s_ps_rx_queue_2, ps_rxbuffer_2,
                &xHigherPriorityTaskWoken);
        }
        if (ps_cnt_2 < PS_RX_LEN - 1U)
        {
            ++ps_cnt_2;
        }
        else
        {
            ps_cnt_2 = 0U;
        }
        (void)HAL_UART_Receive_IT(huart, ps_rxbuffer_2 + ps_cnt_2, 1U);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* 中文名：串口错误恢复回调；作用：huart 指定出错串口；UART7 中止接收、清空候选帧计数并续收单字节，保留最近合法 PS3 值及有效标记。
 * UART10 发起中断方式中止接收并从自身缓存首字节续收；其他串口不处理，无返回值。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &PS_UART)
    {
        (void)HAL_UART_AbortReceive(huart);
        s_ps3_count = 0U;
        (void)HAL_UART_Receive_IT(huart, &s_ps3_byte, 1U);
    }
    else if (huart == &PS_UART_2)
    {
        (void)HAL_UART_AbortReceive_IT(huart);
        (void)HAL_UART_Receive_IT(huart, ps_rxbuffer_2, 1U);
    }
}

/* 中文名：初始化串口接收；作用：无参数，确保 UART7/UART10 帧队列已创建，清零 PS3 候选计数，为 UART7 独立单字节缓存和 UART10 帧缓存启动单字节中断接收；无返回值。 */
void USART_ENABLE(void)
{
    PS_RxQueueEnsureInit();
    PS_RxQueueEnsureInit_2();
    s_ps3_count = 0U;
    HAL_UART_Receive_IT(&PS_UART, &s_ps3_byte, 1U);
    HAL_UART_Receive_IT(&PS_UART_2, (uint8_t *)ps_rxbuffer_2, 1);
}
