/* 文件作用：声明用户串口、PS3协议长度、收发缓存以及UART8接收流和光电查询接口。
 * 发球光电已迁至UART7 PE8 TX/PE7 RX，115200 8N1；PS_RX_LEN沿用用户确认的9字节。
 * 这里只声明接口和共享数据，具体串口中断解析在user_usart.c实现。
 */
#ifndef USER_USART_H
#define USER_USART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "common.h"
#include "usart.h"


    /* 宏：调试串口句柄，当前固定为 huart5，使用中断方式发送。 */
#define DEBUG_UART_HANDLER             (huart5)          
    /* 宏：调试格式化缓冲区大小（字节），当前为 4096 字节。 */
#define DEBUG_BUFFER_SIZE              (4096)
    /* 外部变量：调试日志共享缓冲区；由 DEBUG 宏写入并通过 huart5 中断发送。 */
    extern uint8_t debug_buffer[DEBUG_BUFFER_SIZE];

    /* 宏：格式化调试数据并中断发送；长度为 0 或负数时不会调用 HAL_UART_Transmit_IT。 */
#define DEBUG(...) \
{ \
    int len = 0; \
    len = snprintf((char *)debug_buffer,DEBUG_BUFFER_SIZE,__VA_ARGS__); \
    if (len > 0) \
    { \
        HAL_UART_Transmit_IT(&DEBUG_UART_HANDLER, (uint8_t *)debug_buffer, len); \
    } \
}

    /* 宏：气阀 UART 句柄，当前固定为 huart2。 */
#define VALVE_UART		huart2
    /* 宏：气阀完整发送帧长度，当前为 19 字节（帧头 2 + 数据 15 + 帧尾 2）。 */
#define VALVE_TX_LEN	19
    /* 外部变量：气阀控制量数组，共 15 项；由控制逻辑修改，当前部分索引由任务或被屏蔽代码使用。 */
    extern uint8_t valve_num[VALVE_TX_LEN - 4];

    /* 宏：第一路改接原发球PS设备，UART7：PE8 TX、PE7 RX，115200 8N1。 */
#define PS_UART            huart7
    /* 宏：接收缓存和队列帧长度当前为 9 字节，对应五项数据；UART7 独立解析器采用 PS3_FRAME_LEN。 */
#define PS_RX_LEN          9
    /* PS3_FRAME_LEN：原发球整帧长度，IO + 5个ASCII 0/1 + CRLF。 */
#define PS3_FRAME_LEN      9U
    /* ps_num（光电数值数组）：当前共五项；UART7 PS3 解析器只更新下标 3，保留队列解析函数可更新其余项。 */
    extern uint8_t ps_num[PS_RX_LEN - 4];
    /* ps3_valid（PS3有效标记）：启动后收到过合法帧置1；不增加USB协议字段。 */
    extern volatile uint8_t ps3_valid;
    /* 外部变量：保留的接收缓存，当前为九字节；UART7 PS3 解析器使用独立单字节缓存。 */
    extern uint8_t ps_rxbuffer[PS_RX_LEN];
    /* 外部变量：保留的旧发送缓存；查询帧使用独立9字节静态数组。 */
    extern uint8_t ps_txbuffer[PS_RX_LEN];

    /* 宏：第二路光电传感器 UART 句柄，当前固定为 huart10。 */
#define PS_UART_2			huart10
    /* 外部变量：UART10 光电传感器解析结果，当前共五项；GetPSNum2 从队列取帧后更新。 */
    extern uint8_t ps_num_2[PS_RX_LEN - 4];
    /* 外部变量：UART10 光电传感器接收帧；中断逐字节写入，当前接收模式已启用。 */
    extern uint8_t ps_rxbuffer_2[PS_RX_LEN];
    /* 外部变量：UART10 光电传感器发送缓存，定义长度当前为九字节；SendPs 中相关历史发送代码以 #if 0 保留。 */
    extern uint8_t ps_txbuffer_2[];

    /*
     * UART8 RX via DMA (ReceiveToIdle)
     * - TX: 工程里 VOFA 已使用 HAL_UART_Transmit_DMA(&huart8,...)
     * - RX: 这里提供 DMA+IDLE 收数，并把字节写入一个 StreamBuffer
     */

    /* 中文名：启动UART8接收；作用：无参数，首次创建静态 StreamBuffer，启动 DMA + IDLE 接收并关闭半传输中断；成功返回 0，建流失败返回 -1，启动失败返回 -2。 */
    int User_UART8_RxDmaStart(void);
    /* 中文名：读取UART8接收流；作用：向 out 写入最多 maxLen 字节，按 timeout_ms 毫秒等待（0 不等待，0xFFFFFFFF 无限等待），返回实际读取字节数并消费流中数据；流未创建、空指针或零长度返回 0。 */
    size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms);
    /* 中文名：查询UART8可读长度；作用：无参数，返回接收流内可读字节数，流未创建返回 0；不消费数据。 */
    size_t User_UART8_Available(void);
    /* 中文名：清空UART8接收流；作用：无参数，将已创建的 StreamBuffer 复位并丢弃待读数据；流未创建直接返回，无返回值。 */
    void User_UART8_Flush(void);
    /* 中文名：发送气阀控制帧；作用：无参数，将 valve_num 的 15 项写入全局发送缓存，补齐 QF 帧头和 CRLF 帧尾后向 UART2 发起中断发送；无返回值。 */
    void SendValve(void);
    /* 中文名：发送PS查询；作用：无参数，向 UART7 发起九字节 IO00000 加 CRLF 的中断发送。
     * 静态 query 在发送完成前保持有效；不检查 HAL 返回值，忙时不排队、不重试；无返回值；实现中以 #if 0 保留 UART10 历史发送代码。 */
    void SendPs(void);
    /* 中文名：解析UART7队列帧；作用：无参数，非阻塞取出队列帧并校验 IO/CRLF，将当前九字节帧的五项 ASCII 数据转为数值；在临界区更新 ps_num 中除下标 3 外的项，保留 PS3 解析器结果；无返回值，队列未创建直接返回。 */
    void GetPSNum(void);
    /* 中文名：解析UART10队列帧；作用：无参数，非阻塞取出队列帧并校验 IO/CRLF，将当前九字节帧的五项 ASCII 数据转为数值，在临界区更新 ps_num_2；无返回值，队列未创建直接返回。 */
    void GetPSNum2(void);
    /* 中文名：初始化串口接收；作用：无参数，确保 UART7/UART10 帧队列已创建，清零 PS3 候选计数，为 UART7 独立单字节缓存和 UART10 帧缓存启动单字节中断接收；无返回值。 */
    void USART_ENABLE(void);

#ifdef __cplusplus
}
#endif

#endif
