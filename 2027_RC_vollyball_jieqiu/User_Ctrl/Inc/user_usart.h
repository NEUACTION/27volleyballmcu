#ifndef USER_USART_H
#define USER_USART_H

#ifdef __cplusplus
extern "C" {
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
#define DEBUG(...) {\
    int len = 0;\
    len = snprintf((char *)debug_buffer,DEBUG_BUFFER_SIZE,__VA_ARGS__);\
    if (len > 0){\
        HAL_UART_Transmit_IT(&DEBUG_UART_HANDLER, (uint8_t *)debug_buffer, len);\
    }\
}

/* 宏：气阀 UART 句柄，当前固定为 huart2。 */
#define VALVE_UART		huart2
/* 宏：气阀完整发送帧长度，当前为 19 字节（帧头 2 + 数据 15 + 帧尾 2）。 */
#define VALVE_TX_LEN	19
/* 外部变量：气阀控制量数组，共 15 项；由控制逻辑修改，当前部分索引由任务或被屏蔽代码使用。 */
extern uint8_t valve_num[VALVE_TX_LEN - 4];

/* 宏：第一路光电传感器 UART 句柄，当前固定为 huart7。 */
#define PS_UART			huart7
/* 宏：光电传感器接收帧长度，当前为 20 字节（帧头 2 + 数据 16 + 帧尾 2）。 */
#define PS_RX_LEN 	    9
#define PS_TX_LEN 	    9
/* 外部变量：UART7 光电传感器解析结果，共 16 项；接收回调和 GetPSNum 可更新。 */
extern uint8_t ps_num[PS_RX_LEN - 4];
/* 外部变量：UART7 光电传感器接收帧；中断逐字节写入，当前接收模式已启用。 */
extern uint8_t ps_rxbuffer[PS_RX_LEN];
/* 外部变量：UART7 光电传感器发送帧；SendPs 填充后通过中断发送。 */
extern uint8_t ps_txbuffer[PS_TX_LEN];

/* 宏：第二路光电传感器 UART 句柄，当前固定为 huart10。 */
#define PS_UART_2			huart10
/* 外部变量：UART10 光电传感器解析结果，共 16 项；接收回调可更新。 */
extern uint8_t ps_num_2[PS_RX_LEN - 4];
/* 外部变量：UART10 光电传感器接收帧；中断逐字节写入，当前接收模式已启用。 */
extern uint8_t ps_rxbuffer_2[PS_RX_LEN];
/* 外部变量：UART10 光电传感器发送帧；SendPs 填充后通过中断发送。 */
extern uint8_t ps_txbuffer_2[PS_TX_LEN];

/*
 * UART8 RX via DMA (ReceiveToIdle)
 * - TX: 工程里 VOFA 已使用 HAL_UART_Transmit_DMA(&huart8,...)
 * - RX: 这里提供 DMA+IDLE 收数，并把字节写入一个 StreamBuffer
 */

/* 函数声明：启动 UART8 DMA + IDLE 接收；当前由 User_Task_Init 调用。 */
int User_UART8_RxDmaStart(void);
/* 函数声明：读取 UART8 接收流中的数据；当前声明但工程中未直接调用。 */
size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms);
/* 函数声明：查询 UART8 接收流可读字节数；当前声明但工程中未直接调用。 */
size_t User_UART8_Available(void);
/* 函数声明：清空 UART8 接收流；当前声明但工程中未直接调用。 */
void User_UART8_Flush(void);
/* 函数声明：发送气阀控制帧；在控制逻辑启用时调用，部分直接调用已存在。 */
void SendValve(void);
/* 函数声明：发送两路光电传感器查询帧；当前由任务周期性调用。 */
void SendPs(void);
/* 函数声明：从 UART7 队列解析传感器数据；当前已实现但未直接调用，可恢复任务周期读取时使用。 */
void GetPSNum(void);
/* 函数声明：从 UART10 队列解析传感器数据；当前已实现但未直接调用，可恢复任务周期读取时使用。 */
void GetPSNum2(void);
/* 函数声明：初始化并启动串口接收；当前由 User_Task_Init 调用一次。 */
void USART_ENABLE(void);

#ifdef __cplusplus
}
#endif

#endif
