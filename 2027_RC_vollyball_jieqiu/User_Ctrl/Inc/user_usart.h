#ifndef USER_USART_H
#define USER_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "usart.h"


// DEBUG 发数用
#define DEBUG_UART_HANDLER             (huart5)          
#define DEBUG_BUFFER_SIZE              (4096)
extern uint8_t debug_buffer[DEBUG_BUFFER_SIZE];

//DEBUG重载发数
#define DEBUG(...) {\
    int len = 0;\
    len = snprintf((char *)debug_buffer,DEBUG_BUFFER_SIZE,__VA_ARGS__);\
    if (len > 0){\
        HAL_UART_Transmit_IT(&DEBUG_UART_HANDLER, (uint8_t *)debug_buffer, len);\
    }\
}

#define VALVE_UART		huart2
#define VALVE_TX_LEN	19
extern uint8_t valve_num[VALVE_TX_LEN - 4];

#define PS_UART			huart7
#define PS_RX_LEN 	    20
#define PS_TX_LEN       20
extern uint8_t ps_num[PS_RX_LEN - 4];
extern uint8_t ps_rxbuffer[PS_RX_LEN];
extern uint8_t ps_txbuffer[PS_TX_LEN];

#define PS_UART_2			huart10
extern uint8_t ps_num_2[PS_RX_LEN - 4];
extern uint8_t ps_rxbuffer_2[PS_RX_LEN];
extern uint8_t ps_txbuffer_2[PS_TX_LEN];

/*
 * UART8 RX via DMA (ReceiveToIdle)
 * - TX: 工程里 VOFA 已使用 HAL_UART_Transmit_DMA(&huart8,...)
 * - RX: 这里提供 DMA+IDLE 收数，并把字节写入一个 StreamBuffer
 */

int User_UART8_RxDmaStart(void);
size_t User_UART8_Read(uint8_t *out, size_t maxLen, uint32_t timeout_ms);
size_t User_UART8_Available(void);
void User_UART8_Flush(void);
void SendValve(void);
void SendPs(void);
void GetPSNum(void);
void GetPSNum2(void);
void USART_ENABLE(void);

#ifdef __cplusplus
}
#endif

#endif
