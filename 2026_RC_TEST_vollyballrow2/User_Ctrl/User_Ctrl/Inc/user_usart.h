#ifndef USER_USART_H
#define USER_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "usart.h"


// DEBUG 发数用
// #define DEBUG_UART_HANDLER             (huart8)          
// #define DEBUG_BUFFER_SIZE              (4096)
// extern uint8_t debug_buffer[DEBUG_BUFFER_SIZE];

// //DEBUG重载发数
// #define DEBUG(...) {\
//     int len = 0;\
//     len = snprintf((char *)debug_buffer,DEBUG_BUFFER_SIZE,__VA_ARGS__);\
//     if (len > 0){\
//         HAL_UART_Transmit_IT(&DEBUG_UART_HANDLER, (uint8_t *)debug_buffer, len);\
//     }\
// }


// #ifndef USER_USART_H
// #define USER_USART_H

#include "common.h"
#include "usart.h"
#include <stdio.h>   // 必须包含，snprintf 需要
#include <stdint.h>

// ==================== DEBUG 发数配置（保留原样） ====================
#define DEBUG_UART_HANDLER             (huart8)
#define DEBUG_BUFFER_SIZE              (4096)

void Debug_Init(void);
void Debug_Push(const uint8_t *data, uint16_t len);
void Debug_OnTxComplete(void);
uint32_t Debug_GetDropCount(void);

// ==================== DEBUG 宏重载 ====================
#define DEBUG(...) do { \
    static uint8_t _dbg_fmt[256]; \
    int _dbg_n = snprintf((char *)_dbg_fmt, sizeof(_dbg_fmt), __VA_ARGS__); \
    if (_dbg_n > 0) { \
        if (_dbg_n >= (int)sizeof(_dbg_fmt)) { \
            _dbg_n = (int)sizeof(_dbg_fmt) - 1; \
        } \
        Debug_Push(_dbg_fmt, (uint16_t)_dbg_n); \
    } \
} while (0)

// end  USER_USART_H






#define VALVE_UART		huart5
#define VALVE_TX_LEN	1
extern uint8_t valve_num[20];

#define VALVE2_UART		huart7
#define VALVE_TX_LEN	1
extern uint8_t valve2_num[20];

#define VALVE_RX_LEN 5

#define PS_UART			huart3
#define PS_RX_LEN 	    9
#define PS_TX_LEN       9
extern uint8_t ps_num[PS_RX_LEN - 4];
extern uint8_t ps_rxbuffer[PS_RX_LEN];
extern uint8_t ps_txbuffer[PS_TX_LEN];

/* 调试计数（定位完可删） */
extern volatile uint32_t ps_rx_total;
extern volatile uint32_t ps_rx_ok;
extern volatile uint32_t ps_rx_bad;
extern volatile uint8_t ps_last_raw[PS_RX_LEN];
extern volatile uint8_t ps_last_byte;
extern volatile uint32_t ps_rx_arm_fail;
extern volatile uint32_t ps_rx_err;
extern volatile uint32_t ps_err_code;

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
void GetPSNum2(void);
void USART_ENABLE(void);

#ifdef __cplusplus
}
#endif

#endif
