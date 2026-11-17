#include "vofa.h"

#define MAX_BUFFER_SIZE 1024
#define VOFA_PORT       huart8
uint8_t send_buf[MAX_BUFFER_SIZE] __attribute__((section(".ram_d2"), aligned(32)));

#define MAX_FLOAT_COUNT 16 // 假设最多支持16个浮点数（根据实际需求可以修改）

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:      void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit_DMA(&VOFA_PORT, (uint8_t *)buf, len);
}

/**
 * @brief printf_vofa 函数用于格式化并发送浮点数数据。
 *
 * 该函数接收一个整数参数 float_count，表示后续将要传递的浮点数的数量，
 * 然后接收相应数量的浮点数参数。这些浮点数被存储在一个联合体中，
 * 既可以作为浮点数数组访问，也可以作为字节数组访问。函数会将这些字节
 * 拷贝到 send_buf 缓冲区，并在末尾添加特定的帧尾信息。最后，调用
 * vofa_transmit 函数发送整个数据包。
 *
 * @param float_count 要发送的浮点数的数量，不能超过 MAX_FLOAT_COUNT。
 * @param ... 可变参数列表，包含要发送的浮点数。
 *
 * @note 如果 float_count 超过 MAX_FLOAT_COUNT，函数将不执行任何操作。
 *       发送缓冲区 send_buf 和传输函数 vofa_transmit 需要在函数外部定义。
 */
void printf_vofa(int float_count, ...)
{
    // 检查传入的浮点数数量是否超过最大限制
    if (float_count > MAX_FLOAT_COUNT) {
        return; // 或者处理错误
    }

    union vofa {
        float dataf[MAX_FLOAT_COUNT];                   // 用浮点数组存储数据
        uint8_t datac[MAX_FLOAT_COUNT * sizeof(float)]; // 用字节数组存储数据（每个浮动占4字节，总共支持16个浮点数）
    };

    // 初始化 va_list 处理可变参数
    va_list args;
    va_start(args, float_count);

    // 填充浮点数到 dataf 数组
    union vofa vofa;
    for (uint8_t i = 0; i < float_count; i++) {
        vofa.dataf[i] = va_arg(args, double); // va_arg 获取浮点数参数
    }

    // 结束可变参数处理
    va_end(args);

    // 将数据从 datac 拷贝到 send_buf
    for (uint8_t i = 0; i < float_count * sizeof(float); i++) {
        send_buf[i] = vofa.datac[i];
    }

    // 发送帧尾
    send_buf[float_count * sizeof(float)]       = 0x00;
    send_buf[(float_count * sizeof(float)) + 1] = 0x00;
    send_buf[(float_count * sizeof(float)) + 2] = 0x80;
    send_buf[(float_count * sizeof(float)) + 3] = 0x7f;

    vofa_transmit((uint8_t *)send_buf, (float_count * sizeof(float)) + 4);
}

