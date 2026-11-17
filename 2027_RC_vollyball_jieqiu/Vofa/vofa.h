#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "math.h"
#include "usart.h"
#include <stdarg.h>  // 用于处理可变参数
#include <stdint.h>
#include <string.h>

void printf_vofa(int float_count, ...);

// 计算参数个数
#define COUNT_ARGS(...) (sizeof((float[]){__VA_ARGS__}) / sizeof(float))

// 发送的浮点数用逗号隔开，自动计算参数数量
#define vofa_printf(...) printf_vofa(COUNT_ARGS(__VA_ARGS__), __VA_ARGS__)

#endif /* __VOFA_H__ */














