/* 头文件保护宏：防止 common.h 在同一编译单元中重复展开。 */
#ifndef USER_CTRL_COMMON_H
#define USER_CTRL_COMMON_H

/* C++ 兼容保护：在 C++ 编译时将后续函数声明标记为 C 链接。 */
#ifdef __cplusplus
extern "C" {
#endif

/* C standard */
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* STM32 / HAL */
#include "main.h"
#include "fdcan.h"
#include "tim.h"

/* RTOS */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#ifndef PI
    /* 圆周率常量；仅在工程或库未定义 PI 时启用此定义。 */
    #define PI 3.14159265358979323846
#endif


/* 宏：将蜂鸣器 GPIO 置高，当前用于开启蜂鸣器；定义后可供控制模块调用。 */
#define BEEP_ON  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
/* 宏：将蜂鸣器 GPIO 置低，当前用于关闭蜂鸣器；定义后可供控制模块调用。 */
#define BEEP_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)

/* 函数声明：将毫秒超时换算为 RTOS Tick；在任务与串口代码中使用。 */
TickType_t ms_to_ticks(uint32_t timeout_ms);

/* ------------------------------------------------------------
 * Clamp / Saturation utilities
 * - 通用限幅：把变量限制在 [min, max]
 * - 提供“返回限幅值”和“原地限幅（指针）”两种风格
 * - 避免宏导致的多次求值问题
 * ------------------------------------------------------------ */

/* 函数：返回 int32_t 限幅结果；当前为通用内联工具，供控制代码复用。 */
static inline int32_t clamp_i32(int32_t x, int32_t min, int32_t max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

/* 函数：返回 uint32_t 限幅结果；当前为通用内联工具，供控制代码复用。 */
static inline uint32_t clamp_u32(uint32_t x, uint32_t min, uint32_t max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

/* 函数：返回 float 限幅结果；当前为通用内联工具，供控制代码复用。 */
static inline float clamp_f32(float x, float min, float max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

/* 原地限幅：Saturation_int(&x, min, max) */
/* 函数：对非空 int32_t 指针原地限幅；当前用于需要修改原值的场景。 */
static inline void Saturation_int(int32_t *x, int32_t min, int32_t max)
{
	if (x == NULL) return;
	*x = clamp_i32(*x, min, max);
}

/* 函数：对非空 uint32_t 指针原地限幅；当前为通用内联工具。 */
static inline void Saturation_uint(uint32_t *x, uint32_t min, uint32_t max)
{
	if (x == NULL) return;
	*x = clamp_u32(*x, min, max);
}

/* 函数：对非空 float 指针原地限幅；当前用于控制目标限幅。 */
static inline void Saturation_float(float *x, float min, float max)
{
	if (x == NULL) return;
	*x = clamp_f32(*x, min, max);
}

#ifdef __cplusplus
}
#endif

#endif /* USER_CTRL_COMMON_H */
