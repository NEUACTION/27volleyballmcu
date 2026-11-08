#ifndef USER_CTRL_COMMON_H
#define USER_CTRL_COMMON_H

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
    #define PI 3.14159265358979323846
#endif


#define BEEP_ON  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)

TickType_t ms_to_ticks(uint32_t timeout_ms);

/* ------------------------------------------------------------
 * Clamp / Saturation utilities
 * - 通用限幅：把变量限制在 [min, max]
 * - 提供“返回限幅值”和“原地限幅（指针）”两种风格
 * - 避免宏导致的多次求值问题
 * ------------------------------------------------------------ */

static inline int32_t clamp_i32(int32_t x, int32_t min, int32_t max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

static inline uint32_t clamp_u32(uint32_t x, uint32_t min, uint32_t max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

static inline float clamp_f32(float x, float min, float max)
{
	if (x > max) return max;
	if (x < min) return min;
	return x;
}

/* 原地限幅：Saturation_int(&x, min, max) */
static inline void Saturation_int(int32_t *x, int32_t min, int32_t max)
{
	if (x == NULL) return;
	*x = clamp_i32(*x, min, max);
}

static inline void Saturation_uint(uint32_t *x, uint32_t min, uint32_t max)
{
	if (x == NULL) return;
	*x = clamp_u32(*x, min, max);
}

static inline void Saturation_float(float *x, float min, float max)
{
	if (x == NULL) return;
	*x = clamp_f32(*x, min, max);
}

#ifdef __cplusplus
}
#endif

#endif /* USER_CTRL_COMMON_H */
