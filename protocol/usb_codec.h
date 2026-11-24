/* 文件作用：提供共享协议的纯C小端uint32/float32字节编解码，不访问HAL/ROS或计算轨迹。
 * 调用者保证源/目标至少可访问4字节；不检查空指针和边界，不改变字段物理单位。 */
#ifndef VOLLEYBALL_USB_CODEC_H
#define VOLLEYBALL_USB_CODEC_H
#include <stdint.h>
#include <string.h>
#include "usb_protocol.h"

/* 中文名：写小端32位整数（VbWriteU32）。作用：按低字节在前编码，不依赖宿主端序。
 * 参数：destination为至少4字节的可写区，value为待编码整数（计数/位模式，无量纲）。
 * 返回：无；副作用：改写destination[0..3]，不访问设备。 */
static inline void VbWriteU32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

/* 中文名：读小端32位整数（VbReadU32）。参数：source为至少4字节的只读源区。
 * 返回：按小端拼接的uint32整数（无量纲），计数器不经过float转换。
 * 副作用：无，仅读取source[0..3]，不修改源区。 */
static inline uint32_t VbReadU32(const uint8_t *source)
{
    return ((uint32_t)source[0]) | ((uint32_t)source[1] << 8) |
    ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

/* 中文名：写小端浮点位模式（VbWriteFloat）。参数：destination为至少4字节可写区，
 * value为float32值，单位由位置/速度/电流/增益字段决定。返回：无。
 * 副作用：改写destination[0..3]；按原始位编码，保留负零等位模式，不作数值量纲转换。 */
static inline void VbWriteFloat(uint8_t *destination, float value)
{
    uint32_t bits = 0; /* value的32位原始位暂存（无量纲），不是浮点到整数的数值转换。 */
    memcpy(&bits, &value, sizeof(bits));
    VbWriteU32(destination, bits);
}

/* 中文名：读小端浮点位模式（VbReadFloat）。参数：source为至少4字节只读源区，无对齐要求。
 * 返回：恢复出的float32值，物理单位沿用协议字段；副作用：无，不修改source。 */
static inline float VbReadFloat(const uint8_t *source)
{
    const uint32_t bits = VbReadU32(source); // 源字节拼成的32位浮点位模式，无量纲。
    float value = 0.0f; /* 返回的原始浮点数，单位由字段定义决定。 */
    memcpy(&value, &bits, sizeof(value));
    return value;
}

#endif
