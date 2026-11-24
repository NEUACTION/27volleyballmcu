/* 文件作用：宿主端断言测试小端整数/浮点位模式、十二轴槽布局、帧尾及非对齐复位事务字段。
 * 在宿主运行：cc -std=c11 -Wall -Wextra -Werror codec_test.c -o codec_test && ./codec_test
 * ARM交叉编译只能验证编译；不要把交叉产物当作已运行的测试。
 */
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include "../usb_codec.h"

_Static_assert(sizeof(float) == 4, "Protocol requires float32");
_Static_assert(VB_COMMAND_BYTES == 373, "Command length mismatch");
_Static_assert(VB_FEEDBACK_BYTES == 181, "Feedback length mismatch");
_Static_assert(VB_COMMAND_DATA_OFFSET + 4 * VB_COMMAND_SLOTS == VB_COMMAND_BYTES - 2,
    "Command tail overlaps data");
_Static_assert(VB_FEEDBACK_DATA_OFFSET + 4 * VB_FEEDBACK_SLOTS == VB_FEEDBACK_BYTES - 2,
    "Feedback tail overlaps data");

/* 中文名：运行编解码断言（main）。参数：无；返回：全部断言通过时为0。
 * 副作用：仅修改栈上测试字节数组；失败触发assert终止进程，不访问硬件或ROS。 */
int main(void)
{
    // command为下发整帧测试字节区；feedback为反馈整帧测试字节区，均含头尾与数据槽。
    uint8_t command[VB_COMMAND_BYTES] =
    {
        0
    }
    ;
    uint8_t feedback[VB_FEEDBACK_BYTES] =
    {
        0
    }
    ;
    // bytes为单个32位整数/浮点编码的4字节测试区，无特定物理量单位。
    uint8_t bytes[4] =
    {
        0
    }
    ;
    VbWriteU32(bytes, UINT32_C(0x12345678));
    assert(bytes[0] == 0x78 && bytes[1] == 0x56 && bytes[2] == 0x34 && bytes[3] == 0x12);
    assert(VbReadU32(bytes) == UINT32_C(0x12345678));
    VbWriteFloat(bytes, 1.0f);
    assert(VbReadU32(bytes) == UINT32_C(0x3F800000));
    assert(VbReadFloat(bytes) == 1.0f);
    VbWriteFloat(bytes, -0.0f);
    assert(VbReadU32(bytes) == UINT32_C(0x80000000));
    VbWriteFloat(bytes, -12.5f);
    assert(VbReadFloat(bytes) == -12.5f);
    command[0] = 'S'; command[1] = 'P';
    feedback[0] = 'Q'; feedback[1] = 'R';
    command[VB_COMMAND_BYTES - 2] = feedback[VB_FEEDBACK_BYTES - 2] = '\r';
    command[VB_COMMAND_BYTES - 1] = feedback[VB_FEEDBACK_BYTES - 1] = '\n';
    // axis为协议轴槽序号（0..11：0..7机构轴，8..11舵轮），不是硬件ID。
    for (size_t axis = 0; axis < VB_AXIS_COUNT; ++axis)
    {
        // field为当前轴下发字段序号（0..6，无量纲）。
        for (size_t field = 0; field < VB_COMMAND_FIELDS; ++field)
        {
            const size_t slot = 1 + axis * VB_COMMAND_FIELDS + field; // 数据区槽序号（个），跳过计数槽0。
            const size_t offset = VB_COMMAND_DATA_OFFSET + 4 * slot; // 当前下发字段相对帧首的字节偏移。
            const float value = (float)(axis * 100 + field); // 区分轴/字段的人工测试值，无物理单位。
            VbWriteFloat(command + offset, value);
            assert(VbReadFloat(command + offset) == value);
        }
        // field为当前轴反馈字段序号（0..2，无量纲），与上面的局部field独立。
        for (size_t field = 0; field < VB_FEEDBACK_FIELDS; ++field)
        {
            const size_t offset = VB_FEEDBACK_DATA_OFFSET + 4 * (1 + axis * VB_FEEDBACK_FIELDS + field); // 反馈字段相对帧首的字节偏移。
            VbWriteFloat(feedback + offset, (float)(axis + field));
            assert(VbReadFloat(feedback + offset) == (float)(axis + field));
        }
    }
    assert(command[VB_COMMAND_BYTES - 2] == '\r' && command[VB_COMMAND_BYTES - 1] == '\n');
    assert(feedback[VB_FEEDBACK_BYTES - 2] == '\r' && feedback[VB_FEEDBACK_BYTES - 1] == '\n');
    /* 复位事务位于非对齐控制区，不能覆盖mode/clean或数据槽。 */
    command[VB_RESET_REQUEST_BYTE] = 7;
    VbWriteU32(command + VB_RESET_ID_BYTE, UINT32_C(0x87654321));
    VbWriteU32(feedback + VB_RESET_ACK_BYTE, UINT32_C(0x87654321));
    feedback[VB_PS3_BYTE] = 1;
    feedback[VB_READY_BYTE] = 7;
    assert(command[VB_RESET_REQUEST_BYTE] == 7);
    assert(VbReadU32(command + VB_RESET_ID_BYTE) == UINT32_C(0x87654321));
    assert(VbReadU32(feedback + VB_RESET_ACK_BYTE) == UINT32_C(0x87654321));
    assert(feedback[VB_PS3_BYTE] == 1 && feedback[VB_READY_BYTE] == 7);
    assert(command[VB_COMMAND_DATA_OFFSET + 4] == 0); /* axis0 field0=0.0未覆盖。 */
    return 0;
}
