/* 文件作用：声明八轴主控的USB/CAN接收、软件初始化、5ms执行接口及PVT验证开关。
 * 十二轴合并主控适配层（8机构电机 + 4舵轮）仅执行PC最新目标，不产生本地轨迹或自动复位。
 * 双臂事务的ARM2复位mask位仅表示ARM1收到复位完成反馈后ARM2的SetDriverZero成功入队；
 * 它不代表ARM2机械到位，也不表示驱动器返回了新的复位确认。 */
#ifndef MERGED_CONTROL_H
#define MERGED_CONTROL_H

#include <stdint.h>
#include "fdcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* 腿PVT按用户要求开启；原注释提示的量化/增益尚未完成电驱验证，首次上机须低速小幅度确认。 */
#ifndef MERGED_ALLOW_UNVALIDATED_PVT
#define MERGED_ALLOW_UNVALIDATED_PVT 1
#endif

    /* 中文名：初始化合并控制软件状态。
     * 作用：开机清零命令、反馈、复位事务和收帧状态；默认run为0、各轴等待失能确认，不选择接球或发球，不发复位、置零或使能命令。
     */
    /* 参数：无；先于回调注册调用，清软件状态不代表硬件已失能。 */
    void MergedControl_Init(void);
    /* 中文名：接收并拼接小电脑USB字节流。
     * 作用：data为本次收到的字节，length为字节数；累计SP开头的373字节帧、校验CRLF并提交完整命令，坏帧重新寻找帧头。
     */
    /* 参数：data为仅本次回调有效的只读USB缓冲（NULL忽略）；length为有效字节数，可分包/粘包或为0。 */
    void MergedControl_OnUsbBytes(const uint8_t *data, uint16_t length);
    /* 中文名：合并控制五毫秒执行入口。
     * 作用：用户主任务每5ms调用；取得命令和反馈快照，推进显式复位事务或执行各轴目标，再轮询复位状态并回传反馈。没有PC命令时保持run为0，仅失能与读取状态，不运行本地接球轨迹。
     */
    /* 参数：无；只能由单一控制任务每5ms调用，不可并发或放在接收中断中执行。 */
    void MergedControl_Tick(void);
    /* 中文名：解析八轴驱动器CAN反馈。
     * 作用：仅处理bus为CAN3、identifier为0x281至0x288的回报；length是实际字节数，每8字节解析一次并更新真实反馈及回报序号。
     */
    /* 参数：bus为来源HAL句柄（仅CAN3有效）；identifier为标准ID；data为只读载荷；length为实际字节数0..64，非HAL DLC编码。 */
    void MergedControl_OnCanFrame(FDCAN_HandleTypeDef *bus, uint32_t identifier,
        const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif
#endif
