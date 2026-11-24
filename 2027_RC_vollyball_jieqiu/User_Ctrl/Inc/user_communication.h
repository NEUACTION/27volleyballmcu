/* 文件作用：声明主控USB CDC队列、任务回调、状态查询和合并协议帧长度。
 * 上下行字段使用共享定义；GetPCMsg是接收字节流的转交入口，不在此直接执行电机动作。
 * 保留原联合体与历史发送声明，当前活动八轴解析/反馈由合并模块处理。
 */
/* 头文件保护宏：避免 USB 通信接口重复展开。 */
#ifndef USER_COMMUNICATION_H
#define USER_COMMUNICATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "common.h"
#include "usb_protocol.h"

    /* 宏：原PC接收缓存上限，单位字节；合并解析实际使用261字节独立候选帧缓存。 */
#define MAX_GET_LEN (1024)

    /* 宏：USB 接收帧中 uint8_t 项的数量；当前协议解析使用。 */
#define USB_RX_UINT8_NUM (VB_TX_CONTROL_BYTES)
    /* 宏：USB 接收帧中 32 位数据项数量；当前协议接收布局。 */
#define USB_RX_DATA_NUM  (VB_COMMAND_SLOTS)
    /* 宏：USB 发送帧中 uint8_t 项的数量；当前协议发送布局。 */
#define USB_TX_UINT8_NUM (29)
    /* 宏：USB 发送帧中 32 位数据项数量；当前协议发送布局。 */
#define USB_TX_DATA_NUM  (VB_FEEDBACK_SLOTS)

    /* 宏：按接收协议字段数量计算完整接收帧字节长度。 */
#define USB_RX_LEN (USB_RX_DATA_NUM*4 + USB_RX_UINT8_NUM+4)
    /* 宏：按发送协议字段数量计算完整发送帧字节长度。 */
#define USB_TX_LEN (USB_TX_DATA_NUM*4 + USB_TX_UINT8_NUM+4)

    typedef union
    {
        /* 接收数据的有符号整数视图；与联合体其他视图共享同一存储空间。 */
        int		 dataint[USB_RX_DATA_NUM];
        /* 接收数据的无符号 32 位视图；与浮点和字符视图共用缓冲区。 */
        uint32_t dataint32[USB_RX_DATA_NUM];
        /* 历史接收联合体的浮点视图；当前合并解析使用独立缓存，不读取此视图。 */
        float    dataf[USB_RX_DATA_NUM];
        /* 接收数据的原始字节视图；容量为数据项数乘 4。 */
        char     datac[USB_RX_DATA_NUM*4];
    }
    trans_r;

    typedef union
    {
        /* 发送数据的有符号整数视图；与联合体其他视图共用存储空间。 */
        int		 dataint[USB_TX_DATA_NUM];
        /* 历史发送联合体的无符号 32 位视图；当前合并反馈打包不使用此视图。 */
        uint32_t dataint32[USB_TX_DATA_NUM];
        /* 历史浮点遥测视图，仅保留的已注释 SendPCMsg 代码使用。 */
        float    data_float[USB_TX_DATA_NUM];
        /* 发送数据的原始字节视图；容量为数据项数乘 4。 */
        char     datac[USB_TX_DATA_NUM*4];
    }
    trans_t;

    /* 外部变量：USB 发送协议联合体；仅被已屏蔽的 SendPCMsg 逻辑引用，当前活动发送未使用。 */
    extern trans_t TX_Data;		// 发送联合体
    /* 外部变量：USB 接收协议联合体；仅被已屏蔽的 GetPCMsg 逻辑引用，当前活动接收未使用。 */
    extern trans_r RX_Data;		// 接收联合体


    /* 中文名：创建USB收发资源；作用：无参数，分别创建 TX/RX 字节流和唯一收发任务，已有资源不重复创建；创建失败保留空句柄，无返回值。 */
    void User_Communication_Init(void);
    /* 中文名：检查USB枚举完成；作用：无参数，读取 HS 设备状态和类数据指针，均就绪返回 true，否则 false；不改变链路状态。 */
    bool UsbCdc_IsConfigured(void);

    /* ---------- USB TX ---------- */

    /* 中文名：通知USB发送完成；作用：无参数，在 CDC 完成中断中通知已记录的发送任务继续工作，必要时请求任务切换；任务句柄为空时跳过，无返回值。 */
    void UsbTx_OnCdcTxCpltFromISR(void);
    /* 中文名：查询USB链路就绪；作用：无参数，返回 UsbCdc_IsConfigured 的枚举就绪结果；不检查发送队列余量或 CDC 当前忙闲。 */
    bool UsbTx_IsReady(void);
    /* 中文名：读取USB发送计数；作用：向 enqueued_bytes/dropped_bytes 输出累计入队/丢弃字节数，向 tx_ok_pkts/tx_busy_cnt 输出提交成功次数/忙状态次数；各空指针独立跳过，无返回值。 */
    void UsbTx_GetStats(uint32_t *enqueued_bytes, uint32_t *dropped_bytes, uint32_t *tx_ok_pkts, uint32_t *tx_busy_cnt);
    /* 中文名：将字节入队等待USB发送；作用：复制 data 的 len 字节到 TX 流，timeout_ms 为总等待毫秒数（0 不等待，0xFFFFFFFF 无限等待）。
     * 全部入队返回 0，空数据/零长度/流未创建返回 -1，超时或无法继续入队返回 -2；失败前已入队部分仍保留，更新入队及相应丢弃计数。 */
    int UsbTx_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
    /* 中文名：将字符串加入USB发送流；作用：发送 str 的 strlen 字节，不含终止零且不追加换行；timeout_ms 为入队等待毫秒数，空指针返回 -1，其余返回 UsbTx_Send 的入队结果。 */
    int UsbTx_SendString(const char *str, uint32_t timeout_ms);
    /* 中文名：格式化USB调试文本；作用：按 fmt 及可变参数格式化文本，最多发送 255 字节并以零超时入队；返回入队结果，格式空或格式化失败返回 -1，未枚举返回 -3；不自动添加换行。 */
    int DEBUG_USB(const char *fmt, ...);
    /* 中文名：递增历史设备心跳；作用：无参数，将各执行器、PC 和操作端心跳计数逐次饱和递增至 201，接收方另行清零；无返回值，当前主任务调用已注释。 */
    void HB_Add(void);

    typedef struct
    {
        /* 累计进入 USB 发送队列的字节数。 */
        uint32_t tx_enqueued_bytes;
        /* 入队返回零时累计的剩余字节数；总超时提前返回分支不累加。 */
        uint32_t tx_dropped_bytes;
        /* CDC 返回 USBD_OK 的累计提交批次数，未确认物理传输完成。 */
        uint32_t tx_ok_pkts;
        /* CDC 发送接口返回忙状态的累计次数。 */
        uint32_t tx_busy_cnt;
        /* 发送任务未能成功提交给 CDC 的累计批次数。 */
        uint32_t tx_fail_pkts;
        /* 发送失败涉及的累计字节数。 */
        uint32_t tx_fail_bytes;
        /* USB 接收 ISR 成功写入原始接收流的累计字节数。 */
        uint32_t rx_isr_enqueued_bytes;
        /* USB 接收 ISR 未能写入原始接收流而丢弃的累计字节数。 */
        uint32_t rx_isr_dropped_bytes;
        /* 历史二级应用流转发字节计数；当前未使用二级流，此计数保持初值。 */
        uint32_t app_rx_forwarded_bytes;
        /* 历史二级应用流丢弃字节计数；当前未使用二级流，此计数保持初值。 */
        uint32_t app_rx_dropped_bytes;
    }
    UsbStats_t;



    /* 中文名：读取USB收发统计；作用：将累计收发计数逐字段复制到 out 指向的 UsbStats_t，空指针直接返回；不清零计数，复制过程未加锁，无返回值。 */
    void Usb_GetStats(UsbStats_t *out);

    /* ---------- USB RX ---------- */
    /* 中文名：投递USB中断接收字节；作用：从 data 复制 len 字节到原始 RX 流，累计实际入队和未入队字节，必要时请求 ISR 任务切换；空数据、零长度或未建流直接返回，无返回值。 */
    void UsbRx_OnCdcRxFromISR(const uint8_t *data, uint32_t len);
    /* 中文名：读取USB原始接收流；作用：最多向 out 写入 out_len 字节，以 timeout_ms 毫秒等待（0 不等待，0xFFFFFFFF 无限等待），返回实际读取字节数；参数无效或流未创建返回 0。
     * 本接口与 UsbRxTask 消费同一个 RX 流，读取会取走尚未分发的数据。 */
    uint16_t UsbRx_Read(uint8_t *out, uint16_t out_len, uint32_t timeout_ms);
    /* 中文名：查询USB原始接收积压；作用：无参数，返回 RX 流当前可读字节数，流未创建返回 0；不消费数据。 */
    size_t UsbRx_Available(void);
    /* 中文名：读取USB中断接收计数；作用：向 rx_enqueued_bytes/rx_dropped_bytes 分别输出累计成功入队/丢弃字节数，各指针可独立为空；不清零计数，无返回值。 */
    void UsbRx_GetStats(uint32_t *rx_enqueued_bytes, uint32_t *rx_dropped_bytes);

    /* 回调类型：data 指向本批临时字节缓存，len 为字节数；在接收任务同步调用，数据可为半帧或多帧，回调不得保存临时指针供后续使用。 */
    typedef void (*UsbRx_Callback_t)(const uint8_t *data, uint16_t len);
    /* 中文名：注册USB字节接收回调；作用：将 cb 保存为接收任务的分发入口，NULL 取消回调；当前初始化注册 GetPCMsg，实际调用发生在 UsbRxTask 上下文，无返回值。 */
    void UsbRx_RegisterCallback(UsbRx_Callback_t cb);

    /* ---------- Process ---------- */

    /* 中文名：接收并转交小电脑消息。
     * 作用：USB接收任务（UsbRxTask）调用本回调，将data的len个字节转交合并解析入口；
     * 字节可能属于半帧或多帧；更新合并层拼帧状态和命令快照，无返回值，不直接控制电机。
     */
    void GetPCMsg(const uint8_t *data, uint16_t len);
    /* 中文名：历史PC反馈帧发送入口；作用：无参数，原实现组装心跳、光电和电机遥测后投递 USB 发送流；无返回值，当前实现及调用均已注释，仅保留兼容声明。 */
    void SendPCMsg(void);

#ifdef __cplusplus
}
#endif

#endif // USER_COMMUNICATION_H