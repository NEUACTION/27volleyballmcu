/* 文件作用：实现USB CDC接收、发送队列和任务，管理收发缓存及链路状态。
 * 接收任务通过GetPCMsg将字节流交给合并模块拼帧，发送任务消费已入队的反馈帧。
 * 本文件不计算轨迹、不直接给电机发目标；USB回调也不等于一次完整协议帧。
 */
#include "merged_control.h"
#include "user_communication.h"
#include "user_control.h"
#include "user_usart.h"

#include "usbd_cdc_if.h"
#include "usbd_def.h"

/* 外部变量：HS USB Device 句柄；UsbCdc_IsConfigured 读取其枚举状态和 class data。 */
extern USBD_HandleTypeDef hUsbDeviceHS;

/*
 * ------------------------------------------------------------
 * USB CDC 通信层（user_communication）
 * ------------------------------------------------------------
 * 目标：
 *  - 为上层提供统一的 USB CDC 发送/接收接口，屏蔽 USBD_BUSY/时序细节。
 *
 * 线程模型：
 *  - usbTxTask：唯一的 CDC IN 发送线程。所有发送请求先入队到 s_usb_tx_stream，
 *              再由该线程调用 CDC_Transmit_HS() 发出；发送完成依赖
 *              CDC_TransmitCplt_HS 回调通过 UsbTx_OnCdcTxCpltFromISR() 通知。
 *  - usbRxTask：CDC OUT 接收处理线程。CDC_Receive_HS() 在 USB 协议栈上下文把
 *              Buf/Len 投递到 s_usb_rx_stream（通过 UsbRx_OnCdcRxFromISR()），
 *              再由该线程取出并：
 *                回调 s_usb_rx_cb（当前注册 GetPCMsg）；未注册时取出的字节不再保留。
 *              UsbRx_Read 与此任务共用原始 RX 流，没有二级应用接收流。
 *
 * 注意：
 *  - CDC 是“字节流”，并不自带消息边界；上层需自行定义协议（帧头帧尾）。
 *  - ISR 上下文（*FromISR 函数）必须短小，不做格式化/阻塞操作。
 *  - UsbCdc_IsConfigured() 用于确保枚举完成后再收发，避免未配置时卡住/无回调。
 */

/* 宏：USB 发送任务栈大小；任务创建成功后启用。 */
#define USB_TX_TASK_STACK_SIZE        (512U * 4U)
/* 宏：USB 发送任务优先级；当前高于普通优先级。 */
#define USB_TX_TASK_PRIORITY          ((osPriority_t)osPriorityAboveNormal)

/* 宏：USB 发送流缓冲区容量；发送任务创建时分配使用。 */
#define USB_TX_STREAM_SIZE_BYTES      (2048U)
/* 宏：发送流缓冲唤醒门限；当前 1 字节即可触发。 */
#define USB_TX_TRIGGER_LEVEL          (1U)

/* 宏：USB 接收任务栈大小；任务创建成功后启用。 */
#define USB_RX_TASK_STACK_SIZE        (512U * 4U)
/* 宏：USB 接收任务优先级；当前高于普通优先级。 */
#define USB_RX_TASK_PRIORITY          ((osPriority_t)osPriorityAboveNormal)

/* 宏：USB 原始接收流缓冲区容量；当前接收链路启用时使用。 */
#define USB_RX_STREAM_SIZE_BYTES      (2048U)
/* 宏：原始接收流唤醒门限；当前 1 字节即可触发。 */
#define USB_RX_TRIGGER_LEVEL          (1U)

/* 宏：预留的应用层接收缓冲容量；当前未被有效代码引用。 */
#define USB_APP_RX_STREAM_SIZE_BYTES  (1024U)
/* 宏：预留的应用接收唤醒门限；当前未被有效代码引用。 */
#define USB_APP_RX_TRIGGER_LEVEL      (1U)

//接收帧帧头帧尾
/* 全局常量：历史 USB 接收帧头“SP”和 CRLF 帧尾；引用仅在注释代码中，当前未启用。 */
const char frame_header_read1 = 'S';
const char frame_header_read2 = 'P';
const char frame_tail_read1 = '\r';
const char frame_tail_read2 = '\n';

//发送帧帧头帧尾
/* 全局常量：历史 USB 发送帧头“QR”和 CRLF 帧尾；引用仅在注释代码中，当前未启用。 */
const char frame_header_write1 = 'Q';
const char frame_header_write2 = 'R';
const char frame_tail_write1 = '\r';
const char frame_tail_write2 = '\n';

//发送缓冲区和接收缓冲区
/* 全局传输结构：历史发送/接收缓冲；当前文件内没有直接读写，保留供兼容接口使用。 */
trans_t TX_Data;
trans_r RX_Data;

/*
 * s_usb_tx_stream      : 发送队列（任意线程 -> usbTxTask）
 * s_usb_rx_stream      : 原始接收队列（USB ISR/协议栈 -> usbRxTask）
 */

/* 静态句柄：USB 发送字节流队列；初始化成功后由发送接口和发送任务共同使用。 */
static StreamBufferHandle_t s_usb_tx_stream = NULL;
/* 静态句柄：USB 唯一发送任务；初始化成功后启用。 */
static osThreadId_t s_usb_tx_task = NULL;
/* 静态句柄：发送任务 FreeRTOS 句柄；用于 CDC 完成回调通知。 */
static TaskHandle_t s_usb_tx_task_handle = NULL;

/* 静态句柄：USB 原始接收字节流；初始化成功后由 ISR 与接收任务使用。 */
static StreamBufferHandle_t s_usb_rx_stream = NULL;
/* 静态句柄：USB 接收任务；初始化成功后启用。 */
static osThreadId_t s_usb_rx_task = NULL;
/* 静态回调：应用注册的接收处理函数；未注册时不执行事件回调分发。 */
static UsbRx_Callback_t s_usb_rx_cb = NULL;

/* 发送统计：用于判断是否有丢包/拥塞（调试用途） */
/* 静态统计：发送入队/丢弃字节及发送成功、忙碌、失败计数；由发送路径更新、统计接口读取。 */
/* s_usb_enqueued_bytes：累计写入 TX 流的字节数，由 UsbTx_Send 更新。 */
static volatile uint32_t s_usb_enqueued_bytes = 0;
/* s_usb_dropped_bytes：入队返回零时累计剩余未入队字节；单纯耗尽总超时的提前返回分支不累加。 */
static volatile uint32_t s_usb_dropped_bytes = 0;
/* s_usb_tx_ok_pkts：CDC_Transmit_HS 返回 USBD_OK 的批次数；表示成功提交，不代表已确认传输完成。 */
static volatile uint32_t s_usb_tx_ok_pkts = 0;
/* s_usb_tx_busy_cnt：CDC_Transmit_HS 返回 USBD_BUSY 的累计尝试次数。 */
static volatile uint32_t s_usb_tx_busy_cnt = 0;
/* s_usb_tx_fail_pkts：未成功提交给 CDC 的发送批次数。 */
static volatile uint32_t s_usb_tx_fail_pkts = 0;
/* s_usb_tx_fail_bytes：上述未成功提交批次的累计字节数。 */
static volatile uint32_t s_usb_tx_fail_bytes = 0;

/* 接收统计：记录进入 RX 缓冲的字节数与丢弃字节数（调试用途） */
/* 接收链路维护原始 RX 流计数；应用转发统计仅为兼容预留，当前不更新。 */
/* s_usb_rx_enqueued_bytes：CDC 接收入口成功写入原始 RX 流的累计字节数。 */
static volatile uint32_t s_usb_rx_enqueued_bytes = 0;
/* s_usb_rx_dropped_bytes：CDC 接收入口有有效流和输入时因容量不足丢弃的累计字节数。 */
static volatile uint32_t s_usb_rx_dropped_bytes = 0;
/* s_usb_app_rx_forwarded_bytes：历史应用接收流转发字节计数；当前无二级流，保持初值，仅统计接口读取。 */
static volatile uint32_t s_usb_app_rx_forwarded_bytes = 0;
/* s_usb_app_rx_dropped_bytes：历史应用接收流丢弃字节计数；当前无二级流，保持初值，仅统计接口读取。 */
static volatile uint32_t s_usb_app_rx_dropped_bytes = 0;

/* 中文名：USB队列发送任务；作用：argument 未使用，等待枚举完成后持续消费 TX 流，调用 CDC 发送并等待完成通知，忙时在限定时间内重试；更新提交和失败统计，常驻不返回。 */
static void UsbTxTask(void *argument);
/* 中文名：USB字节分发任务；作用：argument 未使用，等待枚举完成后持续读取原始 RX 流并将本批字节交给已注册回调；没有独立应用接收流，未注册时本批数据取出后不再保留，常驻不返回。 */
static void UsbRxTask(void *argument);

/**
 * @brief  初始化 USB 通信相关资源（任务、缓冲区等）
 * @note   记得调用
 */
/* 中文名：创建USB收发资源；作用：无参数，分别创建 TX/RX 字节流和唯一收发任务，已有资源不重复创建；创建失败保留空句柄，无返回值。 */
void User_Communication_Init(void)
{
    /* 创建 stream buffer（只创建一次）；失败则保持 NULL，上层可据此做降级处理 */
    if (s_usb_tx_stream == NULL)
    {
        s_usb_tx_stream = xStreamBufferCreate(USB_TX_STREAM_SIZE_BYTES, USB_TX_TRIGGER_LEVEL);
    }
    if (s_usb_rx_stream == NULL)
    {
        s_usb_rx_stream = xStreamBufferCreate(USB_RX_STREAM_SIZE_BYTES, USB_RX_TRIGGER_LEVEL);
    }

    if ((s_usb_tx_task == NULL) && (s_usb_tx_stream != NULL))
    {
        /* 启动 USB 发送线程（唯一发送者，避免并发 CDC_Transmit 导致 USBD_BUSY 混乱） */
        /* usbTxTask_attributes：发送任务名称、栈字节数及调度优先级，供创建任务使用。 */
        const osThreadAttr_t usbTxTask_attributes =
        {
            .name = "usbTxTask",
            .stack_size = USB_TX_TASK_STACK_SIZE,
            .priority = USB_TX_TASK_PRIORITY,
        }
        ;
        s_usb_tx_task = osThreadNew(UsbTxTask, NULL, &usbTxTask_attributes);
    }

    if ((s_usb_rx_task == NULL) && (s_usb_rx_stream != NULL))
    {
        /* 启动 USB 接收线程（把 ISR 中的字节流搬运到任务上下文处理） */
        /* usbRxTask_attributes：接收任务名称、栈字节数及调度优先级，供创建任务使用。 */
        const osThreadAttr_t usbRxTask_attributes =
        {
            .name = "usbRxTask",
            .stack_size = USB_RX_TASK_STACK_SIZE,
            .priority = USB_RX_TASK_PRIORITY,
        }
        ;
        s_usb_rx_task = osThreadNew(UsbRxTask, NULL, &usbRxTask_attributes);
    }
}

/**
 * @brief  判断 CDC 是否已配置完成
 * @note   dev_state == CONFIGURED 且 class data 就绪，才允许开始收发。
 */
/* 中文名：检查USB枚举完成；作用：无参数，读取 HS 设备状态和类数据指针，均就绪返回 true，否则 false；不改变链路状态。 */
bool UsbCdc_IsConfigured(void)
{
    return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) && (hUsbDeviceHS.pClassData != NULL);
}

/**
 * @brief  判断 USB CDC 发送是否准备好
 */
/* 中文名：查询USB链路就绪；作用：无参数，返回 UsbCdc_IsConfigured 的枚举就绪结果；不检查发送队列余量或 CDC 当前忙闲。 */
bool UsbTx_IsReady(void)
{
    return UsbCdc_IsConfigured();
}

/**
 * @brief  获取 USB 发送统计信息
 * @param  enqueued_bytes 已入队列的字节数
 * @param  dropped_bytes 丢弃的字节数
 * @param  tx_ok_pkts CDC 成功接受提交的批次数（尚未确认发送完成）
 * @param  tx_busy_cnt 发送忙碌计数
 */
/* 中文名：读取USB发送计数；作用：向 enqueued_bytes/dropped_bytes 输出累计入队/丢弃字节数，向 tx_ok_pkts/tx_busy_cnt 输出提交成功次数/忙状态次数；各空指针独立跳过，无返回值。 */
void UsbTx_GetStats(uint32_t *enqueued_bytes, uint32_t *dropped_bytes, uint32_t *tx_ok_pkts, uint32_t *tx_busy_cnt)
{
    if (enqueued_bytes != NULL)
    {
        *enqueued_bytes = s_usb_enqueued_bytes;
    }
    if (dropped_bytes != NULL)
    {
        *dropped_bytes = s_usb_dropped_bytes;
    }
    if (tx_ok_pkts != NULL)
    {
        *tx_ok_pkts = s_usb_tx_ok_pkts;
    }
    if (tx_busy_cnt != NULL)
    {
        *tx_busy_cnt = s_usb_tx_busy_cnt;
    }
}

/**
 * @brief 获取USB通信统计信息
 * @param out 指向UsbStats_t结构体的指针，用于存储获取到的统计信息
 * @note 如果传入的指针为NULL，则函数直接返回，不进行任何操作
 */
/* 中文名：读取USB收发统计；作用：将累计收发计数逐字段复制到 out 指向的 UsbStats_t，空指针直接返回；不清零计数，复制过程未加锁，无返回值。 */
void Usb_GetStats(UsbStats_t *out)
{
    // 检查输出指针是否有效
    if (out == NULL)
    {
        return;
    }



    // 填充发送相关的统计信息
    out->tx_enqueued_bytes = s_usb_enqueued_bytes;      // 已入队的发送字节数
    out->tx_dropped_bytes = s_usb_dropped_bytes;        // 已丢弃的发送字节数
    out->tx_ok_pkts = s_usb_tx_ok_pkts;                 // 成功发送的数据包数量
    out->tx_busy_cnt = s_usb_tx_busy_cnt;               // 发送忙碌计数
    out->tx_fail_pkts = s_usb_tx_fail_pkts;             // 发送失败的数据包数量
    out->tx_fail_bytes = s_usb_tx_fail_bytes;           // 发送失败的字节数


    // 填充接收相关的统计信息
    out->rx_isr_enqueued_bytes = s_usb_rx_enqueued_bytes;      // 中断服务例程中入队的接收字节数
    out->rx_isr_dropped_bytes = s_usb_rx_dropped_bytes;        // 中断服务例程中丢弃的接收字节数
    out->app_rx_forwarded_bytes = s_usb_app_rx_forwarded_bytes; // 应用层转发的接收字节数
    out->app_rx_dropped_bytes = s_usb_app_rx_dropped_bytes;     // 应用层丢弃的接收字节数
}

/**
 * @brief  CDC IN 传输完成回调触发（ISR上下文调用）
 */
/**
 * @brief USB发送完成中断服务函数
 * 当CDC USB发送完成时，此函数从中断服务程序(ISR)中被调用
 * 它会通知USB发送任务，可能触发任务切换
 */
/* 中文名：通知USB发送完成；作用：无参数，在 CDC 完成中断中通知已记录的发送任务继续工作，必要时请求任务切换；任务句柄为空时跳过，无返回值。 */
void UsbTx_OnCdcTxCpltFromISR(void)
{
    // 创建一个标志变量，用于判断是否需要进行任务切换
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 检查USB发送任务句柄是否有效
    if (s_usb_tx_task_handle != NULL)
    {
        // 从中断服务程序中向任务发送通知
        // 这会使任务从阻塞状态变为就绪状态
        vTaskNotifyGiveFromISR(s_usb_tx_task_handle, &xHigherPriorityTaskWoken);
        // 如果需要，进行任务切换
        // 只有当xHigherPriorityTaskWoken为pdTRUE时才会切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  异步投递数据到USB发送任务
 * @retval 0:成功 负数:失败
 */
/* 中文名：将字节入队等待USB发送；作用：复制 data 的 len 字节到 TX 流，timeout_ms 为总等待毫秒数（0 不等待，0xFFFFFFFF 无限等待）。
 * 全部入队返回 0，空数据/零长度/流未创建返回 -1，超时或无法继续入队返回 -2；失败前已入队部分仍保留，更新入队及相应丢弃计数。 */
int UsbTx_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    /*
     * 将数据投递到发送队列，由 usbTxTask 统一发送。
     * timeout_ms == 0：每次入队不阻塞；若有剩余数据则继续尝试，某次无法入队立即返回。
     */
    if ((s_usb_tx_stream == NULL) || (data == NULL) || (len == 0U))
    {
        return -1;
    }

    /* start_tick：本次整段入队的起始 Tick，用于累计总等待时间。 */
    const TickType_t start_tick = xTaskGetTickCount();
    /* timeout_ticks：总超时换算后的 Tick 数，portMAX_DELAY 表示无限等待。 */
    const TickType_t timeout_ticks = ms_to_ticks(timeout_ms);
    /* offset：已复制入 TX 流的字节数，也是下次待写数据的偏移。 */
    uint32_t offset = 0U;

    while (offset < (uint32_t)len)
    {
        /* remain：本次写 TX 流允许等待的剩余 Tick 数。 */
        TickType_t remain;
        if (timeout_ticks == 0U)
        {
            /* 不阻塞：本次入队等待为零，无法写入时返回失败。 */
            remain = 0U;
        }
        else if (timeout_ticks == portMAX_DELAY)
        {
            remain = portMAX_DELAY;
        }
        else
        {
            /* elapsed：从本次入队开始已消耗的 Tick 数。 */
            const TickType_t elapsed = xTaskGetTickCount() - start_tick;
            if (elapsed >= timeout_ticks)
            {
                return -2;
            }
            remain = timeout_ticks - elapsed;
        }

        /* sent：本次成功复制到 TX 流的字节数，用于累计偏移和入队统计。 */
        const size_t sent = xStreamBufferSend(
            s_usb_tx_stream,
            (const void *)&data[offset],
            (size_t)((uint32_t)len - offset),
            remain);

        if (sent == 0U)
        {
            s_usb_dropped_bytes += (uint32_t)len - offset;
            return -2;
        }
        s_usb_enqueued_bytes += (uint32_t)sent;
        offset += (uint32_t)sent;
    }

    return 0;
}

/**
 * @brief  发送字符串（不自动追加\r\n）
 */
/* 中文名：将字符串加入USB发送流；作用：发送 str 的 strlen 字节，不含终止零且不追加换行；timeout_ms 为入队等待毫秒数，空指针返回 -1，其余返回 UsbTx_Send 的入队结果。 */
int UsbTx_SendString(const char *str, uint32_t timeout_ms)
{
    if (str == NULL)
    {
        return -1;
    }
    return UsbTx_Send((const uint8_t *)str, (uint16_t)strlen(str), timeout_ms);
}

/* 中文名：格式化USB调试文本；作用：按 fmt 及可变参数格式化文本，最多发送 255 字节并以零超时入队；返回入队结果，格式空或格式化失败返回 -1，未枚举返回 -3；不自动添加换行。 */
int DEBUG_USB(const char *fmt, ...)
{
    /* printf 风格调试输出：格式化到栈上缓冲并异步投递（默认不阻塞） */
    if (fmt == NULL)
    {
        return -1;
    }
    if (!UsbCdc_IsConfigured())
    {
        return -3;
    }

    /* out：格式化文本的栈缓存，含终止零在内 256 字节；入队时复制其内容。 */
    char out[256];
    /* args：fmt 对应的可变参数遍历状态，格式化后以 va_end 结束。 */
    va_list args;
    va_start(args, fmt);
    /* n：格式化期望字符数；超出缓存时改为可发送的 255 字节长度。 */
    int n = vsnprintf(out, sizeof(out), fmt, args);
    va_end(args);

    if (n < 0)
    {
        return -1;
    }

    if ((size_t)n >= sizeof(out))
    {
        /* 截断发送（vsnprintf 已确保以 '\0' 结尾） */
        n = (int)sizeof(out) - 1;
    }

    /* 0ms：不阻塞，避免影响实时任务；满了就丢 */
    return UsbTx_Send((const uint8_t *)out, (uint16_t)n, 0U);
}
/**
 * @brief  注册接收回调
 */
/* 中文名：注册USB字节接收回调；作用：将 cb 保存为接收任务的分发入口，NULL 取消回调；当前初始化注册 GetPCMsg，实际调用发生在 UsbRxTask 上下文，无返回值。 */
void UsbRx_RegisterCallback(UsbRx_Callback_t cb)
{
    /* 回调在 usbRxTask 上下文触发；可用于事件式处理（解析命令/统计等） */
    s_usb_rx_cb = cb;
}

/**
 * @brief  USB 接收回调
 * @param data 接收数据
 * @param len 数据长度
 */
/* 中文名：投递USB中断接收字节；作用：从 data 复制 len 字节到原始 RX 流，累计实际入队和未入队字节，必要时请求 ISR 任务切换；空数据、零长度或未建流直接返回，无返回值。 */
void UsbRx_OnCdcRxFromISR(const uint8_t *data, uint32_t len)
{
    /*
     * 在 CDC_Receive_HS() 中调用（USB 协议栈上下文/近似 ISR 约束）：
     * 只做“快速入队 + 统计”，不要做耗时处理。
     */
    if ((s_usb_rx_stream == NULL) || (data == NULL) || (len == 0U))
    {
        return;
    }

    /* xHigherPriorityTaskWoken：本次入流是否唤醒更高优先级任务，供 ISR 尾部调度请求使用。 */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* written：本次从 CDC 数据复制到原始 RX 流的实际字节数。 */
    const size_t written = xStreamBufferSendFromISR(s_usb_rx_stream, data, (size_t)len, &xHigherPriorityTaskWoken);
    s_usb_rx_enqueued_bytes += (uint32_t)written;
    if (written < (size_t)len)
    {
        s_usb_rx_dropped_bytes += (uint32_t)((size_t)len - written);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  供应用层读取 USB 接收数据
 * @param  out 输出缓冲区
 * @param  out_len 输出缓冲区长度
 * @param  timeout_ms 超时时间，单位毫秒
 * @retval uint16_t 接收字节数
 */
/* 中文名：读取USB原始接收流；作用：最多向 out 写入 out_len 字节，以 timeout_ms 毫秒等待（0 不等待，0xFFFFFFFF 无限等待），返回实际读取字节数；参数无效或流未创建返回 0。
 * 本接口与 UsbRxTask 消费同一个 RX 流，读取会取走尚未分发的数据。 */
uint16_t UsbRx_Read(uint8_t *out, uint16_t out_len, uint32_t timeout_ms)
{
    if ((s_usb_rx_stream == NULL) || (out == NULL) || (out_len == 0U))
    {
        return 0;
    }

    /* t：本次原始 RX 流读取允许阻塞的 Tick 数。 */
    const TickType_t t = ms_to_ticks(timeout_ms);
    /* n：本次实际写入调用方 out 缓存的字节数。 */
    const size_t n = xStreamBufferReceive(s_usb_rx_stream, out, (size_t)out_len, t);
    return (uint16_t)n;
}

/**
 * @brief  获取 USB 原始接收流中可用的数据字节数
 * @retval size_t 可用字节数
 */
/* 中文名：查询USB原始接收积压；作用：无参数，返回 RX 流当前可读字节数，流未创建返回 0；不消费数据。 */
size_t UsbRx_Available(void)
{
    if (s_usb_rx_stream == NULL)
    {
        return 0;
    }
    return xStreamBufferBytesAvailable(s_usb_rx_stream);
}

/**
 * @brief  获取 USB 接收统计数据
 * @param rx_enqueued_bytes 已入队字节数
 * @param rx_dropped_bytes 丢弃字节数
 */
/* 中文名：读取USB中断接收计数；作用：向 rx_enqueued_bytes/rx_dropped_bytes 分别输出累计成功入队/丢弃字节数，各指针可独立为空；不清零计数，无返回值。 */
void UsbRx_GetStats(uint32_t *rx_enqueued_bytes, uint32_t *rx_dropped_bytes)
{
    if (rx_enqueued_bytes != NULL)
    {
        *rx_enqueued_bytes = s_usb_rx_enqueued_bytes;
    }
    if (rx_dropped_bytes != NULL)
    {
        *rx_dropped_bytes = s_usb_rx_dropped_bytes;
    }
}

/* 中文名：接收并转交小电脑消息。
 * 作用：USB接收任务（UsbRxTask）调用本回调，把data中的len个字节交给合并控制层拼帧；
 * 更新合并层拼帧状态和完整命令快照，无返回值；不直接控制电机，原322字节机构布局不再参与解析。
 */
void GetPCMsg(const uint8_t *data, uint16_t len)
{
    MergedControl_OnUsbBytes(data, len);
}

/**
 * @brief 发送消息到 PC
 */
// void SendPCMsg(void)
// {
// 	uint8_t data[USB_TX_LEN] = {};
// 	data[0] = frame_header_write1;//帧头2位
// 	data[1] = frame_header_write2;
// 	data[USB_TX_LEN-2] = frame_tail_write1;//帧尾2位
// 	data[USB_TX_LEN-1] = frame_tail_write2;

// 	//心跳包
// 	data[2] = HB.PC;
// 	data[3] = HB.leg1;
// 	data[4] = HB.leg2;
// 	data[5] = HB.leg3;
// 	data[6] = HB.leg4;
// 	data[7] = HB.wheel1;
// 	data[8] = HB.wheel2;
// 	data[9] = HB.wheel3;
// 	data[10]= HB.wheel4;
// 	data[11]= HB.shoulder;
// 	data[12]= HB.elbow;
// 	data[13]= HB.wrist;
// 	data[14] = (ps_num[7] & 0x01) << 0 | (ps_num[11] & 0x01) << 1 |
// 							(ps_num[9] & 0x01) << 2 | (ps_num[3] & 0x01) << 3 |
// 							(ps_num[4] & 0x01) << 4 | (ps_num[8] & 0x01) << 5 |
// 							(ps_num[12] & 0x01) << 6 | (ps_num[13] & 0x01) << 7;
// 	data[15] = (ps_num[2] & 0x01) << 0 | (ps_num[1] & 0x01) << 1 | //  行程开关车尾
// 							(ps_num[0] & 0x01) << 2 | (ps_num[14] & 0x01) << 3 |//  左侧 右侧
// 							(ps_num[10] & 0x01) << 4 | (ps_num[5] & 0x01) << 5 |// 行程开关车头（右左）
// 							(ps_num[6] & 0x01) << 6;// 爪子
// 	data[16] = (ps_num_2[5] & 0x01) << 0 | (ps_num_2[13] & 0x01) << 1 | // 看R1(右左)
// 							(ps_num_2[0] & 0x01) << 2 | (ps_num_2[4] & 0x01) << 3; //侧边行程开关（前后）
// 	data[17] = gRobot.reset_flag;

// 	data[18] = leg1Msg.error;
// 	data[19] = leg2Msg.error;
// 	data[20] = leg3Msg.error;
// 	data[21] = leg4Msg.error;
// 	data[22] = wheel1Msg.error;
// 	data[23] = wheel2Msg.error;
// 	data[24] = wheel3Msg.error;
// 	data[25] = wheel4Msg.error;
// 	data[26] = shoulderMsg.error;

// 	data[27] = leg1Msg.enableState;
// 	data[28] = leg2Msg.enableState;
// 	data[29] = leg3Msg.enableState;
// 	data[30] = leg4Msg.enableState;

// 	TX_Data.data_float[1] = leg1Msg.pos;
// 	TX_Data.data_float[2] = leg1Msg.vel;
// 	TX_Data.data_float[3] = leg1Msg.currQ;

// 	TX_Data.data_float[4] = leg2Msg.pos;
// 	TX_Data.data_float[5] = leg2Msg.vel;
// 	TX_Data.data_float[6] = leg2Msg.currQ;

// 	TX_Data.data_float[7] = leg3Msg.pos;
// 	TX_Data.data_float[8] = leg3Msg.vel;
// 	TX_Data.data_float[9] = leg3Msg.currQ;

// 	TX_Data.data_float[10] = leg4Msg.pos;
// 	TX_Data.data_float[11] = leg4Msg.vel;
// 	TX_Data.data_float[12] = leg4Msg.currQ;

// 	TX_Data.data_float[13] = wheel1Msg.pos;
// 	TX_Data.data_float[14] = wheel1Msg.vel;
// 	TX_Data.data_float[15] = wheel1Msg.currQ;

// 	TX_Data.data_float[16] = wheel2Msg.pos;
// 	TX_Data.data_float[17] = wheel2Msg.vel;
// 	TX_Data.data_float[18] = wheel2Msg.currQ;

// 	TX_Data.data_float[19] = wheel3Msg.pos;
// 	TX_Data.data_float[20] = wheel3Msg.vel;
// 	TX_Data.data_float[21] = wheel3Msg.currQ;

// 	TX_Data.data_float[22] = wheel4Msg.pos;
// 	TX_Data.data_float[23] = wheel4Msg.vel;
// 	TX_Data.data_float[24] = wheel4Msg.currQ;

// 	TX_Data.data_float[25] = shoulderMsg.pos;
// 	TX_Data.data_float[26] = shoulderMsg.vel;
// 	TX_Data.data_float[27] = shoulderMsg.currQ;

// 	TX_Data.data_float[28] = elbowMsg.pos / 2.f / PI;
// 	TX_Data.data_float[29] = elbowMsg.vel / 2.f / PI;

// 	TX_Data.data_float[30] = wristMsg.pos / 2.f / PI;
// 	TX_Data.data_float[31] = wristMsg.vel / 2.f / PI;

// 	//联合体发送
// 	for(uint16_t i = (uint16_t)(2 + USB_TX_UINT8_NUM); i < (uint16_t)(USB_TX_LEN - 2); i++)
// 	{
// 		data[i] = (uint8_t)TX_Data.datac[i - (uint16_t)(2 + USB_TX_UINT8_NUM)];
// 	}

// 	UsbTx_Send(data, USB_TX_LEN, 0U);
// }

/**
 * @brief  USB 发送任务
 * @param argument 任务参数
 * @note   一直运行
 */
/* 中文名：USB队列发送任务；作用：argument 未使用，等待枚举完成后持续消费 TX 流，调用 CDC 发送并等待完成通知，忙时在限定时间内重试；更新提交和失败统计，常驻不返回。 */
static void UsbTxTask(void *argument)
{
    (void)argument;

    /* 记录句柄用于 ISR 通知 */
    s_usb_tx_task_handle = xTaskGetCurrentTaskHandle();

    /* 等待USB枚举到 CONFIGURED 状态，避免过早发送导致异常/无回调 */
    osDelay(200);
    while (!UsbCdc_IsConfigured())
    {
        osDelay(10);
    }

    /* buf：发送任务持有的本批 CDC 数据缓存，取出 TX 流内容后供 USB 提交和重试。 */
    uint8_t buf[256];

    for (;;)
    {
        if (!UsbCdc_IsConfigured())
        {
            osDelay(10);
            continue;
        }

        /* n：从 TX 流取出的本批待发字节数，最多为 buf 容量。 */
        const size_t n = xStreamBufferReceive(s_usb_tx_stream, buf, sizeof(buf), portMAX_DELAY);
        if (n == 0U)
        {
            continue;
        }

        /* 提交一次 IN 传输，并等待发送完成回调通知；避免 1ms 轮询 */
        /* start：当前发送批次首次提交前的 Tick，限制忙状态重试总时长。 */
        const TickType_t start = xTaskGetTickCount();
        /* sent_ok：本批是否已获 CDC 成功提交返回值，用于失败统计；不是完成回调标志。 */
        bool sent_ok = false;
        for (;;)
        {
            /* ret：本次 CDC 提交结果，区分成功、忙和其他错误。 */
            const uint8_t ret = CDC_Transmit_HS(buf, (uint16_t)n);
            if (ret == USBD_OK)
            {
                s_usb_tx_ok_pkts++;
                sent_ok = true;
                /* 最多等待完成通知 100ms；当前代码忽略等待结果，超时后也会结束本批处理。 */
                (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
                break;
            }
            if (ret != USBD_BUSY)
            {
                break;
            }
            s_usb_tx_busy_cnt++;

            /* Busy：等待上一包完成通知，再重试 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(200))
            {
                break;
            }
        }
        if (!sent_ok)
        {
            s_usb_tx_fail_pkts++;
            s_usb_tx_fail_bytes += (uint32_t)n;
        }
    }
}

/**
 * @brief  USB 接收任务
 * @param argument 任务参数
 * @note   一直运行
 */
/* 中文名：USB字节分发任务；作用：argument 未使用，等待枚举完成后持续读取原始 RX 流并将本批字节交给已注册回调；没有独立应用接收流，未注册时本批数据取出后不再保留，常驻不返回。 */
static void UsbRxTask(void *argument)
{
    (void)argument;

    /* 等待USB枚举完成再处理接收（避免没连上时空转） */
    osDelay(200);
    while (!UsbTx_IsReady())
    {
        osDelay(10);
    }

    /* buf：接收任务的单批临时缓存，供同步回调读取；回调不应保存其指针。 */
    uint8_t buf[256];
    for (;;)
    {
        if (!UsbTx_IsReady())
        {
            osDelay(10);
            continue;
        }

        /* n：从原始 RX 流取出的本批字节数，不代表一个完整协议帧。 */
        const size_t n = xStreamBufferReceive(s_usb_rx_stream, buf, sizeof(buf), portMAX_DELAY);
        if (n == 0U)
        {
            continue;
        }

        /* 回调，在任务上下文触发 */
        /* cb：本批分发前读取的注册回调指针；为空时跳过分发。 */
        UsbRx_Callback_t cb = s_usb_rx_cb;
        if (cb != NULL)
        {
            cb(buf, (uint16_t)n);
        }
    }
}