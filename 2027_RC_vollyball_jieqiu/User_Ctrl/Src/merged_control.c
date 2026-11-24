/* 文件作用：实现八台物理电机的USB命令适配、CAN反馈隔离、模式与使能确认，以及显式复位事务。
 * 下行261字节SP帧只更新完整命令；5ms任务执行最新目标，上行133字节QR帧回传反馈。
 * 轴顺序为ARM1、ARM2、盘、丝杆、四腿；仅使用CAN3，不生成运动轨迹、不自动启动复位。
 * ARM2准备位仅表示ARM1已复位且ARM2置零命令入队，不代表ARM2机械到位或驱动确认。
 * ps_num数组长度沿用用户最新PS_RX_LEN配置，byte13只读ps_num[3]原始值。
 *
 * ==================== 命名约定速查 ====================
 * 本文件混有三套来源，先看前缀就能判断归属：
 * 1. s_xxx     文件内静态变量（static），只在本.c可见，跨调用保留状态。
 *              例：s_latest最新命令快照、s_runtime各轴软件确认态、s_reset_stage复位阶段。
 *              注意s_driver_id是static const查表，名字同样带s_。
 * 2. MCU_XXX   本文件新增的具名常量和工具宏，定义在本文件顶部，不对外导出。
 *              例：MCU_MODE_*控制模式、MCU_MASK_*复位轴组合、MCU_RESET_STAGE_*复位阶段。
 * 3. VB_XXX    上下位机共享协议名，定义在protocol/usb_protocol.h，主控与小电脑必须一致。
 *              例：VB_AXIS_COUNT轴数、VB_RUN_BYTE运行字节、VB_READY_BYTE就绪位、VB_ARM1轴槽。
 * 函数：Merged开头是本模块函数；大驼峰是静态辅助函数，全大写是宏。
 * 类型：MergedCommand、MergedFeedback等typedef结构体，定义在文件中部。
 * 变量：内部小写蛇形；axis轴下标，mask位掩码，turns圈，turns_per_second圈每秒，rad弧度。
 * VB_ARM1至VB_LEG4是协议轴槽序号0至7，同时作为s_driver_id下标，不是CAN ID本身。
 * 旧驱动名IDENTIFIER_*、PTP_MODE、ENABLE、DISABLE来自User_Driver，不在本文件定义。
 */

/* ==================== 接手导读（白话版，建议按顺序读） ====================
 * 这个文件负责“把八台电机的控制权，稳稳地交给小电脑（PC）”。
 *
 * 1) 数据怎么流动
 *    小电脑通过USB下发“SP”开头的261字节命令帧 ——> 本文件把它攒齐、解出来、存成一份“最新命令”
 *    ——> 每5毫秒执行一次：按最新命令去决定每台电机该使能/失能、切什么模式、发什么目标
 *    ——> 电机驱动器通过CAN3回报真实状态 ——> 本文件整理成“QR”开头的133字节反馈帧回传给小电脑。
 *
 * 2) 每条帧里有什么
 *    命令帧：每轴7个浮点数（位置=圈，速度=圈/秒，电流=A，位置kp、位置kd、速度kp、速度ki），
 *            外加“是否运行”、“复位请求”、复位事务号。
 *    反馈帧：每轴的错误码、是否使能、复位是否就绪、真实位置/速度/电流，外加心跳和光电传感器值。
 *
 * 3) 八台电机是谁（数组下标就是“轴槽序号”，也是复位掩码的位序号）
 *    0=ARM1 1=ARM2 2=盘(拍板) 3=丝杆 4~7=四条腿
 *    这几台电机在CAN上都挂在3号CAN总线上，驱动器编号是1~8（见 s_driver_id 查表）。
 *
 * 4) 两条互斥的主线
 *    (a) 正常运动：按PC给的“模式+目标”让电机使能并跟随目标（MergedExecuteAxis）。
 *    (b) 显式复位：PC明确要求复位时，先把要复位的电机断电（失能），再一台一台复位，
 *        全部确认完成才回一个回执（MergedRunResetTransaction）。复位期间不做正常运动。
 *
 * 5) 三个最容易踩坑的约定（先记住这三条，看代码会顺很多）
 *    - 安全问题：任何一条命令，都要“先真正确认电机已失能”，才允许切换模式；模式真正确认了，才允许发目标、才允许使能。
 *    - 防重复：小电脑可能重发同一帧，所以用“复位事务号”和“回报序号”判断“这是不是新的”，旧的不算数。
 *    - ARM2特殊：双臂复位时，只对ARM1下发真正的复位命令，ARM2只是“把当前位置记成零点”（置零）。
 */

#include "merged_control.h"
#include "motor_driver.h"
#include "user_communication.h"
#include "user_usart.h"
/* 光电输入保留现有数组；PS byte13按原始ps_num[3]透传。 */
extern uint8_t ps_num[PS_RX_LEN - 4]; /* 光电串口收到的原始字节数组；“3号槽”那一个字节会原样转发给小电脑。 */
/* 相对本文件包含上下位机共用的字段定义与字节转换函数。 */
#include "../../../protocol/usb_codec.h" /* 32位整数和浮点数的“小端”拼装/拆解函数（VbReadU32等），保证主控和小电脑对同一串字节的理解完全一致。 */
#include <math.h> /* 提供 isfinite（判断浮点数是不是正常数字）和 π 常量 PI。 */

/* ==================== 协议与驱动数值的具名定义 ==================== */
#ifndef VB_PS3_BYTE
#define VB_PS3_BYTE 13U /* 小电脑那边如果没定义，就默认：反馈帧第13个字节放光电值。 */
#endif
#ifndef VB_READY_BYTE
#define VB_READY_BYTE 14U /* 小电脑那边如果没定义，就默认：反馈帧第14个字节放“哪些轴已就绪”的位图。 */
#endif

/* 帧头帧尾与帧内字节单位。 */
#define MCU_FRAME_HEADER0           'S'     /* 下行命令帧头第0字节。 */
#define MCU_FRAME_HEADER1           'P'     /* 下行命令帧头第1字节。 */
#define MCU_FRAME_TAIL0             '\r'    /* 两种帧共用的倒数字节1。 */
#define MCU_FRAME_TAIL1             '\n'    /* 两种帧共用的最后字节。 */
#define MCU_FEEDBACK_HEADER0        'Q'     /* 上行反馈帧头第0字节。 */
#define MCU_FEEDBACK_HEADER1        'R'     /* 上行反馈帧头第1字节。 */
#define MCU_FRAME_TAIL_BYTES        2U      /* 帧尾CRLF占2字节。 */

/* 每轴控制模式位：每字节压缩四个轴、每轴2位。 */
/* 白话：命令帧里“每轴用什么模式”不是一轴一个字节，而是把8个轴塞进2个字节，
 * 每个轴只占2位（能表示0~3：禁用/速度/位置/PVT）。所以读的时候要“移位+掩码”取出来。 */
#define MCU_MODE_BITS_PER_AXIS      2U
#define MCU_MODE_AXES_PER_BYTE      4U
#define MCU_MODE_AXIS_MASK          0x03U

/* 下行运行请求和计数限制。 */
#define MCU_RUN_REQUEST_MASK        0x01U  /* 运行字节里只有最低1位是有效的“运行开关”。 */
#define MCU_COUNTER_MASK            0x7FFFFFFFU    /* 计数保持31位正数，便于PC按int32接收。 */

/* USB控制模式值，与ROS消息controlmode字段一致。 */
/* 白话：这四个数字就是“这台电机现在想被怎么控制”，数字来自小电脑。 */
#define MCU_MODE_DISABLE            0U  /* 0：禁用，不给电机出力，保持断电（安全档）。 */
#define MCU_MODE_VELOCITY           1U  /* 1：速度模式，告诉电机“每秒转多少圈”。 */
#define MCU_MODE_POSITION           2U  /* 2：位置模式，告诉电机“转到第多少圈”。 */
#define MCU_MODE_PVT                3U  /* 3：PVT模式（位置+速度+力矩），目前只开放给腿，且默认关闭。 */
#define MCU_MODE_UNKNOWN            0xFFU          /* 复位后尚未重新配置的内部哨兵。 */

/* 使能状态值，来自SetDriverState回报；MCU_MODE_UNKNOWN同样表示尚未确认。 */
#define MCU_STATE_DISABLE           0U  /* 电机驱动器上报：我现在是断电（失能）状态。 */
#define MCU_STATE_ENABLE            1U  /* 电机驱动器上报：我现在是通电（使能）状态。 */

/* 复位掩码：每个物理轴一位，组合用VB_ARM1..VB_LEG4真实槽位拼出。 */
/* 白话：复位掩码就是一个8位字节，第几位是1就代表“这次要复位那台电机”。 */
#define MCU_AXIS_BIT(axis)          ((uint8_t)(1U << (axis))) /* 把“第axis号电机”变成一个只有那一位是1的字节。 */
#define MCU_MASK_ARMS               (uint8_t)(MCU_AXIS_BIT(VB_ARM1) | MCU_AXIS_BIT(VB_ARM2)) /* 只选两台机械臂。 */
#define MCU_MASK_ARM_PLATE          (uint8_t)(MCU_MASK_ARMS | MCU_AXIS_BIT(VB_PLATE)) /* 两台臂加盘。 */
#define MCU_MASK_LEGS               (uint8_t)(MCU_AXIS_BIT(VB_LEG1) | MCU_AXIS_BIT(VB_LEG2) | \
                                             MCU_AXIS_BIT(VB_LEG3) | MCU_AXIS_BIT(VB_LEG4)) /* 四条腿全选。 */
#define MCU_MASK_BODY               (uint8_t)(MCU_AXIS_BIT(VB_PLATE) | MCU_AXIS_BIT(VB_SLIDE)) /* 盘加丝杆（丝杆就是滑台）。 */
#define MCU_MASK_WHEELS             (uint8_t)(MCU_AXIS_BIT(VB_WHEEL1) | MCU_AXIS_BIT(VB_WHEEL2) | \
                                             MCU_AXIS_BIT(VB_WHEEL3) | MCU_AXIS_BIT(VB_WHEEL4)) /* 四个舵轮全选。 */
#define MCU_MASK_ALL                (uint8_t)(MCU_MASK_ARMS | MCU_MASK_BODY | MCU_MASK_LEGS | MCU_MASK_WHEELS) /* 十二台全都选。 */

/* 复位事务阶段。 */
/* 白话：复位不是“发一条命令就完事”，而是一个分步走的小流程，用 s_reset_stage 记住走到哪一步了。 */
#define MCU_RESET_STAGE_DISABLE     0U      /* 确认所选轴已真实失能。（第1步：先断电，确认电源真的断了） */
#define MCU_RESET_STAGE_FIRE        1U      /* 对当前轴发送复位、置零或跳过。（第2步：把“复位”命令发下去） */
#define MCU_RESET_STAGE_WAIT_ACK    2U      /* 等待新序号的真实复位完成回报。（第3步：等驱动器回答“我复位好了”） */
#define MCU_RESET_STAGE_ARM2_ZERO   3U      /* ARM1完成后给ARM2置零。（特殊步：双臂场景下给ARM2置零） */

/* 驱动与反馈数值。增益由小电脑直接给驱动乘之前的原值，本层不再做逆缩放。 */
#define MCU_PULSES_PER_REVOLUTION   4096.0f      /* 驱动位置/速度整数单位，脉冲/圈或脉冲每秒。 */
/* 白话：电机驱动器只认整数“脉冲”，不认“圈”。一圈 = 4096个脉冲，所以圈数要乘4096。 */
#define MCU_PULSE_LIMIT             1024.0f      /* 送入驱动前的圈数/圈每秒上限，避免int32溢出。 */
/* 白话：4096 × 1024 已经是int32比较安全的范围，再大就可能溢出变成反向的大数，很危险，所以先夹住。 */
#define MCU_LEG_POSITION_LIMIT      1.05f        /* 四腿位置安全范围，圈。 */
#define MCU_LEG_VELOCITY_LIMIT      1.05f        /* 四腿PVT实验速度临时范围，圈每秒。 */
#define MCU_RAD_PER_REVOLUTION      (2.0f * (float)PI) /* PVT模式以弧度输入，圈转弧度。 */
#define MCU_GAIN_UNSET              (-1.0f)      /* 驱动哨兵：表示本次不修改该增益。 */
/* 白话：“哨兵值”= 用一个特殊数表达特殊含义。这里-1不等于“增益是负1”，而是告诉驱动库“这次别动这个参数，保持原来的”。 */
#define MCU_ENABLE_BIT              1U           /* 驱动使能状态回传字节中的使能位。 */
#define MCU_SLOT_BYTES              4U           /* 每个32位数据槽占4字节。 */

/* CAN3驱动回报起始标准ID与每轴8字节载荷。 */
/* 白话：每台电机上报状态用的是 CAN ID = 0x280 + 驱动器编号，也就是 0x281(1号) 到 0x288(8号)。
 * 一条CAN-FD报文最长64字节，里面按8字节一小段（子帧）打包，每小段对应一条回报。 */
#define MCU_DRIVER_REPORT_FIRST_ID  (0x280U + 1U) /* 第一台电机（1号驱动器=ARM1）的回报ID：0x281。 */
#define MCU_DRIVER_REPORT_LAST_ID   (0x280U + VB_AXIS_COUNT) /* 最后一台电机（8号驱动器=第四腿）的回报ID：0x288。 */
#define MCU_CAN_PAYLOAD_BYTES       64U  /* 一帧CAN-FD最多带64字节数据。 */
#define MCU_CAN_SUBFRAME_BYTES      8U   /* 每8字节是一段独立的小报文，里面第0字节是“这是哪种回报”。 */

/* 光电字节在ps_num中的槽位。 */
#define MCU_PS3_SLOT                3U /* 我们只关心光电串口数组里的第3个字节，原样转发给小电脑。 */

/* 非参与轴失能确认在等待态最多保留的Tick数；超过后交回复位轮询。
 * 当前复位流程对非参与轴只发一次失能且不阻塞事务，这个上限保留给后续需要限时确认的扩展。 */
#define MCU_NONSELECTED_DISABLE_TIMEOUT 2U /* 备用常量：本文件目前没有使用它，它的作用是给以后“非参与轴失能也要限时确认”的扩展留位置。 */

/* 长度、槽位和阶段值与共享协议/驱动定义的一致性必须在编译期成立。 */
/* 白话：这三行是“编译期自检”。万一以后有人改了共享协议里的轴数或帧长，编译会直接报错，
 * 而不是等到上机器人之后才发现字节错位。 */
_Static_assert(VB_AXIS_COUNT == 12U, "merged control requires eight mechanism axes plus four wheels");
_Static_assert(VB_COMMAND_BYTES == 373U, "PC command frame length mismatch");
_Static_assert(VB_FEEDBACK_BYTES == 181U, "PC feedback frame length mismatch");
/* 与PC每轴七槽严格同序；位置=圈，速度=圈/秒，电流=A。 */
/* 白话：这是“每轴7个数的排列顺序表”。用名字代替数字下标，读代码时不容易数错位置。
 * 例如 value[CMD_VEL] 就是这台电机的“速度”那一格。 */
enum
{
    CMD_POS, CMD_VEL, CMD_CUR, CMD_POS_KP, CMD_POS_KD, CMD_VEL_KP, CMD_VEL_KI /* 依次是圈、圈/秒、A、位置kp、位置kd、速度kp、速度ki的数组索引。 */
}
;
/* 白话：一个“轴命令”= 这台电机这一帧要的7个数 + 用什么模式 + 要不要清积分。 */
typedef struct
{
    float value[VB_COMMAND_FIELDS]; /* 七槽依次是圈、圈/秒、A、位置kp/kd、速度kp/ki，增益按物理轴适配。 */
    uint8_t mode; /* USB模式，取值MCU_MODE_*：禁用、速度、位置或PVT。 */
    uint8_t clean; /* 本轴清积分请求位；位置与PVT按上升沿执行，速度模式沿用原驱动clean输入。 */
    /* 白话：“清积分”把速度环里累积的误差清零，通常在模式切换或者要它精确停住时用。
     * “上升沿”= 从0变成1的那一瞬间才算一次请求，一直保持1不会反复清。 */
}
MergedAxisCommand;
/* 白话：一整帧PC命令 = 8台电机各自的命令 + 运行开关 + 复位选择 + 复位事务号。 */
typedef struct
{
    MergedAxisCommand axis[VB_AXIS_COUNT]; /* 十二个轴槽的完整目标：0..7机构电机，8..11舵轮。 */
    uint32_t pc_counter; /* 原始整数计数槽，限定31位，不经float转换。 */
    /* 白话：小电脑自己的循环计数，只做“它还活着”的心跳判断用；本文件只是原样收下，目前不参与决策。 */
    uint8_t run; /* PC运行请求bit0；为0时禁止普通目标执行，但允许显式复位事务。 */
    uint8_t reset_mask; /* 机构电机复位选择位，每位对应一台机构轴；须通过机构互斥校验。 */
    uint8_t wheel_reset_mask; /* 舵轮复位选择位，每位对应一个舵轮，可任意子集。 */
    uint32_t reset_id; /* PC复位事务号；0不发起，最近完成的同号请求不重复执行，机构轴和舵轮共用。 */
    /* 白话：“事务号”就是这次复位请求的身份证。小电脑重发同一帧时号码不变，我们就知道“这事干过了，别再来一遍”。 */
}
MergedCommand;

/* 白话：驱动器实时回报的“位置、速度、电流”三件套（单位：圈、圈/秒、A）。 */
typedef struct
{
    float pos, vel, current; /* 仅来自0x52，单位分别为圈、圈/秒、A。 */
}
MergedPvt;
/* 白话：一台电机在本模块眼里的完整“真实现状”：
 *   驱动器回报的各字段（位置速度电流、是否使能、什么模式、复位好了没、错误码……）
 *   再加上四个“序号”，用来区分“这条回报是新来的，还是老消息”。 */
typedef struct
{
    driverMsg_t status; /* 八轴各自完整的catch驱动状态，不共用旧四腿快照。 */
    MergedPvt pvt;      /* 隔离0x43/44/49单寄存器原始单位对聚合反馈的覆盖。 */
    /* 白话：子帧0x52是打包好的位置+速度+电流，子帧0x43/44/49是单个原始寄存器值。
     * 这里单独存一份pvt，只让0x52往里写，避免单寄存器把聚合值覆盖掉。 */
    uint32_t state_sequence; /* 仅真实0x01状态回报递增，用来识别新回报。 */
    uint32_t mode_sequence; /* 仅真实0x02模式回报递增。 */
    uint32_t reset_sequence; /* 每次真实0x51复位状态回报递增，隔离旧复位位。 */
    /* 白话：这三个数字是“计数器”，每收到一条对应的新回报就+1。
     * 为什么需要它？因为驱动器回报里的“是否使能/复位标志”在没变化时会一直是同一个值，
     * 光看值分不出“这是刚才那条旧消息”还是“它真的又回答了一次”。序号+1才可信。 */
}
MergedFeedback;
/* 白话：我们“自己认定”的一台电机的软件状态（不代表电机真的照做了，只代表我们发过什么、确认到哪一步）。 */
typedef struct
{
    uint8_t sent_state; /* 最近确认的使能状态，取值MCU_STATE_*；MCU_MODE_UNKNOWN表示尚未确认。 */
    uint8_t sent_mode; /* 最近确认的USB模式，取值MCU_MODE_*；MCU_MODE_UNKNOWN表示复位事务后尚未重新配置。 */
    uint8_t gains_known; /* 当前模式速度增益缓存是否已成功发送；模式确认后清零。 */
    uint8_t clean_latched; /* 位置/PVT清积分按请求上升沿执行，避免持续高电平重复配置。 */
    float vel_kp, vel_ki; /* 已成功入队的速度kp、ki，采用适配到catch接口后的尺度。 */
    uint32_t seen_state_sequence; /* 状态机已消费的最近0x01使能回报序号。 */
    uint32_t seen_mode_sequence; /* 状态机已消费的最近0x02模式回报序号。 */
    uint32_t state_baseline; /* 请求使能/失能前的回报序号，后续确认必须使用新序号。 */
    uint32_t mode_baseline; /* 请求模式切换前的回报序号，排除保留的旧模式值。 */
    uint8_t expected_state; /* 等待确认的驱动使能值，取值MCU_STATE_*。 */
    uint8_t waiting_state_ack; /* 纯状态命令已入队但尚未收到新序号且值匹配的0x01回报。 */
    uint8_t waiting_mode_ack; /* 模式命令已入队但尚未收到新序号且值匹配的0x02回报。 */
    uint8_t mode_confirmed; /* 当前请求模式已由新真实回报确认；为0禁止目标发送。 */
    /* 白话：baseline（基准线）= 我们发命令那一刻的回报序号。
     * 只有当回报序号“大于基准”也就是“变了”，并且值也对，才算这条电机真的确认了我们的命令。 */
}
MergedAxisRuntime;

/* 下面是整个文件共享的“全局状态”。它们都是 static，只在本文件可见，靠调用之间保留。 */
/* s_latest：最新完整PC命令快照；开机run=0，不自行选择动作类别。 */
static MergedCommand s_latest;
/* s_feedback：八台物理电机各自的真实反馈与回报序号。 */
static MergedFeedback s_feedback[VB_AXIS_COUNT];
/* s_runtime：八轴模式配置、使能确认和增益缓存。 */
static MergedAxisRuntime s_runtime[VB_AXIS_COUNT];
/* s_rx：USB候选下行帧缓存，容量为一帧261字节。 */
/* 白话：USB是“字节流”，不保证一次回调刚好给一整帧，所以需要自己攒，攒够261字节再看对不对。 */
static uint8_t s_rx[VB_COMMAND_BYTES];
/* s_rx_length：当前已累计的候选帧字节数。 */
static uint16_t s_rx_length;
/* s_reset_poll_axis：空闲轮询复位状态时的物理轴下标。 */
/* 白话：没在复位的时候，我们也会轮流问一问八台电机“复位好了没”，这样反馈帧里的复位位始终是新的。
 * 这个变量就是“下一轮该问第几台”。 */
static uint8_t s_reset_poll_axis;
/* s_extra_traffic：本周期是否有额外配置流量；非零时让出普通复位轮询。 */
/* 白话：如果这5毫秒里已经排了配置命令（模式/增益/使能），就先不占用队列去“随便问问”，免得把CAN发送队列塞满。 */
static uint8_t s_extra_traffic;
/* s_reset_active：是否正在处理显式复位事务；上电为0。 */
static uint8_t s_reset_active;
/* s_reset_mask：当前复位事务选中的机构轴集合。 */
static uint8_t s_reset_mask;
/* s_wheel_reset_mask：当前复位事务选中的舵轮集合，与机构轴分开校验但共用事务号。 */
static uint8_t s_wheel_reset_mask;
/* s_reset_axis：当前逐轴复位流程处理到的轴下标。 */
static uint8_t s_reset_axis;
/* s_reset_stage：复位步骤：0确认失能、1发送复位、2等待复位、3给ARM2置零。 */
static uint8_t s_reset_stage;
/* s_reset_id：当前正在处理的PC复位事务编号。 */
static uint32_t s_reset_id;
/* s_reset_ack_id：最近已完成的复位事务编号；用于防止同号重传重复复位。 */
static uint32_t s_reset_ack_id;
/* s_reset_seq_start：发出本轴复位命令前的回报序号，用来区分已有回报。 */
static uint32_t s_reset_seq_start;
/* s_reset_action_sent：复位流程保留状态字段；当前仅初始化，未参与步骤判断。 */
static uint8_t s_reset_action_sent;
/* ARM2“已置零”仅代表SetDriverZero成功入队，不是机械到位/驱动ACK。 */
static uint8_t s_arm_pair_zeroed;
/* s_heartbeat：每次5ms入口递增的反馈心跳字节。 */
static uint8_t s_heartbeat;
/* s_mcu_counter：MCU执行周期计数，按31位正数范围循环。 */
static uint32_t s_mcu_counter;

/* 该映射决定USB槽序到物理CAN驱动号的对应；机构轴在CAN3用1..8，舵轮在CAN1用1..4。 */
/* 白话：协议里第几号电机，就对应CAN上第几号驱动器（0号槽->1号驱动器，7号槽->8号驱动器；
 * 8号槽起的舵轮重新从1号算，因为它们在另一条CAN1总线上）。以后改接线顺序只改这里。 */
static const uint8_t s_driver_id[VB_AXIS_COUNT] =
{
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
    1U, 2U, 3U, 4U
}
;

/* 中文名：浮点范围限幅。
 * 作用：将value限制在minimum与maximum之间，返回限幅后的数值；不发送电机命令。
 */
/* 参数：value为待限幅值，minimum/maximum为闭区间下限/上限，三者单位相同；返回值沿用输入单位。 */
/* 白话说明：就是“不超过上限、不低于下限”。比如目标写成99圈，也会被压到1.05圈（腿）或者1024圈（其他轴）。
 * 为什么要夹住？因为后面要把浮点数转成整数发给驱动器，数值太大会溢出变成一个方向相反的巨大值，电机就会飞车。 */
static float MergedClamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum; /* 太小了：直接取下限。 */
    if (value > maximum) return maximum; /* 太大了：直接取上限。 */
    return value; /* 在范围内：原样返回。 */
}

/* 转换前限制到现catch驱动的整数范围，避免异常大浮点数转int溢出。 */
/* 中文名：电机圈数转换为脉冲数。
 * 作用：将revolutions按每圈4096脉冲转换为int32；转换前限制数值范围，避免浮点转整数溢出。
 */
/* 参数：revolutions为圈数或圈/秒；返回位置脉冲或脉冲/秒，保持调用方物理量类别。 */
/* 白话说明：这一步把“人类的圈”翻译成“驱动器认的脉冲”。位置和速度都用同一个单位关系（1圈=4096脉冲）。 */
static int32_t MergedPulse(float revolutions)
{
    return (int32_t)(MergedClamp(revolutions, -MCU_PULSE_LIMIT, MCU_PULSE_LIMIT) * MCU_PULSES_PER_REVOLUTION); /* 先夹住圈数，再乘4096，最后截成整数。 */
}

/* 中文名：判断轴是否属于舵轮。
 * 作用：槽8..11为四个舵轮；返回1表示舵轮轴。
 */
static uint8_t MergedIsWheelAxis(uint8_t axis)
{
    return (uint8_t)(axis >= VB_WHEEL1 && axis < VB_AXIS_COUNT);
}

/* 中文名：取本轴所在CAN总线。
 * 作用：机构轴用CAN3，舵轮用CAN1；返回对应HAL句柄，两者都已在User_FDCAN_Init中启动。
 */
static FDCAN_HandleTypeDef *MergedAxisBus(uint8_t axis)
{
    return MergedIsWheelAxis(axis) ? &hfdcan1 : &hfdcan3;
}

/* 中文名：判断本事务是否选中某轴复位。
 * 作用：机构轴查机构掩码，舵轮查舵轮掩码；返回1表示该轴参与本次复位。
 */
static uint8_t MergedResetAxisSelected(uint8_t axis)
{
    if (MergedIsWheelAxis(axis)) return (uint8_t)((s_wheel_reset_mask & MCU_AXIS_BIT(axis)) != 0U);
    return (uint8_t)((s_reset_mask & MCU_AXIS_BIT(axis)) != 0U);
}

/* 中文名：按机构限制位置目标。
 * 作用：axis为物理轴下标；四腿目标限制在正负1.05圈，双臂、盘、丝杆和舵轮保留传入圈数。
 */
/* 参数：axis为0..11物理轴下标；revolutions为目标位置（圈）；返回该轴适用范围内的圈数。 */
/* 白话说明：腿只能动±1.05圈（机械就这么点行程），超了会把腿顶坏，所以只夹腿；
 * 臂、盘、丝杆的机械范围是另一套，舵轮按虚拟主轮给定值，都不能随便在这里夹，所以原样放过。 */
static float MergedPosition(uint8_t axis, float revolutions)
{
    /* 仅四腿保留原catch位置安全范围；舵轮槽位8..11不能命中该条件。 */
    if (axis >= VB_LEG1 && axis < VB_WHEEL1)
        return MergedClamp(revolutions, -MCU_LEG_POSITION_LIMIT, MCU_LEG_POSITION_LIMIT);
    return revolutions;
}

/* 中文名：解析并保存完整小电脑命令。
 * 作用：从frame读取运行、复位事务和八轴目标；发现非有限浮点数则拒绝整帧，否则在临界区更新最新命令，不直接发送CAN。
 */
/* 参数：frame为已通过SP/CRLF校验的261字节只读下行帧，不允许传入不完整缓冲。 */
/* 白话说明：这是“翻译官”。它把261字节的原始帧拆成程序能用的数字，存到 s_latest 里。
 * 关键态度：要么整帧全收，要么整帧拒收。只要有一个数是NaN（坏数）或无穷大，就整帧丢掉，
 * 绝不能出现“前4台电机更新了、后4台还是上一帧的老目标”这种半新半旧的危险状态。
 * 它只负责“记下来”，不直接给电机发任何指令。 */
static void MergedPublishCommand(const uint8_t *frame)
{
    MergedCommand command /* 完整解码候选；只有七槽全为有限数时才发布整帧。 */ =
    {
        0
    }
    ;
    uint8_t axis; /* 本函数遍历的物理轴槽：0..7机构电机，8..11舵轮。 */
    /* 第1步：取出“是否运行”“PC计数”“复位选择”“复位事务号”。 */
    command.run = frame[VB_RUN_BYTE] & MCU_RUN_REQUEST_MASK; /* 只看最低1位：1=允许电机动，0=所有普通目标都不执行。 */
    command.pc_counter = VbReadU32(frame + VB_COMMAND_DATA_OFFSET) & MCU_COUNTER_MASK; /* 数据区第0格是PC自己的循环计数，保留31位正数。 */
    command.reset_mask = frame[VB_RESET_REQUEST_BYTE]; /* 哪几台机构电机这次要复位（一位一台）。 */
    command.wheel_reset_mask = frame[VB_WHEEL_RESET_REQUEST_BYTE]; /* 哪几个舵轮这次要复位（一位一轮）。 */
    command.reset_id = VbReadU32(frame + VB_RESET_ID_BYTE); /* 本次复位请求的身份证号，机构轴和舵轮共用。 */
    for (axis = 0U; axis < VB_AXIS_COUNT; ++axis) /* 第2步：一台一台把12台电机的命令拆出来。 */
    {
        uint8_t field; /* 每轴七个浮点槽的字段下标，范围0..6。 */
        if (MergedIsWheelAxis(axis))
        {
            /* 舵轮模式与clean在新控制字节里，轮1对应最低两位/最低位。 */
            const uint8_t wheel /* 本舵轮在四个轮子里的序号0..3。 */ = (uint8_t)(axis - VB_WHEEL1);
            command.axis[axis].mode = (uint8_t)((frame[VB_WHEEL_MODE_BYTE]
                >> (wheel * MCU_MODE_BITS_PER_AXIS)) & MCU_MODE_AXIS_MASK);
            command.axis[axis].clean = (uint8_t)((frame[VB_WHEEL_CLEAN_BYTE] >> wheel) & 1U);
        }
        else
        {
            /* 模式：从2个字节里按“每轴2位”抠出这台的模式数字。 */
            command.axis[axis].mode = (uint8_t)((frame[VB_MODE_BYTE + axis / MCU_MODE_AXES_PER_BYTE]
                >> ((axis % MCU_MODE_AXES_PER_BYTE) * MCU_MODE_BITS_PER_AXIS)) & MCU_MODE_AXIS_MASK);
            /* 清积分：一个字节的8个位，一位对应一台机构电机。 */
            command.axis[axis].clean = (uint8_t)((frame[VB_CLEAN_BYTE] >> axis) & 1U);
        }
        for (field = 0U; field < VB_COMMAND_FIELDS; ++field) /* 第3步：把这台的7个浮点数挨个解出来。 */
        {
            const uint32_t slot /* 下行数据区32位槽下标；槽0是PC整数计数。 */ = 1U + axis * VB_COMMAND_FIELDS + field; /* 跳过第0格PC计数，后面每台占7格。 */
            const float value /* 本字段从小端IEEE754位模式解出的原值，单位由field决定。 */ = VbReadFloat(frame + VB_COMMAND_DATA_OFFSET + slot * sizeof(float)); /* 把4个字节按浮点数解释。 */
            /* 拒绝非数/无穷整帧，保证不会产生未定义的浮点转整数或半轴更新。 */
            if (!isfinite(value)) return; /* 发现NaN或无穷大：整帧作废，s_latest 保持上一帧不变。 */
            command.axis[axis].value[field] = value;
        }
    }
    /* USB回调只发布完整快照，不在回调里执行模式切换或CAN发送。 */
    /* 白话：这里加“临界区”是因为USB接收任务在写、5ms控制任务在读，
     * 复制12台电机的结构体不是一条CPU指令，中途被打断就会读到半新半旧的命令。 */
    taskENTER_CRITICAL();
    s_latest = command; /* 用整份新命令替换旧命令。 */
    taskEXIT_CRITICAL();
}

/* 中文名：初始化合并控制软件状态。
 * 作用：开机清零命令、反馈、复位事务和收帧状态；默认run为0、十二台电机都等待失能确认，不选择接球或发球，不发复位、置零或使能命令。
 */
/* 参数：无；必须在接收回调注册前调用，初始化不表示硬件已失能或复位。 */
/* 白话说明：开机先把“记忆”全部清干净，并且让8台电机都处于“我还不知道你是什么状态”的状态。
 * 注意：这里不发任何CAN命令，所以调用完并不等于电机已经断电，只是我们这边不主动做动作。
 * 顺序上必须在注册USB/CAN接收回调之前调用，否则回调可能往刚清零的地方写数据。 */
void MergedControl_Init(void)
{
    uint8_t axis; /* 本函数遍历或解出的物理轴下标：0双臂首轴，7第四腿。 */
    /* 由用户任务初始化函数在注册接收回调前调用，不在此触发机械动作。 */
    memset(&s_latest, 0, sizeof(s_latest)); /* 还没有命令：run=0、复位请求=0，所有目标都是0。 */
    memset(s_feedback, 0, sizeof(s_feedback)); /* 反馈先当作“什么都没收到”，各序号都是0。 */
    memset(s_runtime, 0, sizeof(s_runtime)); /* 软件确认态清零：还没确认过任何使能或模式。 */
    s_rx_length = 0U; /* 收帧缓冲是空的。 */
    s_reset_poll_axis = 0U; /* 空闲轮询从0号轴开始问。 */
    s_extra_traffic = 0U;
    s_reset_active = 0U; /* 当前没有复位事务在进行。 */
    s_reset_mask = 0U; /* 机构轴复位掩码清零。 */
    s_wheel_reset_mask = 0U; /* 舵轮复位掩码清零。 */
    s_reset_axis = 0U;
    s_reset_stage = 0U;
    s_reset_id = 0U;
    s_reset_ack_id = 0U; /* 还没有任何复位事务完成过，所以“最近完成号”是0（协议规定0号不算有效请求）。 */
    s_reset_action_sent = 0U;
    s_arm_pair_zeroed = 0U;
    s_heartbeat = 0U;
    s_mcu_counter = 0U;
    for (axis = 0U; axis < VB_AXIS_COUNT; ++axis) s_runtime[axis].sent_state = MCU_MODE_UNKNOWN; /* 8台电机都标记成“使能状态未知”，逼着后面先做一次真确认。 */
}

/* 中文名：接收并拼接小电脑USB字节流。
 * 作用：data为本次收到的字节，length为字节数；累计SP开头的261字节帧、校验CRLF并提交完整命令，坏帧重新寻找帧头。
 */
/* 参数：data为本次USB只读字节区，NULL时忽略；length为有效字节数，允许0及不对齐的分包。 */
/* 白话说明：USB一次给你多少字节是不确定的——可能只给半个帧，也可能一次给一帧半。
 * 所以这里逐字节“捡”：
 *   - 没在攒帧时，只在看到'S'时才准备开端；
 *   - 接下来必须是'P'，否则重新找；
 *   - 攒够261字节后，检查最后两个字节是不是'\r''\n'；
 *   - 是就交出去解析；不是就说明中间丢了字节或拼错了，从这一堆字节里再找下一个"SP"，重新对齐。
 * 全部在USB接收任务的上下文里执行，不直接给电机发命令。 */
void MergedControl_OnUsbBytes(const uint8_t *data, uint16_t length)
{
    uint16_t index; /* 本次USB回调data内逐字节处理的偏移，单位字节。 */
    if (data == NULL) return; /* 空指针：什么都不做。 */
    /* 单个候选帧缓冲即可处理任意长粘包，256字节USB回调也可累计261字节帧。 */
    /* 白话：缓冲区只有261字节，但因为我们边攒边判断，攒够就立刻处理清空，
     * 所以哪怕上层一次塞过来几千字节，也能顺着流一路处理下去。 */
    for (index = 0U; index < length; ++index)
    {
        const uint8_t byte /* 当前USB原始字节，用于帧头、帧尾和末尾S保留判断。 */ = data[index];
        if (s_rx_length == 0U) /* 情况1：还没开始攒。 */
        {
            if (byte == MCU_FRAME_HEADER0) s_rx[s_rx_length++] = byte; /* 看到'S'才算可能的开头，其它字节一律丢掉。 */
            continue;
        }
        if (s_rx_length == 1U && byte != MCU_FRAME_HEADER1) /* 情况2：刚看到'S'，第二个字节却不是'P'。 */
        {
            s_rx_length = byte == MCU_FRAME_HEADER0 ? 1U : 0U; /* 如果这个字节又是'S'，那它可能是真正的开头，保留它；否则彻底重新开始。 */
            continue;
        }
        s_rx[s_rx_length++] = byte; /* 情况3：正常攒字节。 */
        if (s_rx_length == VB_COMMAND_BYTES) /* 攒够一帧261字节了，开始验收。 */
        {
            if (s_rx[VB_COMMAND_BYTES - MCU_FRAME_TAIL_BYTES] == MCU_FRAME_TAIL0 &&
                s_rx[VB_COMMAND_BYTES - 1U] == MCU_FRAME_TAIL1) /* 检查最后两字节是不是回车换行。 */
            {
                MergedPublishCommand(s_rx); /* 帧完整：交给解析函数保存成最新命令。 */
                s_rx_length = 0U; /* 这一帧处理完，重新开始攒下一帧。 */
            }
            else
            {
                uint16_t next; /* 坏帧内下一处SP候选起点，单位字节；遍历从1开始排除旧头。 */
                /* 坏尾/丢字节时从下一处SP重同步；保留末尾S供下次回调补P。 */
                /* 白话：这一帧废了，但不能把整坨丢掉——里面可能已经藏着下一帧的开头("SP")。
                 * 从第1个字节开始找，"SP"可能存在的位置，找到就把后面的字节往前搬。 */
                for (next = 1U; next + 1U < s_rx_length; ++next)
                {
                    if (s_rx[next] == MCU_FRAME_HEADER0 && s_rx[next + 1U] == MCU_FRAME_HEADER1) break;
                }
                if (next + 1U < s_rx_length) /* 找到了下一个"SP"：从那里开始重新算。 */
                {
                    s_rx_length = (uint16_t)(s_rx_length - next); /* 剩下的字节数。 */
                    memmove(s_rx, s_rx + next, s_rx_length); /* 把这段搬回缓冲区开头。 */
                }
                else /* 没找到：整坨丢弃，但如果最后一个是'S'，先留着等下一批补'P'。 */
                {
                    s_rx_length = byte == MCU_FRAME_HEADER0 ? 1U : 0U;
                    if (s_rx_length != 0U) s_rx[0] = MCU_FRAME_HEADER0; /* 把这个孤零零的'S'放到开头。 */
                }
            }
        }
    }
}

/* 中文名：解析八轴驱动器CAN反馈。
 * 作用：仅处理bus为CAN3、identifier为0x281至0x288的回报；length是实际字节数，每8字节解析一次并更新真实反馈及回报序号。
 */
/* 参数：bus为来源HAL句柄（仅接受hfdcan3）；identifier为标准回报ID；data为只读载荷；length为0..64实际字节数，尾部不足8字节忽略。 */
/* 白话说明：这是“收电报”的地方。每台电机会把好几条状态打包成一条CAN报文发上来，
 * 这里按8字节一段拆开，交给驱动库的 GetDriverMsg 把每一段翻译成某个字段（位置？使能？模式？复位好了没？），
 * 同时对三种关键回报把序号+1，让5ms任务能判断“这是新消息”。
 * 注意：这里只更新“收到的记录”，不给电机发任何东西。 */
void MergedControl_OnCanFrame(FDCAN_HandleTypeDef *bus, uint32_t identifier,
    const uint8_t *data, uint8_t length)
{
    uint8_t offset; /* CAN-FD载荷内当前8字节子帧的起点，单位字节。 */
    uint8_t axis; /* 本函数解出的物理轴槽：0..7机构电机，8..11舵轮。 */
    /* 只认CAN3的机构回报0x281..0x288和CAN1的舵轮回报0x281..0x284，不接受其他总线和驱动器。 */
    /* 白话：第一关过滤。数据是空的、长度超过64、ID不在各自ID段内——一律不理。
     * 机构轴在CAN3上是0x281~0x288，舵轮在CAN1上重新从0x281~0x284算，
     * 因为两条总线各自独立编址，同号ID在不同总线上是不同电机。 */
    if (data == NULL || length > MCU_CAN_PAYLOAD_BYTES) return;
    if (bus == &hfdcan3 && identifier >= MCU_DRIVER_REPORT_FIRST_ID && identifier <= MCU_DRIVER_REPORT_LAST_ID)
    {
        axis = (uint8_t)(identifier - MCU_DRIVER_REPORT_FIRST_ID); /* 0x281->0号轴，0x288->7号轴。 */
    }
    else if (bus == &hfdcan1 && identifier >= MCU_DRIVER_REPORT_FIRST_ID &&
             identifier < MCU_DRIVER_REPORT_FIRST_ID + VB_WHEEL_COUNT)
    {
        axis = (uint8_t)(VB_WHEEL1 + (identifier - MCU_DRIVER_REPORT_FIRST_ID)); /* 0x281->8号轴（舵轮1）。 */
    }
    else
    {
        return;
    }
    for (offset = 0U; (uint16_t)offset + MCU_CAN_SUBFRAME_BYTES <= length; offset = (uint8_t)(offset + MCU_CAN_SUBFRAME_BYTES)) /* 一段8字节一段8字节地处理；尾巴不满8字节就丢掉。 */
    {
        const uint8_t *subframe /* 当前完整8字节子帧，只读且仅在本次接收调用中有效。 */ = data + offset;
        taskENTER_CRITICAL(); /* 进临界区：因为5ms任务可能同时复制 s_feedback，避免读到改到一半的结构体。 */
        GetDriverMsg(&s_feedback[axis].status, subframe); /* 让驱动库按“第0字节是哪种回报”把这段翻译进对应字段。 */
        if (subframe[0] == IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG) /* 这是一条0x52“打包回报”：位置+速度+电流。 */
        {
            /* 现catch GetDriverMsg已经完成旧serve /4096的统一；不可再次除4096。 */
            s_feedback[axis].pvt.pos = s_feedback[axis].status.pos; /* 把翻译好的位置抄到反馈用的槽位（单位：圈）。 */
            s_feedback[axis].pvt.vel = s_feedback[axis].status.vel; /* 速度（单位：圈/秒）。 */
            s_feedback[axis].pvt.current = s_feedback[axis].status.currQ; /* Q轴电流（单位：A），代表电机实际出力。 */
        }
        if (subframe[0] == IDENTIFIER_SET_DRIVER_STATE) ++s_feedback[axis].state_sequence; /* 驱动器回答“我现在是使能还是失能”：序号+1，表示这是新消息。 */
        if (subframe[0] == IDENTIFIER_SET_CTRL_MODE) ++s_feedback[axis].mode_sequence; /* 驱动器回答“我现在的控制模式”：序号+1。 */
        if (subframe[0] == IDENTIFIER_READ_RESET + IDENTIFIER_READ_OFFSET)
            ++s_feedback[axis].reset_sequence; /* 驱动器回答“复位好了没”：序号+1，用来区分“这次复位的结果”和“上一次残留的结果”。 */
        taskEXIT_CRITICAL(); /* 退出临界区。 */
    }
}

/* 中文名：判断单轴复位准备是否完成。
 * 作用：普通轴读取feedback中的真实复位标志；ARM2使用ARM1复位完成且ARM2置零已入队的配对条件，返回1表示该流程已准备完成。
 */
/* 参数：axis为物理轴下标；feedback指向该轴快照；arm1_reset_ok为同批快照的ARM1真实复位位；返回准备判据布尔值。 */
/* 白话说明：这个函数回答一个问题——“这台电机现在可以用了吗？”
 * 普通电机：看驱动器自己报的“复位完成标志”。
 * ARM2（第2台臂）：它不做真正的回零，只做“把当前位置记为零点”，所以它的判据是：
 *   ARM1真的复位好了，并且我们对ARM2的“置零”命令也已经发进队列。
 * 注意：ARM2这个判据只能说明“命令发出去了”，不能说明ARM2机械上真的到位了。 */
static uint8_t MergedResetReady(uint8_t axis, const MergedFeedback *feedback,
    uint8_t arm1_reset_ok)
{
    if (axis == VB_ARM2) /* 第2台臂是特殊情况。 */
        return (uint8_t)(s_arm_pair_zeroed != 0U && arm1_reset_ok != 0U); /* 两个条件都要：ARM2置零命令已发 + ARM1复位完成。 */
    return (uint8_t)(feedback->status.reset_pos_ok_flag != 0U); /* 其他电机：就看驱动器报的复位完成标志。 */
}

/* 中文名：取得门控后的单轴控制模式。
 * 作用：根据command的运行开关与axis请求返回控制模式；未运行或不允许的PVT请求返回0，使该轴禁用。
 */
/* 参数：axis为物理轴下标；command为完整只读PC快照；返回门控后的USB模式0..3。 */
/* 白话说明：“门控”就是加个安全闸：小电脑说“我要这台电机转”，但同时它的运行开关是关的，那就算禁用。
 * 另外PVT模式目前只在腿上有条件开放（默认是关的），如果请求了但没开放，就当成禁用处理，
 * 一定不能偷偷降级成位置模式——那会让电机在没预料的情况下乱跑。 */
static uint8_t MergedRequestedMode(uint8_t axis, const MergedCommand *command)
{
    uint8_t mode /* 本轴经run开关和PVT许可门控后的USB模式。 */ = command->run ? command->axis[axis].mode : MCU_MODE_DISABLE; /* 运行开关为0时，不管小电脑要什么模式，一律按禁用处理。 */
    /* PVT只允许四条腿使用；舵轮用位置环，其它机构也不用PVT。未开放时按失能处理，不降级成位置运动。 */
    if (mode == MCU_MODE_PVT && ((axis < VB_LEG1 || axis >= VB_WHEEL1) || !MERGED_ALLOW_UNVALIDATED_PVT))
        mode = MCU_MODE_DISABLE;
    return mode;
}

/* 中文名：发送单轴本周期目标。
 * 作用：按axis、command和mode调用原速度、位置或PVT驱动接口；完成单位及增益适配，返回CAN发送队列的入队结果，不代表驱动已执行。
 */
/* 参数：axis为物理轴下标；command为该轴七槽命令；mode为门控后的非零USB模式；返回原驱动发送入队结果。 */
/* 白话说明：这是“真正把目标递出去”的一步，按模式分三路：
 *   速度模式：把“圈/秒”换成“脉冲/秒”，电流原样（驱动库内部转成毫安），清积分直接带上。
 *   位置模式：把“圈”换成“脉冲”，增益要按轴区分对待——臂要除回缩小倍数，腿直接用。
 *   PVT模式（默认不开放）：只给腿，单位换成弧度，kp/kd原样传。
 * 返回值只表示“命令成功塞进了CAN发送队列”，不代表电机已经执行。 */
static HAL_StatusTypeDef MergedSendTarget(uint8_t axis, const MergedAxisCommand *command,
    uint8_t mode)
{
    const uint8_t driver /* 本轴CAN驱动编号，机构轴1..8或舵轮1..4。 */ = s_driver_id[axis]; /* 查表：这台电机在本总线上的驱动器编号。 */
    FDCAN_HandleTypeDef *const bus /* 本轴所在总线：机构轴CAN3，舵轮CAN1。 */ = MergedAxisBus(axis);
    const float *value /* 当前轴七槽原始命令数组；位置圈、速度圈/秒、电流A。 */ = command->value;
    if (mode == MCU_MODE_VELOCITY) /* 路线1：速度模式。 */
    {
        /* 圈/秒->整数脉冲/秒；电流保持A，由原VelCtrl内部编码毫安。 */
        return VelCtrl(bus, PTP_MODE, driver, MergedPulse(value[CMD_VEL]),
            value[CMD_CUR], command->clean != 0U); /* 速度换脉冲/秒，电流A，clean非0就带上清积分。 */
    }
    if (mode == MCU_MODE_POSITION) /* 路线2：位置模式。 */
    {
        /* 小电脑直接给驱动乘之前的原始增益，驱动 PosCtrl 内部再放大；本层不再除回去。 */
        return PosCtrl(bus, PTP_MODE, driver,
            MergedPulse(MergedPosition(axis, value[CMD_POS])), value[CMD_POS_KP], value[CMD_POS_KD]); /* 位置先按轴限幅（腿±1.05圈）再换脉冲，然后连同kp/kd一起下发。 */
    }
#if MERGED_ALLOW_UNVALIDATED_PVT
    if (mode == MCU_MODE_PVT && axis >= VB_LEG1) /* 路线3：PVT模式（只有打开实验开关时才会被编译进来，且只给腿）。 */
    {
        /* 原腿VEL_LIMIT=1.05圈/秒只是临时设定，绝非实测安全上限。 */
        const float position /* 腿PVT目标位置，圈限幅后转弧度。 */ = MergedPosition(axis, value[CMD_POS]) * MCU_RAD_PER_REVOLUTION; /* 先限幅，再把圈换成弧度。 */
        const float velocity /* 腿PVT目标速度，临时限幅后转弧度每秒。 */ =
            MergedClamp(value[CMD_VEL], -MCU_LEG_VELOCITY_LIMIT, MCU_LEG_VELOCITY_LIMIT) * MCU_RAD_PER_REVOLUTION; /* 速度同样先限幅再换弧度/秒。 */
        /* 共享帧的kp/kd在PVT模式对应mit_kp/mit_kd，原值直传，不作位置环缩放。 */
        return PVTCtrl(bus, driver, position, velocity,
            value[CMD_POS_KP], value[CMD_POS_KD], value[CMD_CUR]); /* PVT不经过位置环，所以kp/kd原样给。 */
    }
#endif
    return HAL_ERROR; /* 以上都没匹配上（不该发生）：返回错误，也不要让调用方以为发成功了。 */
}

/* 中文名：执行单轴模式与使能状态机。
 * 作用：使用command和真实feedback依次确认失能、模式、预装目标与使能；复位未准备好或模式禁用时只请求失能，稳态发送最新目标。
 */
/* 参数：axis为物理轴下标；command为本周期命令快照；feedback为该轴真实反馈快照；arm1_reset_ok用于ARM2配对门禁。 */
/* 白话说明：这是一台电机的“小管家”，每5毫秒按固定顺序走一遍，顺序很重要，乱一步就可能让电机在没准备好的情况下突然出力。
 * 大致顺序是：
 *   ① 先看驱动器有没有新回报（使能状态、模式）——有就更新我们自己的记忆。
 *   ② 如果要的是“禁用”或者这台电机还没复位好 ——> 只管让它断电，然后本周期就结束。
 *   ③ 如果要切模式 ——> 必须先真正确认它已经断电，再发切换模式；模式没被确认之前什么都不做。
 *   ④ 模式确认后 ——> 必要时更新速度增益/清积分 ——> 发目标 ——> 最后才发“使能”。
 * 关键：这台电机“上一次是什么状态”全记在 s_runtime[axis] 里，跨周期保留。 */
static void MergedExecuteAxis(uint8_t axis, const MergedCommand *command,
    const MergedFeedback *feedback, uint8_t arm1_reset_ok)
{
    /* 我们自己的记忆（可写）。runtime本轴可写的软件状态与已发送增益缓存，不是驱动实际反馈。 */ 
    MergedAxisRuntime *runtime  = &s_runtime[axis]; 
    /* target当前轴本周期命令快照，切换过程中仍读取最新目标。 */
    /* 小电脑这一刻想要什么。 */
    const MergedAxisCommand *target  = &command->axis[axis]; 
    /* 本轴CAN驱动编号1..8，用于原驱动发送接口。 */
    const uint8_t driver  = s_driver_id[axis];
    FDCAN_HandleTypeDef *const bus /* 本轴所在总线：机构轴CAN3，舵轮CAN1。 */ = MergedAxisBus(axis);
    /* 本轴经run和PVT许可门控后的USB模式。 */
    /* 经过安全闸之后，这台电机到底允许用哪种模式。 */
    const uint8_t mode  = MergedRequestedMode(axis, command); 
    /* 驱动实际模式值；USB PVT对应驱动PVT_MODE。 */
    /* USB那边的模式编号和驱动库的模式编号不完全一样，这里做换算（只有PVT不同）。 */
    const uint8_t driver_mode  = mode == MCU_MODE_PVT ? PVT_MODE : mode; 
    /* ---------------- 第①步：吸收新回报，更新自己的记忆 ---------------- */
    /* 小电脑把清积分位放回0了：解除“已经清过”的锁，下次再抬起来才算一次新请求（这就是“上升沿”）。 */
    /*反正下面别管，为了安全。（可以研究研究*/
    if (target->clean == 0U) runtime->clean_latched = 0U; 
    if (feedback->state_sequence != runtime->seen_state_sequence) /* 驱动器又回答了一次“我现在使能/失能”。 */
    {
        runtime->seen_state_sequence = feedback->state_sequence; /* 记下“这条回报我看过了”。 */
        if (runtime->waiting_state_ack && feedback->state_sequence != runtime->state_baseline &&
            feedback->status.enableState == runtime->expected_state) /* 我们正在等使能确认，而且这次回报确实是新的，值也正是我们想要的。 */
        {
            runtime->waiting_state_ack = 0U; /* 不再等了。 */
            runtime->sent_state = runtime->expected_state; /* 记下：这台电机现在是（期望的）使能状态。 */
        }
        else if (!runtime->waiting_state_ack) /* 我们没在等确认，但驱动器主动报了状态（比如别人刚给它断过电）。 */
        {
            /* 非本模块等待中的新真实状态回报也同步本地确认态，不改写driverMsg。 */
            runtime->sent_state = feedback->status.enableState; /* 那就以驱动器为准，更新我们的记忆。 */
        }
    }

    if (feedback->mode_sequence != runtime->seen_mode_sequence) /* 驱动器又回答了一次“我现在是什么模式”。 */
    {
        runtime->seen_mode_sequence = feedback->mode_sequence; /* 记下看过了。 */
        if (runtime->waiting_mode_ack && feedback->mode_sequence != runtime->mode_baseline &&
            feedback->status.driverMode == driver_mode) /* 我们在等模式确认，回报是新的，模式也对上了。 */
        {
            runtime->waiting_mode_ack = 0U; /* 不再等。 */
            runtime->mode_confirmed = 1U; /* 模式确认完成：从此这台电机可以发目标了。 */
            runtime->sent_mode = mode; /* 记下当前确认的模式。 */
            runtime->gains_known = 0U; /* 换过模式后，之前发过的增益不一定还生效，标记为“需要重新下发”。 */
        }
        else if (feedback->status.driverMode != driver_mode) /* 驱动器报的模式不是我们想要的（可能被复位清掉了）。 */
        {
            runtime->mode_confirmed = 0U; /* 取消“模式已确认”，等下次重新切。 */
        }
    }
    /* ---------------- 第②步：不该动的场合，只保证断电 ---------------- */
    if (mode == MCU_MODE_DISABLE || !MergedResetReady(axis, feedback, arm1_reset_ok)) /* 小电脑要禁用，或者这台电机还没复位好。 */
    {
        /* 白话：只要我们已经知道它是失能的，就不重复发；如果还在等失能确认，也不重复发。
         * 只有在“不是失能”或“上次没等到确认”的情况下，才重新发一次失能。 */
        if (runtime->sent_state != MCU_STATE_DISABLE &&
            (runtime->expected_state != MCU_STATE_DISABLE || !runtime->waiting_state_ack))
        {
            runtime->waiting_state_ack = 0U;
            runtime->state_baseline = feedback->state_sequence; /* 记下当前回报序号作为基准线，之后必须看到“更新的”回报才算确认。 */
            runtime->expected_state = MCU_STATE_DISABLE; /* 我们期望它变成失能。 */
            if (SetDriverState(bus, PTP_MODE, driver, DISABLE) == HAL_OK) /* 发“失能”命令。 */
                runtime->waiting_state_ack = 1U; /* 排队成功，开始等驱动器确认。 */
        }
        return; /* 本轴本周期到此结束，不发任何目标。 */
    }
    /* ---------------- 第③步：需要换模式时，先确认断电再切 ---------------- */
    /* 模式未确认/需要切换时，先真实确认失能；待确认使能期间则不重复发命令。 */
    if (!runtime->mode_confirmed || runtime->sent_mode != mode)
    {
        if (runtime->waiting_state_ack) return; /* 还在等上一条失能命令的确认：什么都别做，等下个5毫秒。 */
        if (runtime->sent_state != MCU_STATE_DISABLE) /* 现在不是失能的，必须先断电才能切模式（带电切模式可能让电机猛抖一下）。 */
        {
            runtime->state_baseline = feedback->state_sequence;
            runtime->expected_state = MCU_STATE_DISABLE;
            if (SetDriverState(bus, PTP_MODE, driver, DISABLE) == HAL_OK)
                runtime->waiting_state_ack = 1U;
            return; /* 这一步只负责“断电”，切模式留到下一个5毫秒。 */
        }
        if (!runtime->waiting_mode_ack) /* 已经确认断电了，可以放心切模式。 */
        {
            runtime->mode_baseline = feedback->mode_sequence; /* 记下模式回报序号基准线。 */
            if (SetDriverMode(bus, PTP_MODE, driver, driver_mode,
                MergedPulse(MergedPosition(axis, target->value[CMD_POS]))) == HAL_OK) /* 切模式，并顺手把这个位置作为“预装目标”，避免切完瞬间跳一下。 */
                runtime->waiting_mode_ack = 1U; /* 等驱动器确认模式。 */
        }
        return;
    }
    /* ---------------- 第④步：模式已确认，正常配置增益并下发目标 ---------------- */
    /* 哦终于开始了呵呵*/
    /* 速度模式按每周期命令更新增益；位置/PVT模式只在clean上升沿用负哨兵清积分。 */
    const uint8_t clean_request /* 本周期是否需要把clean状态写入驱动，用于增益缓存与clean_latched比较。 */ =
        target->clean != 0U && !runtime->clean_latched; /* 只有“这次刚抬起”才算一次新请求（上升沿）。 */
    if (mode == MCU_MODE_VELOCITY || (mode == MCU_MODE_POSITION && clean_request)) /* 速度模式每次都要核对增益；位置模式只在要清积分时顺带发一次。 */
    {
        float kp = mode == MCU_MODE_VELOCITY ? target->value[CMD_VEL_KP] : MCU_GAIN_UNSET; /* 速度比例增益；位置clean仅传负哨兵。 */
        float ki = mode == MCU_MODE_VELOCITY ? target->value[CMD_VEL_KI] : MCU_GAIN_UNSET; /* 速度积分增益；位置clean不覆盖原整定参数。 */
        /* 白话：位置模式这里只是想“清积分”，并不想改速度环整定值，所以两个增益都传-1，告诉驱动库“这俩别动”。 */
        /* 小电脑直接给驱动乘之前的原始增益，驱动 SetVelKpKi 内部再乘回去；本层不做逆缩放。 */
        if (!runtime->gains_known || kp != runtime->vel_kp || ki != runtime->vel_ki || clean_request) /* 增益从没发过、或者这次不一样、或者要清积分：才需要发。 */
        {
            if (SetVelKpKi(bus, PTP_MODE, driver, kp, ki, target->clean != 0U) != HAL_OK) return; /* 发增益+清积分命令；发不出去就放弃本次，别乱往下走。 */
            runtime->vel_kp = kp; /* 记住已经发下去的增益值。 */
            runtime->vel_ki = ki;
            runtime->gains_known = 1U; /* 标记“增益已知”，以后值没变就不用每5毫秒重发。 */
            runtime->clean_latched = target->clean != 0U; /* 记住这次已经清过，防止一直高电平反复清。 */
            s_extra_traffic = 1U; /* 标记“本周期已经有额外配置流量”。 */
        }
    }
#if MERGED_ALLOW_UNVALIDATED_PVT
    else if (mode == MCU_MODE_PVT && clean_request) /* PVT模式要清积分：不碰增益，只把清积分位带出去。 */
    {
        if (SetVelKpKi(bus, PTP_MODE, driver, MCU_GAIN_UNSET, MCU_GAIN_UNSET, true) != HAL_OK) return;
        runtime->clean_latched = 1U;
        s_extra_traffic = 1U;
    }
#endif
    /* 发送本周期目标（位置/速度/PVT），失败就本周期算了。 */
    if (MergedSendTarget(axis, target, mode) != HAL_OK) return; 
    /*下面要考虑考虑顺序*/
    /* ---------------- 第⑤步：目标确实发出去了，最后才允许使能 ---------------- */
    /* 白话：“先给目标，再给电”，这样电机一通电就已经知道该去哪儿，而不是先通电再等目标，避免原地乱动。 */
    if (runtime->sent_state != MCU_STATE_ENABLE && !runtime->waiting_state_ack) /* 还没确认它已使能，而且也没有在等确认。 */
    {
        runtime->state_baseline = feedback->state_sequence;
        runtime->expected_state = MCU_STATE_ENABLE; /* 期望它变成使能。 */
        if (SetDriverState(bus, PTP_MODE, driver, ENABLE) == HAL_OK) /* 发“使能”命令。 */
        {
            runtime->waiting_state_ack = 1U; /* 等驱动器确认。 */
            s_extra_traffic = 1U;
        }
    }
}

/* 中文名：校验舵轮复位掩码。
 * 作用：mask低4位对应四个舵轮；四个轮子运动同步但复位各自独立，允许任意非空子集，合法返回1。
 */
/* 参数：mask为舵轮复位选择位（bit0对应舵轮1）；返回1允许受理，0表示空或含保留位。 */
/* 白话说明：舵轮不像腿和双臂那样靠结构互相约束，每个轮子都能单独回零，
 * 所以这里只要求“至少选一个，且不许出现不存在的轮子位”。 */
static uint8_t MergedValidWheelResetMask(uint8_t mask)
{
    const uint8_t known /* 掩码只允许出现在四个舵轮位上。 */ = mask != 0U && (mask & MCU_MASK_WHEELS) == mask;
    return known;
}

/* 中文名：校验互斥机构复位掩码。
 * 作用：mask每一位对应一台物理电机；只允许四腿、双臂、双臂加盘、单盘或单丝杆，拒绝单臂与跨机构混合，合法返回1。
 */
/* 参数：mask为八轴复位选择位（bit0对应ARM1）；返回1允许受理，0表示空或不允许的机构组合。 */
/* 白话说明：复位会让机构物理地动起来，所以“哪些能一起复位”必须守规矩：
 *   - 四条腿：可以一起，也可以只挑其中几条；
 *   - 两台臂：必须成对，可以再加盘，但不许只选一条臂；
 *   - 盘、丝杆：可以单独选其中一个；
 *   - 不许“臂+腿”、不许“盘+丝杆一起”、不许出现不存在的电机位。
 * 原因：这些机构靠结构互相约束，同时复位容易互相顶到。 */
static uint8_t MergedValidResetMask(uint8_t mask)
{
    const uint8_t known /* 掩码只允许出现已知的八轴位，任何保留位都拒绝。 */ = (mask & MCU_MASK_ALL) == mask; /* 有没有选了不存在的位。 */
    const uint8_t legs /* 四腿可单独或成组参与接球测试，空集不算。 */ = mask != 0U && (mask & MCU_MASK_LEGS) == mask; /* 选的都是腿，而且至少选了一条。 */
    const uint8_t body /* 盘或丝杆各自独立，可以只选其中一个。 */ =
        mask == MCU_AXIS_BIT(VB_PLATE) || mask == MCU_AXIS_BIT(VB_SLIDE); /* 只选盘，或者只选丝杆。 */
    const uint8_t arm_pair /* 双臂必须成对，可再带盘；单臂、盘丝同选或混腿都拒绝。 */ =
        mask == MCU_MASK_ARMS || mask == MCU_MASK_ARM_PLATE; /* 双臂，或者双臂+盘。 */
    return (uint8_t)(known && (legs || body || arm_pair)); /* 位都合法，并且组合属于上面允许的几种之一，才放行。 */
}

/* 事务期间仅失能与读取复位状态；一台一台复位，不自动启动动作。 */
/* 中文名：推进所选机构复位事务。
 * 作用：每次调用推进当前步骤；只等待选中轴的真实失能和复位回报，ARM1完成后给ARM2置零；全部准备完成才保存事务回执，不自动起动动作。
 */
/* 参数：feedback为按物理轴顺序排列的八轴只读反馈快照；实际发送状态仍从各轴独立runtime推进。 */
/* 白话说明：这是复位流程的“总控”。它每次被调用只推进一步，然后返回，等下个5毫秒继续，绝不在这卡住等待。
 * 整个流程分四段：
 *   第0段（DISABLE）：确认要复位的电机都真的断电了；顺带把不参与复位的电机也断电，但不为它们等待。
 *   第1段（FIRE）：按顺序挑下一台要复位的电机；如果是双臂里的ARM2，改走“置零”分支。
 *   第3段（ARM2_ZERO）：给ARM2发“把当前位置当零点”的命令，发了就记“已发”。
 *   第2段（WAIT_ACK）：等这台电机报“复位完成”（必须是新回报，不能是老消息）。
 * 全部处理完，才把 s_reset_ack_id 设成这次的事务号，小电脑看到这个号就知道“这一批复位处理完了”。
 * 它不会自动让电机开始运动——复位完就停在原地，等小电脑下一步指令。 */
static void MergedRunResetTransaction(const MergedFeedback *feedback)
{
    uint8_t axis; /* 本函数遍历或解出的物理轴下标：0双臂首轴，7第四腿。 */
    /* ================== 第0段：确认断电 ================== */
    if (s_reset_stage == MCU_RESET_STAGE_DISABLE)
    {
        for (axis = 0U; axis < VB_AXIS_COUNT; ++axis) /* 八台电机全过一遍。 */
        {
            MergedAxisRuntime *runtime  = &s_runtime[axis];
            /* runtime是本轴可写的软件状态与已发送增益缓存，不是驱动实际反馈。 */
            const uint8_t selected /* 本事务是否选中此轴：机构轴查机构掩码，舵轮查舵轮掩码。 */ =
                MergedIsWheelAxis(axis) ? ((s_wheel_reset_mask & MCU_AXIS_BIT(axis)) != 0U)
                    : ((s_reset_mask & MCU_AXIS_BIT(axis)) != 0U);
            /* selected是本事务是否选中此轴；仅选中轴的失能确认会阻塞流程。 */
            if (runtime->waiting_state_ack) /* 还在等这台电机的失能确认。 */
            {
                if (feedback[axis].state_sequence == runtime->state_baseline ||
                    feedback[axis].status.enableState != MCU_STATE_DISABLE) /* 回报没更新，或者更新了但不是“失能”。 */
                {
                    if (selected) return; /* 如果是“要复位”的电机还没确认断电，就必须卡住等它，不能急。 */
                    /* 如果是不参与复位的电机，那就不为它等待，继续往下走（它断电晚一点也没关系）。 */
                }
                else /* 确认了：它真的断电了。 */
                {
                    runtime->waiting_state_ack = 0U; /* 不再等。 */
                    runtime->sent_state = MCU_STATE_DISABLE; /* 记下它现在是失能。 */
                }
            }
            if (runtime->sent_state != MCU_STATE_DISABLE) /* 这台还是“非失能”（比如还没确认过），需要发一次失能。 */
            {
                taskENTER_CRITICAL();
                runtime->state_baseline = s_feedback[axis].state_sequence; /* 记基准线。 */
                taskEXIT_CRITICAL();
                runtime->expected_state = MCU_STATE_DISABLE;
                if (SetDriverState(MergedAxisBus(axis), PTP_MODE, s_driver_id[axis], DISABLE) == HAL_OK) /* 发失能命令。 */
                {
                    if (selected) runtime->waiting_state_ack = 1U; /* 要复位的：必须等它确认断电，所以标记得等。 */
                    else runtime->sent_state = MCU_STATE_DISABLE; /* 非参与轴失能可发，但永不阻塞本事务。 */
                }
                if (selected) return; /* 要复位的电机刚发了失能命令，本周期就返回，等确认。 */
            }
        }
        s_reset_stage = MCU_RESET_STAGE_FIRE; /* 所有要复位的电机都确认断过电（或本轮已发），进入“发送复位命令”阶段。 */
        s_reset_axis = 0U; /* 从0号电机开始一台一台来。 */
    }
    /* ================== 找“下一台要复位的电机” ================== */
    while (s_reset_axis < VB_AXIS_COUNT && !MergedResetAxisSelected(s_reset_axis))
        ++s_reset_axis; /* 跳过没被选中的电机。 */
    if (s_reset_axis >= VB_AXIS_COUNT) /* 八台都处理过了：整批复位结束。 */
    {
        s_reset_active = 0U; /* 事务结束，恢复正常运动处理。 */
        s_reset_ack_id = s_reset_id; /* 把这次的事务号记成“已完成”，小电脑据此知道这批复位完成了。 */
        return;
    }
    /* ================== 第1段：发送复位命令 ================== */
    if (s_reset_stage == MCU_RESET_STAGE_FIRE)
    {
        if (s_reset_axis == VB_ARM2 && (s_reset_mask & MCU_MASK_ARMS) == MCU_MASK_ARMS) /* 舵轮轴不会等于VB_ARM2，不受影响。 */ /* 轮到ARM2，而且这次是两个臂一起复位。 */
        {
            /* 发球机构原流程只对ARM1 ResetDriver，ARM2执行SetDriverZero。 */
            /* 白话：ARM2不真正回零，只把“现在的位置”当零点，所以要单独走第3段。 */
            s_reset_stage = MCU_RESET_STAGE_ARM2_ZERO;
        }
        else /* 普通电机：走真正的“复位/回零”。 */
        {
            taskENTER_CRITICAL();
            s_reset_seq_start = s_feedback[s_reset_axis].reset_sequence; /* 记下当前的复位回报序号。 */
            s_feedback[s_reset_axis].status.reset_pos_ok_flag = 0U; /* 先把旧的“复位完成”标志清掉，防止用老信息误判成功。 */
            taskEXIT_CRITICAL();
            if (ResetDriver(MergedAxisBus(s_reset_axis), PTP_MODE, s_driver_id[s_reset_axis]) == HAL_OK) /* 发送复位命令。 */
                s_reset_stage = MCU_RESET_STAGE_WAIT_ACK; /* 发送成功：进入“等待复位完成回报”阶段。 */
            return;
        }
    }
    /* ================== 第3段：给ARM2置零 ================== */
    if (s_reset_stage == MCU_RESET_STAGE_ARM2_ZERO)
    {
        /* 原发球流程不等待ARM2复位ACK；这里只记录SetDriverZero已成功入队。 */
        if (SetDriverZero(MergedAxisBus(s_reset_axis), PTP_MODE, s_driver_id[s_reset_axis]) == HAL_OK) /* 发“把当前位置设为0”命令。 */
        {
            s_arm_pair_zeroed = 1U; /* 记下“ARM2置零命令已发”（不代表机械到位）。 */
            ++s_reset_axis; /* 处理下一台。 */
            s_reset_stage = MCU_RESET_STAGE_FIRE; /* 回到“发送复位命令”阶段。 */
        }
        return;
    }
    /* ================== 第2段：等待复位完成回报 ================== */
    if (s_reset_stage == MCU_RESET_STAGE_WAIT_ACK &&
        feedback[s_reset_axis].reset_sequence != s_reset_seq_start && /* 复位回报序号变了：说明是新回报，不是老的。 */
        feedback[s_reset_axis].status.reset_pos_ok_flag != 0U) /* 而且驱动器说“复位好了”。 */
    {
        ++s_reset_axis; /* 这台完成，去处理下一台。 */
        s_reset_stage = MCU_RESET_STAGE_FIRE;
    }
}

/* 中文名：组装并回传八轴状态。
 * 作用：从feedback与当前command生成133字节QR帧，包含真实反馈、复位准备位、配置就绪位和PS3；USB可入队才发送，拥塞时丢弃本周期帧。
 */
/* 参数：feedback为八轴只读真实反馈快照；command为本周期PC命令快照，用于判断请求模式与ready是否一致。 */
/* 白话说明：这是“写报告”的地方，把八台电机的真实情况打包成133字节回给小电脑。报告里有：
 *   心跳、光电值、复位回执号、每台电机的错误码、是否使能、复位是否就绪（第12字节）、
 *   “完全就绪”位（第14字节，含义见下面的循环）、真实位置/速度/电流。
 * 只有USB已经连上才发；如果发不出去就丢掉这一帧，下一周期会有新的，不会积压旧数据。 */
static void MergedSendFeedback(const MergedFeedback *feedback, const MergedCommand *command)
{
    uint8_t frame[VB_FEEDBACK_BYTES] /* 本次反馈完整133字节帧；所有保留字节初始为0。 */ =
    {
        0
    }
    ;
    uint8_t axis; /* 本函数遍历或解出的物理轴下标：0双臂首轴，7第四腿。 */
    /* 第1步：填帧头"QR"和帧尾回车换行。 */
    frame[0] = MCU_FEEDBACK_HEADER0;
    frame[1] = MCU_FEEDBACK_HEADER1;
    frame[VB_FEEDBACK_BYTES - MCU_FRAME_TAIL_BYTES] = MCU_FRAME_TAIL0;
    frame[VB_FEEDBACK_BYTES - 1U] = MCU_FRAME_TAIL1;
    frame[VB_HEARTBEAT_BYTE] = s_heartbeat; /* 心跳：每5毫秒+1，小电脑看到它在变就知道主控还活着。 */
    /* 光电byte13保持原始ps_num[MCU_PS3_SLOT]，不作布尔化或电平翻转。 */
    frame[VB_PS3_BYTE] = ps_num[MCU_PS3_SLOT]; /* 光电串口第3个字节，原样转发，不做任何处理。 */
    /* 事务ACK是小端uint32，非对齐写入由共享codec处理。 */
    VbWriteU32(frame + VB_RESET_ACK_BYTE, s_reset_ack_id); /* 告诉小电脑“最近完成的是哪个复位事务”。 */
    /* byte14仅报告具备真实复位、模式、使能反馈且配置成功的轴。 */
    /* 白话：第14字节的每一位代表一台电机“真的可以动了”。要同时满足全部条件才算：
     *   要的模式不是禁用 + 复位真的就绪 + 驱动器真的报使能 + 驱动器报的模式正是我们要的
     *   + 我们的等待都结束了 + 我们看到的最新回报就是驱动器当前的上报（不是漏看了新回报）。
     * 少任何一条，这一位就是0，小电脑就知道“还不能发运动指令”。 */
    for (axis = 0U; axis < VB_AXIS_COUNT; ++axis)
    {
        const uint8_t mode /* 本轴经run和PVT许可门控后的USB模式。 */ = MergedRequestedMode(axis, command);
        const uint8_t expected_mode /* ready校验所需的驱动模式，PVT请求转换为驱动PVT模式。 */ =
            mode == MCU_MODE_PVT ? PVT_MODE : mode; /* 和前面一样，把USB编号换成驱动库编号。 */
        const MergedAxisRuntime *runtime /* 本轴软件确认状态，必须与真实反馈序号一致才算ready。 */ = &s_runtime[axis];
        if (mode != MCU_MODE_DISABLE &&
            MergedResetReady(axis, &feedback[axis], feedback[VB_ARM1].status.reset_pos_ok_flag) &&
            feedback[axis].status.enableState == ENABLE &&
            feedback[axis].status.driverMode == expected_mode &&
            runtime->mode_confirmed && !runtime->waiting_mode_ack &&
            runtime->seen_mode_sequence == feedback[axis].mode_sequence &&
            runtime->sent_mode == mode && runtime->sent_state == MCU_STATE_ENABLE &&
            !runtime->waiting_state_ack &&
            runtime->seen_state_sequence == feedback[axis].state_sequence)
        {
            /* 机构轴写byte14，舵轮写byte25，各自低4位对应轴槽8..11。 */
            if (MergedIsWheelAxis(axis))
                frame[VB_WHEEL_READY_BYTE] |= (uint8_t)(1U << (axis - VB_WHEEL1));
            else
                frame[VB_READY_BYTE] |= MCU_AXIS_BIT(axis);
        }
    }
    VbWriteU32(frame + VB_FEEDBACK_DATA_OFFSET, s_mcu_counter); /* 数据区第0格放主控自己的执行周期计数。 */
    for (axis = 0U; axis < VB_AXIS_COUNT; ++axis) /* 逐台填“真实情况”。 */
    {
        const uint32_t offset /* 当前轴位置反馈在QR完整帧中的字节起点，后续是速度与电流。 */ =
            VB_FEEDBACK_DATA_OFFSET + (1U + axis * VB_FEEDBACK_FIELDS) * sizeof(float); /* 跳过第0格周期计数，每台占3格（位置/速度/电流）。 */
        if (MergedIsWheelAxis(axis))
        {
            /* 舵轮状态写19..25字节：错误码逐轮一字节，使能/复位/就绪各占低4位。 */
            const uint8_t wheel /* 本舵轮在四个轮子里的序号0..3。 */ = (uint8_t)(axis - VB_WHEEL1);
            frame[VB_WHEEL_ERROR_BYTE + wheel] = feedback[axis].status.error;
            if ((feedback[axis].status.enableState & 1U) != 0U)
                frame[VB_WHEEL_ENABLE_BYTE] |= (uint8_t)(1U << wheel);
            if (feedback[axis].status.reset_pos_ok_flag != 0U)
                frame[VB_WHEEL_RESET_BYTE] |= (uint8_t)(1U << wheel);
        }
        else
        {
            frame[VB_ERROR_BYTE + axis] = feedback[axis].status.error; /* 错误码：这台驱动器报过的错误。 */
            frame[VB_ENABLE_BYTE] |= (uint8_t)((feedback[axis].status.enableState & 1U) << axis); /* 使能位图：把第axis位设成它真实的使能状态。 */
            /* 上报准备位复用MergedResetReady：ARM2为配对准备，其余轴为真实复位完成位。 */
            if (MergedResetReady(axis, &feedback[axis], feedback[VB_ARM1].status.reset_pos_ok_flag))
                frame[VB_RESET_BYTE] |= MCU_AXIS_BIT(axis); /* 复位就绪位图（ARM2用的是“ARM1好了+置零已发”这个特殊判据）。 */
        }
        VbWriteFloat(frame + offset, feedback[axis].pvt.pos); /* 真实位置（圈）。 */
        VbWriteFloat(frame + offset + sizeof(float), feedback[axis].pvt.vel); /* 真实速度（圈/秒）。 */
        VbWriteFloat(frame + offset + 2U * sizeof(float), feedback[axis].pvt.current); /* 真实Q轴电流（A）。 */
    }
    if (UsbTx_IsReady()) /* USB已经连上才发，没连上就不浪费时间。 */
    {
        /* 0表示成功入队，不是已发字节数；拥塞丢此周期，不重放旧反馈。 */
        (void)UsbTx_Send(frame, (uint16_t)sizeof(frame), 0U); /* 把整帧塞进USB发送队列；结果不看，塞不进去就算了。 */
    }
}

/* 中文名：合并控制五毫秒执行入口。
 * 作用：用户主任务每5ms调用；取得12轴命令和反馈快照，推进显式复位事务或执行各轴目标，再轮询复位状态并回传反馈。没有PC命令时保持run为0，仅失能与读取状态，不运行本地接球轨迹。
 */
/* 参数：无；调用上下文为唯一用户控制任务，周期5ms，不可并行调用或在USB中断内调用。 */
/* 白话说明：这是整个模块的“心跳”，每5毫秒跑一次，一次跑完做五件事：
 *   1) 拍快照：把PC命令和12台电机（8机构+4舵轮）的反馈整体复制一份，后面只用这份副本，避免中途被别人改。
 *   2) 记账：周期计数+1、心跳+1、清掉“本周期有没有额外流量”的标记。
 *   3) 判断要不要开始一批新的复位（小电脑明确要求、事务号是新的、组合合法、当前不在别的复位里、而且运行开关是关的）。
 *   4) 分支：正在复位就推进复位流程；否则让8台电机各自走一遍“状态机”。
 *   5) 补问一句“复位好了没”，再刷新一次反馈快照，最后把报告发回去。
 * 注意：本函数自己不会自动接球、发球或复位，完全听PC的。 */
void MergedControl_Tick(void)
{
    MergedCommand command; /* 本周期完整命令快照；复制后不受USB回调更新影响。 */
    MergedFeedback feedback[VB_AXIS_COUNT]; /* 本周期八轴反馈快照；发送反馈前再次整体刷新。 */
    uint8_t axis; /* 本函数遍历或解出的物理轴下标：0双臂首轴，7第四腿。 */
    /* 整帧命令和八轴反馈原子快照，临界区外才能排队发送/执行控制。 */
    /* 白话：为什么要复制？因为USB接收任务和CAN接收任务随时可能在改 s_latest / s_feedback。
     * 不加临界区直接读，可能读到一半被改，8台电机的数据就对不上了。 */
    taskENTER_CRITICAL();
    command = s_latest; /* 复制本周期要执行的PC命令。 */
    memcpy(feedback, s_feedback, sizeof(feedback)); /* 复制8台电机的反馈。 */
    taskEXIT_CRITICAL();

    s_mcu_counter = (s_mcu_counter + 1U) & MCU_COUNTER_MASK;//就是自增罢了
    ++s_heartbeat; /* 心跳+1：小电脑借此判断主控有没有卡死。 */
    s_extra_traffic = 0U;//别管
    /* 白话：这是个“本周期有没有做过额外配置”的临时标记，每个周期开头清零，
     * 第⑤步后面用它决定“要不要再去随便问问某台电机的复位状态”，避免把CAN发送队列塞满。 */


    /* 统一准备复位 */
    /* 每个事务ID只受理一次，且只能在run=false时受理互斥的合法轴mask。 */
    /* 白话：这里是在检查“现在可不可以开始一次新的复位”，四个条件缺一不可：
     *   ① 当前没有别的复位在进行；② 小电脑的运行开关是关的（运动中不许复位）；
     *   ③ 小电脑给的复位事务号不是0、而且不是“刚做完的那一个”（防重发）；
     *   ④ 选的电机组合合法（按机构互斥规则）。 */
    if (!s_reset_active && command.run == 0U && command.reset_id != 0U &&
        command.reset_id != s_reset_ack_id &&
        (MergedValidResetMask(command.reset_mask) || MergedValidWheelResetMask(command.wheel_reset_mask)))
        //前面是看现在是不是复位状态，后面是看机构轴或舵轮是不是合法的组合
    {
        s_reset_id = command.reset_id; /* 记住这次复位的事务号，机构轴和舵轮共用。 */
        s_reset_mask = command.reset_mask; /* 记住这次要复位哪几台机构轴。 */
        s_wheel_reset_mask = command.wheel_reset_mask; /* 记住这次要复位哪几个舵轮。 */
        s_reset_axis = 0U; /* 从0号轴开始处理。 */
        s_reset_stage = MCU_RESET_STAGE_DISABLE; /* 第一步：先确认断电。 */
        s_reset_action_sent = 0U;
        if ((s_reset_mask & MCU_MASK_ARMS) == MCU_MASK_ARMS) s_arm_pair_zeroed = 0U;
        //因为臂的复位和设0点是特别的，所以专门列出
        /* 白话：这次只要包含双臂，就要把“ARM2已置零”这个标记清掉重新记，
         * 否则会拿着上一批复位留下的“已置零”当成这次已经做完。 */
        for (axis = 0U; axis < VB_AXIS_COUNT; ++axis) /* 12台电机的软件确认态全部重置。 */
        {
            s_runtime[axis].waiting_state_ack = 0U; /* 取消所有“等确认”。 */
            s_runtime[axis].waiting_mode_ack = 0U;
            s_runtime[axis].mode_confirmed = 0U; /* 模式确认作废：复位之后模式不一定还生效，必须重新确认。 */
            s_runtime[axis].sent_state = MCU_MODE_UNKNOWN; /* 我们不再假定它们的使能状态，当作“未知”。 */
            s_runtime[axis].sent_mode = MCU_MODE_UNKNOWN;
        }
        s_reset_active = 1U; /* 正式进入复位流程。 */
    }
    /* 复位 */
    if (s_reset_active) /* 本周期正在复位：推进复位流程，不做普通运动。 */
    {
        MergedRunResetTransaction(feedback); /* 每次只推进一步。 */
        s_extra_traffic = 1U; /* 复位本身已经在用CAN，本周期不再额外轮询。 */
    }
    /*跑！*/
    else /* 没有复位：8台电机各自走一遍状态机，该断电的断电、该发目标的发目标。 */
    {
        for (axis = 0U; axis < VB_AXIS_COUNT; ++axis)
            MergedExecuteAxis(axis, &command, &feedback[axis],
                feedback[VB_ARM1].status.reset_pos_ok_flag); /* 把ARM1的真实复位状态也传进去，ARM2的判断要用。 */
    }

    /* 复位复位好了没 */
    /* 活跃事务优先轮询当前选中轴，旧位必须等本事务新0x51真实回报才认可。 */
    /* 白话：“轮询”就是主动去问一句“你复位好了没”。平时（没在复位）我们轮流问八台，
     * 这样反馈里的复位位一直是新的；正在复位时只问当前那台，别浪费带宽。 */
    if (s_reset_active) /* 正在复位：只关心当前这台。 */
    {
        while (s_reset_axis < VB_AXIS_COUNT && !MergedResetAxisSelected(s_reset_axis))
            ++s_reset_axis; /* 跳过本次不复位的电机。 */
        if (s_reset_axis < VB_AXIS_COUNT && s_reset_stage == MCU_RESET_STAGE_WAIT_ACK)
            (void)IfResetOkFlag(MergedAxisBus(s_reset_axis), PTP_MODE, s_driver_id[s_reset_axis]); /* 正在等这台复位完成：主动问它一句。 */
    }
    else if (!s_extra_traffic && IfResetOkFlag(MergedAxisBus(s_reset_poll_axis), PTP_MODE, s_driver_id[s_reset_poll_axis]) == HAL_OK) /* 平时：本周期没别的流量，才去问下一台。 */
    {
        s_reset_poll_axis = (uint8_t)((s_reset_poll_axis + 1U) % VB_AXIS_COUNT); /* 问成功了就换下一台，八台轮流问。 */
    }
    /* 重新刷新一次反馈快照：因为刚才发命令、问状态之后，新的回报可能已经到了。 */
    taskENTER_CRITICAL();
    memcpy(feedback, s_feedback, sizeof(feedback));
    taskEXIT_CRITICAL();
    MergedSendFeedback(feedback, &command); /* 把最新的真实情况打包回传给小电脑。 */
}
