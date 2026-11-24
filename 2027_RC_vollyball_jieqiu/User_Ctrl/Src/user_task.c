/* 文件作用：启动用户控制外设和5ms主任务，分发CAN反馈并运行合并执行入口。
 * 当前由小电脑选择唯一动作类别；无PC命令时默认失能，不自动接球或发球。
 * 原接球实验函数和流程保留供核对，主循环continue后的历史流程当前不执行。
 */
#include "user_task.h"
#include "user_control.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_usart.h"
#include "vofa.h"
#include "zlr_math.h"
#include "merged_control.h"

/* 驱动器回报帧 StdId 规则：StdId = 0x280 + DriverNum */
/* 中文名：回报标识转驱动器号；作用：读取标准帧标识 stdId，肘腕标识直接取值，其余扣除回报基址；向 outDriverNum 输出 1～64 号，成功返回 0，空输出指针或越界返回 -1；仅供下方保留的历史解析使用。 */
static int Fdcan_ReportStdIdToDriverNum(uint32_t stdId, uint8_t *outDriverNum)
{
    if (outDriverNum == NULL)
    {
        return -1;
    }

    if (stdId == ELBOW || stdId == WRIST)
    {
        /* dn 是待输出的驱动器编号；仅接受协议支持的 1-64 范围。 */
        const uint32_t dn = stdId;
        if ((dn < 1U) || (dn > 64U))
        {
            return -1;
        }
        *outDriverNum = (uint8_t)dn;
        return 0;
    }
    else
    {
        /* dn：扣除客户端回报基址后的驱动器号，校验后写入输出指针。 */
        const uint32_t dn = stdId - DRIVER_CLIENT_BASE_ID;
        if ((dn < 1U) || (dn > 64U))
        {
            return -1;
        }
        *outDriverNum = (uint8_t)dn;
        return 0;
    }
}

/* 中文名：分发用户CAN接收反馈。
 * 作用：在FDCAN接收任务中将msg交给八轴反馈解析入口；当前仅处理CAN3驱动器，
 * msg携带总线、标识和数据；空消息直接返回；更新合并层反馈快照，无返回值。
 * 下方原接球解析保留但不执行，不在本回调计算轨迹或发送目标。
 */
static void User_FDCAN_RxTaskParse(const UserFDCAN_RxMsg_t *msg)
{
    if ((msg == NULL) || (msg->bus == NULL))
    {
        return;
    }

    /* 合并版由八轴适配层按CAN3/驱动器ID分发；CAN1只预留未来舵轮。
     * 接收任务只更新实测快照，不在此计算或推进轨迹。
     */
    MergedControl_OnCanFrame(msg->bus, msg->header.Identifier, msg->data, msg->data_len);
    return;

    /* 以下保留原接球解析源码作为历史参考，不再重复解析同一帧。 */
    /* dn 是由回报帧标识解出的驱动器号，供总线分发 switch 使用。 */
    uint8_t dn = 0U;
    if (Fdcan_ReportStdIdToDriverNum(msg->header.Identifier, &dn) != 0)
    {
        return;
    }

    /* frame_num 按每个 8 字节子帧计数；msg_data 指向当前子帧起始位置。 */
    int frame_num = msg->data_len / 8;
    for (int i = 0; i < frame_num; i++)
    {
        const uint8_t *msg_data = msg->data + i * 8;
        /* FDCAN1: 上层 CAN（UP_FDCAN）解析：TURN/ROLLER */
        if (msg->bus == &FDCAN_WL)
        {
            switch (dn)
            {
                // case WHEEL1:
                // {
                // 	GetDriverMsg(&wheel1Msg, msg_data);
                // 	HB.wheel1 = 0;	// 收到消息就把对应心跳包清零
                // 	break;
                // }
                // case WHEEL2:
                // {
                // 	GetDriverMsg(&wheel2Msg, msg_data);
                // 	HB.wheel2 = 0;
                // 	break;
                // }
                // case WHEEL3:
                // {
                // 	GetDriverMsg(&wheel3Msg, msg_data);
                // 	HB.wheel3 = 0;
                // 	break;
                // }
                // case WHEEL4:
                // {
                // 	GetDriverMsg(&wheel4Msg, msg_data);
                // 	HB.wheel4 = 0;
                // 	break;
                // }
                case LEG1:
                {
                    GetDriverMsg(&leg1Msg, msg_data);
                    HB.leg1 = 0;
                    break;
                }
                case LEG2:
                {
                    GetDriverMsg(&leg2Msg, msg_data);
                    HB.leg2 = 0;
                    break;
                }
                case LEG3:
                {
                    GetDriverMsg(&leg3Msg, msg_data);
                    HB.leg3 = 0;
                    break;
                }
                case LEG4:
                {
                    GetDriverMsg(&leg4Msg, msg_data);
                    HB.leg4 = 0;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        /* FDCAN3: 轮子 CAN（WHEEL_FDCAN）解析：ONE/TWO/THR */
        // if (msg->bus == &FDCAN_TK)
        // {
        // 	switch (dn)
        // 	{
        // 		case LEG2:
        // 		{
        // 			GetDriverMsg(&leg2Msg, msg_data);
        // 			HB.leg2 = 0;
        // 			break;
        // 		}
        // 		case SHOULDER:
        // 		{
        // 			GetDriverMsg(&shoulderMsg, msg_data);
        // 			HB.shoulder = 0;
        // 			break;
        // 		}
        // 		case ELBOW:
        // 		{
        // 			DMGetMotorMsg(&elbowMsg, msg_data);
        // 			HB.elbow = 0;
        // 			break;
        // 		}
        // 		case WRIST:
        // 		{
        // 			DMGetMotorMsg(&wristMsg, msg_data);
        // 			HB.wrist = 0;
        // 			break;
        // 		}
        // 		default:
        // 		{
        // 			break;
        // 		}
        // 	}
        // }
    }

}

/* 全局句柄：CMSIS 用户任务 ID；User_Task_Init 成功创建后有效。 */
osThreadId_t user_task = NULL;
/* 全局句柄：FreeRTOS 用户任务句柄；任务入口启动后写入，供任务通知等用途。 */
TaskHandle_t user_task_handle = NULL;

/**
 * @brief 将毫秒转换为系统节拍数
 * @param timeout_ms 超时时间，单位毫秒
 * @retval TickType_t 对应的系统节拍数
 */
/* 中文名：毫秒转系统节拍；作用：将 timeout_ms 毫秒换算为 Tick 返回，0 不等待，0xFFFFFFFF 返回 portMAX_DELAY；不修改共享状态。 */
TickType_t ms_to_ticks(uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
    {
        return 0;
    }
    if (timeout_ms == 0xFFFFFFFFU)
    {
        return portMAX_DELAY;
    }
    return pdMS_TO_TICKS(timeout_ms);
}

/* 中文名：推进一轮蜂鸣节拍；作用：time 是每段持续的调用次数，先鸣响 time 次再静音 time 次；改变 time 会重启计数；修改蜂鸣 GPIO 和静态计数，未完成返回 0，完成返回 1。 */
uint8_t Beep(uint32_t time)
{
    /* flag：本轮蜂鸣是否已开始；为 0 时首次调用重置累计节拍。 */
    static uint8_t flag = 0;
    /* lastTime：上一轮传入的段长（调用次数），用于识别时长变更。 */
    static uint32_t lastTime = 0;
    /* beep_Cnt：本轮已推进的调用次数，覆盖鸣响段与静音段。 */
    static uint32_t beep_Cnt = 0;

    if (lastTime != time) flag = 0;//如果跟上次不一样，重新计时

    lastTime = time;

    if (!flag)//判断是否为第一次进入
    {
        flag = 1;
        beep_Cnt = 0;//清零计时器
    }

    if (beep_Cnt < time) BEEP_ON;
    else if (beep_Cnt < time * 2) BEEP_OFF;
    else
    {
        BEEP_OFF;
        flag = 0;//清零等待下一次进入
        return 1;//响完一轮返回1
    }

    beep_Cnt++;
    return 0;
}

/* 全局变量：待执行蜂鸣的剩余节拍计数；Beep_Control 每次调用递减。 */
uint32_t Beep_Time = 0;
/* 中文名：设置持续蜂鸣时长；作用：将 time 个调用节拍写入 Beep_Time，由每 5ms 调用一次的 Beep_Control 消耗；无返回值，time 为 0 时下一节拍关闭蜂鸣。 */
void SetBeep(uint32_t time)
{
    Beep_Time = time;
}
/* 中文名：执行持续蜂鸣倒计时；作用：无参数，Beep_Time 非零时打开蜂鸣 GPIO 并减一，否则关闭蜂鸣；无返回值，由主任务每 5ms 调用。 */
void Beep_Control(void)
{
    if(Beep_Time > 0)
    {
        BEEP_ON;
        Beep_Time --;
    }
    else
    {
        BEEP_OFF;
    }
}
/* 中文名：运行成功双响提示；作用：无参数，仅 gRobot.runSucceedFlag 有效时推进 Beep(100)，累计完成两轮后停止提示；修改 GPIO 和静态轮数，无返回值，当前主任务调用已注释。 */
void Start_Info(void)
{
    /* i：运行成功提示已完成的双段蜂鸣轮数，累计到两轮后不再推进。 */
    static uint8_t i = 0;
    if (i < 2 && gRobot.runSucceedFlag)
    {
        if (Beep(100) == 1) i++;		//定时器响两声表示上位机运行正常
    }
}

/* 中文名：初始化用户外设与控制任务。
 * 作用：开机启动UART、CAN和USB接收，清零合并控制软件状态并创建5ms主任务。
 * 无参数、无返回值；默认不选择接球、发球或测试，不自动复位或使能电机；保留原PWM和蜂鸣初始化。
 */
void User_Task_Init(void)
{
    /* UART8：TX 用 VOFA，RX 用 DMA+IDLE */
    (void)User_UART8_RxDmaStart();

    /* 先清零命令/反馈软件状态，再打开总线接收，避免初始化与回调竞争。 */
    MergedControl_Init();
    // FDCAN 初始化
    FDCAN_ENABLE();
    // 光电串口初始化
    USART_ENABLE();
    // USB通信初始化
    User_Communication_Init();

    // 注册FDCAN回调
    User_FDCAN_RegisterRxCallback(User_FDCAN_RxTaskParse);
    /* USB接收任务只拼帧并更新命令快照；CAN目标由5ms主任务统一执行。 */
    UsbRx_RegisterCallback(GetPCMsg);

    // 设置驱动器初始状态
    RobotInit();

    SendPs();

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1010);

    valve_num[12] = 1;//灯带常亮

    BEEP_ON;
    osDelay(500);
    BEEP_OFF;
    osDelay(2000);
    BEEP_ON;
    osDelay(500);
    BEEP_OFF;

    /* 开启主线程 */
    if ((user_task == NULL) && (user_task_handle == NULL))
    {
        /* userTask_attributes：用户任务名称、栈字节数和调度优先级，供 osThreadNew 创建任务。 */
        const osThreadAttr_t userTask_attributes =
        {
            .name = "user_task",
            .stack_size = 1024U * 4U,
            .priority = osPriorityAboveNormal
        }
        ;
        user_task = osThreadNew(User_Task, NULL, &userTask_attributes);
    }
}

/* 中文名：用户五毫秒控制主任务。
 * 作用：每5ms更新蜂鸣状态并调用MergedControl_Tick处理PC命令；argument未使用。
 * 只开主控时PC命令为零，八轴只请求失能并读取状态，不自动接球、发球或回零。
 * 任务常驻循环不返回；continue隔离下方原接球实验流程；原流程仅保留作参考，当前不会执行。
 */
void User_Task(void *argument)
{
    (void)argument;
    user_task_handle = xTaskGetCurrentTaskHandle();

    /* last_wake：上一次周期唤醒的系统 Tick；由 vTaskDelayUntil 更新，避免相对延时累计漂移。 */
    TickType_t last_wake = xTaskGetTickCount();
    /* period：5ms 对应的系统 Tick 数，作为主任务固定周期。 */
    const TickType_t period = pdMS_TO_TICKS(5);

    /* cnt/colorcnt：历史循环的调试/颜色节拍计数，递增点在 continue 后，当前保持初值。 */
    uint32_t cnt = 0;
    uint32_t colorcnt = 0;


    /* 历史状态：reset_flag 记录回零完成；tarVel 为腿1目标差分速度（圈/s），lastTarPos1 为前次目标（圈）。
     * 使用点均位于 continue 后，当前控制入口不读取或更新这些状态。 */
    static bool reset_flag = false;
    static float tarVel = 0;
    static float lastTarPos1;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, period);

        /* 蜂鸣器：每个 5ms 节拍都要执行一次，Beep_Time 才会递减到 0 */
        Beep_Control();

        /* 合并版只执行小电脑本周期目标并回传真实反馈，不在主控推进接球轨迹。
         * 原实验流程保留在下方源码作核对，continue明确隔离其CAN输出。
         */
        MergedControl_Tick();
        continue;

        /* 这里写每 5ms 需要执行的逻辑 */

        // 心跳包自增
        // HB_Add();
        static int cntTest = 0;
        cntTest++;

        // if(gRobot.runSucceedFlag)
        // {
        // 	// 复位
        // DEBUG("%f\r\n", );
        if(!reset_flag)
        {
            // SetDriverState(&hfdcan3, PTP_MODE, LEG1, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG2, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG3, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG4, ENABLE);

            // SetBeep(100);

            reset_flag = reset_control();

            if(reset_flag)
            {
                /*如果先复位等很久再接就开*/
                // SetDriverState(&hfdcan3, PTP_MODE, LEG1, DISABLE);
                // SetDriverState(&hfdcan3, PTP_MODE, LEG2, DISABLE);
                // SetDriverState(&hfdcan3, PTP_MODE, LEG3, DISABLE);
                // SetDriverState(&hfdcan3, PTP_MODE, LEG4, DISABLE);
            }
            continue;
        }
        // 复位完成后才执行正常控制逻辑

        static bool change_flag = 0;
        if(!change_flag)
        {
            SetDriverMode(&hfdcan3, PTP_MODE , LEG1 , POS_MODE ,0);
            SetDriverMode(&hfdcan3, PTP_MODE , LEG2 , POS_MODE ,0);
            SetDriverMode(&hfdcan3, PTP_MODE , LEG3 , POS_MODE ,0);
            SetDriverMode(&hfdcan3, PTP_MODE , LEG4 , POS_MODE ,0);

            /*如果先复位等很久再接就开*/
            // SetDriverState(&hfdcan3, PTP_MODE, LEG1, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG2, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG3, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, LEG4, ENABLE);
            change_flag = 1;
            continue;
        }
        static bool waiting_flag = 0;
        static uint32_t waiting_cnt = 0;
        if(!waiting_flag)
        {
            waiting_cnt++;
            if(waiting_cnt > 100)
            {
                waiting_flag = 1;
            }
        }
        else
        {
            // gethit_pos();
            getHitPos();
            Leg_PosCtrl(leg1Tar.pos, leg2Tar.pos, leg3Tar.pos, leg4Tar.pos);
        }
        // PosCtrl(&FDCAN_WL, PTP_MODE, LEG1, (int32_t)(leg1Tar.pos * 4096.f), 2.0, 0.15);
        // PosCtrl(&FDCAN_WL, PTP_MODE, LEG2, (int32_t)(leg2Tar.pos * 4096.f), 2.0, 0.15);
        // PosCtrl(&FDCAN_WL, PTP_MODE, LEG3, (int32_t)(leg3Tar.pos * 4096.f), 2.0, 0.15);
        // PosCtrl(&FDCAN_WL, PTP_MODE, LEG4, (int32_t)(leg4Tar.pos * 4096.f), 2.0, 0.15);

        tarVel = (lastTarPos1 - leg1Tar.pos) / 0.005;
        lastTarPos1 = leg1Tar.pos;
        DEBUG("%d %f  %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %d\r\n" , cntTest, -leg1Tar.pos,
            -leg1Msg.pos, leg2Msg.pos, -leg3Msg.pos, leg4Msg.pos,
            tarVel,-leg1Msg.vel, leg2Msg.vel, -leg3Msg.vel,
            leg4Msg.vel, leg1Msg.currQ, leg2Msg.currQ, leg3Msg.currQ, leg4Msg.currQ,
            leg1Msg.error, leg2Msg.error, leg3Msg.error, leg4Msg.error);




        // static float LEGTarPos = 0.76;
        // static float dpos = 0.002f;

        // static float DLimitPos = 5.f / 360.f;

        // static int tim_cnt1 = 0;
        // static int LEGs[4] = {LEG1, LEG2, LEG3, LEG4};
        // static float LEGsPos[4] = {0, 0, 0, 0};
        // static bool end_flag = 0;

        // tim_cnt1++;

        // if (tim_cnt1 >= 150)
        // {
        // 	if(fabsf((LEGsPos[1] - LEGTarPos)) == 0.f)
        // 	{
        // 		for(int i = 0; i < 4; i++)
        // 		{
        // 			SetDriverState(&hfdcan3, PTP_MODE, LEGs[i], DISABLE);
        // 		}
        // 		end_flag = 1;
        // 	}

        // 		LEGsPos[0] -= dpos;
        // 		LEGsPos[1] += dpos;
        // 		LEGsPos[2] -= dpos;
        // 		LEGsPos[3] += dpos;

        // 	if(end_flag == 1)
        // 	{
        // 		LEGsPos[0] = -LEGTarPos;
        // 		LEGsPos[1] = LEGTarPos;
        // 		LEGsPos[2] = -LEGTarPos;
        // 		LEGsPos[3] = LEGTarPos;
        // 	}

        // 	if(fabsf(leg1Msg.pos - leg3Msg.pos) > DLimitPos)
        // 	{
        // 		SetDriverState(&hfdcan3, PTP_MODE, LEG1, DISABLE);
        // 	}
        // 	else if(fabsf(leg2Msg.pos - leg4Msg.pos) > DLimitPos)
        // 	{
        // 		SetDriverState(&hfdcan3, PTP_MODE, LEG2, DISABLE);
        // 	}

        // 	for(int i = 0; i < 4; i++)
        // 	{
        // 		PosCtrl(&FDCAN_WL, PTP_MODE, LEGs[i], (int32_t)(LEGsPos[i] * 4096.f), 2.0, 0.15);
        // 	}

        // }
        // else
        // {
        // 	SetDriverState(&hfdcan3, PTP_MODE, WHEEL2, ENABLE);
        // 	SetDriverState(&hfdcan3, PTP_MODE, WHEEL3, ENABLE);
        // }


        //DEBUG("%f, %f, %f, %f, %f, %f, %f, %f\r\n" , LEGsPos[0], LEGsPos[1], LEGsPos[2], LEGsPos[3], leg1Msg.pos, leg2Msg.pos, leg3Msg.pos, leg4Msg.pos);

        /* 机器人控制 */
        // gRobot.reset_flag = 1;
        // Start_Info();
        // RobotControl();


        // RobotControl();

        // DEBUG("%d, %d, %d, %d\r\n", wheel1Tar.clean, wheel2Tar.clean, wheel3Tar.clean, wheel4Tar.clean);

        // static int cnt = 0;
        // if (cnt == 0)
        // {
        // 	SetDriverMode(&FDCAN_WL, PTP_MODE, LEG4, POS_MODE, (int32_t)(cnt * 4096.f));
        // }
        // cnt++;
        // SetStateAndPosCtrl(&FDCAN_WL, PTP_MODE, LEG4, ENABLE, (int32_t)(cnt * 4096.f));

        // valve_num[0] = 1;
        // valve_num[5] = 1;
        // SendValve();
        // valve_num[3] = 1;
        // static int valve_cnt = 0;
        // valve_cnt++;
        // if (valve_cnt < 25)
        // {
        // 	valve_num[9] = 1;
        // }
        // else if (valve_cnt < 50)
        // {
        // 	valve_num[9] = 0;
        // }
        // else
        // {
        // 	valve_cnt = 0;
        // }
        // SendValve();

        // MotorControl();

        // static int flag = 0;
        // if(flag == 0)
        // {
        // 	leg4Tar.mode = 0;
        // 	flag = 1;
        // }
        // else if (flag == 1)
        // {
        // 	leg4Tar.mode = 2;
        // 	flag = 0;
        // }

        // DEBUG("%d\r\n", leg4Msg.enableState);
        // DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num[0], ps_num[1], ps_num[2], ps_num[3], ps_num[4], ps_num[5], ps_num[6], ps_num[7], ps_num[8], ps_num[9], ps_num[10], ps_num[11], ps_num[12], ps_num[13], ps_num[14], ps_num[15]);
        // DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num_2[0], ps_num_2[1], ps_num_2[2], ps_num_2[3], ps_num_2[4], ps_num_2[5], ps_num_2[6], ps_num_2[7], ps_num_2[8], ps_num_2[9], ps_num_2[10], ps_num_2[11], ps_num_2[12], ps_num_2[13], ps_num_2[14], ps_num_2[15]);
        // DMEnable(&FDCAN_TK, ELBOW, DM_POS_MODE);
        // DMSetPosZero(&FDCAN_TK, ELBOW, DM_POS_MODE);

        // DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
        // DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);

        // static int flag = 0;
        // if(flag == 0)
        // {
        // 	SetDriverZero(&FDCAN_TK,PTP_MODE,SHOULDER);
        // 	flag = 1;
        // }
        // else
        // {
        // 	ReadPVT(&FDCAN_TK, PTP_MODE, SHOULDER);
        // }

        // DEBUG("%f, %f, %f\r\n", shoulderMsg.pos, elbowMsg.pos, wristMsg.pos);

        // static int tim_cnt = 0;
        // tim_cnt++;
        // if (tim_cnt <= 11)
        // {
        // 	valve_num[10] = 1;
        // }
        // else if (tim_cnt <= 22)
        // {
        // 	valve_num[10] = 0;
        // }
        // else
        // {
        // 	tim_cnt = 0;
        // }
        // SendValve();

        // DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);

        // DEBUG("%f\r\n", wristMsg.pos);

        // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1500);

        // static int tim_cnt = 0;
        // if (tim_cnt == 0)
        // {
        // 	// DMEnable(&FDCAN_TK, ELBOW, DM_POS_MODE);
        // 	DMEnable(&FDCAN_TK, WRIST, DM_POS_MODE);
        // 	tim_cnt = 1;
        // }
        // // else if (tim_cnt == 1)
        // // {
        // // 	// DMPosCtrl(&FDCAN_TK, ELBOW, 0, 0.5);
        // // 	DMPosCtrl(&FDCAN_TK, WRIST, 0, 0.5);
        // // 	if (fabs(elbowMsg.pos) < 0.05 && fabs(wristMsg.pos) < 0.05)
        // // 	{
        // // 		tim_cnt = 2;
        // // 	}
        // // }
        // // else
        // // {
        // // 	DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);
        // // 	DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
        // // }
        // else if (tim_cnt <= 10)
        // {
        // 	tim_cnt++;
        // 	// DMSetPosZero(&FDCAN_TK, ELBOW, DM_POS_MODE);
        // 	DMSetPosZero(&FDCAN_TK, WRIST, DM_POS_MODE);
        // }
        // else
        // {
        // 	DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);
        // 	DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
        // }

        /* 获取驱动器最新消息 */
        User_Control_GetDriverMsgsSnapshot(&driverMsgs);
        // SendPCMsg();

        cnt++;
        colorcnt++;
        cnt %= 200;
        colorcnt %= 1800;
    }
}

/* 中文名：历史四腿梯形位置规划；作用：无参数，按 0.005s 步长推进加速、匀速、减速阶段，写入四腿目标圈数及位置环增益；无返回值，当前活动主循环不调用。 */
void gethit_pos()
{
    /* stage：梯形规划阶段，0 加速、1 匀速、2 减速、3 结束。 */
    static int stage = 0;
    /* tarvel：历史梯形曲线目标匀速值，单位圈/s。 */
    static float tarvel = 1.0f;
    /* vel：梯形曲线当前积分速度，单位圈/s。 */
    static float vel = 0;//速度
    /* dt：旧规划每次调用使用的固定积分步长，单位 s。 */
    static float dt = 0.005;//周期
    /* POS：梯形曲线累计电机转数，单位圈。 */
    static float POS = 0;//当前路程
    /* s1：加速段目标转数，单位圈。 */
    static float s1 = 0.33;//加速段路程
    /* s2：匀速段目标转数，单位圈。 */
    static float s2 = 0.34;//匀速段路程
    /* s3：减速段目标转数，单位圈。 */
    static float s3 = 0.33;//减速段
    /* a1：由目标速度和加速段转数计算的加速度，单位圈/s²。 */
    static float a1 = 0;//加速度
    /* a2：由目标速度和减速段转数计算的减速度幅值，单位圈/s²。 */
    static float a2 = 0; //减速度
    a1 = tarvel * tarvel / 2 / s1;
    a2 = tarvel * tarvel / 2 / s3;

    switch(stage)
    {
        case 0:
        {
            vel += a1 * dt;
            leg1Tar.pos_kp = 1.5f;
            leg1Tar.pos_kd = 0.0005f;
            leg2Tar.pos_kp = 1.5f;
            leg2Tar.pos_kd = 0.0005f;
            leg3Tar.pos_kp = 1.5f;
            leg3Tar.pos_kd = 0.0005f;
            leg4Tar.pos_kp = 1.5f;
            leg4Tar.pos_kd = 0.0005f;

            POS += vel * dt;

            leg1Tar.pos = -POS - 0.020f;
            leg2Tar.pos = POS;
            leg3Tar.pos = -POS;
            leg4Tar.pos = POS;
            if(POS >= s1)
            {
                vel = tarvel;
                stage = 1;
            }
            break;
        }

        case 1:
        {
            vel = tarvel;
            leg1Tar.pos_kp = 1.5f;
            leg1Tar.pos_kd = 0.0005f;
            leg2Tar.pos_kp = 1.5f;
            leg2Tar.pos_kd = 0.0005f;
            leg3Tar.pos_kp = 1.5f;
            leg3Tar.pos_kd = 0.0005f;
            leg4Tar.pos_kp = 1.5f;
            leg4Tar.pos_kd = 0.0005f;

            POS += vel * dt;

            leg1Tar.pos = -POS - 0.025f;
            leg2Tar.pos = POS;
            leg3Tar.pos = -POS - 0.005f;
            leg4Tar.pos = POS;
            if(POS >= s1 + s2)
            {
                stage = 2;
            }
            break;
        }
        case 2:
        {
            leg1Tar.pos_kp = 1.5f;
            leg1Tar.pos_kd = 0.0005f;
            leg2Tar.pos_kp = 1.5f;
            leg2Tar.pos_kd = 0.0005f;
            leg3Tar.pos_kp = 1.5f;
            leg3Tar.pos_kd = 0.0005f;
            leg4Tar.pos_kp = 1.5f;
            leg4Tar.pos_kd = 0.0005f;

            POS += vel * dt;

            leg1Tar.pos = -POS - 0.030f;
            leg2Tar.pos = POS;
            leg3Tar.pos = -POS - 0.010f;
            leg4Tar.pos = POS;
            vel -= a2 * dt;
            if(POS > s1 + s2 + s3)
            {
                stage = 3;
            }
            break;
        }
        default:
        break;

    }

    // pos = pos + 0.005f;
    // if(pos >= 0.75f)
    // {
    // 	pos = 0.75f;
    // }
    // return pos;
}

/* 中文名：历史四腿角度域位置规划；作用：无参数，首次初始化角度域轨迹，按 0.005s 步长查询目标圈数并按阶段设置四腿位置环增益；更新 leg1Tar～leg4Tar，无返回值，仅保留流程中调用。 */
void getHitPos()
{
    /* pos：轨迹输出转为 float 后的电机目标圈数，按腿方向分配符号。 */
    static float pos = 0.0f;

    /* dt：角度域轨迹每次推进的时间步长，单位 s。 */
    static double dt = 0.005;//周期
    /* t：当前轨迹查询时刻，单位 s，跨调用累积。 */
    static double t = 0.0;
    /* t_total：初始化时汇总的三段总时长，单位 s，用于停止推进查询时刻。 */
    static double t_total;
    /* initialized：是否已初始化角度域轨迹及总时长，避免每次调用重新起算。 */
    static bool initialized = false;
    if (!initialized)
    {
        // init_motion_profile();
        init_theta_profile();
        t_total = tp.t_acc + tp.t_const + tp.t_dec;
        initialized = true;
    }
    // double revs = get_motor_revs_at_time(t);
    /* revs：当前时刻按角度域轨迹查询到的电机目标圈数。 */
    double revs = get_motor_revs_theta_plan(t);
    if ( t < t_total)
    {

        t += dt;
    }

    pos = (float) revs;
    leg1Tar.pos = -pos;
    leg2Tar.pos = pos;
    leg3Tar.pos = -pos;
    leg4Tar.pos = pos;
    if (t <= tp.t_acc)
    {
        // 加速段 5次多项式
        leg1Tar.pos_kp = 12.0f;
        leg1Tar.pos_kd = 0.0005f;
        leg2Tar.pos_kp = 11.8f;
        leg2Tar.pos_kd = 0.0005f;
        leg3Tar.pos_kp = 10.8f;
        leg3Tar.pos_kd = 0.0005f;
        leg4Tar.pos_kp = 11.5f;
        leg4Tar.pos_kd = 0.0005f;
    }
    else if (t <= tp.t_acc + tp.t_const)
    {
        // 匀速段
        leg1Tar.pos_kp = 10.5f;
        leg1Tar.pos_kd = 0.0005f;
        leg2Tar.pos_kp = 9.8f;
        leg2Tar.pos_kd = 0.0005f;
        leg3Tar.pos_kp = 10.0f;
        leg3Tar.pos_kd = 0.0005f;
        leg4Tar.pos_kp = 10.5f;
        leg4Tar.pos_kd = 0.0005f;
    }
    else if (t <= tp.t_acc + tp.t_const + tp.t_dec)
    {
        // 减速段 5次多项式
        leg1Tar.pos_kp = 9.0f;
        leg1Tar.pos_kd = 0.0005f;
        leg2Tar.pos_kp = 8.8f;
        leg2Tar.pos_kd = 0.0005f;
        leg3Tar.pos_kp = 9.0f;
        leg3Tar.pos_kd = 0.0005f;
        leg4Tar.pos_kp = 9.5f;
        leg4Tar.pos_kd = 0.0005f;
    }
    else
    {
        leg1Tar.pos_kp = 9.0f;
        leg1Tar.pos_kd = 0.0005f;
        leg2Tar.pos_kp = 8.8f;
        leg2Tar.pos_kd = 0.0005f;
        leg3Tar.pos_kp = 9.0f;
        leg3Tar.pos_kd = 0.0005f;
        leg4Tar.pos_kp = 9.5f;
        leg4Tar.pos_kd = 0.0005f;
    }
}

/* 中文名：历史四腿位置速度规划实验；作用：无参数，按 0.005s 步长查询圈数和圈每秒速度，设置 MIT 增益；现有代码先写位置再以速度覆盖四腿 .pos 字段，未写 .vel；无返回值，保留实验实现供核对。 */
void getHitPosPVT()
{
    /* pos：轨迹输出转为 float 后的电机目标圈数，按腿方向分配符号。 */
    static float pos = 0.0f;
    /* vel：轨迹速度转换结果，单位圈/s；本实验后续将其写入位置字段。 */
    static float vel = 0.0f;
    /* dt：角度域轨迹每次推进的时间步长，单位 s。 */
    static double dt = 0.005;//周期
    /* t：当前轨迹查询时刻，单位 s，跨调用累积。 */
    static double t = 0.0;
    /* t_total：初始化时汇总的三段总时长，单位 s，用于停止推进查询时刻。 */
    static double t_total;
    /* initialized：是否已初始化角度域轨迹及总时长，避免每次调用重新起算。 */
    static bool initialized = false;
    if (!initialized)
    {
        // init_motion_profile();
        init_theta_profile();
        t_total = tp.t_acc + tp.t_const + tp.t_dec;
        initialized = true;
    }
    // double revs = get_motor_revs_at_time(t);
    /* revs：当前时刻按角度域轨迹查询到的电机目标圈数。 */
    double revs = get_motor_revs_theta_plan(t);
    /* vel1：轨迹查询返回的电机速度，单位圈/s。 */
    double vel1 = get_motor_vel_at_time(t);

    // float tor =
    if ( t < t_total)
    {

        t += dt;
    }

    pos = (float) revs;
    leg1Tar.pos = -pos;
    leg2Tar.pos = pos;
    leg3Tar.pos = -pos;
    leg4Tar.pos = pos;
    vel = (float) vel1;
    leg1Tar.pos = -vel;
    leg2Tar.pos = vel;
    leg3Tar.pos = -vel;
    leg4Tar.pos = vel;
    if (t <= tp.t_acc)
    {
        // 加速段 5次多项式
        leg1Tar.mit_kp = 1.0f;
        leg1Tar.mit_kd = 1.0f;
        leg2Tar.mit_kp = 1.0f;
        leg2Tar.mit_kd = 1.0f;
        leg3Tar.mit_kp = 1.0f;
        leg3Tar.mit_kd = 1.0f;
        leg4Tar.mit_kp = 1.0f;
        leg4Tar.mit_kd = 1.0f;
    }
    else if (t <= tp.t_acc + tp.t_const)
    {
        // 匀速段
        leg1Tar.mit_kp = 1.0f;
        leg1Tar.mit_kd = 1.0f;
        leg2Tar.mit_kp = 1.0f;
        leg2Tar.mit_kd = 1.0f;
        leg3Tar.mit_kp = 1.0f;
        leg3Tar.mit_kd = 1.0f;
        leg4Tar.mit_kp = 1.0f;
        leg4Tar.mit_kd = 1.0f;
    }
    else if (t <= tp.t_acc + tp.t_const + tp.t_dec)
    {
        // 减速段 5次多项式
        leg1Tar.mit_kp = 1.0f;
        leg1Tar.mit_kd = 1.0f;
        leg2Tar.mit_kp = 1.0f;
        leg2Tar.mit_kd = 1.0f;
        leg3Tar.mit_kp = 1.0f;
        leg3Tar.mit_kd = 1.0f;
        leg4Tar.mit_kp = 1.0f;
        leg4Tar.mit_kd = 1.0f;
    }
    else
    {
        leg1Tar.mit_kp = 1.0f;
        leg1Tar.mit_kd = 1.0f;
        leg2Tar.mit_kp = 1.0f;
        leg2Tar.mit_kd = 1.0f;
        leg3Tar.mit_kp = 1.0f;
        leg3Tar.mit_kd = 1.0f;
        leg4Tar.mit_kp = 1.0f;
        leg4Tar.mit_kd = 1.0f;
    }
}
/* 中文名：历史四腿位置限幅下发；作用：p1～p4 为四腿目标圈数，限幅至正负 1.05 圈，乘 4096 转为位置计数后经 CAN3 下发，使用各腿位置环增益；无返回值，调用位于主循环 continue 后的保留流程。 */
void Leg_PosCtrl(float p1, float p2, float p3, float p4)
{
    /* TAR_LIMIT：位置指令绝对值上限，单位圈。 */
    static float TAR_LIMIT = 1.05f;
    /* safe_flag：允许下发目标的门控；成对位置差保护已注释，当前始终为真。 */
    bool safe_flag = true;
    /* LEGS：四腿驱动器编号表，与输入 p1～p4 顺序一致。 */
    static int LEGS[4] =
    {
        LEG1, LEG2, LEG3, LEG4
    }
    ;
    /* LegsPos：各腿位置指令的本次限幅副本，单位圈。 */
    float LegsPos[4] =
    {
        p1, p2, p3, p4
    }
    ;

    /* DLimitPos：历史同侧腿位置差阈值，5°折合圈数；仅已注释保护分支引用。 */
    static float DLimitPos = 5.0f / 360.f;

    /* i：本次执行限幅的腿数组下标，范围 0～3。 */
    for(int i = 0; i < 4; i++)
    {
        if(LegsPos[i] > TAR_LIMIT)
        {
            LegsPos[i] = TAR_LIMIT;
        }
        else if(LegsPos[i] < -TAR_LIMIT)
        {
            LegsPos[i] = -TAR_LIMIT;
        }
    }

    // if(fabsf(leg1Msg.pos - leg3Msg.pos) > DLimitPos)
    // {
    // 	SetDriverState(&hfdcan3, PTP_MODE, LEG1, DISABLE);
    // 	safe_flag = false;
    // }

    // if(fabsf(leg2Msg.pos - leg4Msg.pos) > DLimitPos)
    // {
    // 	SetDriverState(&hfdcan3, PTP_MODE, LEG2, DISABLE);
    // 	safe_flag = false;
    // }

    if(safe_flag)
    {
        PosCtrl(&hfdcan3, PTP_MODE, LEGS[0], (int32_t)(LegsPos[0] * 4096.f), leg1Tar.pos_kp , leg1Tar.pos_kd);
        PosCtrl(&hfdcan3, PTP_MODE, LEGS[1], (int32_t)(LegsPos[1] * 4096.f), leg2Tar.pos_kp , leg2Tar.pos_kd);
        PosCtrl(&hfdcan3, PTP_MODE, LEGS[2], (int32_t)(LegsPos[2] * 4096.f), leg3Tar.pos_kp , leg3Tar.pos_kd);
        PosCtrl(&hfdcan3, PTP_MODE, LEGS[3], (int32_t)(LegsPos[3] * 4096.f), leg4Tar.pos_kp , leg4Tar.pos_kd);
    }
}
/*注意限幅*/
/* 中文名：四腿PVT限幅下发；作用：p1～p4 为目标圈数，各腿目标速度取自 legNTar.vel（圈每秒）；分别限幅后乘 2π 转为 rad 与 rad/s，连同 MIT 增益和力矩经 CAN3 下发；无返回值，保留旧动作接口。 */
void LegPVTCtrl(float p1, float p2, float p3, float p4)
{
    /* TAR_LIMIT：PVT 位置限幅绝对值，单位圈。 */
    const float TAR_LIMIT = 1.05f;//安全
    /* VEL_LIMIT：旧实验速度限幅绝对值，单位圈/s；保留原设定。 */
    const float VEL_LIMIT = 1.05f;//随便设的
    /* RAD_PER_REV：每圈对应的弧度数，用于圈与 rad、圈/s 与 rad/s 换算。 */
    const float RAD_PER_REV = 2.0f * PI;//换算
    /* safe_flag：允许下发目标的门控；成对位置差保护已注释，当前始终为真。 */
    bool safe_flag = true;
    /* LEGS：按输入顺序排列的四腿驱动器号。 */
    static const uint8_t LEGS[4] =
    {
        LEG1, LEG2, LEG3, LEG4
    }
    ;
    /* LegsPos：各腿位置指令的本次限幅副本，单位圈。 */
    float LegsPos[4] =
    {
        p1, p2, p3, p4
    }
    ;
    /* LegsVel：从四腿目标结构读取的速度限幅副本，单位圈/s。 */
    float LegsVel[4] =
    {
        leg1Tar.vel, leg2Tar.vel, leg3Tar.vel, leg4Tar.vel
    }
    ;

    /* i：本次执行限幅的腿数组下标，范围 0～3。 */
    for(int i = 0; i < 4; i++)
    {
        if(LegsPos[i] > TAR_LIMIT)
        {
            LegsPos[i] = TAR_LIMIT;
        }
        else if(LegsPos[i] < -TAR_LIMIT)
        {
            LegsPos[i] = -TAR_LIMIT;
        }

        if(LegsVel[i] > VEL_LIMIT)
        {
            LegsVel[i] = VEL_LIMIT;
        }
        else if(LegsVel[i] < -VEL_LIMIT)
        {
            LegsVel[i] = -VEL_LIMIT;
        }
    }

    // if(fabsf(leg1Msg.pos - leg3Msg.pos) > DLimitPos)
    // {
    // 	SetDriverState(&hfdcan3, PTP_MODE, LEG1, DISABLE);
    // 	safe_flag = false;
    // }

    // if(fabsf(leg2Msg.pos - leg4Msg.pos) > DLimitPos)
    // {
    // 	SetDriverState(&hfdcan3, PTP_MODE, LEG2, DISABLE);
    // 	safe_flag = false;
    // }

    if(safe_flag)
    {
        PVTCtrl(&hfdcan3, LEGS[0], LegsPos[0] * RAD_PER_REV, LegsVel[0] * RAD_PER_REV, leg1Tar.mit_kp, leg1Tar.mit_kd, leg1Tar.tor);
        PVTCtrl(&hfdcan3, LEGS[1], LegsPos[1] * RAD_PER_REV, LegsVel[1] * RAD_PER_REV, leg2Tar.mit_kp, leg2Tar.mit_kd, leg2Tar.tor);
        PVTCtrl(&hfdcan3, LEGS[2], LegsPos[2] * RAD_PER_REV, LegsVel[2] * RAD_PER_REV, leg3Tar.mit_kp, leg3Tar.mit_kd, leg3Tar.tor);
        PVTCtrl(&hfdcan3, LEGS[3], LegsPos[3] * RAD_PER_REV, LegsVel[3] * RAD_PER_REV, leg4Tar.mit_kp, leg4Tar.mit_kd, leg4Tar.tor);
    }
}