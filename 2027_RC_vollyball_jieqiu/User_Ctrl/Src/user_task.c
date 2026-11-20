#include "user_task.h"
#include "user_control.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_usart.h"
#include "vofa.h"
#include "zlr_math.h"

/* 驱动器回报帧 StdId 规则：StdId = 0x280 + DriverNum */
/* 函数：将反馈帧标准 ID 转换为驱动器编号；肘腕直接取 ID，其余按基址偏移，非法范围返回错误。 */
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
		const uint32_t dn = stdId - DRIVER_CLIENT_BASE_ID;
		if ((dn < 1U) || (dn > 64U))
		{
			return -1;
		}
		*outDriverNum = (uint8_t)dn;
		return 0;
	}
}

/* RXtask 上下文解析：FDCAN1 / FDCAN3 解析函数不同也没问题，在这里分发即可 */
/* 回调：在 FDCAN 接收任务上下文解析腿部反馈；当前仅 FDCAN_WL 的 LEG1-LEG4 分支启用，轮和 FDCAN_TK 解析保持屏蔽。 */
static void User_FDCAN_RxTaskParse(const UserFDCAN_RxMsg_t *msg)
{
	if ((msg == NULL) || (msg->bus == NULL))
	{
		return;
	}

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
/* 函数：将毫秒超时转换为 RTOS Tick；零超时不等待，0xFFFFFFFF 表示永久等待。 */
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

/* 函数：按调用次数产生指定时长的蜂鸣开关节拍；返回 1 表示蜂鸣周期结束。 */
uint8_t Beep(uint32_t time)
{
	static uint8_t flag = 0;
	static uint32_t lastTime = 0;
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
/* 函数：设置蜂鸣剩余节拍数；调用后由周期任务执行 Beep_Control 消耗。 */
void SetBeep(uint32_t time){
    Beep_Time = time;
}
/* 函数：根据 Beep_Time 控制蜂鸣器 GPIO；当前由用户任务周期调用。 */
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
/* 函数：运行成功后提示蜂鸣两次；当前函数保留，主任务调用点已注释。 */
void Start_Info(void)
{
	static uint8_t i = 0;
	if (i < 2 && gRobot.runSucceedFlag)
	{
		if (Beep(100) == 1) i++;		//定时器响两声表示上位机运行正常
	}
}

/**
 * @brief 初始化
 */
/* 函数：启动 UART、FDCAN、USB 及用户控制任务；系统启动时调用一次。 */
void User_Task_Init(void)
{
	/* UART8：TX 用 VOFA，RX 用 DMA+IDLE */
	(void)User_UART8_RxDmaStart();
	
	// FDCAN 初始化
	FDCAN_ENABLE();
	// 光电串口初始化
	USART_ENABLE();
	// USB通信初始化
	User_Communication_Init();

	// 注册FDCAN回调
	User_FDCAN_RegisterRxCallback(User_FDCAN_RxTaskParse);
	// 注册USB回调：在 UsbRxTask 上下文解析PC帧（GetPCMsg 当前已注释，恢复时一并打开）
	// UsbRx_RegisterCallback(GetPCMsg);

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
		const osThreadAttr_t userTask_attributes = {
			.name = "user_task",
			.stack_size = 1024U * 4U,
			.priority = osPriorityAboveNormal
		};
		user_task = osThreadNew(User_Task, NULL, &userTask_attributes);
	}
}

/**
 * @brief 主任务
 * @param argument 任务参数
 */
/* 函数：每 5 ms 执行控制周期任务；当前负责复位、腿部目标更新及调试输出。 */
void User_Task(void *argument)
{
	(void)argument;
	user_task_handle = xTaskGetCurrentTaskHandle();

	/* last_wake/period 用于固定 5 ms 周期；cnt/colorcnt 为循环内调试节拍计数。 */
	TickType_t last_wake = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(5);

	uint32_t cnt = 0;
	uint32_t colorcnt = 0;


	/* reset_flag、tarVel 和 lastTarPos1 保存跨周期控制状态；是否参与当前分支由下面的任务逻辑决定。 */
	static bool reset_flag = false;
	static float tarVel = 0;
	static float lastTarPos1;

	for (;;)
	{
		vTaskDelayUntil(&last_wake, period);

		/* 蜂鸣器：每个 5ms 节拍都要执行一次，Beep_Time 才会递减到 0 */
		Beep_Control();

		/* 这里写每 5ms 需要执行的逻辑 */

		// 心跳包自增
		//HB_Add();

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
					// SetDriverState(&hfdcan3, PTP_MODE, LEG1, DISABLE);
					// SetDriverState(&hfdcan3, PTP_MODE, LEG2, DISABLE);
					// SetDriverState(&hfdcan3, PTP_MODE, LEG3, DISABLE);
					// SetDriverState(&hfdcan3, PTP_MODE, LEG4, DISABLE);
				}
				continue;
			} // 复位完成后才执行正常控制逻辑
			
			static bool change_flag = 0;
			if(!change_flag)
			{	
				SetDriverMode(&hfdcan3, PTP_MODE , LEG1 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG2 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG3 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG4 , POS_MODE ,0);

				
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
			DEBUG("%f  %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %d\r\n" , -leg1Tar.pos,
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

/* 函数：以梯形速度曲线生成四腿位置目标；当前主任务未调用此旧实现。 */
void gethit_pos()
{
	static int stage = 0;
	static float tarvel = 1.0f;
	static float vel = 0;//速度
	static float dt = 0.005;//周期
	static float POS = 0;//当前路程
	static float s1 = 0.33;//加速段路程
	static float s2 = 0.34;//匀速段路程
	static float s3 = 0.33;//减速段
	static float a1 = 0;//加速度
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

/* 函数：按角度域规划计算四腿位置与位置增益；当前存在同名重复定义，须由工程维护者确认意图。 */
void getHitPos()
{
	static float pos = 0.0f;
	
	static double dt = 0.005;//周期
	static double t = 0.0;
	static double t_total;
	static bool initialized = false;
	if (!initialized)
    { 
        // init_motion_profile();
		init_theta_profile();
        t_total = tp.t_acc + tp.t_const + tp.t_dec;
        initialized = true;
    }
	// double revs = get_motor_revs_at_time(t);
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

/* 函数：按角度域规划计算速度与 MIT 增益；该同名定义与前一函数冲突，当前调用状态不确定。 */
void getHitPos()
{
	static float pos = 0.0f;
	static float vel = 0.0f;
	static double dt = 0.005;//周期
	static double t = 0.0;
	static double t_total;
	static bool initialized = false;
	if (!initialized)
    { 
        // init_motion_profile();
		init_theta_profile();
        t_total = tp.t_acc + tp.t_const + tp.t_dec;
        initialized = true;
    }
	// double revs = get_motor_revs_at_time(t);
	double revs = get_motor_revs_theta_plan(t);
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
/* 函数：限幅并向四腿下发位置目标；当前主任务调用。 */
void Leg_PosCtrl(float p1, float p2, float p3, float p4)
{
	static float TAR_LIMIT = 1.05f;
	bool safe_flag = true;
	static int LEGS[4] = {LEG1, LEG2, LEG3, LEG4};
	float LegsPos[4] = {p1, p2, p3, p4};

	static float DLimitPos = 5.0f / 360.f;

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
/* 函数：限幅四腿位置与速度并下发 PVT 控制；当前声明供上层动作流程使用。 */
void LegPVTCtrl(float p1, float p2, float p3, float p4)
{
	const float TAR_LIMIT = 1.05f;
	const float VEL_LIMIT = 1.05f;
	const float RAD_PER_REV = 2.0f * PI;
	bool safe_flag = true;
	static const uint8_t LEGS[4] = {LEG1, LEG2, LEG3, LEG4};
	float LegsPos[4] = {p1, p2, p3, p4};
	float LegsVel[4] = {leg1Tar.vel, leg2Tar.vel, leg3Tar.vel, leg4Tar.vel};

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