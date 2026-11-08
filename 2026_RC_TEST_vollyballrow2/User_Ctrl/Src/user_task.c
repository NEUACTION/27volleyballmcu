#include "user_task.h"
#if WAY == 0

	/*单独测((要改.h))*/


	#include "user_task.h"
	#include "user_control.h"
	#include "motor_driver.h"
	#include "chassis_driver.h"
	#include "user_usart.h"
	#include "vofa.h"
	#include "adc.h"
	#include "math.h"

	bool camp_flag = true;//false红场，true蓝场
	bool star_receive_flag = false;
	uint8_t motor_reset_flag = 0;

	#define LASER_FILTER_SIZE 5  // 激光测距窗口大小
	int32_t laser_buffer[LASER_FILTER_SIZE];
	int32_t laserleft_buffer[LASER_FILTER_SIZE];
	MAFilter laser_filter = {laser_buffer, 0, 0, LASER_FILTER_SIZE, 0};
	MAFilter laserleft_filter = {laserleft_buffer, 0, 0, LASER_FILTER_SIZE, 0};
	void hsvctrl();

	/* ADC 中断回调缓存 */
	static volatile uint32_t adc1_cached = 0;
	static volatile uint32_t adc2_cached = 0;
	static volatile uint32_t adc3_cached = 0;

	volatile bool GetADC1Flag = false;
	volatile bool GetADC2Flag = false;
	volatile bool GetADC3Flag = false;

	int Beep_Time = 0;
	void SetBeep(uint32_t time){
		Beep_Time = time;
	}
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
	/* 驱动器回报帧 StdId 规则：StdId = 0x280 + DriverNum */
	static int Fdcan_ReportStdIdToDriverNum(uint32_t stdId, uint8_t *outDriverNum)
	{
		if (outDriverNum == NULL)
		{
			return -1;
		}
		
		// if (stdId == ELBOW || stdId == WRIST)
		// {
		// 	const uint32_t dn = stdId;
		// 	if ((dn < 1U) || (dn > 64U))
		// 	{
		// 		return -1;
		// 	}
		// 	*outDriverNum = (uint8_t)dn;
		// 	return 0;
		// }
		// else
		// {
			const uint32_t dn = stdId - DRIVER_CLIENT_BASE_ID;
			if ((dn < 1U) || (dn > 64U))
			{
				return -1;
			}
			*outDriverNum = (uint8_t)dn;
			return 0;
		// }
	}

	/* RXtask 上下文解析：FDCAN1 / FDCAN3 解析函数不同也没问题，在这里分发即可 */
	static void User_FDCAN_RxTaskParse(const UserFDCAN_RxMsg_t *msg)
	{
		if ((msg == NULL) || (msg->bus == NULL))
		{
			return;
		}

		uint8_t dn = 0U;
		if (Fdcan_ReportStdIdToDriverNum(msg->header.Identifier, &dn) != 0)
		{
			return;
		}

		int frame_num = msg->data_len / 8;
		for (int i = 0; i < frame_num; i++)
		{
			const uint8_t *msg_data = msg->data + i * 8;
			/* FDCAN1: 上层 CAN（UP_FDCAN）解析：TURN/ROLLER */
			if (msg->bus == &hfdcan1)
			{
				switch (dn)
				{

					default:
					{
						break;
					}
				}
			}
		
			/* FDCAN3: 轮子 CAN（WHEEL_FDCAN）解析：ONE/TWO/THR */
			if (msg->bus == &hfdcan3)
			{
				switch (dn)
				{
					case 1:
					{
						HB.arm1 = 0;
						
						armMsg1.HBStartFlag = 1;
						
						GetDriverMsg(&armMsg1, msg_data);
						
						break;
						
					}
					case 2:
					{
						HB.arm2 = 0;

						armMsg2.HBStartFlag = 1;

						GetDriverMsg(&armMsg2, msg_data);

						break;
					}
					case 3:
					{
						HB.plate = 0;

						plateMsg.HBStartFlag = 1;

						GetDriverMsg(&plateMsg, msg_data);

						break;
					}
					case 4:
					{
						HB.slide = 0;

						slideMsg.HBStartFlag = 1;

						GetDriverMsg(&slideMsg, msg_data);

						break;
					}
					default:
					{
						break;
					}
				}
			}
		}

	}

	osThreadId_t user_task = NULL;
	TaskHandle_t user_task_handle = NULL;

	/**
	 * @brief 将毫秒转换为系统节拍数
	 * @param timeout_ms 超时时间，单位毫秒
	 * @retval TickType_t 对应的系统节拍数
	 */
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

	void Start_Info(void)
	{
		// static uint8_t i = 0;
		// if (i < 2 && gRobot.runSucceedFlag)
		// {
		// 	if (Beep(100) == 1) i++;		//定时器响两声表示上位机运行正常
		// }
	}

	/**
	 * @brief 初始化
	 */
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

		// 注册USB回调：在 UsbRxTask 上下文解析PC帧
		UsbRx_RegisterCallback(GetPCMsg);

		// 设置驱动器初始状态
		RobotInit();

		SendPs();

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1010);

		// valve_num[12] = 1;//灯带常亮

		BEEP_ON;
		osDelay(500);
		BEEP_OFF;
		osDelay(2000);
		BEEP_ON;
		osDelay(500);
		BEEP_OFF;

		MAFilter_Init(&laser_filter, laser_buffer, LASER_FILTER_SIZE);

		/* 初始化 ADC 中断模式（非阻塞） */
		HAL_ADC_Start_IT(&hadc1);
		HAL_ADC_Start_IT(&hadc2);
		HAL_ADC_Start_IT(&hadc3);
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
	void User_Task(void *argument)
	{
		(void)argument;
		user_task_handle = xTaskGetCurrentTaskHandle();

		TickType_t last_wake = xTaskGetTickCount();
		const TickType_t period = pdMS_TO_TICKS(5);

		uint32_t cnt = 0;
		uint32_t colorcnt = 0;

		static int tim_cnt = 0;

		SetLimits(&hfdcan3, PTP_MODE , ARM2 , (int)(65.f * 4096.f / 32768.f * 2.f * PI) , -1 , -1);
		SetLimits(&hfdcan3, PTP_MODE , ARM1 , (int)(65.f * 4096.f / 32768.f * 2.f * PI) , -1 , -1);

		for (;;)
		{
			vTaskDelayUntil(&last_wake, period);

			Beep_Control();

			/* 这里写每 5ms 需要执行的逻辑 */
		

			//心跳包自增
			static bool reset_flag = false;
			static bool out_flag = false;
			// User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			
			// DEBUG(" %d\r\n", ps_num[1]);
			if(!out_flag)
			{
				static bool mode_flag = 0;
				if(!mode_flag)
				{
					
					#if TEST == 0 
						SetDriverState(&hfdcan3, PTP_MODE, SLIDE, ENABLE);
					#elif TEST == 1
						SetDriverState(&hfdcan3, PTP_MODE, PLATE, ENABLE);
					#endif
					mode_flag = 1;
					
				}
				else
				{
					out_flag = true;
					SetBeep(100);
					
					
					#if TEST == 0 
						ResetDriver(&hfdcan3,PTP_MODE,SLIDE);
					#elif TEST == 1
						ResetDriver(&hfdcan3,PTP_MODE,PLATE);
					#endif
				}
				
			}
			// User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			// DEBUG("%f %f %f\r\n", (float)slideTar.pos, (float)slideMsg.pos / 4096.f, (float)slideMsg.vel / 4096.f);
			if(!reset_flag)
			{	
				#if TEST == 0 
					IfResetOkFlag(&hfdcan3,PTP_MODE,SLIDE);
				#elif TEST == 1
					IfResetOkFlag(&hfdcan3,PTP_MODE,PLATE);
				#elif TEST == 2
					IfResetOkFlag(&hfdcan3,PTP_MODE,ARM1);
				#endif	
			}
			#if TEST == 0 
				if(slideMsg.reset_pos_ok_flag && !reset_flag)
			#elif TEST == 1
				if(plateMsg.reset_pos_ok_flag && !reset_flag)
			#elif TEST == 2
				if(armMsg1.reset_pos_ok_flag && !reset_flag)
			#endif
			{
				#if TEST == 2 
					static bool mode_flag = 0;
					if(!mode_flag)
					{
						SetDriverZero(&hfdcan3, PTP_MODE, ARM2);
						mode_flag = 1;
					}
					else
					{
						SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
				#endif
					reset_flag = true;
				#if TEST == 2 
					}
				#endif
				
				
			}
			if(!reset_flag) continue;

			#if TEST == 0
				slideTar.pos = get_slidepos_high();
				PosCtrl(&hfdcan3, PTP_MODE, SLIDE, slideTar.pos*4096.f, 1.0f*1000.f*60, 0.05*100.0f);
			#endif
			

			#if TEST == 2
				static bool pos_ok = false;
				if(!pos_ok)
				{
					
					get_rowpos();
					
					static int cnt_z = 0;
					/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
					if(user_abs(driverMsgs.armMsg1.pos / 4096.f + ARMSTART) <= 0.02f)
					{
						cnt_z++;
						if(cnt_z >= 200)
						{
							cnt_z = 201;
							SetBeep(100);
							pos_ok = true;
						}
					}
					PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
					PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				}
			#endif	


			
			// DEBUG("%d %d %d \r\n", ps_num[0], ps_num[1], ps_num[2]);
			// DEBUG("%d %d %d \r\n", ps_num[3], ps_num[4], ps_num[5]);
			#if TEST == 1
				static bool start = 0;
				static int cnt_start = 0;
				
				if(!start)
				{
					cnt_start++;
					if(cnt_start >= 1000)
					{
						start = 1;
						cnt_start = 1001;
						SetBeep(100);
					}
				}
				else
				{
					platecrl();
					DEBUG("%d \r\n",  ps_num[1]);
			
					
				}
			#endif
			
			#if TEST == 2

				if(pos_ok)
				{
					static bool flag_start = 0;
					if(!flag_start)
					{
						static int cnt_start = 0;
						cnt_start++;
						
						if(cnt_start >= 200)
						{
							flag_start = 1;
						}
						continue;
					}
					else
					{
						float Ssum = pos_row();//需要重复进行时把vel和stage置为0
						armTar1.pos = -ARMSTART + Ssum;
						armTar2.pos = ARMSTART - Ssum - 0.009f;
					
						PosCtrl(&hfdcan3, PTP_MODE, ARM1,armTar1.pos*4096.f, armTar1.pos_kp , armTar1.pos_kd);
						PosCtrl(&hfdcan3, PTP_MODE, ARM2,armTar2.pos*4096.f, armTar2.pos_kp , armTar2.pos_kd);
					}
				}
			#endif
			
		// 	//test();

		// 	//  MotorControl();
		// 	// ChassisControl();
		// // 	/* 获取驱动器最新消息 */
		//  SetDriverState(&hfdcan3, PTP_MODE, SLIDE, ENABLE);
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			
			// DEBUG("%f %f %f %f %f\r\n", (float)armTar1.pos, (float)driverMsgs.armMsg1.pos / 4096.f, (float)driverMsgs.armMsg2.pos / 4096.f,(float)driverMsgs.armMsg1.vel / 4096.f ,(float)driverMsgs.armMsg2.vel / 4096.f);	
			//DEBUG("%f %f %f\r\n",(float)plateTar.vel,(float)plateMsg.pos / 4096.f,(float)plateMsg.vel / 4096.f);
			//DEBUG("%f %f %f\r\n", (float)slideTar.pos, (float)slideMsg.pos / 4096.f, (float)slideMsg.vel / 4096.f);
		}
	}


		/* 参考轨迹：0→4 转匀加速到 tarvel，4→5 转匀速，5 转后以 a2 减速到 0
	* 每 5 ms 调用一次，返回累计参考行程（圈，相对启动点） */


	void test(void)
	{
		static bool start = 0;

		if(!start)
		{
			SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			start = 1;
		}

		
		float dt_t = 0.005f;
		static float vel = 0.0f;
		static float sum = 0.0f;
		
		if(sum < 63.f)
		{
			vel = 3.0f;
		}
		else
		{
			vel = 0.0f;
		}
		armTar1.pos += vel * dt_t;
		armTar2.pos -= vel * dt_t;
		sum = sum + vel * dt_t;
		
		
		PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 2.0f * 1000.f * 60, -1);
		PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 2.2f * 1000.f * 60, -1);
		

	}

	void platecrl(void)
	{
		plateTar.mode = VEL_MODE;
		// plateTar.vel = get_platevel_high(plateMsg.pos/4096.f);
		plateTar.vel = get_platevel_low(plateMsg.pos/4096.f);
		// static bool tormark = 0;
		// if(plateMsg.pos / 4096.f > 0.6f && plateMsg.pos / 4096.f < 1.2f)
		// {
		// 	plateTar.tor = 49.0f;
		// }
		// if(plateMsg.pos / 4096.f > 1.2f && plateMsg.pos / 4096.f < 1.35f)
		// {
		// 	plateTar.tor = 20.0f;
		// 	// tormark = 1;
		// }
		// if(plateMsg.pos / 4096.f > 1.35f && plateMsg.pos / 4096.f < 1.5f)
		// {
		// 	plateTar.tor = 5.0f;
		// 	// tormark = 1;
		// }
		// else
		// {
		//     plateTar.tor = 0.0f;
		// }

		DriverControl(&hfdcan3, PLATE, &plateTar, &driverMsgs.plateMsg);
	}

	float pos_row(void)
	{
		const float tarvel = 57.0f;   // 速度 r/s
		const float s1     = 4.0f;   // 加速段行程 r
		const float s2     = 1.5f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
		
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.5f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
					stage = 1;
				}
				break;

			case 1:       
					// armTar1.pos_kp = 0.001f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.01f*100.f;
					armTar2.pos_kp = 3.5f*1000.f*60.f;//-1为不发期望
					// armTar2.pos_kp = 0.001f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.01f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}

	float user_abs(float x)
	{
		if(x<0) return -x;
		else return x;
	}

	void get_rowpos()
	{
		static float r = 0.0f;
		if(r > -ARMSTART)
		{
			r = r - 0.05f;
			if(r <= -ARMSTART)
			{
				r = -ARMSTART;
			}
		}
		if(r <= -ARMSTART)
		{
			r=-ARMSTART;
		}
		armTar1.pos = r;
		armTar2.pos = -r+0.002f;
	}

	float get_slidepos_high(void)
	{
		static float pos = 0.0f;
		// float tarpos = 394.8f;
		// float tarpos = 185.94f;
		// float tarpos = 241.16f;
		// float tarpos = 308.4f;
		// float tarpos = 73.6f;
		float tarpos = 343.4f;

		pos = pos + 35.0f/200.0f;//1秒35转
		if(pos > tarpos)
		{
			return tarpos;
		}
		return pos;
	}

	float get_slidepos_low(void)
	{
		static float pos = 0.0f;
		pos = pos + 40.0f/200.0f;//1秒70转
		// static float tarpos = 799.53f;
		static float tarpos = 711.42f;
		if(pos >= tarpos)
		{
			return tarpos;
		}
		return pos;
	}

	float get_platevel_high(float r)
	{
	//41 1500 1500
		// float tarvel = 32.9f;//期望转速 r/s
		// float tarvel = 33.4f;//117下午期望转速 r/s
		 float tarvel = 24.8f;
		float a1 = 1500.0f;//加速
		float a2 =1500.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			// if(r>=1.2f && r<=1.4f)
			// {
			// 	// plateTar.vel_kp = 1.0f*1000.f;
			// 	// plateTar.vel_ki = 0.23f*1000.f;
			// 	plateTar.vel_kp = 1.6f*1000.f;
			// 	plateTar.vel_ki = 0.4f*1000.f;
				
			// }
			// if(r>=1.4f && r<=1.5f)
			// {
			// 	// plateTar.vel_kp = 1.0f*1000.f;
			// 	// plateTar.vel_ki = 0.23f*1000.f;
			// 	plateTar.vel_kp = 2.0f*1000.f;
			// 	plateTar.vel_ki = 0.4f*1000.f;
				
			// }
			// if(r>=1.5f && r<=1.6f)
			// {
			// 	plateTar.vel_kp = 10.0f*1000.f;
			// 	plateTar.vel_ki = 0.2f*1000.f;
			// 	// plateTar.vel_kp = 0.1f*1000.f;
			// 	// plateTar.vel_ki = 0.01f*1000.f;

				
			// }
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			if(r>=1.2f && r<1.4f)
			{
				plateTar.vel_kp = 1.6f*1000.f;
				plateTar.vel_ki = 0.4f*1000.f;
			}
			if(r>=1.4f && r<=1.6f)
			{
				plateTar.vel_kp = 0.9f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else 
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.65f*1000.f;

			plateTar.vel_ki = 0.4f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.0f*1000.f;
			plateTar.vel_ki = 0.0f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.0f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_kp = 0.6f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return 0.0f;
		}
		
	}


	float get_platevel_low(float r)
	{
	//41 1500 1500
		float tarvel = 11.2f;//期望转速 r/s
		float a1 = 200.0f;//加速
		float a2 = 200.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0f;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.5f*1000.f;
			if(r>=0.6f)
			{
				plateTar.vel_kp = 1.8f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else 
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.5f*1000.f;
			plateTar.vel_ki = 0.0f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_ki = 1.0f*1000.f;
			return 0.0f;
		}
		
	}

	void valve_ctrl(void){
		
		valve2_number = (valve2_num[11] << 2) + (valve2_num[9] << 1) + (valve2_num[10] << 0) + '0';
		
		valve_number = (valve_num[12] << 1) + (valve_num[10] << 0) + '0';

	}

	void HB_init(void){
		HB.arm1 = 0;
		HB.arm2 = 0;
	}

	void hsvctrl(){


	}

	void MAFilter_Init(MAFilter *f, int32_t *buf, uint16_t window_size) {
		f->buffer = buf;
		f->sum = 0;
		f->index = 0;
		f->size = window_size;
		f->count = 0;
		memset(buf, 0, window_size * sizeof(int32_t)); // 清零缓冲区
	}

	int32_t MAFilter_Put(MAFilter *f, int32_t new_value) {
		// 1. 若窗口已满，先减去将被覆盖的旧值
		if (f->count == f->size) {
			f->sum -= f->buffer[f->index];
		}
		// 2. 写入新值并更新累加和
		f->buffer[f->index] = new_value;
		f->sum += new_value;
		// 3. 更新索引（循环队列核心）
		f->index = (f->index + 1) % f->size;
		// 4. 更新有效数据量（未满时递增）
		if (f->count < f->size) {
			f->count++;
		}
		// 5. 返回当前平均值（未满时分母为count）
		return f->sum / (f->count ? f->count : 1);
	}

	void GetADCMsg()
	{
		/* 非阻塞版本：直接读缓存值（由中断回调更新） */
		LaserDis_right = (int32_t)adc2_cached;
		LaserDis_front = (int32_t)adc3_cached;	
		LaserDis = (int32_t)adc1_cached;
		if(GetADC1Flag){
			GetADC1Flag = false;
			HAL_ADC_Start_IT(&hadc1);
		}
		if(GetADC2Flag){
			GetADC2Flag = false;
			HAL_ADC_Start_IT(&hadc2);
		}
		if(GetADC3Flag){
			GetADC3Flag = false;
			HAL_ADC_Start_IT(&hadc3);
		}

	}

	/**
	 * @brief ADC 转换完成回调（中断上下文）
	 */
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
		if (hadc->Instance == ADC1)
		{
			/* ADC1 转换完成 */
			GetADC1Flag = true;
			adc1_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
		if (hadc->Instance == ADC2)
		{
			/* ADC1 转换完成 */
			GetADC2Flag = true;
			adc2_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
		if (hadc->Instance == ADC3)
		{
			/* ADC1 转换完成 */
			GetADC3Flag = true;
			adc3_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
	}

#elif WAY == 1


/*统合版(要改.h)*/


	#include "user_task.h"
	#include "user_control.h"
	#include "motor_driver.h"
	#include "chassis_driver.h"
	#include "user_usart.h"
	#include "vofa.h"
	#include "adc.h"
	#include "math.h"

	bool camp_flag = true;//false红场，true蓝场
	bool star_receive_flag = false;
	uint8_t motor_reset_flag = 0;

	#define LASER_FILTER_SIZE 5  // 激光测距窗口大小
	int32_t laser_buffer[LASER_FILTER_SIZE];
	int32_t laserleft_buffer[LASER_FILTER_SIZE];
	MAFilter laser_filter = {laser_buffer, 0, 0, LASER_FILTER_SIZE, 0};
	MAFilter laserleft_filter = {laserleft_buffer, 0, 0, LASER_FILTER_SIZE, 0};
	void hsvctrl();

	/* ADC 中断回调缓存 */
	static volatile uint32_t adc1_cached = 0;
	static volatile uint32_t adc2_cached = 0;
	static volatile uint32_t adc3_cached = 0;

	volatile bool GetADC1Flag = false;
	volatile bool GetADC2Flag = false;
	volatile bool GetADC3Flag = false;

	int Beep_Time = 0;
	void SetBeep(uint32_t time){
		Beep_Time = time;
	}
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
	/* 驱动器回报帧 StdId 规则：StdId = 0x280 + DriverNum */
	static int Fdcan_ReportStdIdToDriverNum(uint32_t stdId, uint8_t *outDriverNum)
	{
		if (outDriverNum == NULL)
		{
			return -1;
		}
		
		// if (stdId == ELBOW || stdId == WRIST)
		// {
		// 	const uint32_t dn = stdId;
		// 	if ((dn < 1U) || (dn > 64U))
		// 	{
		// 		return -1;
		// 	}
		// 	*outDriverNum = (uint8_t)dn;
		// 	return 0;
		// }
		// else
		// {
			const uint32_t dn = stdId - DRIVER_CLIENT_BASE_ID;
			if ((dn < 1U) || (dn > 64U))
			{
				return -1;
			}
			*outDriverNum = (uint8_t)dn;
			return 0;
		// }
	}

	/* RXtask 上下文解析：FDCAN1 / FDCAN3 解析函数不同也没问题，在这里分发即可 */
	static void User_FDCAN_RxTaskParse(const UserFDCAN_RxMsg_t *msg)
	{
		if ((msg == NULL) || (msg->bus == NULL))
		{
			return;
		}

		uint8_t dn = 0U;
		if (Fdcan_ReportStdIdToDriverNum(msg->header.Identifier, &dn) != 0)
		{
			return;
		}

		int frame_num = msg->data_len / 8;
		for (int i = 0; i < frame_num; i++)
		{
			const uint8_t *msg_data = msg->data + i * 8;
			/* FDCAN1: 上层 CAN（UP_FDCAN）解析：TURN/ROLLER */
			if (msg->bus == &hfdcan1)
			{
				switch (dn)
				{

					default:
					{
						break;
					}
				}
			}
		
			/* FDCAN3: 轮子 CAN（WHEEL_FDCAN）解析：ONE/TWO/THR */
			if (msg->bus == &hfdcan3)
			{
				switch (dn)
				{
					case 1:
					{
						HB.arm1 = 0;
						
						armMsg1.HBStartFlag = 1;
						
						GetDriverMsg(&armMsg1, msg_data);
						
						break;
						
					}
					case 2:
					{
						HB.arm2 = 0;

						armMsg2.HBStartFlag = 1;

						GetDriverMsg(&armMsg2, msg_data);

						break;
					}
					case 3:
					{
						HB.plate = 0;

						plateMsg.HBStartFlag = 1;

						GetDriverMsg(&plateMsg, msg_data);

						break;
					}
					case 4:
					{
						HB.slide = 0;

						slideMsg.HBStartFlag = 1;

						GetDriverMsg(&slideMsg, msg_data);

						break;
					}
					default:
					{
						break;
					}
				}
			}
		}

	}

	osThreadId_t user_task = NULL;
	TaskHandle_t user_task_handle = NULL;

	/**
	 * @brief 将毫秒转换为系统节拍数
	 * @param timeout_ms 超时时间，单位毫秒
	 * @retval TickType_t 对应的系统节拍数
	 */
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

	void Start_Info(void)
	{
		// static uint8_t i = 0;
		// if (i < 2 && gRobot.runSucceedFlag)
		// {
		// 	if (Beep(100) == 1) i++;		//定时器响两声表示上位机运行正常
		// }
	}

	/**
	 * @brief 初始化
	 */
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

		// 注册USB回调：在 UsbRxTask 上下文解析PC帧
		UsbRx_RegisterCallback(GetPCMsg);

		// 设置驱动器初始状态
		RobotInit();

		SendPs();

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1010);

		// valve_num[12] = 1;//灯带常亮

		BEEP_ON;
		osDelay(500);
		BEEP_OFF;
		osDelay(2000);
		BEEP_ON;
		osDelay(500);
		BEEP_OFF;

		MAFilter_Init(&laser_filter, laser_buffer, LASER_FILTER_SIZE);

		/* 初始化 ADC 中断模式（非阻塞） */
		HAL_ADC_Start_IT(&hadc1);
		HAL_ADC_Start_IT(&hadc2);
		HAL_ADC_Start_IT(&hadc3);
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
	void User_Task(void *argument)
	{
		(void)argument;
		user_task_handle = xTaskGetCurrentTaskHandle();

		TickType_t last_wake = xTaskGetTickCount();
		const TickType_t period = pdMS_TO_TICKS(5);

		SetLimits(&hfdcan3, PTP_MODE , ARM2 , (int)(75.f * 4096.f / 32768.f * 2.f * PI) , -1 , -1);
		SetLimits(&hfdcan3, PTP_MODE , ARM1 , (int)(75.f * 4096.f / 32768.f * 2.f * PI) , -1 , -1);

		for (;;)
		{
			vTaskDelayUntil(&last_wake, period);
			Beep_Control();
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			DEBUG("%d \r\n",ps_num[3]);

			/* 这里写每 5ms 需要执行的逻辑 */
			

			//心跳包自增
			static int mode = HIGHDOWN117;//发球方式

			static bool reset_flag = false; 
			static bool out_flag = false;
			if(!out_flag)
			{
				
				out_flag = true;
				SetBeep(100);
				//ResetDriver(&hfdcan3,PTP_MODE,SLIDE);
				ResetDriver(&hfdcan3,PTP_MODE,PLATE);
				
			}
			if(!reset_flag)
			{	
				IfResetOkFlag(&hfdcan3,PTP_MODE,ARM1);
				IfResetOkFlag(&hfdcan3,PTP_MODE,PLATE);
				//IfResetOkFlag(&hfdcan3,PTP_MODE,SLIDE);
			}
			if(armMsg1.reset_pos_ok_flag && plateMsg.reset_pos_ok_flag && !reset_flag)
			{
				static bool mode_flag = 0;
				if(!mode_flag)
				{
					SetDriverZero(&hfdcan3, PTP_MODE, ARM2);
					mode_flag = 1;
					SetBeep(100);
				}
				else
				{
					SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
					reset_flag = true; 
				}
				
			}
			if(!reset_flag) continue;


			//slideTar.pos = get_slidepos_high();
			//PosCtrl(&hfdcan3, PTP_MODE, SLIDE, slideTar.pos*4096.f, 1.0f*1000.f*60, -1);

			/* 调位阶段：只让 row1 出力，row2 失能（mode=0 会下发 DISABLE） */
			static bool pos_ok = false;
			if(!pos_ok)
			{
				arm_pos1(mode, &pos_ok);
			}

			static bool start = 0;
			static int cnt_start = 0;
			static bool start_arm = false;
			if(!start && pos_ok)
			{
				if(ps_num[3] == 0)
				{
					cnt_start++;
				}
				else
				{
					cnt_start = 0;
				}
				if(cnt_start >= 1000 )
				{
					cnt_start = 1001;
					SetBeep(400);
					start = 1;
				}
			}
			else if (start && pos_ok)
			{
				static bool start2 = 0;
				static int cnt_waiting2 = 0;
				if(!start2)
				{
					cnt_waiting2++;
					if(cnt_waiting2 >= 1400)
					{
						start2 = 1;
						cnt_waiting2 = 1401;
					}
				}
				else
				{
					platecrl(mode);
					start_arm = true;
					DEBUG("%d %d %d %f %f \r\n",ps_num[0], ps_num[1], ps_num[2], armTar1.pos, armMsg1.pos/4096.f,armMsg2.pos/4096.f);
				}
				
				
			}
			else
			{
				continue;
			}

			static bool start1 = 0;
			if(start_arm)
			{
				static int cnt_waiting = 0;
				if(cnt_waiting < GAP_TIME && !start1)
				{
					cnt_waiting++;
				}
				else
				{
					cnt_waiting = GAP_TIME + 1;
					start1 = 1;
				}
				if(start1)
				{
					arm_pos2(mode);
				}
					
			}
			
			//test();
			/* 获取驱动器最新消息 */
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			
			//DEBUG("%f %f %f %f %f\r\n", (float)armTar1.pos, (float)driverMsgs.armMsg1.pos / 4096.f, (float)-driverMsgs.armMsg2.pos / 4096.f,(float)driverMsgs.armMsg1.currQ ,(float)driverMsgs.armMsg2.currQ);	
			//DEBUG("%f %f %f\r\n",(float)plateTar.vel,(float)plateMsg.pos / 4096.f,(float)plateMsg.vel / 4096.f);
			//DEBUG("%f %f %f\r\n", (float)slideTar.pos, (float)slideMsg.pos / 4096.f, (float)slideMsg.vel / 4096.f);
		}
	}


		/* 参考轨迹：0→4 转匀加速到 tarvel，4→5 转匀速，5 转后以 a2 减速到 0
	* 每 5 ms 调用一次，返回累计参考行程（圈，相对启动点） */


	void test(void)
	{
		static bool start = 0;

		if(!start)
		{
			SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			start = 1;
		}

		
		float dt_t = 0.005f;
		static float vel = 0.0f;
		static float sum = 0.0f;
		
		if(sum < 63.f)
		{
			vel = 3.0f;
		}
		else
		{
			vel = 0.0f;
		}
		armTar1.pos += vel * dt_t;
		armTar2.pos -= vel * dt_t;
		sum = sum + vel * dt_t;
		
		
		PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 2.0f * 1000.f * 60, -1);
		PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 2.2f * 1000.f * 60, -1);
		

	}

	void platecrl(int mode)
	{
		plateTar.mode = VEL_MODE;
		if(mode == HIGH)
		{
			plateTar.vel = get_platevel_high(plateMsg.pos/4096.f);
		}
		else if(mode == HIGHDOWN)
		{
			plateTar.vel = get_platevel_highdown(plateMsg.pos/4096.f);
		}
		else if(mode == HIGHDOWN117)
		{
			plateTar.vel = get_platevel_highdown117(plateMsg.pos/4096.f);
		}
		else if(mode == LOW118A)
		{
			plateTar.vel = get_platevel_low118a(plateMsg.pos/4096.f);
		}
		else if(mode == LOW118B)
		{
			plateTar.vel = get_platevel_low118b(plateMsg.pos/4096.f);
		}
		else
		{
			return;
		}
		DriverControl(&hfdcan3, PLATE , &plateTar, &driverMsgs.plateMsg);
	}

	float pos_row(int mode)
	{
		switch(mode)
		{
			case HIGH:
			{
				return pos_row_high();
			}
			case HIGHDOWN:
			{
				return pos_row_highdown();
			}
			case HIGHDOWN117:
			{
				return pos_row_highdown117();
			}
			case LOW118A:
			{
				return pos_row_low118a();
			}
			case LOW118B:
			{
				return pos_row_low118b();
			}
			default:
			{
				return 0.0f;
			}

		}
	}

	float pos_row_high(void)
	{
		//速度环 0.8 0.2
		const float tarvel = 50.0f;   // 速度 r/s
		const float s1     = 4.0f;   // 加速段行程 r
		const float s2     = 1.50f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
			
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
						stage = 1;
				}
				break;

			case 1:       
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.0005f*100.f;
					armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.0005f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					armTar2.pos_kp = -1;//-1为不发期望
					armTar2.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}
	

	float pos_row_highdown(void)
	{
		const float tarvel = 57.0f;   // 速度 r/s
		// const float tarvel = 60.0f;   // 速度 r/s
		const float s1     = 4.3f;   // 加速段行程 r
		const float s2     = 1.70f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
		
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
					stage = 1;
				}
				break;

			case 1:       
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.0005f*100.f;
					armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.0005f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					armTar2.pos_kp = -1;//-1为不发期望
					armTar2.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}

	float pos_row_highdown117(void)
	{
		const float tarvel = 60.0f;   // 速度 r/s
		const float s1     = 4.3f;   // 加速段行程 r
		const float s2     = 1.80f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
		
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
					stage = 1;
				}
				break;

			case 1:       
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.0005f*100.f;
					armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.0005f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					armTar2.pos_kp = -1;//-1为不发期望
					armTar2.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}
	
	float pos_row_low118a(void)
	{
		const float tarvel = 50.0f;   // 速度 r/s
		const float s1     = 4.3f;   // 加速段行程 r
		const float s2     = 1.80f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
		
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
					stage = 1;
				}
				break;

			case 1:       
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.0005f*100.f;
					armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.0005f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					armTar2.pos_kp = -1;//-1为不发期望
					armTar2.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}

	float pos_row_low118b(void)
	{
		const float tarvel = 60.0f;   // 速度 r/s
		const float s1     = 4.3f;   // 加速段行程 r
		const float s2     = 1.80f;   // 匀速段行程 r
		const float dt     = 0.005f; // 调用周期 5 ms                   
		const float t1 = 2.0f * s1 / tarvel;   // 加速时间
		const float a1 = tarvel / t1;          // = tarvel^2 / (2*s1) = 1.125 r/s^2
		
		static float s_ref = 0.0f;   // 累计参考位置 r
		static float vel   = 0.0f;   // 参考速度 r/s
		static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

		switch (stage)
		{
			case 0:                                  /* 匀加速 */
				vel += a1 * dt;
				armTar1.pos_kp = 3.327241f*1000.f*60.f;//-1为不发期望
				armTar1.pos_kd = 0.0005f*100.f;
				armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
				armTar2.pos_kd = 0.0005f*100.f;
				if (vel >= tarvel)
				{
					vel   = tarvel;
					stage = 1;
				}
				break;

			case 1:       
					armTar1.pos_kp = 3.6814f*1000.f*60.f;//-1为不发期望
					armTar1.pos_kd = 0.0005f*100.f;
					armTar2.pos_kp = 3.1202f*1000.f*60.f;//-1为不发期望
					armTar2.pos_kd = 0.0005f*100.f;                           /* 匀速，走满 s1+s2 后进减速 */
					if (s_ref >= s1 + s2)
					{
						stage = 2;
					}
					break;

			case 2:                                  /* 匀减速 失能*/
					vel = 0.f;
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM1 , DISABLE , 0.0f);
					SetStateAndPosCtrl(&hfdcan3, PTP_MODE, ARM2 , DISABLE , 0.0f);
					armTar1.pos_kp = -1;//-1为不发期望
					armTar1.pos_kd = -1;
					armTar2.pos_kp = -1;//-1为不发期望
					armTar2.pos_kd = -1;
					stage = 3; 	
					break;

			default:                                 /* 结束后保持不动 */
					break;
		}

		s_ref += vel * dt; // 累计参考位置
		return s_ref;
	}

	float user_abs(float x)
	{
		if(x<0) return -x;
		else return x;
	}

	void arm_pos1(int mode , bool *pos_ok)
	{
		switch(mode)
		{
			case HIGH:
			{
				get_rowpos_high();
				static int cnt_z = 0;
				/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
				if(user_abs(driverMsgs.armMsg1.pos / 4096.f + 1.75f) <= 0.02f)
				{
					cnt_z++;
					if(cnt_z >= 200)
					{
						cnt_z=201;
						SetBeep(100);
						*pos_ok=true;
					}
				}
				PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				break;
			}
			case HIGHDOWN:
			{
				get_rowpos_highdown();
				static int cnt_z = 0;
				/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
				if(user_abs(driverMsgs.armMsg1.pos / 4096.f + 2.0f) <= 0.02f)
				{
					cnt_z++;
					if(cnt_z >= 200)
					{
						cnt_z=201;
						SetBeep(100);
						*pos_ok=true;
					}
				}
				PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				break;
			}
			case HIGHDOWN117:
			{
				get_rowpos_highdown117();
				static int cnt_z = 0;
				/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
				if(user_abs(driverMsgs.armMsg1.pos / 4096.f + 2.0f) <= 0.02f)
				{
					cnt_z++;
					if(cnt_z >= 200)
					{
						cnt_z=201;
						SetBeep(100);
						*pos_ok=true;
					}
				}
				PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				break;
			}
			case LOW118A:
			{
				get_rowpos_low118a();
				static int cnt_z = 0;
				/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
				if(user_abs(driverMsgs.armMsg1.pos / 4096.f + STARTLOW118A) <= 0.02f)
				{
					cnt_z++;
					if(cnt_z >= 200)
					{
						cnt_z=201;
						SetBeep(100);
						*pos_ok=true;
					}
				}
				PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				break;
			}
			case LOW118B:
			{
				get_rowpos_low118b();
				static int cnt_z = 0;
				/* driverMsgs.row1.pos 是 1/4096 圈的原始脉冲值，先换算成圈再判到位 */
				if(user_abs(driverMsgs.armMsg1.pos / 4096.f + STARTLOW118B) <= 0.02f)
				{
					cnt_z++;
					if(cnt_z >= 200)
					{
						cnt_z=201;
						SetBeep(100);
						*pos_ok=true;
					}
				}
				PosCtrl(&hfdcan3, PTP_MODE, ARM1, armTar1.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				PosCtrl(&hfdcan3, PTP_MODE, ARM2, armTar2.pos * 4096.f, 1.2f * 1000.f * 60, -1);
				break;
			}
			default:
			{
				*pos_ok=false;
				break;
			}

		}
	}

	void arm_pos2(int mode)
	{
		float Ssum = pos_row(mode);//需要重复进行时把vel和stage置为0
		switch(mode)
		{
			case HIGH:
				armTar1.pos = STARTHIGH + Ssum;
				armTar2.pos = -STARTHIGH - Ssum;
				break;
				
			case HIGHDOWN:
				armTar1.pos = STARTHIGHDOWN + Ssum;
				armTar2.pos = -STARTHIGHDOWN - Ssum;
				break;
				
			case HIGHDOWN117:
				armTar1.pos = STARTHIGHDOWN + Ssum;
				armTar2.pos = -STARTHIGHDOWN - Ssum - 0.009f;
				break;
			case LOW118A:
				armTar1.pos = STARTLOW118A + Ssum;
				armTar2.pos = -STARTLOW118A - Ssum;
				break;
				
			case LOW118B:
				armTar1.pos = STARTLOW118B + Ssum;
				armTar2.pos = -STARTLOW118B - Ssum;
				break;
				
			default:
				return; // 对应你原本的 else { return; }
		}

		PosCtrl(&hfdcan3, PTP_MODE, ARM1,armTar1.pos*4096.f, armTar1.pos_kp , armTar1.pos_kd);
		PosCtrl(&hfdcan3, PTP_MODE, ARM2,(armTar2.pos)*4096.f, armTar2.pos_kp , armTar2.pos_kd);
	}

	void get_rowpos_high()
	{
		static float r = 0.0f;
		if(r > STARTHIGH)
		{
			r = r - 0.05f;
			if(r <= STARTHIGH)
			{
				r = STARTHIGH;
			}
		}
		if(r <= STARTHIGH)
		{
			r=STARTHIGH;
		}
		armTar1.pos = r;
		armTar2.pos = -r;
	}

	void get_rowpos_highdown117()
	{
		static float r = 0.0f;
		if(r > STARTHIGHDOWN)
		{
			r = r - 0.05f;
			if(r <= STARTHIGHDOWN)
			{
				r = STARTHIGHDOWN;
			}
		}
		if(r <= STARTHIGHDOWN)
		{
			r=STARTHIGHDOWN;
		}
		armTar1.pos = r;
		armTar2.pos = -r;
	}


	void get_rowpos_highdown()
	{
		static float r = 0.0f;
		if(r > STARTHIGHDOWN)
		{
			r = r - 0.05f;
			if(r <= STARTHIGHDOWN)
			{
				r = STARTHIGHDOWN;
			}
		}
		if(r <= STARTHIGHDOWN)
		{
			r=STARTHIGHDOWN;
		}
		armTar1.pos = r;
		armTar2.pos = -r;
	}

	void get_rowpos_low118a()
	{
		static float r = 0.0f;
		if(r > STARTLOW118A)
		{
			r = r - 0.05f;
			if(r <= STARTLOW118A)
			{
				r = STARTLOW118A;
			}
		}
		if(r <= STARTLOW118A)
		{
			r=STARTLOW118A;
		}
		armTar1.pos = r;
		armTar2.pos = -r;
	}

	void get_rowpos_low118b()
	{
		static float r = 0.0f;
		if(r > STARTLOW118B)
		{
			r = r - 0.05f;
			if(r <= STARTLOW118B)
			{
				r = STARTLOW118B;
			}
		}
		if(r <= STARTLOW118B)
		{
			r=STARTLOW118B;
		}
		armTar1.pos = r;
		armTar2.pos = -r;
	}


	float get_slidepos_high(void)
	{
		static float pos = 0.0f;
		// float tarpos = 394.8f;
		// float tarpos = 185.94f;
		float tarpos = 241.16f;
		// float tarpos = 308.4f;
		pos = pos + 35.0f/200.0f;//1秒35转
		if(pos > tarpos)
		{
			return tarpos;
		}
		return pos;
	}

	float get_slidepos_low(void)
	{
		static float pos = 0.0f;
		pos = pos + 40.0f/200.0f;//1秒80转
		static float tarpos = 711.42f;
		if(pos >= tarpos)
		{
			return tarpos;
		}
		return pos;
	}

	float get_slidepos_highdown(void)
	{
		static float pos = 0.0f;
		float tarpos = 241.17f;
		pos = pos + 35.0f/200.0f;//1秒35转
		if(pos > tarpos)
		{
			return tarpos;
		}
		return pos;
	}

	float get_platevel_high(float r)
	{
	//41 1500 1500
		float tarvel = 32.5f;//期望转速 r/s
		float a1 = 1500.0f;//加速
		float a2 =1500.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			if(r>=1.2f && r<1.4f)
			{
				plateTar.vel_kp = 1.6f*1000.f;
				plateTar.vel_ki = 0.4f*1000.f;
			}
			if(r>=1.4f && r<=1.6f)
			{
				plateTar.vel_kp = 0.9f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.65f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.0f*1000.f;
			plateTar.vel_ki = 0.2f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_kp = 0.6f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return 0.0f;
		}
		return 0.0f;
	}


	float get_platevel_highdown(float r)
	{
		float tarvel = 35.80f;//期望转速 r/s
		float a1 = 1500.0f;//加速
		float a2 =1500.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			if(r>=1.2f && r<1.4f)
			{
				plateTar.vel_kp = 1.6f*1000.f;
				plateTar.vel_ki = 0.4f*1000.f;
			}
			if(r>=1.4f && r<=1.6f)
			{
				plateTar.vel_kp = 0.9f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.65f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.0f*1000.f;
			plateTar.vel_ki = 0.2f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_kp = 0.6f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return 0.0f;
		}
		return 0.0f;
	}

	float get_platevel_highdown117(float r)
	{
		// float tarvel = 35.4f;//旧球期望转速 r/s
		float tarvel = 33.5f;//期望转速 r/s
		// float tarvel = 50.0f;//期望转速 r/s//失败！！！

		float a1 = 1500.0f;//加速
		float a2 =1500.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			if(r>=1.2f && r<1.4f)
			{
				plateTar.vel_kp = 1.6f*1000.f;
				plateTar.vel_ki = 0.4f*1000.f;
			}
			if(r>=1.4f && r<=1.6f)
			{
				plateTar.vel_kp = 0.9f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.65f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.0f*1000.f;
			plateTar.vel_ki = 0.2f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_kp = 0.6f*1000.f;
			plateTar.vel_ki = 0.4f*1000.f;
			return 0.0f;
		}
		return 0.0f;
	}

	float get_platevel_low118a(float r)
	{
	//41 1500 1500
		float tarvel = 11.2f;//期望转速 r/s
		float a1 = 200.0f;//加速
		float a2 = 200.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0f;
		}
		if(r <= 1.60f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			plateTar.vel_ki = 0.5f*1000.f;
			if(r>=0.6f)
			{
				plateTar.vel_kp = 1.8f*1000.f;
				plateTar.vel_ki = 0.2f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else 
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.5f*1000.f;
			plateTar.vel_ki = 0.0f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_ki = 1.0f*1000.f;
			return 0.0f;
		}
		
	}

	float get_platevel_low118b(float r)
	{
		return 0.0f;
	}
	
	float get_platevel_low(float r)
	{
	//41 1500 1500
		float tarvel = 20.0f;//期望转速 r/s
		float a1 = 400.0f;//加速
		float a2 =400.0f;//减速
		if(r <= 0.0f)
		{
			r = 0.0;
		}
		if(r <= 1.60f)
		{
			if(r>=1.2f)
			{
				plateTar.vel_kp = 2.5f*1000.f;
			}
			if(sqrt(2 * r * a1) < 0.3f)
				return 0.3f;  // 返回0.3，表示速度接近于0，给一个小速度
			if(sqrt(2 * r * a1) < tarvel)
				return sqrt(2 * r * a1);  // 返回计算值
			else 
				return tarvel;        // 返回目标速度
		}
		if(r > 1.60f && r <= 1.70f)
		{
			plateTar.vel_kp = 1.8f*1000.f;
			return tarvel;
		}
		if(r>1.70f && r <= 2.95f)
		{
			plateTar.vel_kp = 2.5f*1000.f;
			plateTar.vel_ki = 0.0f*1000.f;
			// 计算基于加速度a2的速度，判断是否接近于0
			if(tarvel * tarvel - 2 * a2 * (r - 1.70f) < 0.f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			if(sqrt(tarvel * tarvel - 2 * a2 * (r - 1.70f)) < 0.0001f)
			{
				return 0.0f;  // 返回0，表示速度接近于0
			}
			else
			{
				// 返回计算得到的速度值
				return sqrt(tarvel * tarvel-2 * a2 * (r - 1.70f));
			}
			
		}
		if(r > 2.95f)
		{
			plateTar.vel_ki = 1.0f*1000.f;
			return 0.0f;
		}
		return 0.0f;
	}

	void valve_ctrl(void){
		
		valve2_number = (valve2_num[11] << 2) + (valve2_num[9] << 1) + (valve2_num[10] << 0) + '0';
		
		valve_number = (valve_num[12] << 1) + (valve_num[10] << 0) + '0';

	}

	void HB_init(void){
		HB.arm1 = 0;
		HB.arm2 = 0;
		HB.plate = 0;
		HB.slide = 0;
	}

	void hsvctrl(){


	}

	void MAFilter_Init(MAFilter *f, int32_t *buf, uint16_t window_size) {
		f->buffer = buf;
		f->sum = 0;
		f->index = 0;
		f->size = window_size;
		f->count = 0;
		memset(buf, 0, window_size * sizeof(int32_t)); // 清零缓冲区
	}

	int32_t MAFilter_Put(MAFilter *f, int32_t new_value) {
		// 1. 若窗口已满，先减去将被覆盖的旧值
		if (f->count == f->size) {
			f->sum -= f->buffer[f->index];
		}
		// 2. 写入新值并更新累加和
		f->buffer[f->index] = new_value;
		f->sum += new_value;
		// 3. 更新索引（循环队列核心）
		f->index = (f->index + 1) % f->size;
		// 4. 更新有效数据量（未满时递增）
		if (f->count < f->size) {
			f->count++;
		}
		// 5. 返回当前平均值（未满时分母为count）
		return f->sum / (f->count ? f->count : 1);
	}

	void GetADCMsg(){
		/* 非阻塞版本：直接读缓存值（由中断回调更新） */
		LaserDis_right = (int32_t)adc2_cached;
		LaserDis_front = (int32_t)adc3_cached;	
		LaserDis = (int32_t)adc1_cached;
		if(GetADC1Flag){
			GetADC1Flag = false;
			HAL_ADC_Start_IT(&hadc1);
		}
		if(GetADC2Flag){
			GetADC2Flag = false;
			HAL_ADC_Start_IT(&hadc2);
		}
		if(GetADC3Flag){
			GetADC3Flag = false;
			HAL_ADC_Start_IT(&hadc3);
		}

	}

	/**
	 * @brief ADC 转换完成回调（中断上下文）
	 */
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
		if (hadc->Instance == ADC1)
		{
			/* ADC1 转换完成 */
			GetADC1Flag = true;
			adc1_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
		if (hadc->Instance == ADC2)
		{
			/* ADC1 转换完成 */
			GetADC2Flag = true;
			adc2_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
		if (hadc->Instance == ADC3)
		{
			/* ADC1 转换完成 */
			GetADC3Flag = true;
			adc3_cached = HAL_ADC_GetValue(hadc);
			/* 立即启动下一次转换 */
			// HAL_ADC_Start_IT(hadc);
		}
	}
#endif