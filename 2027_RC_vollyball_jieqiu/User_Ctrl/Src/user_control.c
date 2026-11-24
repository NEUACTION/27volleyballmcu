#include "user_control.h"
#include "user_usart.h"

/* 全局变量：设备心跳计数；接收回调清零，周期逻辑递增。 */
Heart_t HB;

/* 全局反馈：以下消息对象由对应驱动器接收解析更新。 */
driverMsg_t leg1Msg, /* 腿部电机 1 反馈；当前接收分支更新。 */
            leg2Msg, /* 腿部电机 2 反馈；当前接收分支更新。 */
            leg3Msg, /* 腿部电机 3 反馈；当前接收分支更新。 */
            leg4Msg, /* 腿部电机 4 反馈；当前接收分支更新。 */
            wheel1Msg, /* 轮子 1 反馈；当前接收解析分支屏蔽。 */
            wheel2Msg, /* 轮子 2 反馈；当前接收解析分支屏蔽。 */
            wheel3Msg, /* 轮子 3 反馈；当前接收解析分支屏蔽。 */
            wheel4Msg, /* 轮子 4 反馈；当前接收解析分支屏蔽。 */
            shoulderMsg, /* 肩部反馈；接收解析分支当前屏蔽。 */
            elbowMsg, /* 肘部反馈；接收解析分支当前屏蔽。 */
            wristMsg; /* 腕部反馈；接收解析分支当前屏蔽。 */

/* 全局目标：以下对象由任务填写，电机控制路径读取。 */
motorTarget_t leg1Tar, /* 腿部电机 1 目标；当前任务控制使用。 */
              leg2Tar, /* 腿部电机 2 目标；当前任务控制使用。 */
              leg3Tar, /* 腿部电机 3 目标；当前任务控制使用。 */
              leg4Tar, /* 腿部电机 4 目标；当前任务控制使用。 */
              wheel1Tar, /* 轮子 1 目标；当前有效控制路径可能屏蔽。 */
              wheel2Tar, /* 轮子 2 目标；当前有效控制路径可能屏蔽。 */
              wheel3Tar, /* 轮子 3 目标；当前有效控制路径可能屏蔽。 */
              wheel4Tar, /* 轮子 4 目标；当前有效控制路径可能屏蔽。 */
              shoulderTar, /* 肩部目标；当前任务赋值/发送路径需进一步确认。 */
              elbowTar, /* 肘部目标；当前任务赋值/发送路径需进一步确认。 */
              wristTar; /* 腕部目标；当前任务赋值/发送路径需进一步确认。 */

/* 全局快照：保存一组驱动反馈及采样时刻；由快照接口写入，调用方读取。 */
DriverMsgsSnapshot_t driverMsgs;

/* 全局机器人状态：供各控制任务共享机器人模式与状态；当前启用。 */
gRobot_t gRobot;

/* 全局电调输出比较值；由 EscControl 写入 TIM8 通道 4，当前启用。 */
int esc = 0;

/* 函数：在临界区内复制驱动反馈并记录系统 Tick；空输出指针时不操作。 */
void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out)
{
	if (out == NULL)
	{
		return;
	}

	/* 关中断拷贝，避免结构体撕裂（rxTask 正在更新时被读到一半） */
	taskENTER_CRITICAL();
	{
		/* tick 记录快照复制时的 RTOS 时间，供上层判断数据新鲜度。 */
		out->leg1Msg  = leg1Msg;
        out->leg2Msg  = leg2Msg;
        out->leg3Msg  = leg3Msg;
        out->leg4Msg  = leg4Msg;
        out->wheel1Msg  = wheel1Msg;
        out->wheel2Msg  = wheel2Msg;
        out->wheel3Msg  = wheel3Msg;
        out->wheel4Msg  = wheel4Msg;
        out->shoulderMsg  = shoulderMsg;
        out->elbowMsg  = elbowMsg;
        out->wristMsg  = wristMsg;
		out->tick = (uint32_t)xTaskGetTickCount();
	}
	taskEXIT_CRITICAL();
}

/**
 * @brief 初始化
 */
/**
 * @brief 机器人初始化函数
 * 该函数用于初始化机器人各个部件的状态和模式
 * 包括腿部、轮子、肩部、肘部和腕部的初始设置
 */
/* 函数：初始化腿、轮和上肢驱动的使能/模式/清理状态；腿轮默认禁用，上肢关节默认位置模式。 */
void RobotInit(void)
{

    // 初始化腿部1的状态和模式
    leg1Msg.enableState = DISABLE;    // 禁用使能状态
    leg1Msg.driverMode  = DISABLE;    // 禁用驱动模式
    leg1Tar.mode = DISABLE;           // 设置目标模式为禁用
    leg1Tar.last_mode = DISABLE;      // 设置上次模式为禁用
    leg1Msg.enable_error = 0;         // 清除使能错误
    leg1Tar.clean = 0;                // 清除标志位



    // 初始化腿部2的状态和模式
    leg2Msg.enableState = DISABLE;    // 禁用使能状态
    leg2Msg.driverMode  = DISABLE;    // 禁用驱动模式
    leg2Tar.mode = DISABLE;           // 设置目标模式为禁用
    leg2Tar.last_mode = DISABLE;      // 设置上次模式为禁用
    leg2Msg.enable_error = 0;         // 清除使能错误
    leg2Tar.clean = 0;

    leg3Msg.enableState = DISABLE;
    leg3Msg.driverMode  = DISABLE;
    leg3Tar.mode = DISABLE;
    leg3Tar.last_mode = DISABLE;
    leg3Msg.enable_error = 0;
    leg3Tar.clean = 0;
    
    leg4Msg.enableState = DISABLE;
    leg4Msg.driverMode  = DISABLE;
    leg4Tar.mode = DISABLE;
    leg4Tar.last_mode = DISABLE;
    leg4Msg.enable_error = 0;
    leg4Tar.clean = 0;

    wheel1Msg.enableState = DISABLE;
    wheel1Msg.driverMode  = DISABLE;
    wheel1Tar.mode = DISABLE;
    wheel1Tar.last_mode = DISABLE;
    wheel1Tar.vel_kp = -1;
    wheel1Tar.last_vel_kp = -1;
    wheel1Tar.vel_ki = -1;
    wheel1Tar.last_vel_ki = -1;
    wheel1Msg.enable_error = 0;
    wheel1Tar.clean = 0;

    wheel2Msg.enableState = DISABLE;
    wheel2Msg.driverMode  = DISABLE;
    wheel2Tar.mode = DISABLE;
    wheel2Tar.last_mode = DISABLE;
    wheel2Tar.vel_kp = -1;
    wheel2Tar.last_vel_kp = -1;
    wheel2Tar.vel_ki = -1;
    wheel2Tar.last_vel_ki = -1;
    wheel2Msg.enable_error = 0;
    wheel2Tar.clean = 0;

    wheel3Msg.enableState = DISABLE;
    wheel3Msg.driverMode  = DISABLE;
    wheel3Tar.mode = DISABLE;
    wheel3Tar.last_mode = DISABLE;
    wheel3Tar.vel_kp = -1;
    wheel3Tar.last_vel_kp = -1;
    wheel3Tar.vel_ki = -1;
    wheel3Tar.last_vel_ki = -1;
    wheel3Msg.enable_error = 0;
    wheel3Tar.clean = 0;

    wheel4Msg.enableState = DISABLE;
    wheel4Msg.driverMode  = DISABLE;
    wheel4Tar.mode = DISABLE;
    wheel4Tar.last_mode = DISABLE;
    wheel4Tar.vel_kp = -1;
    wheel4Tar.last_vel_kp = -1;
    wheel4Tar.vel_ki = -1;
    wheel4Tar.last_vel_ki = -1;
    wheel4Msg.enable_error = 0;
    wheel4Tar.clean = 0;

    shoulderMsg.enableState = ENABLE;
    shoulderMsg.driverMode  = POS_MODE;
    shoulderTar.mode = POS_MODE;
    shoulderTar.last_mode = POS_MODE;
    shoulderMsg.enable_error = 0;
    shoulderTar.clean = 0;

    elbowMsg.enableState = ENABLE;
    elbowMsg.driverMode  = POS_MODE;
    elbowTar.mode = POS_MODE;
    elbowTar.last_mode = POS_MODE;
    elbowTar.clean = 0;

    wristMsg.enableState = ENABLE;
    wristMsg.driverMode  = POS_MODE;
    wristTar.mode = POS_MODE;
    wristTar.last_mode = POS_MODE;
    wristTar.clean = 0;
}

/**
 * @brief IO口控制
 */
/* 函数：GPIO 控制预留入口；当前函数体为空，未执行具体输出控制。 */
void GPIOControl(void)
{

}

/**
 * @brief damiao控制指令
 * @param hfdcan CAN总线
 * @param driverNum 驱动器ID
 * @param driverTarget 驱动器期望
 */
/* 函数：按目标模式变化切换达妙电机使能状态，否则发送位置/速度目标；当前用于达妙驱动路径。 */
void DMControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget)
{
    if(driverTarget->mode != driverTarget->last_mode)
    {
        if(driverTarget->mode == 0)
        {
            DMDisable(hfdcan, driverNum, DM_POS_MODE);
        }
        else
        {
            DMEnable(hfdcan, driverNum, DM_POS_MODE);
        }
        driverTarget->last_mode = driverTarget->mode;
    }
    else
    {
        DMPosCtrl(hfdcan, driverNum, driverTarget->pos, driverTarget->vel);
    }
}

/**
 * @brief 驱动器控制指令
 * @param hfdcan CAN总线
 * @param driverNum 驱动器ID
 * @param driverTarget 驱动器期望
 * @param driverMsg 驱动器消息
 */
/* 函数：处理普通驱动器使能和速度/位置/PVT命令；mode 0-3 选择禁用、速度、位置、PVT，旧的备选逻辑保持屏蔽。 */
void DriverControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget, driverMsg_t *driverMsg)
{
	/* 当前 mode 与 last_mode 用于模式切换；反馈 enableState 和错误计数决定是否重发使能状态。 */
	if(driverTarget->mode != driverTarget->last_mode)
	{
        //控制状态切换
		if(driverTarget->mode == 0)
		{
			SetStateAndPosCtrl(hfdcan, PTP_MODE, driverNum, DISABLE, (int32_t)(driverTarget->pos * 4096.f));
		}
		if(driverTarget->last_mode == 0)
		{
			SetStateAndPosCtrl(hfdcan, PTP_MODE, driverNum, ENABLE, (int32_t)(driverTarget->pos * 4096.f));
		}

        //控制模式切换
        if (driverTarget->mode == 1)
        {
            SetDriverMode(hfdcan, PTP_MODE, driverNum, VEL_MODE, (int32_t)(driverTarget->pos * 4096.f));
        }
        if (driverTarget->mode == 2)
        {
            SetDriverMode(hfdcan, PTP_MODE, driverNum, POS_MODE, (int32_t)(driverTarget->pos * 4096.f));
        }
        if (driverTarget->mode == 3)
        {
            SetDriverMode(hfdcan, PTP_MODE, driverNum, PVT_MODE, (int32_t)(driverTarget->pos * 4096.f));
        }

		driverTarget->last_mode = driverTarget->mode;
	}
    else if (driverMsg->enableState == 0 && driverTarget->mode != 0)
    {
        driverMsg->enable_error++;
        SetStateAndPosCtrl(hfdcan, PTP_MODE, driverNum, ENABLE, (int32_t)(driverTarget->pos * 4096.f));
    }
    else if (driverMsg->enableState == 1 && driverTarget->mode == 0)
    {
        SetStateAndPosCtrl(hfdcan, PTP_MODE, driverNum, DISABLE, (int32_t)(driverTarget->pos * 4096.f));
    }
    else
    {
        switch (driverTarget->mode)
        {
            case 0:
            {
                ReadPVT(hfdcan, PTP_MODE, driverNum);
                break;
            }
            case 1:
            {
                // if (driverMsg->enableState == 0)
                // {
                //     SetDriverState(hfdcan, PTP_MODE, driverNum, ENABLE);
                // }
                // else
                // {
                //     if (driverTarget->vel_kp != driverTarget->last_vel_kp || driverTarget->vel_ki != driverTarget->last_vel_ki)
                //     {
                //         SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki);
                //         driverTarget->last_vel_kp = driverTarget->vel_kp;
                //         driverTarget->last_vel_ki = driverTarget->vel_ki;
                //     }
                //     else
                //     {
                //         VelCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->vel * 4096.f), driverTarget->tor);
                //     }
                // }
                if (driverTarget->vel_kp != driverTarget->last_vel_kp || driverTarget->vel_ki != driverTarget->last_vel_ki)
                {
                    SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki, driverTarget->clean);
                    driverTarget->last_vel_kp = driverTarget->vel_kp;
                    driverTarget->last_vel_ki = driverTarget->vel_ki;
                }
                else
                {
                    VelCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->vel * 4096.f), driverTarget->tor, driverTarget->clean);
                }
                break;
            }
            case 2:
            {
                // if (driverMsg->enableState == 0)
                // {
                //     SetDriverState(hfdcan, PTP_MODE, driverNum, ENABLE);
                // }
                // else
                // {
                //     if (driverTarget->vel_kp != driverTarget->last_vel_kp || driverTarget->vel_ki != driverTarget->last_vel_ki)
                //     {
                //         SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki);
                //         driverTarget->last_vel_kp = driverTarget->vel_kp;
                //         driverTarget->last_vel_ki = driverTarget->vel_ki;
                //     }
                //     else
                //     {
                //         PosCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->pos * 4096.f), driverTarget->pos_kp, driverTarget->pos_kd);
                //     }
                // }

                if (driverTarget->vel_kp != driverTarget->last_vel_kp || driverTarget->vel_ki != driverTarget->last_vel_ki)
                {
                    SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki, driverTarget->clean);
                    driverTarget->last_vel_kp = driverTarget->vel_kp;
                    driverTarget->last_vel_ki = driverTarget->vel_ki;
                }
                else
                {
                    PosCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->pos * 4096.f), driverTarget->pos_kp, driverTarget->pos_kd);
                }
                break;
            }
            case 3:
            {
                PVTCtrl(hfdcan, driverNum, driverTarget->pos, driverTarget->vel, driverTarget->mit_kp, driverTarget->mit_kd, driverTarget->tor);
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

/**
 * @brief 心跳包计数
 */
/* 函数：将所有执行器和通信心跳计数饱和递增至 201；周期调用以便接收侧清零。 */
void HB_Add(void)
{
	if(HB.leg1 <= 200)HB.leg1 ++;
    if(HB.leg2 <= 200)HB.leg2 ++;
    if(HB.leg3 <= 200)HB.leg3 ++;
    if(HB.leg4 <= 200)HB.leg4 ++;
    if(HB.wheel1 <= 200)HB.wheel1 ++;
    if(HB.wheel2 <= 200)HB.wheel2 ++;
    if(HB.wheel3 <= 200)HB.wheel3 ++;
    if(HB.wheel4 <= 200)HB.wheel4 ++;
    if(HB.shoulder <= 200)HB.shoulder ++;
    if(HB.elbow <= 200)HB.elbow ++;
    if(HB.wrist <= 200)HB.wrist ++;
	if(HB.PC <= 200)HB.PC ++;
    if(HB.op <= 200)HB.op ++;
}

/**
 * @brief 驱动器复位
 * @return 是否复位完成（可以开始正常控制了）
 */
/* 函数：轮询四条腿的回零完成标志；旧的直接复位及肘腕回零代码处于注释屏蔽状态。 */
bool reset_control(void)
{
    // static int reset_flag = 0;

	// if(reset_flag == 0)
	// {
	// 	ResetDriver(&FDCAN_WL,PTP_MODE,LEG1);
    //     ResetDriver(&FDCAN_WL,PTP_MODE,LEG2);
    //     ResetDriver(&FDCAN_WL,PTP_MODE,LEG3);
    //     ResetDriver(&FDCAN_WL,PTP_MODE,LEG4);
    //     // SetDriverZero(&FDCAN_TK,PTP_MODE,SHOULDER);
	// 	reset_flag = 1;
	// }
    // else if (reset_flag == 1)
    // {
    //     DMEnable(&FDCAN_TK,ELBOW,DM_POS_MODE);
    //     DMEnable(&FDCAN_TK,WRIST,DM_POS_MODE);
    //     SetDriverState(&FDCAN_WL, PTP_MODE, WHEEL1, ENABLE);
    //     SetDriverState(&FDCAN_TK, PTP_MODE, WHEEL2, ENABLE);
    //     SetDriverState(&FDCAN_WL, PTP_MODE, WHEEL3, ENABLE);
    //     SetDriverState(&FDCAN_WL, PTP_MODE, WHEEL4, ENABLE);
    //     SetDriverState(&FDCAN_TK, PTP_MODE, SHOULDER, ENABLE);
    //     reset_flag = 3;
    // }
    // if (reset_flag == 0)
    // {
        if(!leg1Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,LEG1);
	    if(!leg2Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,LEG2);
	    if(!leg3Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,LEG3);
	    if(!leg4Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,LEG4);
        // if(!elbowMsg.reset_pos_ok_flag)
        // {
        //     DMPosCtrl(&FDCAN_TK,ELBOW,0,0.5);
        //     if (fabs(elbowMsg.pos) < 0.05)
        //     {
        //         elbowMsg.reset_pos_ok_flag = 1;
        //     }
        // }
        // if(!wristMsg.reset_pos_ok_flag)
        // {
        //     DMPosCtrl(&FDCAN_TK,WRIST,0,0.5);
        //     if (fabs(wristMsg.pos) < 0.05)
        //     {
        //         wristMsg.reset_pos_ok_flag = 1;
        //     }
        // }
    // }
    if(leg1Msg.reset_pos_ok_flag && leg2Msg.reset_pos_ok_flag && leg3Msg.reset_pos_ok_flag && leg4Msg.reset_pos_ok_flag)
        // && elbowMsg.reset_pos_ok_flag && wristMsg.reset_pos_ok_flag
    // if(elbowMsg.reset_pos_ok_flag && wristMsg.reset_pos_ok_flag)
    // if (leg1Msg.reset_pos_ok_flag && leg2Msg.reset_pos_ok_flag && leg3Msg.reset_pos_ok_flag && leg4Msg.reset_pos_ok_flag)
    {
        return true;
    }
    else 
    {
        return false;
    } 
}

/**
 * @brief 电调控制
 */
/* 函数：将全局 esc 值写入 TIM8 通道 4；当前启用。 */
void EscControl(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, esc);
}

/**
 * @brief 电机控制
 */
// void MotorControl(void)
// {
//     DriverControl(&FDCAN_WL, WHEEL1, &wheel1Tar, &wheel1Msg);
//     DriverControl(&FDCAN_TK, WHEEL2, &wheel2Tar, &wheel2Msg);
//     DriverControl(&FDCAN_WL, WHEEL3, &wheel3Tar, &wheel3Msg);
//     DriverControl(&FDCAN_WL, WHEEL4, &wheel4Tar, &wheel4Msg);

//     DriverControl(&FDCAN_WL, LEG1, &leg1Tar, &leg1Msg);
//     DriverControl(&FDCAN_TK, LEG2, &leg2Tar, &leg2Msg);
//     DriverControl(&FDCAN_WL, LEG3, &leg3Tar, &leg3Msg);
//     DriverControl(&FDCAN_WL, LEG4, &leg4Tar, &leg4Msg);

//     DriverControl(&FDCAN_TK, SHOULDER, &shoulderTar, &shoulderMsg);
//     DMControl(&FDCAN_TK, WRIST, &wristTar);
//     DMControl(&FDCAN_TK, ELBOW, &elbowTar);
// }

/* 函数：按顺序发送气阀、GPIO、电机和电调控制；当前机器人控制调度入口。 */
void RobotControl(void)
{
    SendValve();
    GPIOControl();
    MotorControl();
    EscControl();
}

/**
 * @brief 紧急停车
 */
/* 函数：紧急停车预留接口；当前函数体为空，不执行停车动作。 */
void ErrorStop(void)
{

    
}

/**
 * @brief 紧急失能
 */
/* 函数：全局失能预留接口；当前函数体为空，不执行失能动作。 */
void DisableAll(void)
{
    
}