#include "user_control.h"
#include "user_usart.h"

Heart_t HB;

driverMsg_t leg1Msg, leg2Msg, leg3Msg, leg4Msg, wheel1Msg, wheel2Msg, wheel3Msg, wheel4Msg, shoulderMsg, elbowMsg, wristMsg;

motorTarget_t leg1Tar, leg2Tar, leg3Tar, leg4Tar, wheel1Tar, wheel2Tar, wheel3Tar, wheel4Tar, shoulderTar, elbowTar, wristTar;

DriverMsgsSnapshot_t driverMsgs;

gRobot_t gRobot;

int esc = 0;

void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out)
{
	if (out == NULL)
	{
		return;
	}

	/* 关中断拷贝，避免结构体撕裂（rxTask 正在更新时被读到一半） */
	taskENTER_CRITICAL();
	{
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
void GPIOControl(void)
{

}

/**
 * @brief damiao控制指令
 * @param hfdcan CAN总线
 * @param driverNum 驱动器ID
 * @param driverTarget 驱动器期望
 */
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
void DriverControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget, driverMsg_t *driverMsg)
{
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
void ErrorStop(void)
{

    
}

/**
 * @brief 紧急失能
 */
void DisableAll(void)
{
    
}