#include "user_control.h"
#include "user_usart.h"
#include "user_task.h"

Heart_t HB;
SteeringWheelMsg_t wheel1Msg, wheel2Msg, wheel3Msg;
driverMsg_t armMsg1 , armMsg2 , plateMsg, slideMsg;

SteeringWheelTar_t wheel1Tar, wheel2Tar, wheel3Tar;
motorTarget_t armTar1, armTar2, plateTar, slideTar;

DriverMsgsSnapshot_t driverMsgs;

int LaserDis = 0;
int LaserDis_front = 0;
int LaserDis_right = 0;
int buffer_valve_flag = 0;

int Esc1 = 0.0;
int Esc2 = 0.0;

int LightwheelH = 0;
int LightwheelS = 0;
int LightwheelV = 0;

char valve_number = '0';
char valve2_number = '0';

void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out)
{
	if (out == NULL)
	{
		return;
	}

	/* 关中断拷贝，避免结构体撕裂（rxTask 正在更新时被读到一半） */
	taskENTER_CRITICAL();
	{
        out->armMsg1 = armMsg1;
        out->armMsg2 = armMsg2;
        out->plateMsg = plateMsg;
        out->slideMsg = slideMsg;
		out->tick = (uint32_t)xTaskGetTickCount();
	}
	taskEXIT_CRITICAL();
}

/**
 * @brief 初始化
 */
void RobotInit(void)
{
    armTar1.mode = 0;
    armTar1.pos = 0.f;
    armTar1.vel = 0.f;
    armTar1.last_mode = 0;
    

    armTar2.mode = 0;
    armTar2.pos = 0.f;
    armTar2.vel = 0.f;
    armTar2.last_mode = 0;

    plateTar.mode = 0;
    plateTar.pos = 0.f;
    plateTar.vel = 0.f;
    plateTar.last_mode = 0;
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
        SetStateAndPosCtrl(hfdcan, PTP_MODE, driverNum, ENABLE, (int32_t)(driverTarget->pos * 4096.f));
        // driverMsg->enableState = 1;
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
                if (driverTarget->vel_kp != driverTarget->last_vel_kp || driverTarget->vel_ki != driverTarget->last_vel_ki)
                {
                    SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki);
                    driverTarget->last_vel_kp = driverTarget->vel_kp;
                    driverTarget->last_vel_ki = driverTarget->vel_ki;
                }
                else
                {
                    VelCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->vel * 4096.f), driverTarget->tor);
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
                        SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki);
                        driverTarget->last_vel_kp = driverTarget->vel_kp;
                        driverTarget->last_vel_ki = driverTarget->vel_ki;
                    }
                    else
                    {
                        if(driverTarget->pos_kp != driverTarget->last_pos_kp || driverTarget->pos_kd != driverTarget->last_pos_kd)
                        {
                            PosCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->pos * 4096.f), driverTarget->pos_kp, driverTarget->pos_kd);
                            driverTarget->last_pos_kp = driverTarget->pos_kp;
                            driverTarget->last_pos_kd = driverTarget->pos_kd;
                        }
                        else
                        {
                            PosCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->pos * 4096.f), -1.f, -1.f);
                        }
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

}

/**
 * @brief 驱动器复位
 * @return 是否复位完成（可以开始正常控制了）
 */
bool reset_control(void)
{

}

/**
 * @brief 电调控制
 */
void EscControl(void)
{
	// __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, esc);
}

/**
 * @brief 电机控制
 */
void MotorControl(void)
{
   
}

/**
 * @brief 舵轮控制
 */
// void ChassisControl(void)
// {
    
//     SteeringWheelCtrl(&FDCANWHEEL1, WHEEL1, wheel1Tar.vel, wheel1Tar.pos, wheel1Tar.hsv);
//     SteeringWheelCtrl(&FDCANWHEEL3, WHEEL3, wheel3Tar.vel, wheel3Tar.pos, wheel3Tar.hsv);
//     SteeringWheelCtrl(&FDCANWHEEL2, WHEEL2, wheel2Tar.vel, wheel2Tar.pos, wheel2Tar.hsv); 
     
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