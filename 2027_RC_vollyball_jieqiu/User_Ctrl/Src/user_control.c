#include "user_control.h"
#include "user_usart.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_task.h"

#include <math.h>

/* 全局变量：设备心跳计数；接收回调清零，周期逻辑递增。 */
Heart_t HB;

/* 全局反馈：以下消息对象由对应驱动器接收解析更新。 */
driverMsg_t leg1Msg, /* 腿部电机 1 反馈；当前接收分支更新。 */
            leg2Msg, /* 腿部电机 2 反馈；当前接收分支更新。 */
            leg3Msg, /* 腿部电机 3 反馈；当前接收分支更新。 */
            leg4Msg, /* 腿部电机 4 反馈；当前接收分支更新。 */
            slideMsg, /* 肩部反馈；接收解析分支当前屏蔽。 */
            plateMsg, /* 肘部反馈；接收解析分支当前屏蔽。 */
            arm1Msg, /* 腕部反馈；接收解析分支当前屏蔽。 */
            arm2Msg; /* 腕部反馈；接收解析分支当前屏蔽。 */

/* 全局目标：以下对象由任务填写，电机控制路径读取。 */
motorTarget_t leg1Tar, /* 腿部电机 1 目标； */
              leg2Tar, /* 腿部电机 2 目标；*/
              leg3Tar, /* 腿部电机 3 目标； */
              leg4Tar, /* 腿部电机 4 目标；*/
              slideTar, 
              plateTar, 
              arm1Tar, 
              arm2Tar; 
SteeringWheelTar_t
              wheel1Tar, /* 轮子 1 目标； */
              wheel2Tar, /* 轮子 2 目标； */
              wheel3Tar, /* 轮子 3 目标；。 */
              wheel4Tar; /* 轮子 4 目标；。 */
SteeringWheelMsg_t 
              wheel1Msg, /* 轮子 1 实际； */
              wheel2Msg, /* 轮子 2 ； */
              wheel3Msg, /* 轮子 3 ； */
              wheel4Msg; /* 轮子 4 ；不知道是哪个轮 */             
/* 全局快照：保存一组驱动反馈及采样时刻；由快照接口写入，调用方读取。 */
DriverMsgsSnapshot_t driverMsgs;

/* 全局机器人状态：供各控制任务共享机器人模式与状态；当前启用。 */
gRobot_t gRobot;

volatile bool  armok = 0;
volatile bool plateok = 0;
volatile bool serveok = 0;
volatile bool legok = 0;
volatile bool wheelok = 0;
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
        out->slideMsg  = slideMsg;
        out->plateMsg  = plateMsg;
        out->arm1Msg  = arm1Msg;
        out->arm2Msg  = arm2Msg;
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



    slideMsg.enableState = DISABLE;//AsK
    slideMsg.driverMode  = POS_MODE;
    slideTar.mode = POS_MODE;
    slideTar.last_mode = POS_MODE;
    slideMsg.enable_error = 0;
    slideTar.clean = 0;

    plateMsg.enableState = DISABLE;
    plateMsg.driverMode  = POS_MODE;
    plateTar.mode = POS_MODE;
    plateTar.last_mode = POS_MODE;
    plateTar.clean = 0;

    arm1Msg.enableState = DISABLE;
    arm1Msg.driverMode  = POS_MODE;
    arm1Tar.mode = POS_MODE;
    arm1Tar.last_mode = POS_MODE;
    arm1Tar.clean = 0;

    arm2Msg.enableState = DISABLE;
    arm2Msg.driverMode  = POS_MODE;
    arm2Tar.mode = POS_MODE;
    arm2Tar.last_mode = POS_MODE;
    arm2Tar.clean = 0;
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
// void DMControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget)
// {
//     if(driverTarget->mode != driverTarget->last_mode)
//     {
//         if(driverTarget->mode == 0)
//         {
//             DMDisable(hfdcan, driverNum, DM_POS_MODE);
//         }
//         else
//         {
//             DMEnable(hfdcan, driverNum, DM_POS_MODE);
//         }
//         driverTarget->last_mode = driverTarget->mode;
//     }
//     else
//     {
//         DMPosCtrl(hfdcan, driverNum, driverTarget->pos, driverTarget->vel);
//     }
// }

void ChassisControl(void)
{
    
    SteeringWheelCtrl(&hfdcan1, WHEEL1, wheel1Tar.vel, wheel1Tar.pos, wheel1Tar.hsv);
    SteeringWheelCtrl(&hfdcan1, WHEEL2, wheel3Tar.vel, wheel3Tar.pos, wheel3Tar.hsv);
    SteeringWheelCtrl(&hfdcan3, WHEEL3, wheel2Tar.vel, wheel2Tar.pos, wheel2Tar.hsv); 
    SteeringWheelCtrl(&hfdcan3, WHEEL4, wheel4Tar.vel, wheel4Tar.pos, wheel4Tar.hsv);
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
                // if (driverMsg->enableState == 0)//AsK
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
                    SetVelKpKi(hfdcan, PTP_MODE, driverNum, driverTarget->vel_kp, driverTarget->vel_ki, driverTarget->clean);//AsK
                    driverTarget->last_vel_kp = driverTarget->vel_kp;
                    driverTarget->last_vel_ki = driverTarget->vel_ki;
                }
                else
                {
                    VelCtrl(hfdcan, PTP_MODE, driverNum, (int32_t)(driverTarget->vel * 4096.f), driverTarget->tor, driverTarget->clean);//AsK
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
                    PosCtrl(hfdcan, PTP_MODE, driverNum, driverTarget->pos, driverTarget->pos_kp, driverTarget->pos_kd);
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
    if(HB.slide <= 200)HB.slide ++;
    if(HB.plate <= 200)HB.plate ++;
    if(HB.arm1 <= 200)HB.arm1 ++;
    if(HB.arm2 <= 200)HB.arm2 ++;
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
    switch (gRobot.mode)
    {
        case LEGPVT://都是腿，顺延用
        case LEGPOSE:
        {
            SetDriverState(&hfdcan1, PTP_MODE, LEG1, ENABLE);
            SetDriverState(&hfdcan1, PTP_MODE, LEG2, ENABLE);
            SetDriverState(&hfdcan1, PTP_MODE, LEG3, ENABLE);
            SetDriverState(&hfdcan1, PTP_MODE, LEG4, ENABLE);
            if(!leg1Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan1,PTP_MODE,LEG1);
            if(!leg2Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan1,PTP_MODE,LEG2);
            if(!leg3Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan1,PTP_MODE,LEG3);
            if(!leg4Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan1,PTP_MODE,LEG4);
            
            if(leg1Msg.reset_pos_ok_flag && leg2Msg.reset_pos_ok_flag && leg3Msg.reset_pos_ok_flag && leg4Msg.reset_pos_ok_flag)
            {
                SetDriverState(&hfdcan1, PTP_MODE, LEG1, DISABLE);
                SetDriverState(&hfdcan1, PTP_MODE, LEG2, DISABLE);
                SetDriverState(&hfdcan1, PTP_MODE, LEG3, DISABLE);
                SetDriverState(&hfdcan1, PTP_MODE, LEG4, DISABLE);
                return true;
            }
            else 
            {
                return false;
            } 
        }
        case WHEEL:
        {
            // SetDriverState(&hfdcan3, PTP_MODE, WHEEL1, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, WHEEL2, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, WHEEL3, ENABLE);
            // SetDriverState(&hfdcan3, PTP_MODE, WHEEL4, ENABLE);
            // if(!wheel1Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,WHEEL1);
            // if(!wheel2Msg.reset_pos_ok_flag)IfResetOkFlag(,PTP_MODE,WHEEL2);
            // if(!wheel3Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,WHEEL3);
            // if(!wheel4Msg.reset_pos_ok_flag)IfResetOkFlag(&FDCAN_WL,PTP_MODE,WHEEL4);
            
            // if(wheel1Msg.reset_pos_ok_flag && wheel2Msg.reset_pos_ok_flag && wheel3Msg.reset_pos_ok_flag && wheel4Msg.reset_pos_ok_flag)
            // {
            //     return true;
            // }
            // else 
            // {
            //     return false;
            // } 
            return true;
        }
        case ARM:
        {
            SetDriverState(&hfdcan3, PTP_MODE, ARM1, ENABLE);
            SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
            if(!slideMsg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE,SLIDE);
            if(!plateMsg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, PLATE);
            if(!arm1Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, ARM1);
            if(!arm2Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, ARM2);
            
            if(slideMsg.reset_pos_ok_flag && plateMsg.reset_pos_ok_flag && arm1Msg.reset_pos_ok_flag && arm2Msg.reset_pos_ok_flag)
            {
                return true;
            }
            else 
            {
                return false;
            } 
        }
        case SLIDE:
        {
            SetDriverState(&hfdcan3, PTP_MODE, PLATE, ENABLE);
            ResetDriver(&hfdcan3, PTP_MODE, PLATE);
            if(!slideMsg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE,SLIDE);
            if(slideMsg.reset_pos_ok_flag)
            {
                return true;
            }
            else 
            {
                return false;
            } 
        }
        case PLATE:
        {
            SetDriverState(&hfdcan3, PTP_MODE, PLATE, ENABLE);
            ResetDriver(&hfdcan3, PTP_MODE, PLATE);
            if(!plateMsg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, PLATE);
            
            if(plateMsg.reset_pos_ok_flag)
            {
                return true;
            }
            else 
            {
                return false;
            } 
        }
        case SERVE:
        {
            SetDriverState(&hfdcan3, PTP_MODE, PLATE, ENABLE);
            SetDriverState(&hfdcan3, PTP_MODE, ARM1, ENABLE);
            SetDriverState(&hfdcan3, PTP_MODE, ARM2, ENABLE);
            ResetDriver(&hfdcan3, PTP_MODE, PLATE);
            if(!plateMsg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, PLATE);
            if(!arm1Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, ARM1);
            if(!arm2Msg.reset_pos_ok_flag)IfResetOkFlag(&hfdcan3,PTP_MODE, ARM2);
            if(plateMsg.reset_pos_ok_flag && arm1Msg.reset_pos_ok_flag && arm2Msg.reset_pos_ok_flag)
            {
                return true;
            }
            else 
            {
                return false;
            } 
        }
        default:
        {
            return false;
        }
    }
    
}
void changeMode()
{
    switch (gRobot.mode)
    {
        case LEGPVT:
        {
            legok = 0;
			if(!legok)
			{	
				SetDriverMode(&hfdcan3, PTP_MODE , LEG1 , PVT_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG2 , PVT_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG3 , PVT_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG4 , PVT_MODE ,0);

				
				SetDriverState(&hfdcan3, PTP_MODE, LEG1, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG2, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG3, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG4, ENABLE);
				
			}
			static bool waiting_flag = 0;
			static uint32_t waiting_cnt = 0;
			if(!waiting_flag)//不确定必要性
			{
				waiting_cnt++;
				if(waiting_cnt > 100)
				{
					waiting_flag = 1;
				}
			}
            else
            {
                legok = 1;
            }
            break;

        }
        case LEGPOSE:
        {
            legok = 0;
			if(!legok)
			{	
				SetDriverMode(&hfdcan3, PTP_MODE , LEG1 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG2 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG3 , POS_MODE ,0);
				SetDriverMode(&hfdcan3, PTP_MODE , LEG4 , POS_MODE ,0);

				
				SetDriverState(&hfdcan3, PTP_MODE, LEG1, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG2, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG3, ENABLE);
				SetDriverState(&hfdcan3, PTP_MODE, LEG4, ENABLE);
				
			}
			static bool waiting_flag = 0;
			static uint32_t waiting_cnt = 0;
			if(!waiting_flag)//不确定必要性
			{
				waiting_cnt++;
				if(waiting_cnt > 100)
				{
					waiting_flag = 1;
				}
			}
            else
            {
                legok = 1;
            }
            break;
        }
        case WHEEL:
        {
            wheelok = 0;
			if(!wheelok)
			{	
				hsvctrl();
		        ChassisControl();
				
			}
            /*如果到位了*/
            // if ()
            // {
                    // wheelok = 1;
            // }
            break;
        }
        case ARM:
        {
            armok = 0;
            PosCtrl(&hfdcan3, PTP_MODE, ARM1, arm1Tar.pos,
                        1.2f, -1);
            PosCtrl(&hfdcan3, PTP_MODE, ARM2, arm2Tar.pos,
                        1.2f, -1);
        
            static uint32_t waiting_cnt = 0;
            if (fabs(arm1Msg.pos - arm1Tar.pos) <=
                0.02f)
            {
                waiting_cnt++;
                if (waiting_cnt >= 2000)
                {
                    waiting_cnt = 2001;
                    SetBeep(100);
                    armok = 1;
                }
            }
            break;
        }
        case SLIDE:
        {
            
            break;
        }
        case PLATE:
        {

            static bool waiting_flag = 0;
			static uint32_t waiting_cnt = 0;
			if(!waiting_flag)
			{
				waiting_cnt++;
				if(waiting_cnt > 1000)
				{
					waiting_flag = 1;

				}
			}
            else
            {
                plateok = 1;
            }
            break;
        }
        case SERVE:
        {
            PosCtrl(&hfdcan3, PTP_MODE, ARM1, arm1Tar.pos,
                        1.2f, -1);
            PosCtrl(&hfdcan3, PTP_MODE, ARM2, arm2Tar.pos,
                        1.2f, -1);
        
            static uint32_t waiting_cnt = 0;
            if (fabs(arm1Msg.pos - arm1Tar.pos) <=
                0.02f  && fabs(arm2Msg.pos - arm2Tar.pos) <= 0.02f && waiting_cnt != 2001)
            {
                waiting_cnt++;
                if (waiting_cnt >= 2000)
                {
                    waiting_cnt = 2001;
                    SetBeep(100);
                }
            }
            static uint32_t startCount = 0;
            static uint32_t startok = 0;
            if (startok == 0)
            {

                // DEBUG("x %d \r\n",ps_num[3]);
                if (ps_num[3] == 0)
                {
                    startCount++;
                }
                else
                {
                    startCount = 0;
                }
                if (startCount >= 1000)
                {
                    startCount = 1001;
                    SetBeep(400);
                    startok = 1;
                }
            }
            if (waiting_cnt == 2001  && startok == 1)
            {
                static bool waiting_flag = 0;
                static uint32_t waiting_cnt = 0;
                if(!waiting_flag)//不确定必要性
                {
                    waiting_cnt++;
                    if(waiting_cnt > 1000)
                    {
                        waiting_flag = 1;
                    }
                }
                else
                {
                    serveok = 1;
                }
            }
            
            break;
        }
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

//     DriverControl(&FDCAN_TK, slide, &slideTar, &slideMsg);
//     DMControl(&FDCAN_TK, arm1, &arm1Tar);
//     DMControl(&FDCAN_TK, arm2, &arm2Tar);
//     DMControl(&FDCAN_TK, plate, &plateTar);
// }
void MotorControl(void)
{
    switch (gRobot.mode)
    {
        case LEGPVT:
        {
            LegPVTCtrl();
        }
        case LEGPOSE:
        {
            Leg_PosCtrl();
            break;
        }
        case WHEEL:
        {
            hsvctrl();
		
            ChassisControl();
            break;
        }
        case ARM:
        {
        
            PosCtrl(&hfdcan3, PTP_MODE, ARM1, arm1Tar.pos, arm1Tar.pos_kp, arm1Tar.pos_kd);
            PosCtrl(&hfdcan3, PTP_MODE, ARM2, arm2Tar.pos, arm2Tar.pos_kp, arm2Tar.pos_kd);
            break;
        }
        case OSLIDE:
        {
            PosCtrl(&hfdcan3, PTP_MODE, SLIDE, slideTar.pos, slideTar.pos_kp, slideTar.pos_kd);
            break;
        }
        case OPLATE:
        {
            DriverControl(&hfdcan3, PLATE, &plateTar, &plateMsg);
            break;
        }
        case SERVE:
        {
            DriverControl(&hfdcan3, PLATE, &plateTar, &plateMsg);
            PosCtrl(&hfdcan3, PTP_MODE, ARM1, arm1Tar.pos, arm1Tar.pos_kp, arm1Tar.pos_kd);
            PosCtrl(&hfdcan3, PTP_MODE, ARM2, arm2Tar.pos, arm2Tar.pos_kp, arm2Tar.pos_kd);
            break;
        }
    }
}

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