#include "chassis_driver.h"

/**
 * @brief  舵轮位置速度控制
 * @param  CANx：所使用的CAN通道编号
 * @param  CMDmode: 指令模式，范围：
 *				PTP_MODE: 点对点模式
 *				BROADCAST_MODE: 广播模式
 * @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64   
 * @param  vel: 速度，范围：-2147483 ~ 2147483      单位r/s 
 * @param  pos: 位置，范围：-2147483 ~ 2147483      单位°
 * @param  hsv_cmd: 舵轮灯板颜色 H: 0 ~ 180 S: 0 ~ 255 V: 0 ~ 255
 * @author ACTION
 */
/* 函数：限幅并打包舵轮位置、速度和 HSV 灯色后发送；工程内未发现有效调用点。 */
void SteeringWheelCtrl(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, float velAim, float PosAim, HSV_Cmd hsv_cmd)
{
    /* stdId 由驱动器编号加服务基址构成；vel/pos 是经过限幅和单位缩放的帧内整数。 */
    uint32_t stdId = DriverNum + DRIVER_SERVER_BASE_ID;
    int32_t vel = 0;
    int32_t pos = 0;
    uint8_t data[16] = {};

    /* 限幅 */
    // int32 -2,147,483,648 ~ 2,147,483,647
    Saturation_float(&velAim, -2147483.f, 2147483.f);
    Saturation_float(&PosAim, -2147483.f, 2147483.f);

    vel = (int32_t)(velAim * 1000.f);// 提高精度
    pos = (int32_t)(PosAim * 10.f);

    data[0]  = IDENTIFIER_POS_SPD_CTRL;
    data[1]  = (pos >> 24) & 0xFF;
    data[2]  = (pos >> 16) & 0xFF;
    data[3]  = (pos >> 8) & 0xFF;
    data[4]  = pos & 0xFF;
    data[5]  = (vel >> 24) & 0xFF;
    data[6]  = (vel >> 16) & 0xFF;
    data[7]  = (vel >> 8) & 0xFF;
    data[8]  = vel & 0xFF;
    data[9]  = hsv_cmd.h;
    data[10] = hsv_cmd.s;
    data[11] = hsv_cmd.v;
    data[12] = IDENTIFIER_SILENCE;

	// 发送
    User_FDCAN_SendMsg(hfdcan, data, 16, stdId, 0);
}

/**
 * @brief  驱动器控制命令
 * @param  CMDmode: 指令模式，命令：
 *				PTP_MODE: 点对点模式
 *				BROADCAST_MODE: 广播模式
 * @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
 * @param  ILIMmode:限流模式，命令：
 *				TEST_MODE :限流10A，限温35°，用于测试 
 *				NORMAL_MODE :限流80A，限温45°，正常使用
 *				EXTREME_MODE :限流150A，限温55°，拒绝平庸，挑战极限！
 * @param  wheelstate
 * @param  turnstate:使能/失能，命令（电机的失能为驱动器停止对电机的驱动，可灵活使用）
 *				ENABLE_ALL :使能
 *				DISABLE_ALL :失能
 * @param  clearintergral: 清除积分标志
 * @author ZBW
 * @note	  
 */
/* 函数：打包舵轮加速度、限流、状态和积分清除位并发送；工程内未发现有效调用点。 */
void DriverStateChassis(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, ACC_MODE acc_mode,     \
                CurrQLim_Mode ILIMmode, DriverState_Mode wheelstate, DriverState_Mode turnstate, uint8_t clearintergral)
{
    /* stdId 标识目标驱动器/命令模式；data 编码状态位并作为本次 CAN 负载。 */
    uint32_t stdId = DriverNum + CMDmode + DRIVER_SERVER_BASE_ID;
    uint8_t data[16] = {};

    data[0]  = IDENTIFIER_SILENCE;
    data[12] = IDENTIFIER_SW_SET;
    data[13] = ((acc_mode & 0x03) << 6) | ((ILIMmode & 0x03) << 4) | 
        ((wheelstate & 0x01) << 3) | ((turnstate & 0x01) << 2) | (clearintergral & 0x03);

    User_FDCAN_SendMsg(hfdcan, data, 16, stdId, 0);
}


/**
 * @brief  舵轮自动回复位置速度
 * @param  driverMsg：对应电机结构体的地址
 * @param  data8: 接收驱动器消息数组的首地址
 * @author ACTION
 */
/* 函数：按反馈标识解析舵轮位置、速度或状态；空参数时返回，工程内未发现有效调用点。 */
void GetSteeringWheelMsg(SteeringWheelMsg_t *driverMsg, const uint8_t *data8)
{
    if ((driverMsg == NULL) || (data8 == NULL))
    {
        return;
    }

    /* identifier 决定当前帧更新位置、速度还是状态字段；未知标识不修改反馈结构。 */
    uint32_t identifier = data8[0];
    switch (identifier)
    {
        case IDENTIFIER_MT1_POS_READ:
        {
            int32_t pos_int = (int32_t)(((data8[1] & 0xFF) << 24) | ((data8[2] & 0xFF) << 16) 
                | ((data8[3] & 0xFF) << 8) | ((data8[4] & 0xFF) << 0));
            driverMsg->pos = (float)pos_int / 10.f;
            break;
        }
        case IDENTIFIER_MT2_SPD_READ:
        {
            int32_t vel_int = (int32_t)(((data8[1] & 0xFF) << 24) | ((data8[2] & 0xFF) << 16) 
                | ((data8[3] & 0xFF) << 8) | ((data8[4] & 0xFF) << 0));
            driverMsg->vel = (float)vel_int / 1000.f;
            break;
        }
        case IDENTIFIER_DRIVER_STATE:
        {
            driverMsg->status = data8[1];
            break;
        }
        case IDENTIFIER_SILENCE:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}