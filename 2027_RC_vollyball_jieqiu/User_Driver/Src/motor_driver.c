#include "motor_driver.h"
#include "user_usart.h"

/* 函数：将无符号打包值线性映射到浮点物理量范围；由达妙反馈解析使用。 */
static float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	/* span 和 offset 描述线性映射区间及起点；均仅服务于本次解包换算。 */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* 函数：将浮点物理量限幅并量化为指定位宽无符号值；当前由 PVT 控制组帧使用。 */
static uint32_t float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* span 决定量化范围宽度；bits 非法或范围非正时函数提前返回。 */
    float span = x_max - x_min;

    // 基本保护
    if (bits <= 0) return 0;
    if (span <= 0.0f) return 0;

    // 限幅
    if (x_float < x_min) x_float = x_min;
    if (x_float > x_max) x_float = x_max;

    // levels = (2^bits - 1)，注意 bits==32 时不能用 (1u<<bits)
    /* levels 是目标整数最大值，normalized/q 为归一化和量化临时值；该函数当前无调用点。 */
    uint32_t levels = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);

    // 归一化到 [0,1] 再量化到 [0,levels]
    float normalized = (x_float - x_min) / span;       // [0,1]
    float q = normalized * (float)levels;

    // 四舍五入并夹紧
    uint64_t x_int = (uint64_t)(q + 0.5f);
    if (x_int > (uint64_t)levels) x_int = (uint64_t)levels;

    return (uint32_t)x_int;
}


/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便用户使用 
*/
/* 函数：按普通驱动器反馈标识解析状态、位置、速度、电流及错误；当前由 FDCAN 接收任务调用。 */
void GetDriverMsg(driverMsg_t *driverMsg, const uint8_t *data8)
{
    /* identifier 选择反馈字段，FDCAN_Data 将帧内四字节拼为有符号数据；由本次报文解析使用。 */
    uint32_t identifier = data8[0];
	FDCAN_Trans FDCAN_Data = {};
		
	for (int i = 0; i < 4; i++)
	{
		FDCAN_Data.datac[i] = data8[4 - i];
	}
  
    switch (identifier)
    {
        case IDENTIFIER_SILENCE:
        {
            break;
        }
        case IDENTIFIER_SET_DRIVER_STATE:
        {
            driverMsg->enableState = data8[1] & 0x01;
            break;
        }
        case IDENTIFIER_SET_CTRL_MODE:
        {
            driverMsg->driverMode  = data8[1] & 0x0F;
            break;
        }
        case IDENTIFIER_READ_VEL + IDENTIFIER_READ_OFFSET:
        {
            //获取脉冲速度
            driverMsg->vel = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_POS + IDENTIFIER_READ_OFFSET:
        {
            //获取脉冲位置
            driverMsg->pos = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_ACC + IDENTIFIER_READ_OFFSET:
        {
            //获取加速度
            driverMsg->acc = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_ENCODER_POS + IDENTIFIER_READ_OFFSET:
        {
            //获取编码器脉冲
            driverMsg->encoder = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_CURR_D + IDENTIFIER_READ_OFFSET:
        {
            //获取D轴电流
            driverMsg->currD = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_CURR_Q + IDENTIFIER_READ_OFFSET:
        {
            //获取Q轴电流
            driverMsg->currQ = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_CURR_G + IDENTIFIER_READ_OFFSET:
        {
            //获取母线电流
            driverMsg->currG = FDCAN_Data.datai;
            break;
        }
        case IDENTIFIER_READ_RESET + IDENTIFIER_READ_OFFSET:
        {
            driverMsg->reset_pos_ok_flag = data8[1] & 0x0F;
            //复位情况
            break;
        }
        case IDENTIFIER_RETURN_MSG + IDENTIFIER_READ_OFFSET:
        {
            //pvt，需要注意，和原始版不同，要看电驱
            // int32_t raw = (data8[1] << 16) | (data8[2] << 8) | (data8[3] & 0xFF);
            uint32_t raw = ((uint32_t)data8[1] << 16) | ((uint32_t)data8[2] << 8) | 
                            ((uint32_t)data8[3] & 0xFF);//采取了更安全的写法
            
            if (raw & 0x800000)
            {
                raw |= 0xFF000000;
            }

            // int pos_int = raw;
            int32_t pos_int = (int32_t)raw; //采取了更安全的写法

            int16_t vel_int = (data8[4] << 8) | (data8[5] & 0xFF);
            int16_t cur_int = (data8[6] << 8) | (data8[7] & 0xFF);
            
            float pos = (float)pos_int / 32768.f * 4096.f / 100.f;
            float vel = (float)vel_int / 10.f;
            
            driverMsg->pos = pos;
            driverMsg->vel = vel / 2.f / PI;
            driverMsg->currQ = cur_int / 10.f;
            
            break;
        }
        case IDENTIFIER_REPORT_ERROR:
        {
            driverMsg->error = data8[1];
            break;
        }
        default:
        {

        }
    }
}



// 不管大喵

// /* 函数：解析达妙电机反馈帧并更新位置、速度和扭矩；当前接收分发中的调用已注释。 */
// void DMGetMotorMsg(driverMsg_t *driverMsg, const uint8_t *data)
// { 
// 	// TODO 解算DM8009P的数据
// 	/// unpack ints from can buffer /// 从can的缓冲区得到数据
// 	//没用上的数据
// 	/* id_err 拆分电机编号和错误码；当前位置/速度/扭矩的打包量随后映射为物理量。 */
// 	uint8_t id_err = data[0];
// 	uint8_t id = id_err & 0x0F; // 低4位：电机ID（CAN_ID 低8位）
// 	uint8_t err = id_err >> 4; // 高4位：错误码
// 	(void)id;
// 	(void)err;

// 	uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
// 	uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
// 	uint16_t t_int = ((uint16_t)(data[4] & 0xF) << 8) | data[5];

// 	/// convert uints to floats /// 将uint转换为float
// 	float p = uint_to_float(p_int, P_DM_MIN, P_DM_MAX, 16);
// 	float v = uint_to_float(v_int, V_DM_MIN, V_DM_MAX, 12);
// 	float t = uint_to_float(t_int, T_DM_MIN, T_DM_MAX, 12);

// 	// ----- 写入结构体 -----
// 	driverMsg->pos = p;
// 	driverMsg->vel = v;
// 	driverMsg->torque = t; 
// }

// /*******************************控制驱动器命令************************************/
// /* 函数：按 DM_POS_MODE 发送达妙位置/速度目标；当前达妙控制路径使用。 */
// HAL_StatusTypeDef DMPosCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel)
// {	
//     uint32_t canId = DM_POS_MODE + DriverNum;
// 	uint8_t data[8] = {0};
	
// 	memcpy(&data[0], &pos, 4);
//     memcpy(&data[4], &vel, 4);	
//     return User_FDCAN_SendMsg(hfdcan, data, 8, canId, 0);
// }

// /* 函数：发送达妙电机设置零位命令；工程内调用仅见注释回零代码，当前未启用。 */
// HAL_StatusTypeDef DMSetPosZero(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode)
// {		
// 	uint32_t canId = DriverNum + CtrlMode;
//     uint8_t data[8] = {0};
	
// 	data[0] = 0xFF;
// 	data[1] = 0xFF;
// 	data[2] = 0xFF;
// 	data[3] = 0xFF;
// 	data[4] = 0xFF;
// 	data[5] = 0xFF;
// 	data[6] = 0xFF;
// 	data[7] = 0xFE;
	
// 	return User_FDCAN_SendMsg(hfdcan, data, 8, canId, 0);
// }

// /* 函数：发送达妙电机使能命令；模式由 CtrlMode 指定，当前模式切换路径使用。 */
// HAL_StatusTypeDef DMEnable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode)
// {	
//     uint32_t canId = DriverNum + CtrlMode;
// 	uint8_t data[8] = {0};
	
// 	data[0] = 0xFF;
// 	data[1] = 0xFF;
// 	data[2] = 0xFF;
// 	data[3] = 0xFF;
// 	data[4] = 0xFF;
// 	data[5] = 0xFF;
// 	data[6] = 0xFF;
// 	data[7] = 0xFC;

//     return User_FDCAN_SendMsg(hfdcan, data, 8, canId, 0);
// }

// /* 函数：发送达妙电机失能命令；模式由 CtrlMode 指定，当前模式切换路径使用。 */
// HAL_StatusTypeDef DMDisable(FDCAN_HandleTypeDef *hfdcan, uint16_t DriverNum, uint16_t CtrlMode)
// {
// 	uint32_t canId = DriverNum + CtrlMode;
// 	uint8_t data[8] = {0};
	
// 	data[0] = 0xFF;
// 	data[1] = 0xFF;
// 	data[2] = 0xFF;
// 	data[3] = 0xFF;
// 	data[4] = 0xFF;
// 	data[5] = 0xFF;
// 	data[6] = 0xFF;
// 	data[7] = 0xFD;
	
// 	return User_FDCAN_SendMsg(hfdcan, data, 8, canId, 0);
// }



/**
 * @brief  驱动器复位
 * @param  hfdcan     所使用的 CAN 外设句柄
 * @param  CMDmode    指令模式（PTP_MODE / BROADCAST_MODE）
 * @param  DriverNum  驱动器 ID（广播模式下为组号），范围：1~64
 */
/* 函数：请求普通驱动器回零/复位；工程内调用仅见注释代码，当前未启用。 */
HAL_StatusTypeDef ResetDriver(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};
    
    data[0] = IDENTIFIER_SET_RESET;
    
    data[8]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;
    
    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}
/**
 * @brief  询问驱动器复位是否完成
 * @param  hfdcan     所使用的 CAN 外设句柄
 * @param  CMDmode    指令模式（PTP_MODE / BROADCAST_MODE）
 * @param  DriverNum  驱动器 ID（广播模式下为组号），范围：1~64
 */
/* 函数：读取指定驱动器回零完成状态；当前 reset_control 对腿部调用。 */
HAL_StatusTypeDef IfResetOkFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};
    
    data[0] = IDENTIFIER_READ_RESET + IDENTIFIER_READ_OFFSET;
    
    data[8]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;
    
    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}
/**
 * @brief  驱动器消空程
 * @param  hfdcan     所使用的 CAN 外设句柄
 * @param  CMDmode    指令模式（PTP_MODE / BROADCAST_MODE）
 * @param  DriverNum  驱动器 ID（广播模式下为组号），范围：1~64
 */
/* 函数：清除/归零普通驱动器距离数据；工程内未发现有效调用点。 */
HAL_StatusTypeDef EmptyDriverDistance(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};
    
    data[0] = IDENTIFIER_ELIMINATE_EMPTY_DIATANCE;
    
    data[8]  = IDENTIFIER_SILENCE;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;
    
    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
 * @brief  设置驱动器状态（使能/失能）
 * @param  hfdcan     所使用的 CAN 外设句柄
 * @param  CMDmode    指令模式（PTP_MODE / BROADCAST_MODE）
 * @param  DriverNum  驱动器 ID（广播模式下为组号），范围：1~64
 * @param  state      状态（ENABLE / DISABLE）
 * @note   失能表示驱动器停止对电机驱动；本函数只下发“状态”指令，不切换控制模式。
 * @note   帧格式：data[0]=IDENTIFIER_SET_DRIVER_STATE，data[1]=state，其余填充静默字。
 */
/* 函数：设置普通驱动器使能状态；根据 PTP/广播模式组帧，工程内当前调用点均为注释。 */
HAL_StatusTypeDef SetDriverState(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
                    FunctionalState state)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    data[0] = IDENTIFIER_SET_DRIVER_STATE;
    data[1] = (uint8_t)state;

    data[8]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/* 函数：组合设置驱动器使能状态并下发位置目标；模式切换和状态校正路径使用。 */
HAL_StatusTypeDef SetStateAndPosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
       FunctionalState state, int32_t pos)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    data[0] = IDENTIFIER_SET_DRIVER_STATE;
    data[1] = (uint8_t)state;

    Saturation_int(&pos, -1024 * 4096, 1024 * 4096);

    data[8] = IDENTIFIER_SET_POS_ABS;
    memcpy(&data[9], &pos, sizeof(pos));   // data[9..12] = int32 pos

    // 最后问一下实际
    data[16] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
 * @brief  切换驱动器控制模式
 * @param  hfdcan     所使用的 CAN 外设句柄
 * @param  CMDmode    指令模式（PTP_MODE / BROADCAST_MODE）
 * @param  DriverNum  驱动器 ID（广播模式下为组号），范围：1~64
 * @param  CtlMode    控制模式（VEL_MODE / POS_MODE / PVT_MODE 等）
 * @note   本函数只下发“控制模式”指令，不改变使能/失能状态。
 * @note   帧格式：data[0]=IDENTIFIER_SET_CTRL_MODE，data[1]=CtlMode，其余填充静默字。
 */
/* 函数：切换普通驱动器控制模式并附带位置目标；DriverControl 模式变更分支使用。 */
HAL_StatusTypeDef SetDriverMode(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
                   uint8_t CtlMode, int32_t pos)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};
    
    data[0] = IDENTIFIER_SET_CTRL_MODE;
    data[1] = CtlMode;

    if (CtlMode == VEL_MODE)
    {

    }
    else if (CtlMode == POS_MODE)
    {
        Saturation_int(&pos, -1024 * 4096, 1024 * 4096);
        data[8] = IDENTIFIER_SET_POS_ABS;//AsK
        memcpy(&data[9], &pos, sizeof(pos));
    }

    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
* @brief  电机速度控制和前馈电流
* @param  vel: 速度(脉冲/秒)，范围：-1024*4096 ~ 1024*4096
* @param  kp/ki: 速度环参数（不想设置就给 -1）
*/
/* 函数：下发普通驱动器速度目标及力矩约束；DriverControl 的速度模式调用。 */
HAL_StatusTypeDef VelCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
        int32_t vel, float curr, bool clean)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    Saturation_int(&vel, -1024 * 4096, 1024 * 4096);

    data[0] = IDENTIFIER_SET_VEL;
    memcpy(&data[1], &vel, sizeof(vel));   // data[1..4] = int32 vel

    if (fabs(curr) > 50.0f)
    {
        data[8] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[8] = IDENTIFIER_SET_FEEDFORWARD_CURR;
        float curr_send = curr * 1000.0f;
        memcpy(&data[9], &curr_send, sizeof(curr_send)); // data[9..12] = float
    }   

    // 最后问一下实际
    data[16] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;

    if (clean)
    {
        data[24] = IDENTIFIER_SET_INTEGRAL_CLEARED;
    }
    else
    {
        data[24] = IDENTIFIER_SILENCE;
    }

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
* @brief  电机速度模式调节参数
* @param  kp/ki: 速度环参数（不想设置就给 -1）
*/
/* 函数：更新普通驱动器速度环 KP/KI 参数；参数目标变化时由 DriverControl 调用。 */
HAL_StatusTypeDef SetVelKpKi(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
        float kp, float ki, bool i)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

        if (kp < 0.0f)
        {
            data[0] = IDENTIFIER_SILENCE;
        }
        else
        {
            data[0] = IDENTIFIER_SET_SPD_KP;
            float kp_send = kp * 1000.0f;
            memcpy(&data[1], &kp_send, sizeof(kp_send)); // data[9..12] = float
        }

        // ki
        if (ki < 0.0f)
        {
            data[8] = IDENTIFIER_SILENCE;
        }
        else
        {
            data[8] = IDENTIFIER_SET_SPD_KI;
            float ki_send = ki * 1000.0f;
            memcpy(&data[9], &ki_send, sizeof(ki_send)); // data[17..20] = float
        }


    // 最后问一下实际
    data[16] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;

    if (i)
    {
        data[24] = IDENTIFIER_SET_INTEGRAL_CLEARED;
    }
    else
    {
        data[24] = IDENTIFIER_SILENCE;
    }

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/* 以下旧版速度控制函数定义已注释屏蔽，不会编译或发送命令。 */
// HAL_StatusTypeDef VelCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
//         int32_t vel, float kp, float ki)
// {
//     uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
//     uint8_t  data[32] = {0};

//     Saturation_int(&vel, -1024 * 4096, 1024 * 4096);

//     data[0] = IDENTIFIER_SET_VEL;
//     memcpy(&data[1], &vel, sizeof(vel));   // data[1..4] = int32 vel

//     // kp
//     if (kp < 0.0f)
//     {
//         data[8] = IDENTIFIER_SILENCE;
//     }
//     else
//     {
//         data[8] = IDENTIFIER_SET_SPD_KP;
//         float kp_send = kp * 1000.0f;
//         memcpy(&data[9], &kp_send, sizeof(kp_send)); // data[9..12] = float
//     }

//     // ki
//     if (ki < 0.0f)
//     {
//         data[16] = IDENTIFIER_SILENCE;
//     }
//     else
//     {
//         data[16] = IDENTIFIER_SET_SPD_KI;
//         float ki_send = ki * 1000.0f;
//         memcpy(&data[17], &ki_send, sizeof(ki_send)); // data[17..20] = float
//     }

//     // 最后问一下实际
//     data[24] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;

//     return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
// }


/**
* @brief  电机位置控制
* @param  pos: 位置(脉冲)，范围：-1024*4096 ~ 1024*4096
* @param  kp/kd: 位置环参数（不想设置就给 -1）
*/
/* 函数：下发普通驱动器位置目标及位置环增益；DriverControl 的位置模式调用。 */
HAL_StatusTypeDef PosCtrl(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
        int32_t pos, float kp, float kd)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    Saturation_int(&pos, -1024 * 4096, 1024 * 4096);

    data[0] = IDENTIFIER_SET_POS_ABS;
    memcpy(&data[1], &pos, sizeof(pos));   // data[1..4] = int32 pos

    // kp
    if (kp < 0.0f)
    {
        data[8] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[8] = IDENTIFIER_SET_POS_KP;
        float kp_send = kp * 1000.0f * 60.0f; 
        memcpy(&data[9], &kp_send, sizeof(kp_send)); // data[9..12] = float
    }

    // kd
    if (kd < 0.0f)
    {
        data[16] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[16] = IDENTIFIER_SET_POS_KD;
        float kd_send = kd * 1000.0f;
        memcpy(&data[17], &kd_send, sizeof(kd_send)); // data[17..20] = float
    }

    // 最后问一下实际
    data[24] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}


/**
* @brief  配置电机最大速度/最大力矩/最大Q轴电流
* @param  VelLimit：速度限制(脉冲/秒)，范围：0 ~ 1024*4096（不想设置就给 -1）
* @param  TrqLimit：力矩限制（暂未实现，不想设置就给 -1）
* @param  CurLimit：电流限制(A)，范围：0 ~ 160（不想设置就给 -1）
*/
/* 函数：配置普通驱动器速度/位置等限制值；工程内未发现有效调用点。 */
HAL_StatusTypeDef SetLimits(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
        int32_t VelLimit, int32_t TrqLimit, float CurLimit)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    // 最大速度
    if (VelLimit < 0)
    {
        data[0] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[0] = IDENTIFIER_SET_VEL_LIMIT;
        Saturation_int(&VelLimit, 0, 1024 * 4096);
        memcpy(&data[1], &VelLimit, sizeof(VelLimit)); // data[1..4]
    }

    // 最大力矩（暂时没有）
    if (TrqLimit < 0)
    {
        data[8] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[8] = IDENTIFIER_SILENCE;
    }

    // 最大电流
    if (CurLimit < 0.0f)
    {
        data[16] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[16] = IDENTIFIER_SET_CURRENTQ_LIMIT;
        Saturation_float(&CurLimit, 0.0f, 160.0f);
        float cur_send = CurLimit * 1000.0f;
        memcpy(&data[17], &cur_send, sizeof(cur_send)); // data[17..20]
    }

    // 空闲位
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}


/**
* @brief  配置电机最大加速度/最大减速度
* @param  AccLimit：加速度限制(脉冲/s^2)，范围：0 ~ 1024*4096（不想设置就给 -1）
* @param  DecLimit：减速度限制(脉冲/s^2)，范围：0 ~ 1024*4096（不想设置就给 -1）
*/
/* 函数：配置普通驱动器速度加减速参数；工程内未发现有效调用点。 */
HAL_StatusTypeDef SetVelAcc(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum,
        int32_t AccLimit, int32_t DecLimit)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    // 加速度
    if (AccLimit < 0)
    {
        data[0] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[0] = IDENTIFIER_SET_VEL_ACC;
        Saturation_int(&AccLimit, 0, 1024 * 4096);
        memcpy(&data[1], &AccLimit, sizeof(AccLimit)); // data[1..4]
    }

    // 减速度
    if (DecLimit < 0)
    {
        data[8] = IDENTIFIER_SILENCE;
    }
    else
    {
        data[8] = IDENTIFIER_SET_VEL_DEC;
        Saturation_int(&DecLimit, 0, 1024 * 4096);
        memcpy(&data[9], &DecLimit, sizeof(DecLimit)); // data[9..12]
    }

    // 空闲位
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
 * @brief  PVT 控制（MIT 打包格式）
 * @param  hfdcan     FDCAN 句柄
 * @param  DriverNum  驱动器 ID，范围：1~64
 * @param  pos        位置指令 单位：弧度
 * @param  vel        速度指令 单位：弧度/秒
 * @param  kp         位置环 Kp
 * @param  kd         位置环 Kd
 * @param  t          力矩，现用作电流 单位：A
 * @author YYY
 * @note  MIT 常见 8 字节负载打包（12/12/12/12/8 bit）：
 *        byte(1) p[11:4]
 *        byte(2) p[3:0] | v[11:8]
 *        byte(3) v[7:0]
 *        byte(4) kp[11:4]
 *        byte(5) kp[3:0] | kd[11:8]
 *        byte(6) kd[7:0]
 *        byte(7) t[7:0]
 */
/* 函数：向普通驱动器发送 PVT 位置、速度、增益及前馈量；DriverControl 的 PVT 模式调用。 */
HAL_StatusTypeDef PVTCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, float pos, float vel, float kp, float kd, float t)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + DriverNum;
    uint8_t  data[32] = {0};
    
    uint32_t p_int     = float_to_uint(pos * 1000, P_K_MIN,  P_K_MAX,  12);
    uint32_t v_int     = float_to_uint(vel * 1000, V_K_MIN,  V_K_MAX,  12);
    uint32_t kp_int    = float_to_uint(kp  * 1000, KP_K_MIN, KP_K_MAX, 12);
    uint32_t kd_int    = float_to_uint(kd  * 1000, KD_K_MIN, KD_K_MAX, 12);
    uint32_t t_int     = float_to_uint(t   * 1000, T_K_MIN,  T_K_MAX,  8);
    
    // 存入发送数组
    data[0] = IDENTIFIER_SET_TORQUE;
    data[1] = (uint8_t)(p_int >> 4);                                      // p[11:4]
    data[2] = (uint8_t)(((p_int & 0x0F) << 4)  | (uint8_t)(v_int >> 8));  // p[3:0] + v[11:8]
    data[3] = (uint8_t)(v_int & 0xFF);                                    // v[7:0]
    data[4] = (uint8_t)(kp_int >> 4);                                     // kp[11:4]
    data[5] = (uint8_t)(((kp_int & 0x0F) << 4) | (uint8_t)(kd_int >> 8)); // kp[3:0] + kd[11:8]
    data[6] = (uint8_t)(kd_int & 0xFF);                                   // kd[7:0]
    data[7] = (uint8_t)(t_int  & 0xFF);                                   // t[7:0]
    
    // 空闲位 （以后觉得精度不够从这里拓展出来）
    data[8]  = IDENTIFIER_SILENCE;
    data[16] = IDENTIFIER_SILENCE;
    
    // 最后问一下实际
    data[24] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    
    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}



/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机位置，速度，力矩（电流）
* @param  hfdcan：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author YYY
* @note   单位：脉冲
*/
/* 函数：请求读取普通驱动器 PVT 返回数据；当前 DriverControl 禁用模式分支调用。 */
HAL_StatusTypeDef ReadPVT(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    data[0] = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    // 空闲位
    data[8]  = IDENTIFIER_SILENCE;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/**
* @brief  读取电机编码器值，母线电流，Q轴电流，D轴电流（想改成啥自己改）
* @param  hfdcan：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author YYY
* @note   单位：脉冲，A * 1000
*/
/* 函数：请求读取普通驱动器状态；工程内未发现有效调用点。 */
HAL_StatusTypeDef ReadDriverStatus(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};
    
    data[0]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_READ_ENCODER_POS;
    data[8]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_READ_CURR_G;
    data[16] = IDENTIFIER_READ_OFFSET + IDENTIFIER_READ_CURR_Q;
    data[24] = IDENTIFIER_READ_OFFSET + IDENTIFIER_READ_CURR_D;
    
    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}

/* 函数：请求普通驱动器设定当前位置为零点；工程内调用仅见注释回零代码，当前未启用。 */
HAL_StatusTypeDef SetDriverZero(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
    uint32_t canId    = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    uint8_t  data[32] = {0};

    data[0] = IDENTIFIER_SET_ZERO;
    // 空闲位
    data[8]  = IDENTIFIER_READ_OFFSET + IDENTIFIER_RETURN_MSG;
    data[16] = IDENTIFIER_SILENCE;
    data[24] = IDENTIFIER_SILENCE;

    return User_FDCAN_SendMsg(hfdcan, data, 32, canId, 0);
}