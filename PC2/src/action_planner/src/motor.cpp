/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-12-01 16:24:49
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/motor.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "action_planner/motor.hpp"
#include "action_planner/fpath.hpp"
#include "action_planner/kfs.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

/*-----------------------------------------------------------------------------*/
/*******电机相关参数******* */
float BASE_MOTOR::GetTarVel()
{
    return tarData.Vel;
}

float BASE_MOTOR::GetTarPos()
{
    return tarData.Pos;
}

float BASE_MOTOR::GetTarTor()
{
    return tarData.Tor;
}
float BASE_MOTOR::GetTarPos_Kp()
{
    return tarData.Pos_Kp;
}

float BASE_MOTOR::GetTarPos_Kd()
{
    return tarData.Pos_Kd;
}

float BASE_MOTOR::GetTarVel_Kp()
{
    return tarData.Vel_Kp;
}
float BASE_MOTOR::GetTarVel_Ki()
{
    return tarData.Vel_Ki;
}
/**
 * @brief 获取目标驱动器状态
 * @return 返回目标驱动器状态的布尔值
 *         - true: 驱动器处于目标状态
 *         - false: 驱动器不在目标状态
 */
float BASE_MOTOR::GetActVel()
{
    return actData.Vel;
}

float BASE_MOTOR::GetActPos()
{
    return actData.Pos;
}

float BASE_MOTOR::GetActTor()
{
    return actData.Tor;
}

float BASE_MOTOR::GetActPos_Kp()
{
    return actData.Pos_Kp;
}

float BASE_MOTOR::GetActPos_Kd()
{
    return actData.Pos_Kd;
}
float BASE_MOTOR::GetActVel_Kp()
{
    return actData.Vel_Kp;
}
float BASE_MOTOR::GetAvtVel_Ki(){
    return actData.Vel_Ki;
}

float BASE_MOTOR::GetExCur()
{
    return actData.ex_cur;
}

void BASE_MOTOR::SetTarPara(float tarvel,float tarpos, float tartor)
{
    tarData.Vel = tarvel;
    tarData.Pos = tarpos;
    tarData.Tor = tartor;
}

void BASE_MOTOR::SetTarPosKpKd(float pos_kp, float pos_kd)
{
    tarData.Pos_Kp = pos_kp;
    tarData.Pos_Kd = pos_kd;
}

void BASE_MOTOR::SetTarVelKpKi(float vel_kp, float vel_ki)
{
    tarData.Vel_Kp = vel_kp;
    tarData.Vel_Ki = vel_ki;
}

void BASE_MOTOR::SetCtrlMode(int tarctrlmode)
{
    tarData.tarControlMode = tarctrlmode;
}

void BASE_MOTOR::SetTarClean(bool clean)
{
    tarData.clean = clean;
}

int BASE_MOTOR::GetTarClean()
{
    return tarData.clean;
}

int BASE_MOTOR::GetTarCtrlMode()
{
    return tarData.tarControlMode;
}

void BASE_MOTOR::SetActPara(float actvel,float actpos, float acttor)
{
    actData.Vel = actvel;
    actData.Pos = actpos;
    actData.Tor = acttor;

}

float BASE_MOTOR::CalculationTime(float v, float first_pos, float last_pos, float a, float a2)
{
    float total_distance = fabs(last_pos - first_pos);//总共的路程
    float const_time = (total_distance - (2 * a * v / kPi) - (2 * a2 * v / kPi)) / v;//匀速行驶所需的时间

    return (const_time + a2 + a); 
}

float BASE_MOTOR::QuerySpeed(float v, float first_pos, float last_pos, float a, float a2, int cnt)
{
    float start_t = cnt * 0.005;
    float total_distance = fabs(last_pos - first_pos);//总共的路程
    float current_speed = 0.0;//目前速度
    float distance_covered = 0.0;//目前距离

    float const_time = (total_distance - 2 * a / kPi * v - 2 * a2 / kPi * v) / v;//匀速行驶所需的时间

    int flag = fabs(last_pos - first_pos) / (last_pos - first_pos);
    if (start_t < a)
    {
        
        current_speed = v * sin(kPi * start_t / (2.0 * a));//t0时候的速度(0~PI/2)
        return current_speed * flag;
    }
  
    else if (start_t >= a && start_t <= a + const_time)
    {
        return v * flag;//匀速时的速度
    }
    
    else if (start_t > a + const_time && start_t <= a2 + a + const_time)
    {
        current_speed = v * sin(kPi / 2 * (1 - (start_t - (a + const_time)) / a2));
        return current_speed * flag;
    }
    else if (start_t > a2 + a + const_time && start_t <= a2 + a + const_time + 5)
    {
        return 0;
    }
}

bool BASE_MOTOR::PlanControl(float a, float a2, float v_max, float last_pos, int tim_cnt)
{
    float tarVel = 0;
    float tarPos = 0;
    
    if (tim_cnt == 0)
    {
        this->first_pos = this->GetActPos();
        this->all_time = this->CalculationTime(v_max, first_pos, last_pos, a, a2);
        this->SetTarPara(5, first_pos, 0);
        SetCtrlMode(POS_MODE);
    }

    if (tim_cnt * 0.005 <= all_time)
    {
        tarVel = this->QuerySpeed(v_max, first_pos, last_pos, a, a2, tim_cnt);
        tarPos = tarVel * 0.005 + this->GetTarPos();
        SetTarPara(5, tarPos, 0.0f);
    }
    else if (tim_cnt * 0.005 > all_time)
    {
        SetTarPara(5, last_pos, 0.0f);

        if(abs(GetActPos() - this->tarData.Pos) <= 0.1)
        {
            return true;
        }
    }
    return false;
}

void WHEEL_MOTOR::SetFeedCurr(float curr)
{
    tarData.Tor = curr;
}
