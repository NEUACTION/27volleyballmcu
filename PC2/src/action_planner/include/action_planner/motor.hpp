/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-12-01 16:19:04
 * @FilePath: /ROBOCON2026_base/src/action_planner/include/action_planner/motor.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
/*
  ******************************************************************************
  * @file    motor.hpp
  * @author  wky, noFuckCarrot
  * @last_version V0.0.0._alpha
  * @date    2023.11
  * @now_version V0.0.0._beta
  * @date    2024.12
  * @brief   definition & init of wheel and motor
  *          
  ******************************************************************************
  */
#ifndef _MOTOR_H
#define _MOTOR_H

#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#define DISABLE  (0)
#define VEL_MODE (1)
#define POS_MODE (2)
#define MIT_MODE (4)

struct MotorData
{
    float ex_cur;             // 驱动器计算出的期望电流
    float Vel;                // 速度（r/s）  
    float Pos;                // 位置（r）   
    float Tor;                // 力矩（Nm）
    float Pos_Kp;             // pos比例系数
    float Pos_Kd;             // pos微分系数
    float Vel_Kp;
    float Vel_Ki;
    bool clean;               //清积分
    int tarControlMode;       // 电机控制模式（失能/速度模式/位置模式/MIT模式）
};

/*----------------------------------------------------------------------------*/
//上层电机类，用于控制上层电机的运动。
class BASE_MOTOR
{
public:
    MotorData actData;           //实际参数
    MotorData tarData;           //期望参数
    float first_pos;
    float all_time;

public:
    //构造函数，参数:减速比，最大力矩
    BASE_MOTOR()
    :tarData({0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.6f, 0.5, 0.6, 0, 2}){}
    
    //设置期望PVT参数
    void SetTarPara(float tarvel, float tarpos, float tartor);//设置电机各项期望
    void SetTarPosKpKd(float pos_kp, float pos_kd);
    void SetTarVelKpKi(float vel_kp, float vel_ki);
    void SetCtrlMode(int tarctrlmode);//设置期望控制模式
    void SetActPara(float actvel, float actpos, float acttor);//设置电机实际状态数据
    void SetTarClean(bool clean);
    
    //获取PVT参数
    float GetTarVel();//获取期望速度
    float GetTarPos();//获取期望位置
    float GetTarTor();//获取期望力矩
    float GetTarPos_Kp();//获取期望Kp
    float GetTarPos_Kd();//获取期望Kd
    float GetTarVel_Kp();
    float GetTarVel_Ki();

    float GetActVel();//获取实际速度
    float GetActPos();//获取实际位置
    float GetActTor();//获取实际力矩
    float GetActPos_Kp();//获取实际Kp
    float GetActPos_Kd();//获取实际Kd
    float GetActVel_Kp();
    float GetAvtVel_Ki();
    float GetExCur();//获取实际期望电流

    int GetTarCtrlMode();//获取期望控制模式
    int GetTarClean();

    float CalculationTime(float v, float first_pos, float last_pos, float a, float a2);
    float QuerySpeed(float v, float first_pos, float last_pos, float a, float a2, int cnt);
    bool PlanControl(float a, float a2, float v_max, float last_pos, int tim_cnt);

    float GetAcc(float vlast, float vnext , float s);
    float GetAcc(float vlast, float vnext , float s);
    float GetTimeAtAV(float vlast, float vnext , float a);
    float GetTimeAtSV(float s, float v);
    bool IsArrive()
    {
        return (fabs(this->GetActPos() - this->GetTarPos()) < 1.f ? 1 : 0);
    }
};

//腿电机类
class LEG_MOTOR : public BASE_MOTOR
{
public:
    float SpeedAdjust(float adjust);
    float vel_limit;

public:
    LEG_MOTOR() :BASE_MOTOR() {}
};

//电机类
class SWORD_MOTOR : public BASE_MOTOR
{
public:
    float SpeedAdjust(float adjust);
    float PlateVelPlan( float v_max, float s1, float s2 , float s3 , float s4 , int tim_cnt , float *kp, float *ki);
    bool PlateCtrlPlan(int tim_cnt);
    int vel_limit;

public:
    SWORD_MOTOR() :BASE_MOTOR() {}
};

//轮电机类
class WHEEL_MOTOR : public BASE_MOTOR
{
public:
    float SpeedAdjust(float adjust);
    int vel_limit;

public :
    WHEEL_MOTOR() :BASE_MOTOR() {}

    void SetFeedCurr(float curr);
};

#endif
