/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-30 21:43:20
 * @FilePath: /ROBOCON2026_base/src/control/include/control/movement.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _MOVEMENT_H
#define _MOVEMENT_H
#include <cmath>
#include "control/controlnode.hpp"
// #include "action_planner/motor.hpp"

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define AXIS_LENGTH 304

class WHEEL_MOTOR
{
public:
    float speed;    // 轮子速度
    float kp;
    float ki;

public:
    WHEEL_MOTOR()
    :speed(0.0f){}

    void SetSpeed(float tarSpeed);
    void SetKpKi(float kp, float ki);
    float GetSpeed(void);
    float GetKp(void);
    float GetKi(void);
};

class ROBOT_MOVEMENT
{
public:
    WHEEL_MOTOR wheel[4];
    float maxVel;
    float maxOmg;
    float maxAcc;
    float velDir;
    float velSum;
    float nowYaw;
    float velX;
    float velY;
    float velOmega;

public:
    ROBOT_MOVEMENT()
    {
        for (int i = 0; i < 4; i++)
        {
            wheel[i] = WHEEL_MOTOR();
        }
        maxAcc = 10;
        maxVel = 10000.f;
        maxOmg = 3600.f;
    }

    void Stop();
    void SetVelLimit(float velLim, float omgLim);
    void VelLimit(float &tarVelDir, float &tarOmg);
    void AccLimit(float &tarVelDir, float &tarOmg);
    void Resolutionofvelocity(float &tarVelDir, float &tarVelSum, float &tarOmg);

    void SetVelocity(float velDir, float velSum, float velOmega, float nowAngle);
    float GetVelX(void);
    float GetVelY(void);
    float GetVelOmega(void);
};

extern ROBOT_MOVEMENT myMovement;
#endif