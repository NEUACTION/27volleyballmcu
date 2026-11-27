/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-30 21:44:56
 * @FilePath: /ROBOCON2026_base/src/control/src/movement.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "control/movement.hpp" 

#define VehicleRadius 342.965f // 轮子距车中心距离
#define SquareRootOfTwo  1.414213562373095f // 根号2
#define WheelRadius   101.6f
#define PI 3.14159265358979323846f

ROBOT_MOVEMENT myMovement;

void WHEEL_MOTOR::SetSpeed(float tarSpeed)
{
    this->speed = tarSpeed;
}

void WHEEL_MOTOR::SetKpKi(float kp, float ki)
{
    this->kp = kp;
    this->ki = ki;
}

float WHEEL_MOTOR::GetSpeed(void)
{
    return this->speed;
}

float WHEEL_MOTOR::GetKp(void)
{
    return this->kp;
}

float WHEEL_MOTOR::GetKi(void)
{
    return this->ki;
}

void ROBOT_MOVEMENT::Stop()
{
    for(int i = 0; i <= 2; i++)
    {
        wheel[i].speed = 0;
    }

	this->velX = 0;
	this->velY = 0;
	this->velOmega = 0;
}

void ROBOT_MOVEMENT::SetVelLimit(float velLim, float omgLim)
{
    this->maxVel = fabs(velLim);
    this->maxOmg = fabs(omgLim);
}

void ROBOT_MOVEMENT::SetVelocity(float velSum, float velDir, float velOmega, float nowAngle)
{
    this->velDir = velDir;
    this->velSum = velSum;
    this->nowYaw = nowAngle;
    this->velOmega = velOmega;
}

float ROBOT_MOVEMENT::GetVelOmega(void)
{
    return this->velOmega;
}

float ROBOT_MOVEMENT::GetVelX(void)
{
    return this->velX;
};

float ROBOT_MOVEMENT::GetVelY(void)
{
    return this->velY;
};

void ROBOT_MOVEMENT::VelLimit(float &tarVelSum, float &tarOmg)
{
    if (tarVelSum > this->maxVel)
    {
        tarVelSum = this->maxVel;
    }
    if (tarVelSum < -this->maxVel)
    {
        tarVelSum = -this->maxVel;
    }

	if(tarOmg > this->maxOmg)
	{
		tarOmg = this->maxOmg;
	}
	if(tarOmg < -this->maxOmg)
	{
		tarOmg = -this->maxOmg;
	}
}

void ROBOT_MOVEMENT::AccLimit(float &tarVelSum, float &tarOmg)
{
    static float last_vel = 0;

    if (tarVelSum - last_vel > maxAcc)
    {
        tarVelSum = last_vel + maxAcc;
    }
    else if (tarVelSum - last_vel < -maxAcc)
    {
        tarVelSum = last_vel - maxAcc;
    }
    last_vel = tarVelSum;
}

void ROBOT_MOVEMENT::Resolutionofvelocity(float &tarVelSum, float &tarVelDir, float &tarOmg)
{
    VelLimit(tarVelSum, tarOmg);

    this->velDir = (tarVelDir - this->nowYaw) / 180 * M_PI;
    this->velX = -1.f * tarVelSum * sinf(this->velDir);
	this->velY = tarVelSum * cosf(this->velDir);
    this->velOmega = tarOmg; 
    //this->velOmega = 0; 
    std::cout << "vel: " << this->velX << " " << this->velY << " " << this->velOmega << std::endl;
    //计算轮子速度
    this->wheel[0].SetSpeed((this->velY - this->velX + ((this->velOmega / 180.0) * PI * (230.0 + 245.0))) / WheelRadius / PI); //右前
    this->wheel[1].SetSpeed((this->velY + this->velX + ((this->velOmega / 180.0) * PI * (230.0 + 245.0))) / WheelRadius / PI); //右后
    this->wheel[2].SetSpeed((this->velY - this->velX - ((this->velOmega / 180.0) * PI * (230.0 + 245.0))) / WheelRadius / PI); //左后
    this->wheel[3].SetSpeed((this->velY + this->velX - ((this->velOmega / 180.0) * PI * (230.0 + 245.0))) / WheelRadius / PI); //左前
    std::cout << "wheelvel: " << this->wheel[0].GetSpeed() << " " << this->wheel[1].GetSpeed() << " " << this->wheel[2].GetSpeed() << " " << this->wheel[3].GetSpeed() << std::endl;
}