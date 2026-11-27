#ifndef _MOTIONCARD_H
#define _MOTIONCARD_H
#include <stdint.h>
#include <stdlib.h>
#include "math.h"
#include <unistd.h>
#include <stdio.h>
#include "action_planner/action_planner_node.hpp"

//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    (0.01745329251994f)   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     (57.29577951308232f)
#define ADJUST_KP (1.0f)
#define ENDOFMYROAD 3000.f //路径终点
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define WAYBEGINY 2652.5f
#define WAYLASTY 8547.5f
#define WAYRIGHTX 5347.5f
#define WAYLEFTX  625.5f

extern bool CalculateLenFlag;
extern float lengthWheelTraveled;
extern float lengthSum;
// extern robotVel_t targetVel;

//机器人速度结构体
typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;
typedef struct
{
	float module;
	float direction;
}vector_t;


Path_t GetPosPresent();
float GetPath(void);
inline float CalculatePoint2PointDistance(Point_t point1, Point_t point2);
float CalculateAngleSub(float minuend, float subtrahend);
float AngleControl(float anglePresent,float angleTarget,float kp,float kd);
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd);
vector_t CalculateVectorAdd(vector_t vector1 , vector_t vector2);
robotVel_t GetAdjustVel(Point_t robotPos,Path_t adjustTarget,float vell);
void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel);
void ClearPathLen(void);
void VelCtrlTask(void);
void VelControl(robotVel_t actVel);
Path_t SearchVirPosPoint();
Path_t SearchVirTarPoint(int viewTime);
float OldAngleControl(float anglePresent,float angleTarget,float kp,float kd);
float SimpleAngleControl(float actAngle,float tarAngle, float kp, float kd, float ki);
// robotVel_t GetAdjustVel_PI(Point_t robotPos,Path_t adjustTarget,float vell);
void MoveAsPlanned(const float percent, Path_t posCmd);
vel_data_t PosControl(float tar_x, float act_x, float tar_y, float act_y, float kp1, float kd1, float kp2, float kd2, float actYaw);
int PathFollowing(float percent);

#pragma once

#endif