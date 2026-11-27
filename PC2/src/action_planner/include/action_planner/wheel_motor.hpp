#ifndef _WHEEL_MOTOR_HPP_
#define _WHEEL_MOTOR_HPP_

#include "stdint.h"
#include "action_planner/motor.hpp"
#include "action_planner/action_planner_node.hpp"
#include <vector>
#include <cstdio>
#include <fstream>

class WHEEL
{
public:
    WHEEL_MOTOR WheelOne;
    WHEEL_MOTOR WheelTwo;
    WHEEL_MOTOR WheelThree;
    WHEEL_MOTOR WheelFour;
   
public:
    WHEEL()
    : WheelOne(), WheelTwo(), WheelThree(), WheelFour(){}

    void SetAllCtrlMode(int tarctrlmode);
};

extern WHEEL wheel;

class CHASSIS_MOTOR
{
public:
    float TarRunVel;
    float TarTurnPos;
    float ActRunVel;
    float ActTurnPos;
    CHASSIS_MOTOR()
    : TarRunVel(0.0f),TarTurnPos(0.0f),ActRunVel(0.0f),ActTurnPos(0.0f){}
};

CHASSIS_MOTOR wheel[4];
#endif