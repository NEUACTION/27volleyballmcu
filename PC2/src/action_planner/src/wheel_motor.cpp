#include "action_planner/wheel_motor.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>

WHEEL wheel;

void WHEEL::SetAllCtrlMode(int tarctrlmode)
{
    this->WheelOne.SetCtrlMode(tarctrlmode);
    this->WheelTwo.SetCtrlMode(tarctrlmode);
    this->WheelThree.SetCtrlMode(tarctrlmode);
    this->WheelFour.SetCtrlMode(tarctrlmode);
}
