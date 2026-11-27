/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-12-01 16:44:36
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/leg.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "action_planner/leg.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>

LEG leg;

void LEG::SetAllCtrlMode(int tarctrlmode)
{
    this->LegOne.SetCtrlMode(tarctrlmode);
    this->LegTwo.SetCtrlMode(tarctrlmode);
    this->LegThree.SetCtrlMode(tarctrlmode);
    this->LegFour.SetCtrlMode(tarctrlmode);
}