/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-30 10:39:02
 * @FilePath: /ROBOCON2026_base/src/action_planner/include/action_planner/leg.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _LEG_HPP_
#define _LEG_HPP_

#include "stdint.h"
#include "action_planner/motor.hpp"
#include "action_planner/action_planner_node.hpp"
#include <vector>
#include <cstdio>
#include <fstream>

#define Gear0       (0.0)                // 腿初始档位，0mm
#define GearUp1     (2.228)              // 腿1号档位，210mm
#define GearUp2     (4.350)              // 腿2号档位，410mm
#define GearDown1   (2.122)             //200mm
#define GearDown2   (4.244)             //400mm
#define Gear3       (2.33427)              // 腿3号档位，220mm

#define Gear4       (3.7136)             // 310mm
#define Gear5       (1.91)            // 180mm
#define Gear6       (3.5014)            // 330mm
#define Gear7       (1.305)

class LEG
{
public:
    LEG_MOTOR LegOne;
    LEG_MOTOR LegTwo;
    LEG_MOTOR LegThree;
    LEG_MOTOR LegFour;
   
public:
    LEG()
    : LegOne(), LegTwo(), LegThree(), LegFour(){}

    void SetAllCtrlMode(int tarctrlmode);
};

extern LEG leg;

#endif
