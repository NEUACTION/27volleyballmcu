/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2026-01-21 11:22:32
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/task.cpp
 * @Description:
 *
 * Copyright (c) 2025 by Action, All Rights Reserved.
 */
#include "action_planner/task.hpp"
#include "action_planner/frecord.hpp"
#include "action_planner/pathfollow.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/leg.hpp"
#include "action_planner/wheel_motor.hpp"
#include "action_planner/sword.hpp"
#include "math.h"

// 状态参量赋值以及初始化

void UpperTask()
{
    switch (plannerNode->UpperTask)
    {
        case uppertest:
        {
            if (true)
            {
                
                sword.Plate.SetTarVelKpKi(1, 1);
                sword.Plate.SetTarPara(10, 0, 0);
                plannerNode->logger()->info("{}", sword.Plate.GetTarVel()); // 注意：原代码此处是 word，可能也是笔误，保留原样
            }
            break;
        }
        default:
            break;
    }
}

void ChassisTask()
{
    switch (plannerNode->ChassisTask)
    { 
        case chassis_test:
        {

            leg.SetAllCtrlMode(DISABLE);

            static int flag = 0;
            if (flag == 0)
            {
                if (Up400())
                {
                    flag = 1;
                }
            }
            else if (flag == 1)
            {
                plannerNode->tarVel.SetVel2(0, 200, 0);
                plannerNode->tarPos.x = plannerNode->nowPos.x;
                plannerNode->tarPos.yaw = plannerNode->nowPos.yaw;
            }

            static bool leg_flag = 0;
            if (!leg_flag)
            {
                // 预留腿部控制逻辑
            }
            break;
        }
        default:
            break;
    }
}

bool CheckSelf(void)
{
    return false;
}

bool DockWeapon(void)
{
    return false;
}

bool Turn_left(void)
{
    return false;
}

bool Turn_right(void)
{
    return false;
}

bool Up400(void)
{
    const auto set_target = [](LEG_MOTOR &motor) {
        motor.SetCtrlMode(POS_MODE);
        motor.SetTarPara(5.0f, GearDown2, 0.0f);
        return motor.IsArrive();
    };

    const bool leg_one_arrived = set_target(leg.LegOne);
    const bool leg_two_arrived = set_target(leg.LegTwo);
    const bool leg_three_arrived = set_target(leg.LegThree);
    const bool leg_four_arrived = set_target(leg.LegFour);

    return leg_one_arrived && leg_two_arrived && leg_three_arrived && leg_four_arrived;
}