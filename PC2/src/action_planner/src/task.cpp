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
#include "action_planner/trunk.hpp"
#include "math.h"

// 状态参量赋值以及初始化

void UpperTask()
{
    switch (plannerNode->UpperTask)
    {
        case uppertest:
        {
           if ()
           {

                start = 1
                sword.Plate.SetTarVelKpKi(1, 1);
                SetTarPara(10, 0, 0);
           }


            break;
        }

        case get_kfs:
        {
            static bool flag = 0;
            static int trunk_pose = 0;
            static int num = 0;
            static int save_flag = 0;
            plannerNode->myFlags.uppertask = 2;

            if (!save_flag)
            {
                num = plannerNode->lrz.trunk_num;
                save_flag = 1;
            }

            if (!flag)
            {
                if ((trunk_pose = plannerNode->lrz.SearchTrunk()) == 0)
                {
                    plannerNode->UpperTask = error_stop;
                    plannerNode->ChassisTask = no_chassis_task;

                    break;
                }
                flag = 1;
            }

            // static int cnt = 0;
            
            // if (plannerNode->myFlags.throw_flag)
            // {
            //     cnt++;
            //     if (cnt >= 5)
            //     {
            //         plannerNode->Raw_Ctrl(0);
            //         plannerNode->myFlags.throw_flag = 0;
            //         cnt = 0;
            //     }
            // }

            if (plannerNode->lrz.trunk_num == 2 && trunk_pose == 1)
            {
                trunk_pose = 2;
            }
            
            if (TrunkTakePiece(trunk_pose))
            {
                plannerNode->lrz.trunk_num--;

                if (trunk_pose == 4 && plannerNode->lrz.trunk_num < (num - 1))
                {
                    plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
                }

                if (plannerNode->lrz.trunk_num == 0)
                {
                    plannerNode->UpperTask = recive_kfs;
                }
                else if (plannerNode->lrz.trunk_num == num - 1 && plannerNode->re_area_num != 1)
                {
                    plannerNode->UpperTask = save_kfs;

                    if (trunk_pose == 2)
                    {
                        plannerNode->myFlags.saveflag = 1;
                    }
                    else if (trunk_pose == 3 || trunk_pose == 1)
                    {
                        plannerNode->myFlags.saveflag = 3;
                    }
                    else if (trunk_pose == 4)
                    {
                        plannerNode->myFlags.saveflag = 2;
                    }
                }
                else
                {
                    plannerNode->UpperTask = throw_kfs;
                }

                flag = 0;
                trunk_pose = 0;
            }

            break;
        }

        case save_kfs:
        {
            plannerNode->myFlags.uppertask = 3;

            if (plannerNode->myFlags.saveflag == 3)
            {
                static int flag = 0;
                if (flag == 0)
                {
                    if (TrunkTakePiece(8))
                    {
                        flag = 1;
                    }
                }
                else if (flag == 1)
                {
                    if (plannerNode->myFlags.save_ready_flag == 1)
                    {
                        if (TrunkSave(plannerNode->myFlags.saveflag))
                        {
                            flag = 2;
                        }
                    }
                }
                else if (flag == 2)
                {
                    if (plannerNode->myFlags.save_start_flag == 1)
                    {
                        if (TrunkSave(1))
                        {
                            plannerNode->UpperTask = get_kfs;
                        }
                    }
                }
            }
            else
            {
                if (TrunkSave(plannerNode->myFlags.saveflag))
                {
                    plannerNode->UpperTask = get_kfs;
                }
            }

            break;
        }

        case throw_kfs:
        {
            plannerNode->myFlags.uppertask = 4;
            static int flag = 0;
            static int cnt = 0;
            if (flag == 0)
            {
                if (TrunkTakePiece(5))
                {
                    flag = 2;
                }

                if (trunk.Shoulder.GetActPos() >= 4.5)
                {
                    trunk.Elbow.SetTarPara(0.25, 0, 0);
                }

                if (trunk.Shoulder.GetActPos() >= TRUNK_LTH200_SHOULDER)
                {
                    trunk.Elbow.SetTarPara(0.3, 0, 0);
                }

                if (fabs(trunk.Shoulder.GetActPos() - TRUNK_THROW_SHOULDER) < 0.5)
                {
                    plannerNode->Raw_Ctrl(0);
                }
            }
            else if (flag == 1)
            {
                plannerNode->Raw_Ctrl(0);
                flag = 2;
            }
            else if (flag == 2)
            {
                cnt++;
                if (cnt >= 15)
                {
                    flag = 0;
                    cnt = 0;
                    plannerNode->UpperTask = get_kfs;
                }
            }

            break;
        }

        case recive_kfs:
        {
            plannerNode->myFlags.uppertask = 5;
            if (TrunkReceive(5.4, 0.2, 0.2))
            {
                plannerNode->UpperTask = no_upper_task;
            }

            break;
        }

        case get_weapon:
        {
            wheel.WheelOne.SetTarVelKpKi(1.2, 3.0);
            wheel.WheelTwo.SetTarVelKpKi(1.2, 3.0);
            wheel.WheelThree.SetTarVelKpKi(1.2, 3.0);
            wheel.WheelFour.SetTarVelKpKi(1.2, 3.0);
            
            plannerNode->myFlags.uppertask = 1;

            if (GetWeapon())
            {
                plannerNode->chassiswork = dockweapon;
                // plannerNode->chassiswork = nochassiswork;
                plannerNode->UpperTask = no_upper_task;
            }

            break;
        }

        case put_kfs_first:
        {
            plannerNode->myFlags.uppertask = 6;

            switch (plannerNode->upperwork)
            {
                case ready_put:
                {
                    // if (plannerNode->chassiswork == no_r1)
                    // {
                    if (TrunkTakePiece(6))
                    {
                        plannerNode->upperwork = noupperwork;
                    }
                    // }
                    // else
                    // {
                    //     if (TrunkTakePiece(9))
                    //     {
                    //         plannerNode->upperwork = noupperwork;
                    //     }
                    // }
                    break;
                }
                case put_into:
                {
                    static int flag = 0;
                    if (flag == 0)
                    {
                        // if (plannerNode->chassiswork == no_r1)
                        // {
                        if (TrunkPutinto())
                        {
                            flag = 1;
                            trunk.Wrist.SetTarPara(0.6, 0.35, 0);
                        }
                        // }
                        // else
                        // {
                        //     plannerNode->Raw_Ctrl(0);
                        //     trunk.Wrist.SetTarPara(0.6, 0.35, 0);
                        //     flag = 1;
                        // }
                    }
                    else if (flag == 1)
                    {
                        static int cnt = 0;
                        cnt++;
                        if (cnt >= 30)
                        {
                            if (TrunkTakePiece(7))
                            {
                                cnt = 0;
                                plannerNode->UpperTask = put_kfs_second;

                                // if (plannerNode->chassiswork == no_r1)
                                // {
                                plannerNode->upperwork = ready_put; 
                                // }
                                // else
                                // {
                                //     plannerNode->upperwork = noupperwork;
                                // }
                                // plannerNode->upperwork = ready_put;
                                // plannerNode->chassiswork = down350;
                            }
                        }
                    }

                    break;
                }
                case noupperwork:
                {

                    break;
                }
            }
            break;
        }

        case put_kfs_second:
        {
            plannerNode->myFlags.uppertask = 7;

            switch (plannerNode->upperwork)
            {
                case ready_put:
                {
                    if (plannerNode->mode == "re_area3" && plannerNode->re_area_num == 1)
                    {
                        static int flag = 0;
                        if (flag == 0)
                        {
                            if (TrunkTakePiece(7))
                            {
                                flag = 1;
                            }
                        }
                        else if (flag == 1)
                        {
                            if (TrunkPutPiece())
                            {
                                plannerNode->upperwork = noupperwork;
                            }    
                        }
                    }
                    else
                    {
                        if (TrunkPutPiece())
                        {
                            plannerNode->upperwork = noupperwork;
                        }
                    }

                    break;
                }
                case put_into:
                {
                    static int flag = 0;
                    static int cnt = 0;

                    if (flag == 0)
                    {
                        if (TrunkPutinto())
                        {
                            flag = 1;
                            trunk.Wrist.SetTarPara(0.3, 0.30, 0);
                        }
                    }
                    else if (flag == 1)
                    {
                        cnt++;
                        if (cnt >= 50)
                        {
                            if (TrunkReceive(6.8, 0.4, 0.6))
                            {
                                cnt = 0;
                                plannerNode->UpperTask = no_r2_get_kfs;
                                plannerNode->upperwork = noupperwork;
                            }
                        }
                    }

                    break;
                }
                case noupperwork:
                {
                    
                    break;
                }
            }
            break;
        }

        case no_r2_get_kfs:
        {
            switch (plannerNode->upperwork)
            {
                case get_other_kfs:
                {
                    if (plannerNode->ChassisTask == move_to_get_r1_kfs)
                    {
                        if (TrunkTakePiece(1))
                        {
                            plannerNode->upperwork = ready_put;
                        }
                    }
                    else
                    {
                        if (TrunkTakePiece(2))
                        {
                            plannerNode->upperwork = ready_put;
                        }
                    }

                    break;
                }
                // case recive_kfs:
                // {
                //     if (TrunkReceive(2.4, 0.4, 0.5))
                //     {
                //         plannerNode->upperwork = ready_put; 
                //     }
                //     break;
                // }
                case ready_put:
                {
                    if (TrunkTakePiece(6))
                    {
                        plannerNode->upperwork = noupperwork;
                    }
                    break;
                }
                case put_into:
                {
                    static int flag = 0;
                    static int cnt = 0;
                    if (flag == 0)
                    {
                        if (TrunkPutinto())
                        {
                            flag = 1;
                        }
                    }
                    else if (flag == 1)
                    {
                        cnt++;
                        if (cnt >= 50)
                        {
                            if (TrunkReceive(7.2, 0.6, 0.6))
                            {
                                flag = 0;
                                plannerNode->upperwork = noupperwork;
                            }
                        }
                    }

                    break;
                }
                case noupperwork:
                {
                    break;
                }
            }
            break;
        }
        case no_upper_task:
        {
            plannerNode->myFlags.uppertask = 8;
            break;
        }
    }
}

void ChassisTask()
{
    switch (plannerNode->ChassisTask)
    { 
        case chassis_test:
        {
            wheel.SetAllCtrlMode(DISABLE);
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
                //omega = YawControl(plannerNode->nowPos.yaw, 0, 6.8, 1.2);
             }
            // else if (flag == 1)
            // {
            //     if (!plannerNode->Ps_Num[13])
            //     {
            //         cnt++;
            //     }
            //     if (cnt >= 50)
            //     {
            //         flag = 2;
            //     }
            // }
            // else if (flag == 2)
            // {
            //     plannerNode->ChassisTask = area3_move;
            //     plannerNode->chassiswork = upr1;
            // }

            // leg.SetAllCtrlMode(POS_MODE);
            
            
            // static bool flag = 0;

            // if (!flag)
            // {
            //     if (UpR1())
            //     {
            //         flag = 1;
            //         plannerNode->mystep = hind_leg_down;
            //     }
            // }
            // else if (flag == 1)
            // {
            //     if (DownR1())
            //     {
            //         flag = 2;
            //     }
            // }
            
            // if (!flag)
            // {
            //     leg.SetAllCtrlMode(POS_MODE);

            //     leg.LegOne.SetTarPosKpKd(3.2, 0.6);
            //     leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
            //     leg.LegThree.SetTarPosKpKd(4.0, 0.6);
            //     leg.LegFour.SetTarPosKpKd(3.0, 0.6);

            //     static int tim_cnt = -1;
            //     float leg_max_vel = 6.0;
            //     float leg_a1 = 0.1;
            //     float leg_a2 = 0.1;

            //     tim_cnt++;

            //     if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear5, tim_cnt) &
            //         leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear5, tim_cnt) &
            //         leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear5, tim_cnt) &
            //         leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear5, tim_cnt))
            //     {
            //         flag = 1;
            //     }
            // }
            static bool leg_flag = 0;

            // plannerNode->Vavle_Chassis_Ctrl(0, 1);

            if (!leg_flag)
            {

                // if (UpR1())
                // {
                //     leg_flag = 1;
                // }


                // leg.SetAllCtrlMode(POS_MODE);
            
                // leg.LegOne.SetTarPosKpKd(3.2, 0.6);
                // leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
                // leg.LegThree.SetTarPosKpKd(3.4, 0.6);
                // leg.LegFour.SetTarPosKpKd(3.0, 0.6);

                // static int tim_cnt = -1;
                // float leg_max_vel = 11.0;
                // float leg_a1 = 0.2;
                // float leg_a2 = 0.2;

                // tim_cnt++;

                // if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                //     leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                //     leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                //     leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt))
                // {
                //     leg_flag = 1;
                // }

                // leg.SetAllCtrlMode(POS_MODE);

                // leg.LegOne.SetTarPosKpKd(3.2, 0.6);
                // leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
                // leg.LegThree.SetTarPosKpKd(4.0, 0.6);
                // leg.LegFour.SetTarPosKpKd(3.0, 0.6);

                // static int tim_cnt = -1;
                // float leg_max_vel = 10.0;
                // float leg_a1 = 0.1;
                // float leg_a2 = 0.1;

                // tim_cnt++;

                // if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                //     leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                //     leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                //     leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt))
                // {
                //    leg_flag = 1;
                // }

            }

            // plannerNode->tarVel.SetVel2(0, 100, 0);
            
            break;
        }
        case area1_move:
// ==================== 区域：区域1底盘任务处理 开始 ====================
{
    plannerNode->myFlags.chassistask = 1;
    // 标记底盘任务状态为1

    switch (plannerNode->chassiswork)
    // 根据当前底盘工作状态进行分支切换
    {
        case moveweapon:
        // ==================== 区域：底盘工作状态：移动武器 开始 ====================
        {
            wheel.WheelOne.SetTarVelKpKi(4.0, 2.6);
            // 设置轮1速度环PID参数Kp=4.0, Ki=2.6
            wheel.WheelTwo.SetTarVelKpKi(4.0, 2.0);
            // 设置轮2速度环PID参数Kp=4.0, Ki=2.0
            wheel.WheelThree.SetTarVelKpKi(3.8, 2.6);
            // 设置轮3速度环PID参数Kp=3.8, Ki=2.6
            wheel.WheelFour.SetTarVelKpKi(4.2, 2.6);//4.0，2.6
            // 设置轮4速度环PID参数Kp=4.2, Ki=2.6（原注释：4.0，2.6）

            plannerNode->myFlags.chassistask = 11;
            // 标记底盘任务状态为11

            if (MoveWeapon())
            // 判断移动武器动作是否完成
            {
                plannerNode->UpperTask = get_weapon;
                // 动作完成后切换上层任务为获取武器
                plannerNode->chassiswork = nochassiswork;
                // 底盘工作状态置为无
                // plannerNode->PublishToYyy(502, plannerNode->nowPos.y, plannerNode->nowPos.z, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, -90.0f, true);
                // correctLidar(502, -1, -90);
            }
            break;
            // 跳出当前分支
        }
        // ==================== 区域：底盘工作状态：移动武器 结束 ====================

        case dockweapon:
        // ==================== 区域：底盘工作状态：对接武器 开始 ====================
        {
            plannerNode->myFlags.chassistask = 12;
            // 标记底盘任务状态为12

            static bool clean_flag = 0;
            // 静态变量，用于记录是否已经初始化清理操作
            
            if (!clean_flag)
            // 首次进入时设置轮子清理标志
            {
                wheel.WheelOne.SetTarClean(1);
                // 设置轮1清理标志为1
                wheel.WheelTwo.SetTarClean(1);
                // 设置轮2清理标志为1
                wheel.WheelThree.SetTarClean(1);
                // 设置轮3清理标志为1
                wheel.WheelFour.SetTarClean(1);
                // 设置轮4清理标志为1
                clean_flag = 1;
                // 标记清理操作已初始化
            }
            else
            // 后续进入时清除轮子清理标志
            {
                wheel.WheelOne.SetTarClean(0);
                // 设置轮1清理标志为0
                wheel.WheelTwo.SetTarClean(0);
                // 设置轮2清理标志为0
                wheel.WheelThree.SetTarClean(0);
                // 设置轮3清理标志为0
                wheel.WheelFour.SetTarClean(0);
                // 设置轮4清理标志为0
            }

            wheel.WheelOne.SetTarVelKpKi(3.6, 2.2);
            // 设置轮1速度环PID参数Kp=3.6, Ki=2.2
            wheel.WheelTwo.SetTarVelKpKi(3.8, 2.2);
            // 设置轮2速度环PID参数Kp=3.8, Ki=2.2
            wheel.WheelThree.SetTarVelKpKi(3.6, 2.2);
            // 设置轮3速度环PID参数Kp=3.6, Ki=2.2
            wheel.WheelFour.SetTarVelKpKi(3.6, 2.0);//3.0，2.8
            // 设置轮4速度环PID参数Kp=3.6, Ki=2.0（原注释：3.0，2.8）
            
            if (DockWeapon())
            // 判断对接武器动作是否完成
            {
                // plannerNode->PublishToYyy(502, plannerNode->nowPos.y, plannerNode->nowPos.z, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, 90.0f, true);
                // plannerNode->Raw_Ctrl(1);
                // plannerNode->chassiswork = nochassiswork;
                plannerNode->chassiswork = waitr1;
                // 对接完成后切换底盘工作状态为等待R1
                plannerNode->mystep = wheel_pos;
                // 设置腿部步骤为轮子位置控制
            }

            break;
            // 跳出当前分支
        }
        // ==================== 区域：底盘工作状态：对接武器 结束 ====================

        case waitr1:
        // ==================== 区域：底盘工作状态：等待R1 开始 ====================
        {
            plannerNode->myFlags.chassistask = 13;
            // 标记底盘任务状态为13
            if (WaitR1())
            // 判断是否等到R1
            {
                // plannerNode->chassiswork = nochassiswork;
                plannerNode->chassiswork = gomove;
                // 等到R1后切换底盘工作状态为去移动
            }
            break;
            // 跳出当前分支
        }
        // ==================== 区域：底盘工作状态：等待R1 结束 ====================

        case gomove:
        // ==================== 区域：底盘工作状态：去移动 开始 ====================
        {
            plannerNode->myFlags.chassistask = 14;
            // 标记底盘任务状态为14
            wheel.WheelOne.SetTarVelKpKi(3.0, 2.4);
            // 设置轮1去移动时速度环PID参数Kp=3.0, Ki=2.4
            wheel.WheelTwo.SetTarVelKpKi(3.2, 2.8);
            // 设置轮2去移动时速度环PID参数Kp=3.2, Ki=2.8
            wheel.WheelThree.SetTarVelKpKi(3.2, 2.8);
            // 设置轮3去移动时速度环PID参数Kp=3.2, Ki=2.8
            wheel.WheelFour.SetTarVelKpKi(3.0, 2.4);//3.2，2.6
            // 设置轮4去移动时速度环PID参数Kp=3.0, Ki=2.4（原注释：3.2，2.6）
            static int num = 0;
            // 静态变量，用于存储选择的路径编号
            static bool flag = 0;
            // 静态标志，记录是否已完成路径选择

            if (flag == 0)
            // 如果尚未完成路径选择
            {
                if (plannerNode->lrz.R2.way_num == 1)
                // 如果R2路径编号为1
                {
                    num = 3; 
                    // 对应选择路径3
                }
                if (plannerNode->lrz.R2.way_num == 2)
                // 如果R2路径编号为2
                {
                    num = 2;   
                    // 对应选择路径2
                }
                if (plannerNode->lrz.R2.way_num == 3)
                // 如果R2路径编号为3
                {
                    num = 1;
                    // 对应选择路径1
                }

                if (plannerNode->lrz.trunk_first[0] == 1)
                // 如果躯干优先标志第0位为1
                {
                    num = 3;
                    // 强制选择路径3
                }
                if (plannerNode->lrz.trunk_first[1] == 1)
                // 如果躯干优先标志第1位为1
                {
                    num = 2;
                    // 强制选择路径2
                }
                if (plannerNode->lrz.trunk_first[2] == 1)
                // 如果躯干优先标志第2位为1
                {
                    num = 1;
                    // 强制选择路径1
                }

                if (((plannerNode->lrz.R2.FirsrR2KFS >> 0) & 0x01) && ((plannerNode->lrz.R2.FirsrR2KFS >> 1) & 0x01) && plannerNode->lrz.R2.way_num == 3)
                // 如果满足特殊KFS组合条件且R2路径为3
                {
                    num = 3;
                    // 强制选择路径3
                }

                flag = 1;
                // 标记路径选择已完成
            }

            // if (GoMove(3))
            // {
            //     plannerNode->ChassisTask = area2_move3;
            //     plannerNode->chassiswork = up400;
            //     plannerNode->mystep = all_leg_down;
            // }

            if (GoMove(num))
            // 判断去移动动作是否完成
            {
                plannerNode->lxData.mode = 1;
                // 去移动完成后切换通信模式为1

                if (num == 3 && plannerNode->lrz.R2.way_num != 1)
                // 如果选择路径3但与R2实际路径不匹配
                {
                    plannerNode->ChassisTask = area2_change_move;
                    // 进入区域2换道移动任务
                }
                else if (num == 3)
                // 如果选择路径3且路径匹配
                {
                    plannerNode->ChassisTask = area2_move3;
                    // 进入区域2移动3任务
                    plannerNode->chassiswork = up400;
                    // 设置底盘工作状态为抬升400
                }

                if (num == 2 && plannerNode->lrz.R2.way_num != 2)
                // 如果选择路径2但与R2实际路径不匹配
                {
                    plannerNode->ChassisTask = area2_change_move;
                    // 进入区域2换道移动任务
                }
                else if (num == 2)
                // 如果选择路径2且路径匹配
                {
                    plannerNode->ChassisTask = area2_move2;
                    // 进入区域2移动2任务
                    plannerNode->chassiswork = up200; 
                    // 设置底盘工作状态为抬升200
                }

                if (num == 1 && plannerNode->lrz.R2.way_num != 3)
                // 如果选择路径1但与R2实际路径不匹配
                {
                    plannerNode->ChassisTask = area2_change_move;
                    // 进入区域2换道移动任务
                }
                else if (num == 1)
                // 如果选择路径1且路径匹配
                {
                    plannerNode->ChassisTask = area2_move1;
                    // 进入区域2移动1任务
                    plannerNode->chassiswork = up400;
                    // 设置底盘工作状态为抬升400
                }

                flag = 0;
                // 重置路径选择标志
                plannerNode->mystep = all_leg_down;
                // 设置腿部步骤为全腿放下
            }
        
            break;
            // 跳出当前分支
        }
        // ==================== 区域：底盘工作状态：去移动 结束 ====================

        case nochassiswork:
        // ==================== 区域：底盘工作状态：无底盘工作 开始 ====================
        {
            plannerNode->myFlags.chassistask = 15;
            // 标记底盘任务状态为15
            break;
            // 跳出当前分支
        }
        // ==================== 区域：底盘工作状态：无底盘工作 结束 ====================
    }
    // 内层switch结束
    break;
    // 跳出外层case
}
// ==================== 区域：区域1底盘任务处理 结束 ====================
        case area2_change_move:
        {
            plannerNode->myFlags.chassistask = 2;
            static int i = 0;

            if (plannerNode->lrz.trunk_first[0] == 1 && plannerNode->lrz.R2.way_num == 3) //先最左后最右
            {
                i = 1;
            }
            else if (plannerNode->lrz.trunk_first[0] == 1 && plannerNode->lrz.R2.way_num == 2) //先最左后中间
            {
                i = 2;
            }
            else if (plannerNode->lrz.trunk_first[1] == 1 && plannerNode->lrz.R2.way_num == 3) //先中间后最右
            {
                i = 3;
            }
            else if (plannerNode->lrz.trunk_first[1] == 1 && plannerNode->lrz.R2.way_num == 1) //先中间后最左
            {
                i = 4;
            }
            else if (plannerNode->lrz.trunk_first[2] == 1 && plannerNode->lrz.R2.way_num == 1) //先最右后最左
            {
                i = 5;
            }
            else if (plannerNode->lrz.trunk_first[2] == 1 && plannerNode->lrz.R2.way_num == 2) //先最右后中间
            {
                i = 6;
            }

            if (((plannerNode->lrz.R2.FirsrR2KFS >> 0) & 0x01) && ((plannerNode->lrz.R2.FirsrR2KFS >> 2) & 0x01) && plannerNode->lrz.R2.way_num == 2) //先两边后中间
            {
                i = 7;
            }
            else if (((plannerNode->lrz.R2.FirsrR2KFS >> 0) & 0x01) && ((plannerNode->lrz.R2.FirsrR2KFS >> 1) & 0x01) && plannerNode->lrz.R2.way_num == 3) //后最左
            {
                i = 8;
            }
            else if (((plannerNode->lrz.R2.FirsrR2KFS >> 1) & 0x01) && ((plannerNode->lrz.R2.FirsrR2KFS >> 2) & 0x01) && plannerNode->lrz.R2.way_num == 1) //后最右
            {
                i = 9;
            }

            static bool flag = 0;
            if (!flag)
            {
                if (i == 1 || i == 2 || i == 8)
                {
                    plannerNode->tarPos.x = 4225 * plannerNode->myFlags.RB_flag;
                }
                else if (i == 3 || i == 4)
                {
                    plannerNode->tarPos.x = 3025 * plannerNode->myFlags.RB_flag;
                }
                else if (i == 5 || i == 6 || i == 7 || i == 9)
                {
                    plannerNode->tarPos.x = 1825 * plannerNode->myFlags.RB_flag;
                }
                flag = 1;
            }

            static float tarvelx = 0;

            float omega = YawControl(plannerNode->nowPos.yaw, 0, 4.8, 0.8);

            if (plannerNode->myFlags.trunk_flag)
            {
                static int tim_cnt = 0;
                static bool right_flag = 0;
                static bool left_flag = 0;
                static int flag = 0;
                static bool first_flag = 0;

                if (plannerNode->myFlags.aimFlag == 0)
                {
                    if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                    {
                        tim_cnt = 0;
                        right_flag = 1;
                        plannerNode->tarVel.SetVel2(-300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                    {
                        tim_cnt = 0;
                        left_flag = 1;
                        plannerNode->tarVel.SetVel2(300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                    {
                        plannerNode->lxData.start_test = 1;
                        plannerNode->myFlags.aimFlag = 1;
                    }
                    else
                    {
                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 3.6, 0.6, 0, 0).x;
                        plannerNode->tarVel.SetVel2(tarvelx, 250, omega);
                    }
                }
                else
                {
                    plannerNode->tarPos.x = plannerNode->nowPos.x + plannerNode->lxData.kfs_dist;

                    tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.5, 0.8, 0, 0).x;

                    plannerNode->tarVel.SetVel2(tarvelx, 250, omega);
                }
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 3.6, 0.8, 0, 0).x;
                plannerNode->tarVel.SetVel2(tarvelx, 300, omega);
            }

            if (NoUp(i))
            {
                plannerNode->myFlags.save_ready_flag = 1;

                plannerNode->myFlags.aimFlag = 0;
                
                if (plannerNode->lrz.R2.way_num == 1)
                {
                    plannerNode->ChassisTask = area2_move3;
                    plannerNode->chassiswork = up400;
                }
                if (plannerNode->lrz.R2.way_num == 2)
                {
                    plannerNode->ChassisTask = area2_move2;
                    plannerNode->chassiswork = up200;
                }
                if (plannerNode->lrz.R2.way_num == 3)
                {
                    plannerNode->ChassisTask = area2_move1;
                    plannerNode->chassiswork = up400;
                }
            }

            break;
        }
        case area2_move1:
        {
            plannerNode->myFlags.chassistask = 3;
            plannerNode->LastChassisTask = area2_move1;
            static float tarvelx = 0;
            float omega = YawControl(plannerNode->nowPos.yaw, 0, 6.8, 1.2);

            static bool pos_flag = 0;
            if (!pos_flag)
            {
                plannerNode->tarPos.x = 1825 * plannerNode->myFlags.RB_flag;
                pos_flag = 1;
            }

            if (plannerNode->chassiswork == up400 && plannerNode->mystep == all_leg_down)
            {
                plannerNode->tarPos.x = 1825 * plannerNode->myFlags.RB_flag;
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                plannerNode->tarVel.SetVel2(tarvelx, 1000, omega);
            }
            else if (plannerNode->myFlags.trunk_flag && plannerNode->myFlags.getKFS_flag)
            {
                static int tim_cnt = 0;
                static bool right_flag = 0;
                static bool left_flag = 0;
                static int flag = 0;
                static bool first_flag = 0;

                if (plannerNode->myFlags.aimFlag == 0)
                {
                    if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                    {
                        tim_cnt = 0;
                        right_flag = 1;
                        plannerNode->tarVel.SetVel2(-300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                    {
                        tim_cnt = 0;
                        left_flag = 1;
                        plannerNode->tarVel.SetVel2(300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                    {
                        if (!plannerNode->myFlags.htl_flag)
                        {
                            plannerNode->lxData.start_test = 1;
                            plannerNode->myFlags.aimFlag = 1;
                        }
                        else
                        {
                            // if (tim_cnt == 0 && first_flag == 1)
                            // {
                            //     plannerNode->myFlags.aimFlag = 1;
                            //     flag = 1;
                            // }

                            tim_cnt++;

                            if (tim_cnt >= 5)
                            {
                                plannerNode->tarPos.x = plannerNode->nowPos.x;
                                plannerNode->myFlags.aimFlag = 1;
                                flag = 0;
                                tim_cnt = 0;
                            }
                        }
                        plannerNode->tarVel.SetVel2(0, 0, omega);
                    }
                    else
                    {
                        tim_cnt = 0;
                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                }
                else
                {
                    if (!plannerNode->myFlags.htl_flag)
                    {
                        plannerNode->tarPos.x = plannerNode->nowPos.x + plannerNode->lxData.kfs_dist;

                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;

                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                    else
                    {
                        if (!flag)
                        {
                            static float pos_x[10] = {0};
                            static int cnt = 0;
                            static float sum = 0;

                            pos_x[cnt] = plannerNode->nowPos.x;
                            cnt++;
                            
                            if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                            {
                                if (left_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(-250, 0, omega);
                            }
                            else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                            {
                                if (right_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(250, 0, omega);
                            }

                            if (cnt == 10)
                            {
                                for(int i = 0; i < cnt; i++)
                                {
                                    sum += pos_x[i];
                                }
                                plannerNode->tarPos.x = sum / cnt;
                                flag = 1;
                                sum = 0;
                                cnt = 0;
                            }
                        }
                        else
                        {
                            if (fabs(plannerNode->tarPos.x - plannerNode->nowPos.x) <= 20)
                            {
                                if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
                                {
                                    plannerNode->myFlags.aimFlag = 0;
                                    right_flag = 0;
                                    left_flag = 0;
                                }
                            }

                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                            right_flag = 0;
                            left_flag = 0;
                            plannerNode->tarVel.SetVel2(tarvelx, 500, omega);

                        }   
                    }
                }
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
            }

            switch (plannerNode->chassiswork)
            {
                case up400:
                {
                    plannerNode->myFlags.chassistask = 31;

                    static bool flag = 0;
                    if (!flag)
                    {
                        if ((plannerNode->lrz.R2.FirsrR2KFS >> 2) & 0x01)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }
                        flag = 1;
                    }
                    
                    if (Up400())
                    {
                        static bool yyy_flag = 0;
                        if (!yyy_flag)
                        {
                            // plannerNode->PublishToYyy(plannerNode->nowPos.x, plannerNode->nowPos.y, 400, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, plannerNode->nowPos.yaw, true);
                            yyy_flag = 1;
                        }

                        if (plannerNode->lrz.trunk_flag[1][1] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_left;
                            plannerNode->lastchassiswork = up200;
                        }
                        else if (plannerNode->lrz.trunk_flag[1][2] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_right;
                            plannerNode->lastchassiswork = up200;
                        }
                        else
                        {
                            plannerNode->chassiswork = up200;
                        }

                        if (plannerNode->lrz.trunk_flag[1][3] != 0)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }

                        plannerNode->mystep = all_leg_down;
                    }
                    break;
                }
                case up200:
                {
                    plannerNode->myFlags.chassistask = 32;
                    if (Up200())
                    {
                        static bool yyy_flag = 0;
                        if (!yyy_flag)
                        {
                            // plannerNode->PublishToYyy(plannerNode->nowPos.x, plannerNode->nowPos.y, 600, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, plannerNode->nowPos.yaw, true);
                            yyy_flag = 1;
                        }

                        if (plannerNode->lrz.trunk_flag[2][3] != 0)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }

                        if (plannerNode->lrz.trunk_flag[2][1] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_left;
                            plannerNode->lastchassiswork = down200;
                        }
                        else if (plannerNode->lrz.trunk_flag[2][2] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_right;
                            plannerNode->lastchassiswork = down200;
                        }
                        else
                        {
                            plannerNode->chassiswork = down200;
                        }

                        plannerNode->mystep = front_leg_down;
                    }
                    break;
                }
                case down200:
                {
                    plannerNode->myFlags.chassistask = 33;
                    if (Down200())
                    {
                        static int next_flag = 0;

                        static bool flag = 0;
                        if (!flag)
                        {
                            if (plannerNode->lrz.trunk_flag[3][3] != 0)
                            {
                                plannerNode->myFlags.getKFS_flag = 1;
                            }
                            else
                            {
                                plannerNode->myFlags.getKFS_flag = 0;
                            }
                            flag = 1;
                        }
                        next_flag++;

                        if (next_flag == 1)
                        {
                            static bool yyy_flag = 0;
                            if (!yyy_flag)
                            {
                                // plannerNode->PublishToYyy(plannerNode->nowPos.x, plannerNode->nowPos.y, 400, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, plannerNode->nowPos.yaw, true);
                                yyy_flag = 1;
                            }
                            if (plannerNode->lrz.trunk_flag[3][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = down200;
                                if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == -1)
                                {
                                    plannerNode->myFlags.turn_leg_flag = 1;
                                }
                            }
                            else if (plannerNode->lrz.trunk_flag[3][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = down200;
                                if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == 1)
                                {
                                    plannerNode->myFlags.turn_leg_flag = 1;
                                }
                            }
                            else
                            {
                                plannerNode->chassiswork = down200;
                            }
                        }

                        if (next_flag == 2)
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                            plannerNode->myFlags.last_platform_flag = 1;
                            static bool yyy_flag = 0;
                            if (!yyy_flag)
                            {
                                // plannerNode->PublishToYyy(plannerNode->nowPos.x, plannerNode->nowPos.y, 200, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, plannerNode->nowPos.yaw, true);
                                yyy_flag = 1;
                            }
                            if (plannerNode->lrz.trunk_flag[4][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = down200;
                            }
                            else if (plannerNode->lrz.trunk_flag[4][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = down200;
                            }
                            else
                            {
                                plannerNode->chassiswork = down200;
                            }
                        }

                        if (next_flag == 3)
                        {
                            plannerNode->ChassisTask = area3_move;
                            plannerNode->chassiswork = carup;
                            // plannerNode->chassiswork = nochassiswork;
                            next_flag = 0;
                        }

                        plannerNode->mystep = front_leg_down;
                    }

                    break;
                }
                case nochassiswork:
                {
                    plannerNode->myFlags.chassistask = 34;
                    
                    wheel.SetAllCtrlMode(DISABLE);
                    trunk.Elbow.SetCtrlMode(DISABLE);
                    trunk.Shoulder.SetCtrlMode(DISABLE);
                    trunk.Wrist.SetCtrlMode(DISABLE);
                    leg.SetAllCtrlMode(DISABLE);
                    break;
                }
                break;
            }

            if ((plannerNode->nowPos.y > 2600 && plannerNode->nowPos.y < 3200) 
                || (plannerNode->nowPos.y > 3850 && plannerNode->nowPos.y < 4400)
                || (plannerNode->nowPos.y > 5050 && plannerNode->nowPos.y < 5600)
                || (plannerNode->nowPos.y > 6250 && plannerNode->nowPos.y < 6800)
                || (plannerNode->nowPos.y > 7450 && plannerNode->nowPos.y < 8000))
            {
                plannerNode->myFlags.pre_deceleration = 1;
            }
            else
            {
                plannerNode->myFlags.pre_deceleration = 0;
            }

            break;
        }
        case area2_move2:
        {
            plannerNode->myFlags.chassistask = 4;
            plannerNode->LastChassisTask = area2_move2;
            static float tarvelx = 0;
            float omega = YawControl(plannerNode->nowPos.yaw, 0, 6.8, 1.2);

            static bool pos_flag = 0;
            if (!pos_flag)
            {
                plannerNode->tarPos.x = 3025 * plannerNode->myFlags.RB_flag;
                pos_flag = 1;
            }

            if (plannerNode->chassiswork == up200 && plannerNode->mystep == all_leg_down)
            {
                static int flag = 0;
                if (flag == 0)
                {
                    plannerNode->tarPos.x = 3025 * plannerNode->myFlags.RB_flag;
                    flag = 1;
                }

                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 3.6, 0.8).x;

                plannerNode->tarVel.SetVel2(tarvelx, 1000, omega);
            }
            else if (plannerNode->myFlags.trunk_flag && plannerNode->myFlags.getKFS_flag)
            {
                static int tim_cnt = 0;
                static bool right_flag = 0;
                static bool left_flag = 0;
                static int flag = 0;
                static bool first_flag = 0;

                if (plannerNode->myFlags.aimFlag == 0)
                {
                    if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                    {
                        tim_cnt = 0;
                        right_flag = 1;
                        plannerNode->tarVel.SetVel2(-300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                    {
                        tim_cnt = 0;
                        left_flag = 1;
                        plannerNode->tarVel.SetVel2(300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                    {
                        if (!plannerNode->myFlags.htl_flag)
                        {
                            plannerNode->lxData.start_test = 1;
                            plannerNode->myFlags.aimFlag = 1;
                        }
                        else
                        {
                            // if (tim_cnt == 0 && first_flag == 1)
                            // {
                            //     plannerNode->myFlags.aimFlag = 1;
                            //     flag = 1;
                            // }

                            tim_cnt++;

                            if (tim_cnt >= 5)
                            {
                                plannerNode->tarPos.x = plannerNode->nowPos.x;
                                plannerNode->myFlags.aimFlag = 1;
                                flag = 0;
                                tim_cnt = 0;
                            }
                        }

                        plannerNode->tarVel.SetVel2(0, 0, omega);
                    }
                    else
                    {
                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                }
                else
                {
                    if (!plannerNode->myFlags.htl_flag)
                    {
                        plannerNode->tarPos.x = plannerNode->nowPos.x + plannerNode->lxData.kfs_dist;

                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;

                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                    else
                    {
                        if (!flag)
                        {
                            static float pos_x[10] = {0};
                            static int cnt = 0;
                            static float sum = 0;

                            pos_x[cnt] = plannerNode->nowPos.x;
                            cnt++;
                            
                            if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                            {
                                if (left_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(-250, 0, omega);
                            }
                            else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                            {
                                if (right_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(250, 0, omega);
                            }

                            if (cnt == 10)
                            {
                                for(int i = 0; i < cnt; i++)
                                {
                                    sum += pos_x[i];
                                }
                                plannerNode->tarPos.x = sum / cnt;
                                flag = 1;
                                sum = 0;
                                cnt = 0;
                            }
                        }
                        else
                        {
                            if (fabs(plannerNode->tarPos.x - plannerNode->nowPos.x) <= 20)
                            {
                                if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
                                {
                                    plannerNode->myFlags.aimFlag = 0;
                                    right_flag = 0;
                                    left_flag = 0;
                                }
                            }
                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                            right_flag = 0;
                            left_flag = 0; 
                            plannerNode->tarVel.SetVel2(tarvelx, 500, omega);

                        }   
                    }
                }
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 2.4, 0.4).x;
                plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
            }

            switch (plannerNode->chassiswork)
            {
                case up200:
                {
                    static bool flag = 0;
                    
                    if (!flag)
                    {
                        if ((plannerNode->lrz.R2.FirsrR2KFS >> 1) & 0x01)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }
                        flag = 1;
                    }
                    
                    plannerNode->myFlags.chassistask = 41;
                    if (Up200())
                    {
                        static int next_flag = 0;
                        
                        next_flag++;
                        
                        if (next_flag == 1)
                        {
                            // if (plannerNode->UpperTask == no_upper_task)
                            // {
                            //     plannerNode->UpperTask = get_kfs;
                            // }

                            if (plannerNode->lrz.trunk_flag[1][3] != 0)
                            {
                                plannerNode->myFlags.getKFS_flag = 1;
                            }
                            else
                            {
                                plannerNode->myFlags.getKFS_flag = 0;
                            }

                            if (plannerNode->lrz.trunk_flag[1][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = up200;
                            }
                            else if (plannerNode->lrz.trunk_flag[1][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = up200;
                            }
                            else
                            {
                                plannerNode->chassiswork = up200;
                            }

                            plannerNode->mystep = all_leg_down;
                        }

                        if (next_flag == 2)
                        {
                            if (plannerNode->lrz.trunk_flag[2][3] != 0)
                            {
                                plannerNode->myFlags.getKFS_flag = 1;
                            }
                            else
                            {
                                plannerNode->myFlags.getKFS_flag = 0;
                            }

                            if (plannerNode->lrz.trunk_flag[2][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == 1)
                                {
                                    plannerNode->myFlags.turn_leg_flag = 1;
                                }
                                plannerNode->lastchassiswork = up200;
                            }
                            else if (plannerNode->lrz.trunk_flag[2][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = up200;
                                if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == -1)
                                {
                                    plannerNode->myFlags.turn_leg_flag = 1;
                                }
                            }
                            else
                            {
                                plannerNode->chassiswork = up200;
                            }
                            
                            plannerNode->mystep = all_leg_down;
                        }

                        if (next_flag == 3)
                        {
                            if (plannerNode->lrz.trunk_flag[3][3] != 0)
                            {
                                plannerNode->myFlags.getKFS_flag = 1;
                            }
                            else
                            {
                                plannerNode->myFlags.getKFS_flag = 0;
                            }

                            if (plannerNode->lrz.trunk_flag[3][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = down200;
                            }
                            else if (plannerNode->lrz.trunk_flag[3][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = down200;
                            }
                            else
                            {
                                plannerNode->chassiswork = down200;
                            }

                            plannerNode->mystep = front_leg_down;

                            next_flag = 0;
                        }
                    }

                    break;
                }
                case down200:
                {
                    plannerNode->myFlags.chassistask = 42;
                    if (Down200())
                    {
                        plannerNode->myFlags.getKFS_flag = 0;
                        if (plannerNode->lrz.trunk_flag[4][1] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_left;
                            plannerNode->lastchassiswork = down400;
                        }
                        else if (plannerNode->lrz.trunk_flag[4][2] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_right;
                            plannerNode->lastchassiswork = down400;
                        }
                        else
                        {
                            plannerNode->chassiswork = down400;
                        }
                        plannerNode->mystep = front_leg_down;
                    }
                    break;
                }
                case down400:
                {
                    plannerNode->myFlags.chassistask = 43;
                    if (Down400())
                    {
                        plannerNode->ChassisTask = area3_move;
                        plannerNode->chassiswork = carup;
                        // plannerNode->chassiswork = nochassiswork;
                        // plannerNode->mystep = nostep;
                    }
                    break;
                }
                case nochassiswork:
                {
                    plannerNode->myFlags.chassistask = 44;
                    wheel.SetAllCtrlMode(DISABLE);
                    trunk.Elbow.SetCtrlMode(DISABLE);
                    trunk.Shoulder.SetCtrlMode(DISABLE);
                    trunk.Wrist.SetCtrlMode(DISABLE);
                    leg.SetAllCtrlMode(DISABLE);
                    break;
                }
            }

            if (fabs(plannerNode->nowPos.yaw) > 5)
            {
                plannerNode->tarVel.SetVelY(0);
            }

            if ((plannerNode->nowPos.y > 2650 && plannerNode->nowPos.y < 3200) 
                || (plannerNode->nowPos.y > 3850 && plannerNode->nowPos.y < 4400)
                || (plannerNode->nowPos.y > 5050 && plannerNode->nowPos.y < 5600)
                || (plannerNode->nowPos.y > 6250 && plannerNode->nowPos.y < 6800)
                || (plannerNode->nowPos.y > 7550 && plannerNode->nowPos.y < 8000))
            {
                plannerNode->myFlags.pre_deceleration = 1;
            }
            else
            {
                plannerNode->myFlags.pre_deceleration = 0;
            }

            break;
        }
        case area2_move3:
        {
            plannerNode->myFlags.chassistask = 5;
            plannerNode->LastChassisTask = area2_move3;
            static float tarvelx = 0;
            static float lastvely = 0;
            float omega = YawControl(plannerNode->nowPos.yaw, 0, 6.8, 1.2);

            static bool pos_flag = 0;
            if (!pos_flag)
            {
                plannerNode->tarPos.x = 4225 * plannerNode->myFlags.RB_flag;
                pos_flag = 1;
            }

            if (plannerNode->chassiswork == up400 && plannerNode->mystep == all_leg_down)
            {
                plannerNode->tarPos.x = 4225 * plannerNode->myFlags.RB_flag;

                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 2.8, 0.4).x;

                plannerNode->tarVel.SetVel2(tarvelx, 1000, omega);
            }
            else if (plannerNode->myFlags.trunk_flag && plannerNode->myFlags.getKFS_flag)
            {
                static int tim_cnt = 0;
                static bool right_flag = 0;
                static bool left_flag = 0;
                static int flag = 0;
                static bool first_flag = 0;

                if (plannerNode->myFlags.aimFlag == 0)
                {
                    if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                    {
                        tim_cnt = 0;
                        right_flag = 1;
                        plannerNode->tarVel.SetVel2(-300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                    {
                        tim_cnt = 0;
                        left_flag = 1;
                        plannerNode->tarVel.SetVel2(300, 0, omega);
                    }
                    else if (!plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                    {
                        if (!plannerNode->myFlags.htl_flag)
                        {
                            plannerNode->lxData.start_test = 1;
                            plannerNode->myFlags.aimFlag = 1;
                        }
                        else
                        {
                            // if (tim_cnt == 0 && first_flag == 1)
                            // {
                            //     plannerNode->myFlags.aimFlag = 1;
                            //     flag = 1;
                            // }

                            tim_cnt++;

                            if (tim_cnt >= 5)
                            {
                                plannerNode->tarPos.x = plannerNode->nowPos.x;
                                plannerNode->myFlags.aimFlag = 1;
                                flag = 0;
                                tim_cnt = 0;
                            }
                        }

                        plannerNode->tarVel.SetVel2(0, 0, omega);
                    }
                    else
                    {
                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                }
                else
                {
                    if (!plannerNode->myFlags.htl_flag)
                    {
                        plannerNode->tarPos.x = plannerNode->nowPos.x + plannerNode->lxData.kfs_dist;

                        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;

                        plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
                    }
                    else
                    {
                        if (!flag)
                        {
                            static float pos_x[10] = {0};
                            static int cnt = 0;
                            static float sum = 0;

                            pos_x[cnt] = plannerNode->nowPos.x;
                            cnt++;
                            
                            if (plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
                            {
                                if (left_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(-250, 0, omega);
                            }
                            else if (plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                            {
                                if (right_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_x[i];
                                    }
                                    plannerNode->tarPos.x = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVel2(250, 0, omega);
                            }

                            if (cnt == 10)
                            {
                                for(int i = 0; i < cnt; i++)
                                {
                                    sum += pos_x[i];
                                }
                                plannerNode->tarPos.x = sum / cnt;
                                flag = 1;
                                sum = 0;
                                cnt = 0;
                            }
                        }
                        else
                        {
                            if (fabs(plannerNode->tarPos.x - plannerNode->nowPos.x) <= 20)
                            {
                                if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
                                {
                                    plannerNode->myFlags.aimFlag = 0;
                                    right_flag = 0;
                                    left_flag = 0;
                                }
                            }
                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 0, 0).x;
                            right_flag = 0;
                            left_flag = 0;
                            plannerNode->tarVel.SetVel2(tarvelx, 500, omega);

                        }   
                    }
                }
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 4.8, 1.0, 3.0, 0.6).x;
                plannerNode->tarVel.SetVel2(tarvelx, 500, omega);
            }

            switch (plannerNode->chassiswork)
            {
                case up400:
                {
                    static bool flag = 0;

                    if (!flag)
                    {
                        if ((plannerNode->lrz.R2.FirsrR2KFS >> 0) & 0x01)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }

                        flag = 1;
                    }

                    plannerNode->myFlags.chassistask = 51;
                    
                    if (Up400())
                    {
                        // if (plannerNode->UpperTask == no_upper_task)
                        // {
                        //     plannerNode->UpperTask = get_kfs;
                        // }

                        if (plannerNode->lrz.trunk_flag[1][3] != 0)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }

                        if (plannerNode->lrz.trunk_flag[1][1] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_left;
                            plannerNode->lastchassiswork = down200;
                        }
                        else if (plannerNode->lrz.trunk_flag[1][2] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_right;
                            plannerNode->lastchassiswork = down200;
                        }
                        else
                        {
                            plannerNode->chassiswork = down200;
                        }
                        plannerNode->mystep = front_leg_down;
                    }
                    break;
                }
                case down200:
                {
                    plannerNode->myFlags.chassistask = 52;
                    if (Down200())
                    {
                        static int nextflag = 0;
                        
                        nextflag++;

                        if (nextflag == 1)
                        {
                            if (plannerNode->lrz.trunk_flag[2][3] != 0)
                            {
                                plannerNode->myFlags.getKFS_flag = 1;
                            }
                            else
                            {
                                plannerNode->myFlags.getKFS_flag = 0;
                            }

                            if (plannerNode->lrz.trunk_flag[2][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = up200;
                            }
                            else if (plannerNode->lrz.trunk_flag[2][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = up200;
                            }
                            else
                            {
                                plannerNode->chassiswork = up200;
                            }   
                            plannerNode->mystep = all_leg_down;
   
                        }
                        else if (nextflag == 2)
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                            plannerNode->myFlags.last_platform_flag = 1;

                            if (plannerNode->lrz.trunk_flag[4][1] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_left;
                                plannerNode->lastchassiswork = down200;
                            }
                            else if (plannerNode->lrz.trunk_flag[4][2] != 0)
                            {
                                plannerNode->ChassisTask = turn;
                                plannerNode->chassiswork = turn_right;
                                plannerNode->lastchassiswork = down200;
                            }
                            else
                            {
                                plannerNode->chassiswork = down200;
                            }
                            plannerNode->mystep = front_leg_down;
                        }
                        else
                        {
                            plannerNode->ChassisTask = area3_move;
                            plannerNode->chassiswork = carup;
                            // plannerNode->chassiswork = nochassiswork;
                        }
                    }

                    break;
                }
                case up200:
                {
                    plannerNode->myFlags.chassistask = 53;
                    if (Up200())
                    {
                        if (plannerNode->lrz.trunk_flag[3][3] != 0)
                        {
                            plannerNode->myFlags.getKFS_flag = 1;
                        }
                        else
                        {
                            plannerNode->myFlags.getKFS_flag = 0;
                        }

                        if (plannerNode->lrz.trunk_flag[3][1] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_left;
                            plannerNode->lastchassiswork = down200;
                            if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == 1)
                            {
                                plannerNode->myFlags.turn_leg_flag = 1;
                            }
                        }
                        else if (plannerNode->lrz.trunk_flag[3][2] != 0)
                        {
                            plannerNode->ChassisTask = turn;
                            plannerNode->chassiswork = turn_right;
                            plannerNode->lastchassiswork = down200;
                            if (plannerNode->lrz.trunk_num == 2 && plannerNode->myFlags.RB_flag == -1)
                            {
                                plannerNode->myFlags.turn_leg_flag = 1;
                            }
                        }
                        else
                        {
                            plannerNode->chassiswork = down200;
                        }   

                        plannerNode->mystep = front_leg_down;
                    }

                    break;
                } 

                case nochassiswork:
                {
                    plannerNode->myFlags.chassistask = 54;
                    wheel.SetAllCtrlMode(DISABLE);
                    trunk.Elbow.SetCtrlMode(DISABLE);
                    trunk.Shoulder.SetCtrlMode(DISABLE);
                    trunk.Wrist.SetCtrlMode(DISABLE);
                    leg.SetAllCtrlMode(DISABLE);
                    break;
                }

            }

            if ((plannerNode->nowPos.y > 2575 && plannerNode->nowPos.y < 3200) 
                || (plannerNode->nowPos.y > 3900 && plannerNode->nowPos.y < 4400)
                || (plannerNode->nowPos.y > 5050 && plannerNode->nowPos.y < 5600)
                || (plannerNode->nowPos.y > 6300 && plannerNode->nowPos.y < 6800)
                || (plannerNode->nowPos.y > 7550 && plannerNode->nowPos.y < 8000))
            {
                plannerNode->myFlags.pre_deceleration = 1;
            }
            else
            {
                plannerNode->myFlags.pre_deceleration = 0;
            }

            break;
        }
        case area3_move:
        {
            plannerNode->myFlags.chassistask = 6;

            switch (plannerNode->chassiswork)
            {
                case carup:
                {
                    wheel.WheelOne.SetTarVelKpKi(3.4, 2.4);
                    wheel.WheelTwo.SetTarVelKpKi(3.2, 2.8);
                    wheel.WheelThree.SetTarVelKpKi(3.4, 2.8);
                    wheel.WheelFour.SetTarVelKpKi(3.4, 2.8);
                    
                    plannerNode->myFlags.chassistask = 61;
                    plannerNode->lxData.start_test = 2;
                    
                    if(CarGo2())
                    {
                        if (plannerNode->tlrData.start)
                        {
                            plannerNode->chassiswork = yes_r1;
                            
                        }
                        else
                        {
                            plannerNode->chassiswork = no_r1;
                        }
                    }
                    break;
                }
                case yes_r1:
                {
                    static bool flag = 0;
                    static float omega = 0;
                    static float tarVelx = 0;
            
                    if (!flag)
                    {
                        omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 3.6, 0.6);
                        plannerNode->tarVel.SetVel2(0, 0, omega);

                        if (plannerNode->R1StarData.r1movedirection != 0)
                        {
                            plannerNode->tarVel.SetVel2(0, 0, 0);
                            flag = 1;
                        }
                    }
                    else
                    {
                        if (YesR1())
                        {
                            plannerNode->chassiswork = upr1;
                            plannerNode->mystep = all_leg_down;
                        }
                    }

                    break;
                }
                case upr1:
                {
                    plannerNode->lxData.mode = 1;
                    plannerNode->myFlags.chassistask = 62;
                    
                    static bool align_flag = false;
                    static int align_cnt = 0;
                    static bool up_flag = false;
                    static float lastpos_x = 0;
                    static float lastpos_y = 0;
                    static float tarvelx = 0;
                    static float tarvely = 0;
                    static float omega = 0;
                    static bool up_ok = 0;

                    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 3.6, 0.6);

                    if (!up_flag)
                    {
                        plannerNode->Vavle_Chassis_Ctrl(0, 1);

                        if (!plannerNode->R1StarData.r1platehigh || plannerNode->tlrData.r1_is_putting)
                        {
                            tarvelx = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).x;
                            tarvely = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).y;
                            
                            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

                            if (fabs(fabs(plannerNode->nowPos.x) - 2800) <= 50 && fabs(plannerNode->nowPos.y - 10750) <= 50)
                            {
                                plannerNode->tarVel.SetVel2(0, 0, 0);
                            }
                        }
                        else
                        {
                            if(!up_flag)
                            {
                                if (!align_flag)
                                {
                                    if (plannerNode->myFlags.RB_flag == 1)
                                    {
                                        static bool flag = 0;

                                        if (!flag)
                                        {
                                            plannerNode->tarPos.x = plannerNode->nowPos.x - (plannerNode->tlrData.depth - 450) * plannerNode->myFlags.RB_flag;
                                            if (plannerNode->tlrData.depth <= 400 && plannerNode->tlrData.distance != 0)
                                            {
                                                plannerNode->tarPos.x = plannerNode->nowPos.x + 200 * plannerNode->myFlags.RB_flag;
                                                flag = 1;
                                            }
                                        }
                                    }
                                    else if (plannerNode->myFlags.RB_flag == -1)
                                    {
                                        plannerNode->tarPos.x = plannerNode->nowPos.x - (plannerNode->tlrData.depth - 450) * plannerNode->myFlags.RB_flag;
                                        if (fabs(plannerNode->tarPos.x) <= 1500)
                                        {
                                            plannerNode->tarPos.x = -1500;
                                        }
                                    }
                                    plannerNode->tarPos.y = plannerNode->nowPos.y + plannerNode->tlrData.distance * plannerNode->myFlags.RB_flag;
                                    align_flag = 1;
                                }
                                else
                                {
                                    if (plannerNode->myFlags.RB_flag == 1)
                                    {
                                        if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) >= 400)
                                        {
                                            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.4, 1.8, 0.4).y;
                                        }
                                        else
                                        {
                                            if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) >= 100)
                                            {
                                                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 2.2, 0.4).y;
                                            }
                                            else
                                            {
                                                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 4.5, 1.0).y;     
                                            }
                                        }

                                        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) >= 400)
                                        {
                                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 2.8, 0.4).x;
                                        }
                                        else
                                        {
                                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.4, 2.4, 0.4).x;

                                            if (fabs(plannerNode->tarPos.x - plannerNode->nowPos.x) <= 50)
                                            {
                                                up_ok = 1;
                                            }
                                        }
                                    }
                                    else if (plannerNode->myFlags.RB_flag == -1)
                                    {
                                        if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) >= 400)
                                        {
                                            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.4, 2.4, 0.6).y;
                                        }
                                        else
                                        {
                                            if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) >= 100)
                                            {
                                                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 2.8, 0.4).y;    
                                            }
                                            else
                                            {
                                                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 3.8, 0.6).y;    
                                            }
                                        }

                                        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) >= 400)
                                        {
                                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.6, 2.8, 0.4).x;
                                        }
                                        else
                                        {
                                            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.0, 0.6, 2.4, 0.4).x;

                                            if (fabs(plannerNode->tarPos.x - plannerNode->nowPos.x) <= 50)
                                            {
                                                up_ok = 1;
                                            }
                                        }
                                    }
                                }

                                if (plannerNode->tlrData.depth != lastpos_x || plannerNode->tlrData.distance != lastpos_y)
                                {
                                    align_flag = 0;
                                }

                                lastpos_x = plannerNode->tlrData.depth;
                                lastpos_y = plannerNode->tlrData.distance;

                                plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

                                static float real_vely = 0;
                                real_vely = (-wheel.WheelOne.GetActVel() + wheel.WheelTwo.GetActVel() - wheel.WheelThree.GetActVel() + wheel.WheelFour.GetActVel()) * PI * 101.6 / 4.0; 

                                if(fabs(plannerNode->tlrData.distance) <= 10 && plannerNode->tlrData.distance != 0 && fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) <= 3 && fabs(real_vely) <= 50 && up_ok)
                                {
                                    align_cnt++;
                                    if (align_cnt >= 9)
                                    {
                                        // plannerNode->tarVel.SetVel2(0, 0, 0);
                                        if (plannerNode->R1StarData.r1platehigh)
                                        {
                                            up_flag = true;
                                            plannerNode->tarPos.y = plannerNode->nowPos.y;
                                        }
                                        else if (!plannerNode->R1StarData.r1platehigh)
                                        {
                                            plannerNode->tarVel.SetVel2(0, 0, 0);
                                        }
                                    }
                                }
                                else
                                {
                                    align_cnt = 0;
                                }
                            }
                        }
                    }

                    if (up_flag)
                    {   
                        // plannerNode->Vavle_Upper_Ctrl(0, 0, 1, 0);
                        // plannerNode->tarVel.SetVel2(0, 0, 0);
                        plannerNode->lxData.start_test = 2;

                        plannerNode->lxData.lx_posz = leg.LegOne.GetActPos() * 30 * PI;

                        static float dif_posy = 0;
                        static float last_distance = 0;
                        static bool first_flag = 0;
                        
                        if (plannerNode->lxData.lx_posx > 1000)
                        {
                            plannerNode->logger()->info("lx_posx:{}", plannerNode->lxData.lx_posx);
                            plannerNode->logger()->info("last_lx_posx:{}", last_distance);
                            plannerNode->logger()->info("dif_posy:{}", dif_posy);
                            plannerNode->logger()->info("tarPosy:{}", plannerNode->tarPos.y);

                            if (last_distance != plannerNode->lxData.col_second && plannerNode->lxData.col_second != 0)
                            {
                                last_distance = plannerNode->lxData.col_second;
                                
                                dif_posy = 10750 - (last_distance * plannerNode->myFlags.RB_flag);

                                if (fabs(last_distance) >= 800)
                                {
                                    dif_posy = plannerNode->nowPos.y;
                                }
                                else
                                {
                                    correctLidar(-1, dif_posy, 33605);

                                    if (!first_flag)
                                    {
                                        plannerNode->tarPos.y = plannerNode->nowPos.y;
                                        first_flag = 1;
                                    }
                                }
                            }
                        }

                        if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) >= 30)
                        {
                            tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 4.8, 0.6).y;
                            plannerNode->logger()->info("tarvy1:{}", tarvely);
                        }
                        else
                        {
                            tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 3.6, 0.6).y;
                            plannerNode->logger()->info("tarvy2:{}", tarvely);
                        }

                        plannerNode->logger()->info("upR1");

                        plannerNode->tarVel.SetVel2(-375 * plannerNode->myFlags.RB_flag, tarvely, omega);
                        
                        if(UpR1())
                        {
                            align_flag = 0;
                            align_cnt = 0;
                            up_flag = 0;
                            plannerNode->myFlags.upr1_flag = 1;
                            plannerNode->chassiswork = up200;
                        }
                    }

                    break;
                }
                case up200:
                {
                    plannerNode->myFlags.chassistask = 63;
                    static int cnt = 0;
                    static int tim_cnt = 0;
                    static int flag = 0;
                    static float firstpos_x = 0;
                    static float firstpos_y = 0;
                    static float firstpos_z = 0;
                    static float lastpos_x = 0;
                    static float lastpos_y = 0;
                    static float lastpos_z = 0;

                    if (flag == 0)
                    {
                        if (UpPut(Gear5))
                        {
                            flag = 1;
                            plannerNode->upperwork = ready_put;
                        }
                    }
                    else if (flag == 1)
                    {
                        if (cnt == 0)
                        {
                            plannerNode->R1StarData.R1PlatformFlag = 1;
                            if (plannerNode->R1StarData.R1PlatformFlag && plannerNode->upperwork != ready_put && plannerNode->upperwork != put_into)
                            {
                                cnt = 1;
                                plannerNode->myFlags.upr1_flag = 0;
                                plannerNode->upperwork = put_into;
                            }
                        }
                        else if (cnt == 1)
                        {
                            static int r1_flag = 0;                           

                            if (!plannerNode->R1StarData.R1MoveFlag && r1_flag == 0)
                            {
                                r1_flag = 1;
                            }
                            if (r1_flag == 1 && plannerNode->R1StarData.R1MoveFlag)
                            {
                                r1_flag = 2;
                            }

                            if (!plannerNode->R1StarData.R1PlatformFlag && plannerNode->upperwork != put_into)
                            {
                                plannerNode->mystep = hind_leg_down;
                                plannerNode->chassiswork = down200;
                                plannerNode->myFlags.downr1_mode = 0;
                                cnt = 0;
                            }

                            if (r1_flag == 2)
                            {
                                if (plannerNode->upperwork != ready_put)
                                {
                                    plannerNode->upperwork = put_into;
                                    plannerNode->myFlags.downr1_mode = 1;
                                    cnt = 3;
                                }
                            }
                        }
                        else if (cnt == 3)
                        {
                            plannerNode->logger()->info("cnt: {}", cnt);
                            if (!plannerNode->R1StarData.R1PlatformFlag)
                            {
                                plannerNode->mystep = hind_leg_down;
                                plannerNode->chassiswork = down200;
                                cnt = 0;
                            }
                        }
                    }
                    
                    break;
                }
                case down200:
                {
                    wheel.WheelFour.SetTarVelKpKi(3.0, 2.8);

                    plannerNode->myFlags.chassistask = 64;

                    static bool clean_flag = 0;
                    if (!clean_flag)
                    {
                        wheel.WheelOne.SetTarClean(1);
                        wheel.WheelTwo.SetTarClean(1);
                        wheel.WheelThree.SetTarClean(1);
                        wheel.WheelFour.SetTarClean(1);
                        clean_flag = 1;
                    }
                    else
                    {
                        wheel.WheelOne.SetTarClean(0);
                        wheel.WheelTwo.SetTarClean(0);
                        wheel.WheelThree.SetTarClean(0);
                        wheel.WheelFour.SetTarClean(0);
                    }
                    
                    if (DownPut())
                    {
                        plannerNode->chassiswork = downr1;
                    }

                    break;
                }
                case downr1:
                {
                    plannerNode->myFlags.chassistask = 65;
                    static int flag = 0;
                    static float tarvelx = 0;
                    static float tarvely = 0;
                    static float omega = 0;

                    // wheel.SetAllCtrlMode(VEL_MODE);
                    wheel.WheelOne.SetTarVelKpKi(3.0, 2.4);
                    wheel.WheelTwo.SetTarVelKpKi(3.0, 2.8);
                    wheel.WheelThree.SetTarVelKpKi(3.0, 3.0);

                    plannerNode->tarVel.SetVel2(200 * plannerNode->myFlags.RB_flag, 0, 0);

                    if (flag == 0)
                    {
                        if(DownR1())
                        {
                            // plannerNode->upperwork = ready_put;
                            // plannerNode->chassiswork = gotoputsecond;
                            
                            // if (plannerNode->UpperTask == no_r2_get_kfs)
                            // {
                            //     plannerNode->ChassisTask = no_chassis_task;
                            // }
                            // plannerNode->chassiswork = nochassiswork;
                            flag = 1;
                        }
                    }
                    else if (flag == 1)
                    {
                        tarvelx = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).x;
                        tarvely = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).y;
                        omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 3.6, 0.6);
                        
                        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

                        if (fabs(fabs(plannerNode->nowPos.x) - 2800) <= 50 && fabs(plannerNode->nowPos.y - 10750) <= 50)
                        {
                            plannerNode->tarVel.SetVel2(0, 0, 0);
                            flag = 2;
                        }
                    }
                    else if (flag == 2)
                    {
                        plannerNode->tarVel.SetVel2(0, 0, 0);

                        if (!plannerNode->R1StarData.r1platehigh)
                        {
                            if (plannerNode->tlrData.kfs_start)
                            {
                                plannerNode->ChassisTask = move_to_get_ground_kfs;
                            }
                            else
                            {
                                plannerNode->ChassisTask = move_to_get_r1_kfs;
                            }
                        }
                    }
                
                    break;
                }
                case no_r1:
                {
                    plannerNode->myFlags.chassistask = 66;
                    
                    static int flag = 0;

                    if (flag == 0)
                    {
                        if (NoR1())
                        {
                            flag = 1;
                        }
                    }
                    else if (flag == 1)
                    {
                        if (plannerNode->tlrData.start)
                        {
                            plannerNode->chassiswork = upr1;
                            plannerNode->mystep = all_leg_down;
                        }
                    }


                    break;
                }
                case gotoputsecond:
                {
                    plannerNode->myFlags.chassistask = 67;
                    if (GoToPutSecond())
                    {
                        plannerNode->chassiswork = nochassiswork;
                    }

                    break;
                }
                case re_area3:
                {
                    static float tarvelx = 0;
                    static float tarvely = 0;
                    static float omega = 0;

                    tarvelx = PosControl(3000 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).x;
                    tarvely = PosControl(3000 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).y;
                    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 3.6, 0.6);

                    plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

                    if (fabs(fabs(plannerNode->nowPos.x) - 3000) <= 100 && fabs(plannerNode->nowPos.y - 10750) <= 100)
                    {
                        plannerNode->tarVel.SetVel2(0, 0, 0);
                    }
                    break;
                }
                case nochassiswork:
                {
                    plannerNode->myFlags.chassistask = 68;
                    plannerNode->tarVel.Stop();
                    break;
                }
                break;
            }
            break;
        }
        case move_to_get_ground_kfs:
        {
            if (MoveToGetGroundKfs())
            {
                plannerNode->ChassisTask = area3_move;
                plannerNode->chassiswork = gotoputsecond;
            }
            break;
        }
        case move_to_get_r1_kfs:
        {
            if (MoveToGetR1KFS())
            {
                plannerNode->ChassisTask = area3_move;
                plannerNode->chassiswork = gotoputsecond;
            }
            break;
        }
        case turn:
        {
            plannerNode->myFlags.chassistask = 7;
            wheel.WheelOne.SetTarVelKpKi(2.4, 2.5);
            wheel.WheelTwo.SetTarVelKpKi(2.6, 2.5);
            wheel.WheelThree.SetTarVelKpKi(2.6, 2.8);
            wheel.WheelFour.SetTarVelKpKi(2.6, 2.8);

            static bool flag = false;
            static float tarposx = 0;
            static float tarvelx = 0;
            static float tarposy = 0;
            static float tarvely = 0;
            static int turn_flag = 1;

            if (plannerNode->chassiswork == turn_left)
            {
                turn_flag = 1 * plannerNode->myFlags.RB_flag;
            }
            else if (plannerNode->chassiswork == turn_right)
            {
                turn_flag = -1 * plannerNode->myFlags.RB_flag; 
            }

            if (plannerNode->myFlags.trunk_flag && plannerNode->myFlags.turn_flag)
            {
                static int tim_cnt = 0;
                static bool right_flag = 0;
                static bool left_flag = 0;
                static int y_flag = 0;

                static bool first_flag = 0;

                if (plannerNode->myFlags.aimFlag == 0)
                {
                    if (y_flag == 0)
                    {
                        plannerNode->tarPos.y = plannerNode->nowPos.y;
                        y_flag = 1;
                    }

                    if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                    {
                        tim_cnt = 0;
                        right_flag = 1;
                        plannerNode->tarVel.SetVelY(-300 * turn_flag);
                    }
                    else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                    {
                        tim_cnt = 0;
                        left_flag = 1;
                        plannerNode->tarVel.SetVelY(300 * turn_flag);
                    }
                    else if (!plannerNode->Ps_Num[12] && !plannerNode->Ps_Num[11])
                    {
                        if (!plannerNode->myFlags.htl_flag)
                        {
                            plannerNode->myFlags.aimFlag = 1;
                            plannerNode->lxData.start_test = 1;
                        }
                        else
                        {
                            // if (tim_cnt == 0 && first_flag == 1)
                            // {
                            //     plannerNode->myFlags.aimFlag = 1;
                            //     flag = 1;
                            // }

                            tim_cnt++;

                            if (tim_cnt >= 5)
                            {
                                plannerNode->tarPos.y = plannerNode->nowPos.y;
                                plannerNode->myFlags.aimFlag = 1;
                                flag = 0;
                                tim_cnt = 0;
                            }
                        }

                        plannerNode->tarVel.SetVelY(0);
                    }
                    else
                    {
                        tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 2.6, 0.4).y;
                        plannerNode->tarVel.SetVelY(tarvely);
                    }
                }
                else if (plannerNode->myFlags.aimFlag == 1)
                {
                    y_flag = 0;

                    if (!plannerNode->myFlags.htl_flag)
                    {
                        plannerNode->tarPos.y = plannerNode->nowPos.y + plannerNode->lxData.kfs_dist * turn_flag;

                        tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 2.6, 0.4).y;

                        plannerNode->tarVel.SetVelY(tarvely);
                    }
                    else
                    {
                        if (!flag)
                        {
                            static float pos_y[10] = {0};
                            static int cnt = 0;
                            static float sum = 0;

                            pos_y[cnt] = plannerNode->nowPos.y;
                            cnt++;
                            
                            if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
                            {
                                if (left_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_y[i];
                                    }
                                    plannerNode->tarPos.y = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVelY(-200 * turn_flag);
                            }
                            else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
                            {
                                if (right_flag)
                                {
                                    for(int i = 0; i < cnt; i++)
                                    {
                                        sum += pos_y[i];
                                    }
                                    plannerNode->tarPos.y = sum / cnt;
                                    flag = 1;
                                    sum = 0;
                                    cnt = 0;
                                }

                                plannerNode->tarVel.SetVelY(200 * turn_flag);
                            }

                            if (cnt == 10)
                            {
                                for(int i = 0; i < cnt; i++)
                                {
                                    sum += pos_y[i];
                                }
                                plannerNode->tarPos.y = sum / cnt;
                                flag = 1;
                                sum = 0;
                                cnt = 0;
                            }
                        }
                        else
                        {
                            if (fabs(plannerNode->tarPos.y - plannerNode->nowPos.y) <= 20)
                            {
                                if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
                                {
                                    plannerNode->myFlags.aimFlag = 0;
                                    right_flag = 0;
                                    left_flag = 0;
                                }
                            }
                            tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 2.0, 0.4).y;
                            right_flag = 0;
                            left_flag = 0;
                            first_flag = 1; 
                            plannerNode->tarVel.SetVelY(tarvely);
                        }
                    }
                }
            }

            switch (plannerNode->chassiswork)
            {
                case turn_left:
                {
                    plannerNode->myFlags.chassistask = 71;
                    if (Turn_left())
                    {
                        plannerNode->myFlags.aimFlag = 0;
                        plannerNode->myFlags.turn_flag = 0;
                        plannerNode->ChassisTask = plannerNode->LastChassisTask;
                        plannerNode->chassiswork = plannerNode->lastchassiswork;
                        // plannerNode->ChassisTask = no_chassis_task;
                        // plannerNode->chassiswork = nochassiswork;
                    }
                    break;
                }
                case turn_right:
                {
                    plannerNode->myFlags.chassistask = 72;
                    if (Turn_right())
                    {
                        plannerNode->myFlags.aimFlag = 0;
                        plannerNode->myFlags.turn_flag = 0;
                        plannerNode->ChassisTask = plannerNode->LastChassisTask;
                        plannerNode->chassiswork = plannerNode->lastchassiswork;
                    }
                    break;
                }
                break;
            }
            break;
        }
        case no_chassis_task:
        {
            plannerNode->myFlags.chassistask = 8;
            plannerNode->tarVel.SetVel2(0, 0, 0);
            break;
        }
    }
}

bool CheckSelf(void)
{
    static int check_step = -1;

    static int tim_cnt = -1;
    float leg_max_vel = 2.0;
    float leg_a1 = 0.3;
    float leg_a2 = 0.2;

    if (check_step == -1)
    {
        plannerNode->tarVel.SetVel2(0, 300, 0);
        tim_cnt++;
        leg.SetAllCtrlMode(POS_MODE);
        if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
            leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
            leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
            leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt))
        {
            check_step = 0;
            tim_cnt = -1;
        }
    }
    else if (check_step == 0)
    {
        plannerNode->tarVel.SetVel2(0, -300, 0);
        tim_cnt++;
        if (tim_cnt >= 200)
        {
            check_step = 1;
            leg.SetAllCtrlMode(DISABLE);
            wheel.SetAllCtrlMode(DISABLE);
            tim_cnt = -1;
        }
    }
    else if (check_step == 1)
    {
        if (TrunkTakePiece(2))
        {
            check_step = 2;
            trunk.Shoulder.SetCtrlMode(DISABLE);
            trunk.Elbow.SetCtrlMode(DISABLE);
            trunk.Wrist.SetCtrlMode(DISABLE);
        }
    }
    else if (check_step == 2)
    {
        static bool valve_chassis_flag = 1;//13 14 是车头行程开关
        if (!plannerNode->Ps_Num[13] && valve_chassis_flag && !plannerNode->Ps_Num[14])
        {
            valve_chassis_flag = 0;
            plannerNode->Vavle_Chassis_Ctrl(1, 0);
        }

        static bool valve_upper_flag = 1;//车尾
        if (plannerNode->Ps_Num[16] && valve_upper_flag && plannerNode->Ps_Num[17]) //右
        {
            valve_upper_flag = 0;
            plannerNode->Vavle_Upper_Ctrl(1, 1, 1, 1);
        }

        if (!valve_chassis_flag && !valve_upper_flag)
        {
            // plannerNode->Led_Ctrl(1, 1);
            // return true;
            check_step = 3;
        }
    }
    else if (check_step == 3)
    {
        static int ps_flag = 0;

        if (ps_flag == 0 && plannerNode->Ps_Num[1])
        {
            plannerNode->Raw_Ctrl(0);
            ps_flag = 1;
        }
        else if (ps_flag == 1 && plannerNode->Ps_Num[2])
        {
            plannerNode->Vavle_Chassis_Ctrl(0, 0);
            ps_flag = 2;
        }
        else if (ps_flag == 2 && plannerNode->Ps_Num[3])
        {
            plannerNode->Vavle_Chassis_Ctrl(0, 1);
            ps_flag = 3;
        }
        else if (ps_flag == 3 && plannerNode->Ps_Num[4])
        {
            plannerNode->Vavle_Upper_Ctrl(0, 1, 1, 1);
            ps_flag = 4;
        }
        else if (ps_flag == 4 && plannerNode->Ps_Num[5])
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 1, 1);
            ps_flag = 5;
        }
        else if (ps_flag == 5 && plannerNode->Ps_Num[6])
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
            ps_flag = 8;
        }
        else if (ps_flag == 8 && !plannerNode->Ps_Num[9] && !plannerNode->Ps_Num[10])
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
            ps_flag = 9;
        }
        else if (ps_flag == 9 && !plannerNode->Ps_Num[20] && !plannerNode->Ps_Num[21])
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
            plannerNode->Led_Ctrl(1, 1);
            return true;
        }
    }
    return false;
}

bool DockWeapon(void)
{
    static int yaw_flag = 1;
    static int leave_flag = 0;
    static float tar_velx = 0;
    static float tar_vely = 0;
    static float tar_omega = 0;
    static int cnt = 0;
    plannerNode->tarPos.x = 625 * plannerNode->myFlags.RB_flag;
    plannerNode->tarPos.y = 685;

    if (plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag <= -160 || plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag >= 100)
    {
        plannerNode->tarPos.y = 685 + 35;
    }
    else if (plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag >= 70 && plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag <= 100)
    {
        plannerNode->tarPos.y = 685 + 70;
    }

    if (leave_flag == 0)
    {
        tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 4.2, 0.8, 4.6, 0.8).x;
        tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 4.2, 0.8, 4.6, 0.8).y;

        tar_omega = YawControl(plannerNode->nowPos.yaw, -90.0 * plannerNode->myFlags.RB_flag, 1.5, 0.5);

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 45 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 85)
        {
            tar_velx = 0;
            tar_vely = 0;
            tar_omega = 0;
            
            leave_flag = 1;
        }
    }
    else if (leave_flag == 1)
    {
        tar_omega -= 25.0 * plannerNode->myFlags.RB_flag;

        if (plannerNode->myFlags.RB_flag == 1)
        {
            if (tar_omega < -450)
            {
                tar_omega = -450;
            }
        }
        else
        {
            if (tar_omega > 450)
            {
                tar_omega = 450;
            }
        }

        tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.2, 0.5, 2.5, 0.4).x;

        // tar_vely = PosControl(650, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.2, 0.5, 2.5, 0.4).y;

        if (plannerNode->nowPos.y >= (plannerNode->tarPos.y + 30))
        {
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 7.2, 0.8).y;
        }
        else
        {
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 3.6, 0.4).y;
        }

        if (plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag <= 150 && plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag > 0)
        {
            plannerNode->Vavle_Chassis_Ctrl(1, 0);
            leave_flag = 2;
        }
    }
    else if (leave_flag == 2)
    {
        static int yaw_flag = 0;
        static bool clean_flag = 0;
        static int cnt = 0;
        cnt++;
        if (cnt >= 5)
        {
            cnt = 5;
            plannerNode->Vavle_Chassis_Ctrl(0, 0);
        }
                    
        if (!clean_flag)
        {
            wheel.WheelOne.SetTarClean(1);
            wheel.WheelTwo.SetTarClean(1);
            wheel.WheelThree.SetTarClean(1);
            wheel.WheelFour.SetTarClean(1);
            clean_flag = 1;
        }
        else
        {
            wheel.WheelOne.SetTarClean(0);
            wheel.WheelTwo.SetTarClean(0);
            wheel.WheelThree.SetTarClean(0);
            wheel.WheelFour.SetTarClean(0);
        }

        if (plannerNode->nowPos.yaw * plannerNode->myFlags.RB_flag >= 95 && yaw_flag == 0)
        {
            tar_omega = YawControl(plannerNode->nowPos.yaw, 90.0 * plannerNode->myFlags.RB_flag, 2.4, 0.6);

            if (plannerNode->nowPos.y >= (plannerNode->tarPos.y + 30))
            {
                tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 7.2, 0.8).y;
            }
            else
            {
                tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 3.2, 0.5).y;
            }
        }
        else
        {
            yaw_flag = 1;
            tar_omega = YawControl(plannerNode->nowPos.yaw, 90.0 * plannerNode->myFlags.RB_flag, 1.8, 0.6);
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.8, 0.6, 2.8, 0.8).y;
        }
        // tar_omega = YawControl(plannerNode->nowPos.yaw, 90.0, 0.5, 0.2);

        tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 1.2, 0.4).x;

        if (fabs(plannerNode->nowPos.yaw - 90.0 * plannerNode->myFlags.RB_flag) <= 8.0)
        {
            tar_velx = -550 * plannerNode->myFlags.RB_flag;
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.6, 0.5, 1.8, 0.6).y;
            plannerNode->Vavle_Upper_Ctrl(0, 0, 1, 0);
            leave_flag = 3;
        }
    }
    else if (leave_flag == 3)
    {
        static bool flag = 0;
        
        if (!flag)
        {
            tar_vely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 2.8, 0.6).y;
            tar_omega = YawControl(plannerNode->nowPos.yaw, 90.0 * plannerNode->myFlags.RB_flag, 2.6, 0.6);
        }
        else
        {
            tar_vely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 2.8, 0.4).y;
            // tar_vely = 0;

            tar_velx = -250 * plannerNode->myFlags.RB_flag;
            tar_omega = 8;

        }

        if (!plannerNode->Ps_Num[13] && plannerNode->Ps_Num[14])
        {
            yaw_flag = -1;
        }

        if (!plannerNode->Ps_Num[14] && plannerNode->Ps_Num[13])
        {
            yaw_flag = 1;
        }

        if (!plannerNode->Ps_Num[13] || !plannerNode->Ps_Num[14])
        {
            tar_velx = -250 * plannerNode->myFlags.RB_flag;
            flag = 1;
        }

        static int wait_cnt = 0;
        
        if (!plannerNode->Ps_Num[13] || !plannerNode->Ps_Num[14])
        {
            tar_vely = 0;
        }

        if (!plannerNode->Ps_Num[13] && !plannerNode->Ps_Num[14])
        {
            tar_vely = 0;
            tar_omega = 0;
            flag = 1;

            wait_cnt ++;
            if (wait_cnt >= 8)
            {
                plannerNode->Led_Ctrl(1, 1);
                wait_cnt = 0;
                tar_velx = 0;
                tar_vely = 0;
                return true;
            }
        }
        else
        {
            wait_cnt = 0;
        }
    }
    plannerNode->tarVel.SetVel2(tar_velx, tar_vely, tar_omega * yaw_flag);

    return false;
}

bool Turn_left(void)
{
    leg.SetAllCtrlMode(POS_MODE);
    static bool leg_flag = 0;
    static float tarposx = 0;
    static float tarposy = 0;
    static float tarvelx = 0;
    static float tarvely = 0;
    static float omega = 0;
    static int flag = 0;
    static int step = 1;
    static bool trunk_flag = 0;
    static int leg_up = 0;
    static bool down_flag = 0;

    if (trunk.Shoulder.GetActPos() > 9.9)
    {
        trunk_flag = 1;
    }

    if (flag == 0)
    {
        plannerNode->Vavle_Chassis_Ctrl(1, 0);
        tarposx = plannerNode->nowPos.x;
        flag = 1;
    }

    if (fabs(plannerNode->nowPos.y - 3800) <= 600)
    {
        tarposy = 3800;
    }
    else if (fabs(plannerNode->nowPos.y - 5000) <= 600)
    {
        tarposy = 5050;
    }
    else if (fabs(plannerNode->nowPos.y - 6200) <= 600)
    {
        tarposy = 6250;
    }
    else if (fabs (plannerNode->nowPos.y - 7400) <= 600)
    {
        tarposy = 7450;
    }

    if (leg.LegOne.GetActPos() <= 0.5 && leg.LegTwo.GetActPos() <= 0.5 && leg.LegThree.GetActPos() <= 0.5 && leg.LegFour.GetActPos() <= 0.5)
    {
        leg_flag = 0;
    }
    else
    {
        leg_flag = 1;
    }

    if (plannerNode->myFlags.turn_leg_flag)
    {
        leg_flag = 1;
    }

    if (step == 1)
    {
        static bool turn_flag = 0;
        
        if (fabs(plannerNode->nowPos.y - tarposy) >= 50)
        {
            tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 2.0, 0.4, 2.4, 0.5).x;
            tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 2.0, 0.4, 2.4, 0.5).y;
        }
        else
        {
            tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 1.0, 0.4, 1.8, 0.5).x;
            tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 1.0, 0.4, 1.8, 0.5).y;
        }
        
        if (!leg_flag)
        {
            if (!turn_flag)
            {
                omega += 20 * plannerNode->myFlags.RB_flag;
                if (fabs(omega) >= 300)
                {
                    omega = 300 * plannerNode->myFlags.RB_flag;
                }
                if (fabs(plannerNode->nowPos.yaw) > 40)
                {
                    turn_flag = 1;
                }
            }
            else
            {
                if (fabs(fabs(plannerNode->nowPos.yaw) - 90) >= 10)
                {
                    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 1.0, 0.4);
                }
                else
                {
                    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.6, 0.4);
                }
            }
        }
        else
        {
            omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.4, 0.6);
        }

        if (fabs(fabs(plannerNode->nowPos.yaw) - 90) <= 15)
        {
            wheel.WheelOne.SetTarVelKpKi(2.8, 2.4);
            wheel.WheelTwo.SetTarVelKpKi(2.8, 2.6);
            wheel.WheelThree.SetTarVelKpKi(2.8, 2.6);
            wheel.WheelFour.SetTarVelKpKi(2.8, 2.4);
        }

        if (fabs(fabs(plannerNode->nowPos.yaw) - 90) <= 3
            && fabs(wheel.WheelOne.GetActVel()) <= 0.2
            && fabs(wheel.WheelTwo.GetActVel()) <= 0.2
            && fabs(wheel.WheelThree.GetActVel()) <= 0.2
            && fabs(wheel.WheelFour.GetActVel()) <= 0.2)
        {
            plannerNode->myFlags.turn_flag = 1;
            plannerNode->myFlags.aimFlag = 0;
            step = 2;
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
            tarvelx = 0;
            tarvely = 0;
            omega = 0;
            turn_flag = 0;
        }

        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }
    else if (step == 2)
    {   
        omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.6, 0.6);
        
        plannerNode->tarVel.SetOmega(omega);

        if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
        {
            plannerNode->tarVel.SetVelX(-450 * plannerNode->myFlags.RB_flag);
        }
        else
        {
            plannerNode->tarVel.SetVelX(-50 * plannerNode->myFlags.RB_flag);
        }

        if (trunk.Shoulder.GetActPos() >= 9.9)
        {
            plannerNode->Vavle_Chassis_Ctrl(1, 0);

            if (plannerNode->Ps_Num[15])
            {
                plannerNode->tarVel.SetVelX(-50 * plannerNode->myFlags.RB_flag);
            }
        }

        if (plannerNode->UpperTask == recive_kfs)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
        }

        if (plannerNode->Vavle_Num[9])
        {
            plannerNode->tarVel.SetVelX(0);
            step = 3;
            tarvelx = 0;
            omega = 0;
        }
    }
    else if (step == 3)
    {
        static int tim_cnt = 0;
        
        tim_cnt++;

        wheel.WheelOne.SetTarVelKpKi(2.6, 2.4);
        wheel.WheelTwo.SetTarVelKpKi(2.8, 2.4);
        wheel.WheelThree.SetTarVelKpKi(2.8, 2.4);
        wheel.WheelFour.SetTarVelKpKi(2.8, 2.4);
        
        tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 1.6, 0.4, 1.2, 0.4).x;
        tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 1.6, 0.4, 1.2, 0.4).y;
        omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.2, 0.4);
        
        // if((trunk_flag && plannerNode->UpperTask == get_kfs) || plannerNode->UpperTask == recive_kfs)
        if (trunk.Shoulder.GetActPos() < 4.8 || (plannerNode->UpperTask == recive_kfs && !trunk_flag))
            {
                tim_cnt = 0;
                tarvelx = 0;
                tarvely = 0;
                omega = 0;
                step = 4;
                
                if (plannerNode->lastchassiswork == down200 && plannerNode->myFlags.turn_leg_flag)
                {
                    down_flag = 1;
                }
            }
        
        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }
    else if (step == 4)
    {
        wheel.WheelOne.SetTarVelKpKi(2.8, 2.6);
        wheel.WheelTwo.SetTarVelKpKi(2.6, 2.8);
        wheel.WheelThree.SetTarVelKpKi(2.6, 2.8);
        wheel.WheelFour.SetTarVelKpKi(2.8, 2.6);

        static bool turn_flag = 0;
        tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 2.8, 0.5, 2.4, 0.4).x;
        tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 2.8, 0.5, 2.4, 0.4).y;
        
        if (!leg_flag)
        {
            if (!turn_flag)
            {
                omega -= 20 * plannerNode->myFlags.RB_flag;
                
                if (fabs(omega) >= 300)
                {
                    omega = -300 * plannerNode->myFlags.RB_flag;
                }
                if (fabs(plannerNode->nowPos.yaw) < 40)
                {
                    turn_flag = 1;
                }
            }
            else
            {
                omega = YawControl(plannerNode->nowPos.yaw, 0, 1.4, 0.2);
            }
        }
        else
        {
            omega = YawControl(plannerNode->nowPos.yaw, 0, 2.4, 0.4);
        }

        if (fabs(plannerNode->nowPos.yaw) <= 15)
        {
            wheel.WheelOne.SetTarVelKpKi(2.5, 2.8);
            wheel.WheelTwo.SetTarVelKpKi(2.5, 2.8);
            wheel.WheelThree.SetTarVelKpKi(2.5, 3.0);
            wheel.WheelFour.SetTarVelKpKi(2.5, 3.2);
        }

        if(fabs(plannerNode->nowPos.yaw) <= 3)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
            plannerNode->Vavle_Chassis_Ctrl(0, 1);
            plannerNode->tarVel.SetVel2(0, 0, 0);

            wheel.WheelOne.SetTarVelKpKi(2.5, 2.8);
            wheel.WheelTwo.SetTarVelKpKi(3.0, 2.8);
            wheel.WheelThree.SetTarVelKpKi(3.0, 3.0);
            wheel.WheelFour.SetTarVelKpKi(3.0, 3.2);

            if (plannerNode->myFlags.getKFS_flag)
            {
                plannerNode->tarVel.SetVel2(0, 0, 0);

                if (plannerNode->myFlags.trunk_flag)
                {
                    tarvelx = 0;
                    tarvely = 0;
                    omega = 0;
                    step = 1;
                    turn_flag = 0;
                    if (plannerNode->myFlags.turn_leg_flag)
                    {
                        if (leg_up == 1 && plannerNode->lastchassiswork == up200)
                        {
                            return true;
                        }
                        else if (leg_up == 2)
                        {
                            return true;
                        }
                    }
                    else
                    {
                        return true;
                    }
                }
            }
            else
            {
                tarvelx = 0;
                tarvely = 0;
                omega = 0;
                step = 1;
                turn_flag = 0;
                if (plannerNode->myFlags.turn_leg_flag)
                {
                    if (leg_up == 1 && plannerNode->lastchassiswork == up200)
                    {
                        return true;
                    }
                    else if (leg_up == 2)
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }
        }
        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }

    if (leg_up == 0 && plannerNode->myFlags.turn_leg_flag)
    {
        leg.SetAllCtrlMode(POS_MODE);

        static int leg_up_cnt1 = -1;
        float leg_max_vel = 8.0;
        float leg1_a1 = 0.1;
        float leg1_a2 = 0.1;

        leg_up_cnt1++;

        if (leg.LegOne.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegTwo.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegThree.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegFour.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1))
        {
            leg_up_cnt1 = -1;
            leg_up = 1;
            plannerNode->mystep = front_leg_up;
        }
    }

    if (leg_up == 1 && down_flag)
    {
        leg.SetAllCtrlMode(POS_MODE);

        static int leg_up_cnt2 = -1;
        float leg_max_vel = 8.0;
        float leg1_a1 = 0.1;
        float leg1_a2 = 0.1;

        leg_up_cnt2++;

        if (leg.LegOne.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegTwo.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegThree.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegFour.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2))
        {
            leg_up_cnt2 = -1;
            leg_up = 2;
            plannerNode->mystep = front_leg_down;
        }
    }

    return false;
}

bool Turn_right(void)
{
    leg.SetAllCtrlMode(POS_MODE);
    static float tarposx = 0;
    static float tarposy = 0;
    static float tarvelx = 0;
    static float tarvely = 0;
    static float omega = 0;
    static int flag = 0;
    static int step = 1;
    static bool trunk_flag = 0;
    static bool leg_flag = 0;
    static int leg_up = 0;
    static bool down_flag = 0;

    if (trunk.Shoulder.GetActPos() > 9.9)
    {
        trunk_flag = 1;
    }

    if (flag == 0)
    {
        plannerNode->Vavle_Chassis_Ctrl(1, 0);
        tarposx = plannerNode->nowPos.x;
        flag = 1;
    }

    if (fabs(plannerNode->nowPos.y - 3800) <= 600)
    {
        tarposy = 3800;
    }
    else if (fabs(plannerNode->nowPos.y - 5000) <= 600)
    {
        tarposy = 5000;
    }
    else if (fabs(plannerNode->nowPos.y - 6200) <= 600)
    {
        tarposy = 6200;
    }
    else if (fabs (plannerNode->nowPos.y - 7400) <= 600)
    {
        tarposy = 7450;
    }

    if (leg.LegOne.GetActPos() <= 0.5 && leg.LegTwo.GetActPos() <= 0.5 && leg.LegThree.GetActPos() <= 0.5 && leg.LegFour.GetActPos() <= 0.5)
    {
        leg_flag = 0;
    }
    else
    {
        leg_flag = 1;
    }

    if (plannerNode->myFlags.turn_leg_flag)
    {
        leg_flag = 1;
    }

    if (step == 1)
    {
        static bool turn_flag = 0;
        
        if (fabs(plannerNode->nowPos.y - tarposy) >= 50)
        {
            tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 2.0, 0.4, 3.2, 0.5).x;
            tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 2.0, 0.4, 3.2, 0.5).y;
        }
        else
        {
            tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 1.0, 0.4, 1.5, 0.5).x;
            tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 1.0, 0.4, 1.5, 0.5).y;
        }
        
        if (!leg_flag)
        {
            if (!turn_flag)
            {
                omega -= 20 * plannerNode->myFlags.RB_flag;
                if (fabs(omega) >= 300)
                {
                    omega = -300 * plannerNode->myFlags.RB_flag;
                }
                if (fabs(plannerNode->nowPos.yaw) > 40)
                {
                    turn_flag = 1;
                }
            }
            else
            {
                if (fabs(fabs(plannerNode->nowPos.yaw) - 90) >= 10)
                {
                    omega = YawControl(plannerNode->nowPos.yaw, -90 * plannerNode->myFlags.RB_flag, 1.0, 0.4);
                }
                else
                {
                    omega = YawControl(plannerNode->nowPos.yaw, -90 * plannerNode->myFlags.RB_flag, 2.6, 0.4);
                }
            }
        }
        else
        {
            omega = YawControl(plannerNode->nowPos.yaw, -90 * plannerNode->myFlags.RB_flag, 2.4, 0.6);
        }

        if (fabs(fabs(plannerNode->nowPos.yaw) - 90) <= 15)
        {
            wheel.WheelOne.SetTarVelKpKi(2.6, 2.4);
            wheel.WheelTwo.SetTarVelKpKi(2.6, 2.6);
            wheel.WheelThree.SetTarVelKpKi(2.6, 2.6);
            wheel.WheelFour.SetTarVelKpKi(2.8, 2.4);
        }

        if (fabs(fabs(plannerNode->nowPos.yaw) - 90) <= 3
            && fabs(wheel.WheelOne.GetActVel()) <= 0.2
            && fabs(wheel.WheelTwo.GetActVel()) <= 0.2
            && fabs(wheel.WheelThree.GetActVel()) <= 0.2
            && fabs(wheel.WheelFour.GetActVel()) <= 0.2)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
            plannerNode->myFlags.turn_flag = 1;
            plannerNode->myFlags.aimFlag = 0;
            step = 2;
            tarvelx = 0;
            tarvely = 0;
            omega = 0;
            turn_flag = 0;
        }

        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }
    else if (step == 2)
    {   
        wheel.WheelOne.SetTarVelKpKi(2.6, 2.4);
        wheel.WheelTwo.SetTarVelKpKi(2.8, 2.4);
        wheel.WheelThree.SetTarVelKpKi(2.8, 2.4);
        wheel.WheelFour.SetTarVelKpKi(2.8, 2.4);
        omega = YawControl(plannerNode->nowPos.yaw, -90 * plannerNode->myFlags.RB_flag, 2.6, 0.6);
        plannerNode->tarVel.SetOmega(omega);
        if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
        {
            plannerNode->tarVel.SetVelX(450 * plannerNode->myFlags.RB_flag);
        }
        else
        {
            plannerNode->tarVel.SetVelX(50 * plannerNode->myFlags.RB_flag);
        }

        if (trunk.Shoulder.GetActPos() >= 9.9)
        {
            plannerNode->Vavle_Chassis_Ctrl(1, 0);
            if (plannerNode->Ps_Num[15])
            {
                plannerNode->tarVel.SetVelX(50 * plannerNode->myFlags.RB_flag);
            }
        }

        if (plannerNode->UpperTask == recive_kfs)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
        }

        if (plannerNode->Vavle_Num[9])
        {
            plannerNode->tarVel.SetVelX(0);
            step = 3;
            tarvelx = 0;
            omega = 0;
        }
    }
    else if (step == 3)
    {
        static int tim_cnt = 0;
        
        tim_cnt++;

        wheel.WheelOne.SetTarVelKpKi(2.2, 2.8);
        wheel.WheelTwo.SetTarVelKpKi(2.2, 2.8);
        wheel.WheelThree.SetTarVelKpKi(2.2, 2.8);
        wheel.WheelFour.SetTarVelKpKi(2.2, 2.8);
        
        tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 1.6, 0.4, 1.0, 0.4).x;
        tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 1.6, 0.4, 1.0, 0.4).y;
        omega = YawControl(plannerNode->nowPos.yaw, -90 * plannerNode->myFlags.RB_flag, 2.2, 0.4);
        
        if (trunk.Shoulder.GetActPos() < 4.8 || (plannerNode->UpperTask == recive_kfs && !trunk_flag))
        {
            tim_cnt = 0;
            tarvelx = 0;
            tarvely = 0;
            omega = 0;
            step = 4;
            if (plannerNode->lastchassiswork == down200 && plannerNode->myFlags.turn_leg_flag)
            {
                down_flag = 1;
            }
        }
        
        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }
    else if (step == 4)
    {
        wheel.WheelOne.SetTarVelKpKi(2.2, 2.8);
        wheel.WheelTwo.SetTarVelKpKi(2.6, 2.8);
        wheel.WheelThree.SetTarVelKpKi(2.6, 2.8);
        wheel.WheelFour.SetTarVelKpKi(2.6, 2.8);

        static bool turn_flag = 0;
        tarvelx = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 2.8, 0.5, 1.8, 0.4).x;
        tarvely = PosControl(tarposx, plannerNode->nowPos.x, tarposy + 100, plannerNode->nowPos.y, 2.8, 0.5, 1.8, 0.4).y;
        
        if (!turn_flag)
        {
            omega += 20 * plannerNode->myFlags.RB_flag;
            if (fabs(omega) >= 300)
            {
                omega = 300 * plannerNode->myFlags.RB_flag;
            }
            if (fabs(plannerNode->nowPos.yaw) < 40)
            {
                turn_flag = 1;
            }
        }
        else
        {
            omega = YawControl(plannerNode->nowPos.yaw, 0, 1.4, 0.2);
        }

        if (fabs(plannerNode->nowPos.yaw) <= 15)
        {
            wheel.WheelOne.SetTarVelKpKi(2.5, 2.8);
            wheel.WheelTwo.SetTarVelKpKi(2.5, 2.8);
            wheel.WheelThree.SetTarVelKpKi(2.5, 3.0);
            wheel.WheelFour.SetTarVelKpKi(2.5, 3.2);
        }

        if(fabs(plannerNode->nowPos.yaw) <= 3)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
            plannerNode->Vavle_Chassis_Ctrl(0, 1);
            plannerNode->tarVel.SetVel2(0, 0, 0);

            wheel.WheelOne.SetTarVelKpKi(2.6, 2.8);
            wheel.WheelTwo.SetTarVelKpKi(3.0, 2.8);
            wheel.WheelThree.SetTarVelKpKi(3.0, 3.0);
            wheel.WheelFour.SetTarVelKpKi(3.0, 3.2);

            if (plannerNode->myFlags.getKFS_flag)
            {
                plannerNode->tarVel.SetVel2(0, 0, 0);

                if (plannerNode->myFlags.trunk_flag)
                {
                    tarvelx = 0;
                    tarvely = 0;
                    omega = 0;
                    step = 1;
                    trunk_flag = 0;
                    turn_flag = 0;
                    if (plannerNode->myFlags.turn_leg_flag)
                    {
                        if (leg_up == 1 && plannerNode->lastchassiswork == up200)
                        {
                            return true;
                        }
                        else if (leg_up == 2)
                        {
                            return true;
                        }
                    }
                    else
                    {
                        return true;
                    }
                }
            }
            else
            {
                tarvelx = 0;
                tarvely = 0;
                omega = 0;
                step = 1;
                trunk_flag = 0;
                turn_flag = 0;
                if (plannerNode->myFlags.turn_leg_flag)
                {
                    if (leg_up == 1 && plannerNode->lastchassiswork == up200)
                    {
                        return true;
                    }
                    else if (leg_up == 2)
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }
        }
        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
    }

    if (leg_up == 0 && plannerNode->myFlags.turn_leg_flag)
    {
        leg.SetAllCtrlMode(POS_MODE);

        static int leg_up_cnt1 = -1;
        float leg_max_vel = 8.0;
        float leg1_a1 = 0.1;
        float leg1_a2 = 0.1;

        leg_up_cnt1++;

        if (leg.LegOne.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegTwo.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegThree.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegFour.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1))
        {
            leg_up_cnt1 = -1;
            leg_up = 1;
            plannerNode->mystep = front_leg_up;
        }
    }

    if (leg_up == 1 && down_flag)
    {
        leg.SetAllCtrlMode(POS_MODE);

        static int leg_up_cnt2 = -1;
        float leg_max_vel = 8.0;
        float leg1_a1 = 0.1;
        float leg1_a2 = 0.1;

        leg_up_cnt2++;

        if (leg.LegOne.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegTwo.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegThree.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2) &
            leg.LegFour.PlanControl(leg1_a1, leg1_a2, leg_max_vel, Gear0, leg_up_cnt2))
        {
            leg_up_cnt2 = -1;
            leg_up = 2;
            plannerNode->mystep = front_leg_down;
        }
    }

    return false;
}

bool MoveWeapon(void)
{
    leg.LegTwo.SetCtrlMode(POS_MODE);
    leg.LegThree.SetCtrlMode(POS_MODE);
    leg.LegTwo.SetTarPara(0, 0.01, 0);
    leg.LegThree.SetTarPara(0, 0.01, 0);

    static float taromega = 0;
    static float taryaw = 0;
    if (plannerNode->myFlags.RB_flag == 1)
    {
        taryaw = -80;
    }
    else
    {
        taryaw = -100;
    }
    static float tar_velx = 0.0f;
    static float tar_vely = 0.0f;
    static float omega = 0.0f;
    static float last_posx = 0;
    static int time_cnt = 0;
    static int moveflag = 0;
    static float first_posx = 0;
    static int tim_cnt1 = 0;
    static int tim_cnt2 = 0;
    
    time_cnt++;

    if(time_cnt == 1)
    {
        plannerNode->Vavle_Upper_Ctrl(1, 1, 0, 0);
        plannerNode->Vavle_Chassis_Ctrl(1, 0);
        first_posx = plannerNode->nowPos.x;
    }

    if(moveflag == 0)
    {
        if (plannerNode->myFlags.RB_flag == 1)
        {
            if (!plannerNode->Ps_Num[18] && !plannerNode->Ps_Num[19])
            {
                taromega = 0;
                correctLidar(-1, 331, -90);
                // plannerNode->PublishToYyy(plannerNode->nowPos.x, 331, plannerNode->nowPos.z, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, -90.0f, true);
            }
            else if (!plannerNode->Ps_Num[18] && plannerNode->Ps_Num[19])
            {
                taromega = 5;
                taryaw = -75;
                tim_cnt1++;
            }
            else if (!plannerNode->Ps_Num[19] && plannerNode->Ps_Num[18])
            {
                taromega = -5;
                taryaw = -75;
                tim_cnt2++;
            }
            else if (plannerNode->Ps_Num[18] && plannerNode->Ps_Num[19])
            {
                taryaw = -75;
                tim_cnt1 = 0;
                tim_cnt2 = 0;
            }
        }
        else if (plannerNode->myFlags.RB_flag == -1)
        {
            if (!plannerNode->Ps_Num[20] && !plannerNode->Ps_Num[21])
            {
                taromega = 0;
                correctLidar(-1, 331, 90);
                // plannerNode->PublishToYyy(plannerNode->nowPos.x, 331, plannerNode->nowPos.z, plannerNode->nowPos.roll, plannerNode->nowPos.pitch, -90.0f, true);
            }
            else if (!plannerNode->Ps_Num[20] && plannerNode->Ps_Num[21])
            {
                taromega = -5;
                taryaw = -105;
                tim_cnt2++;
            }
            else if (!plannerNode->Ps_Num[21] && plannerNode->Ps_Num[20])
            {
                taromega = 5;
                taryaw = -105;
                tim_cnt1++;
            }
            else if (plannerNode->Ps_Num[20] && plannerNode->Ps_Num[21])
            {
                taryaw = -105;
                tim_cnt1 = 0;
                tim_cnt2 = 0;
            }
        }

        if (tim_cnt1 != 0)
        {
            taromega += tim_cnt1 * 0.1;
            
            if (taromega >= 15)
            {
                taromega = 15;
            }
        }
        else if (tim_cnt2 != 0)
        {
            taromega -= tim_cnt2 * 0.1;
            
            if (taromega <= -15)
            {
                taromega = -15;
            }
        }


        static float tarvel[107] = {0,0.003697624,0.024239634,0.058666917,0.094612475,0.130558033,0.166503591,0.20244915,0.238394708,0.274340266,0.310285824,0.346231382,0.38217694,0.418122499,0.454068057,0.490013615,0.525959173,0.561904731,0.597850289,0.633795848,0.669741406,0.705686964,0.741632522,0.77757808,0.813523639,0.849469197,0.885414755,0.921360313,0.957305871,0.993251429,1.029196988,1.065142546,1.101088104,1.137033662,1.17297922,1.208924778,1.244870337,1.280815895,1.316761453,1.352707011,1.388652569,1.424598128,1.460543686,1.496489244,1.532434802,1.56838036,1.604325918,1.640271477,1.676217035,1.712162593,1.748108151,1.784053709,1.819999268,1.855944826,1.891890384,1.927835942,1.9637815,1.999727058,2.035672617,2.071618175,2.107563733,2.143509291,2.179454849,2.215400407,2.251345966,2.287291524,2.323237082,2.35918264,2.395128198,2.431073757,2.467019315,2.502964873,2.538610158,2.569459339,2.589825981,2.598636442,2.6,2.6,2.581127343,2.541942964,2.502380692,2.462818421,2.423256149,2.383693877,2.344131606,2.304569334,2.265007063,2.225444791,2.185882519,2.146320248,2.106757976,2.067195704,2.027633433,1.988071161,1.94850889,1.908946618,1.869384346,1.829822075,1.790259803,1.750697531,1.71113526,1.671572988,1.632010716,1.592448445,1.552886173,1.513948253,1.5};
        static int cnt = 0;
        static float cur_one = 0;
        static float cur_two = 0;
        static float cur_three = 0;
        static float cur_four = 0;
        
        cnt++; 
        
        if (cnt > 106)
        {
            cnt = 106;
        } 
        if (cnt <= 37)
        {
            cur_two = -25;
            cur_three = -15;
            wheel.WheelTwo.SetFeedCurr(cur_two);
            wheel.WheelThree.SetFeedCurr(cur_three);
        }
        else
        {
            cur_two += 5;
            cur_three += 3;
            if ((cur_two) >= 0)
            {
                cur_two = 0;
            }
            if ((cur_three) >= 0)
            {
                cur_three = 0;
            }
            wheel.WheelTwo.SetFeedCurr(cur_two);
            wheel.WheelThree.SetFeedCurr(cur_three);
        } 

        if (cnt <= 37)
        {
            cur_one = -35;
            cur_four = -25;
            wheel.WheelOne.SetFeedCurr(cur_one);
            wheel.WheelFour.SetFeedCurr(cur_four);
        }
        else
        {
            cur_one += 7;
            cur_four += 5;
            if ((cur_one) >= 0)
            {
                cur_one = 0;
            }
            if ((cur_four) >= 0)
            {
                cur_four = 0;
            }
            wheel.WheelOne.SetFeedCurr(cur_one);
            wheel.WheelFour.SetFeedCurr(cur_four);
        } 

        plannerNode->tarVel.SetVel1(-tarvel[cnt] * 1250.f * plannerNode->myFlags.RB_flag, taryaw, taromega);

        last_posx = plannerNode->nowPos.x;

        if(fabs(plannerNode->nowPos.x - first_posx) >= 550)
        {
            plannerNode->Vavle_Chassis_Ctrl(0, 0);
        }
        
        if ((!plannerNode->Ps_Num[9] || !plannerNode->Ps_Num[10]) && fabs(plannerNode->nowPos.x - (535 * plannerNode->myFlags.RB_flag)) <= 200)
        {
            moveflag = 1;
            return true;
        }
    }
    else if(moveflag == 1)
    {
        static bool next_flag = false;
        static int retry_flag = -1;
        static int retry_step = 0;
        plannerNode->lxData.task = 2;

        omega = YawControl(plannerNode->nowPos.yaw, -90.0 * plannerNode->myFlags.RB_flag, 2.0, 1.2);
        if (!next_flag)
        {
            retry_flag++;
            next_flag = true;
        }

        plannerNode->Vavle_Upper_Ctrl(1, 0, 0, 0);
        plannerNode->tarPos.y = 531.5 + 200 * retry_flag;
        if(retry_step == 0)
        {
            tar_velx = 0;
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.8, 2.8, 0.6).y;
            plannerNode->tarVel.SetVel2(tar_velx, tar_vely, omega);

            if(fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 20  && 
                plannerNode->lxData.dist_y != 0)
            {
                retry_step = 1;
            }
        }
        if(retry_step == 1)
        {
            tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, plannerNode->lxData.dist_y, 2.5, 0.8, 4.0, 0.6).y * plannerNode->myFlags.RB_flag;
            
            if (plannerNode->nowPos.y >= 1600 && tar_vely > 0)
            {
                tar_vely = 0;
            }

            plannerNode->tarVel.SetVel2(tar_velx, tar_vely, omega);
            if(fabs(plannerNode->lxData.dist_y) <= 3 && plannerNode->lxData.dist_y != 0)
            {
                static int flag = 0;
                flag++;
                if(flag >= 5)
                {
                    plannerNode->Vavle_Upper_Ctrl(1, 1, 0, 0);
                    next_flag = false;
                    retry_step = 0;
                    flag = 0;
                    return true;
                }
            }
        }
    }
    plannerNode->Logger()->info("Weapon1 executing...");

    return false;
}

bool GetWeapon(void)
{
    static int time_cnt = 0;
    static float tar_velx = 0.0f;
    static float tar_vely = 0.0f;
    static int retry_flag = 0;
    static int flag = 0;
    static int flag2 = 0;
    static float omega = 0;

    if(!plannerNode->Ps_Num[9] || !plannerNode->Ps_Num[10])
    {
        flag++;
    }
    else
    {
        flag = 0;
    }

    if(flag >= 3 && flag2 == 0)
    {
        flag2 = 1;
    }

    if(flag2 == 1)
    {
        static float tarVelx = -1200 * plannerNode->myFlags.RB_flag;
        tarVelx += 50 * plannerNode->myFlags.RB_flag;

        if (time_cnt >= 24)
        {
            tarVelx = 0;
        }

        plannerNode->tarVel.SetVel2(tarVelx, 0.0, 0.0);

        time_cnt++;
        if(time_cnt <= 15)
        {
            plannerNode->Vavle_Upper_Ctrl(1, 1, 1, 0);
        }
        else if (time_cnt <= 30)
        {
            plannerNode->lxData.task = 1;
            plannerNode->Vavle_Upper_Ctrl(0, 1, 1, 0);
        }

        if (time_cnt > 30)
        {
            if (plannerNode->lxData.weapon_state == 0)
            {
                flag2 = 2;
                time_cnt = 0;
                plannerNode->chassiswork = moveweapon;
                plannerNode->UpperTask = no_upper_task;
            }
            else if (plannerNode->lxData.weapon_state == 1)
            {
                retry_flag = 0;
                time_cnt = 0;
                return true;
            }
        }
    }
    else if(flag2 == 2)
    {
        static int cnt  = 0;
        cnt++;
        plannerNode->logger()->info("cnt:{}", cnt);
        if(cnt <= 10)
        {
            plannerNode->logger()->info("waiting for retry...");
        }

        if (cnt >= 20 && cnt <= 60)
        {
            plannerNode->lxData.task = 1;
            plannerNode->logger()->info("jia");
            plannerNode->Vavle_Upper_Ctrl(1, 1, 1, 0);
        }

        if (cnt >= 60)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 1, 1, 0);
        }

        if(cnt >= 60)
        {
            if (plannerNode->lxData.weapon_state == 0)
            {
                flag2 = 2;
                plannerNode->chassiswork = moveweapon;
                plannerNode->UpperTask = no_upper_task;
                cnt = 0;
            }
            if (plannerNode->lxData.weapon_state == 1)
            {
                cnt = 0;
                return true;
            }  
        } 
    }

    return false;
}

bool WaitR1(void)
{
    switch (plannerNode->mystep)
    {
    case wheel_pos:
    {
        static float tar_velx = -120 * plannerNode->myFlags.RB_flag;
        static float tar_vely = 0;
        static float tar_omega = 0;
        plannerNode->lxData.task = 3;
        static int yaw_flag = 0;

        if (!plannerNode->Ps_Num[13] && plannerNode->Ps_Num[14])
        {
            yaw_flag = -1;
        }
        else if (!plannerNode->Ps_Num[14] && plannerNode->Ps_Num[13])
        {
            yaw_flag = 1;
        }
        else
        {
            yaw_flag = 0;
        }

        if (!plannerNode->Ps_Num[13] && !plannerNode->Ps_Num[14])
        {
            tar_omega = 0;
            tar_velx += 20 * plannerNode->myFlags.RB_flag;
            if (tar_velx * plannerNode->myFlags.RB_flag >= 0)
            {
                wheel.WheelOne.SetTarVelKpKi(1.2, 2.0);
                wheel.WheelTwo.SetTarVelKpKi(1.2, 2.0);
                wheel.WheelThree.SetTarVelKpKi(1.2, 2.0);
                wheel.WheelFour.SetTarVelKpKi(1.2, 2.0);
                tar_velx = 0;
            }
        }
        else
        {
            tar_omega = 8 * yaw_flag;
            tar_velx = -120 * plannerNode->myFlags.RB_flag;
        }

        plannerNode->tarVel.SetVel2(tar_velx, tar_vely, tar_omega);

        if (plannerNode->lxData.ok_assemble)
        {
            static int cnt = 0;
            cnt++;

            if (cnt == 1)
            {
                correctLidar(502 * plannerNode->myFlags.RB_flag, -1, 90 * plannerNode->myFlags.RB_flag);
            }

            plannerNode->lxData.mode = 1;
            plannerNode->Logger()->info("Dock Weapon over");
            plannerNode->Led_Ctrl(1, 0);
            
            if (cnt >= 30)
            {
                plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
                plannerNode->mystep = wheel_vel;
            }
        }

        break;
    }
    case wheel_vel:
    {
        wheel.SetAllCtrlMode(VEL_MODE);

        static bool clean_flag = 0;
        if (!clean_flag)
        {
            wheel.WheelOne.SetTarClean(1);
            wheel.WheelTwo.SetTarClean(1);
            wheel.WheelThree.SetTarClean(1);
            wheel.WheelFour.SetTarClean(1);
            clean_flag = 1;
        }
        else
        {
            wheel.WheelOne.SetTarClean(0);
            wheel.WheelTwo.SetTarClean(0);
            wheel.WheelThree.SetTarClean(0);
            wheel.WheelFour.SetTarClean(0);
        }

        plannerNode->tarVel.SetVel2(0.0, 0.0, 0.0);
        static int flag = 0;
        if (flag == 0)
        {
            if ((plannerNode->Ps_Num[16] && plannerNode->myFlags.RB_flag == 1) || (plannerNode->Ps_Num[17] && plannerNode->myFlags.RB_flag == -1))
            {
                flag = 1;
            }
        }
        else if (flag == 1)
        {
            static int cnt = 0;
            if ((!plannerNode->Ps_Num[16] && plannerNode->myFlags.RB_flag == 1) || (!plannerNode->Ps_Num[17] && plannerNode->myFlags.RB_flag == -1))
            {
                cnt++;
                if (cnt >= 60)
                {
                    return true;
                }
            }
        }
        
        break;
    }
    }
    return false;
}

bool GoMove(int way)
{
    static int step = 0;
    static float tarposx = 0;
    static float tarposy = 0;
    static float taryaw = 90.0 * plannerNode->myFlags.RB_flag;
    static bool re_flag = 0;
    if (plannerNode->mode == "re_area2" && !re_flag)
    // if (!re_flag)
    {
        taryaw = -90.0 * plannerNode->myFlags.RB_flag;
        re_flag = 1;
    }
    static float omega = 0.0f;
    static float velx = 0.0f;
    static float vely = 0.0f;
    static float tar_velx = 0.0f;
    static float tar_vely = 0.0f;
    static float last_posx = 0;
    static float last_posy = 1600;
    static float kp_x = 1.2;
    static float kd_x = 0.5;
    static float kp_y = 1.2;
    static float kd_y = 0.5;
    static int time_cnt = 0;
    static float dif_x = 0;
    static bool flag = 0;
    static float distance_y = 0;
    static float distance_x = 0;

    if(time_cnt == 0)
    {
        plannerNode->tarPos.x = plannerNode->nowPos.x;//470
        plannerNode->tarPos.y = plannerNode->nowPos.y;//760
        distance_y = last_posy - plannerNode->nowPos.y;
        if (way == 1)
        {
            distance_x = fabs(1825 * plannerNode->myFlags.RB_flag - plannerNode->nowPos.x);
        }
        else if (way == 2)
        {
            distance_x = fabs(3025 * plannerNode->myFlags.RB_flag - plannerNode->nowPos.x);
        }
        else if (way == 3)
        {
            distance_x = fabs(4225 * plannerNode->myFlags.RB_flag - plannerNode->nowPos.x);
        }
    }
    time_cnt++;
    if (way == 1)
    {
        vely = 400 * 2.0;
        velx = distance_x / distance_y * vely * plannerNode->myFlags.RB_flag;
        plannerNode->tarPos.x += velx / 200;//distance_x
        plannerNode->tarPos.y += 2 * 2.0;//distance_y
        dif_x = 750;
        last_posx = 1825 * plannerNode->myFlags.RB_flag;
        if (plannerNode->nowPos.x * plannerNode->myFlags.RB_flag >= 600)
        {
            taryaw = 0.0;
        }
        kp_x = 2.8;
        kd_x = 0.5;
        kp_y = 2.6;
        kd_y = 0.5;
    }
    else if (way == 2)
    {
        vely = 300 * 2.0;
        velx = distance_x / distance_y * vely * plannerNode->myFlags.RB_flag;
        plannerNode->tarPos.x += velx / 200;//distance_x
        plannerNode->tarPos.y += 1.5 * 2.0;//840
        dif_x = 950;
        last_posx = 3025 * plannerNode->myFlags.RB_flag;
        if (plannerNode->nowPos.x * plannerNode->myFlags.RB_flag >= 1700)
        {
            taryaw = 0.0;
        }
        kp_x = 2.8;
        kd_x = 0.5;
        kp_y = 2.4;
        kd_y = 0.5;
    }
    else if (way == 3)
    {
        vely = 200 * 2.0;
        velx = distance_x / distance_y * vely * plannerNode->myFlags.RB_flag;
        plannerNode->tarPos.x += velx / 200;//3755
        plannerNode->tarPos.y += 1 * 2.0;//840
        dif_x = 850;
        last_posx = 4225 * plannerNode->myFlags.RB_flag;
        if (plannerNode->nowPos.x * plannerNode->myFlags.RB_flag >= 2700)
        {
            taryaw = 0.0;
        }
        kp_x = 2.8;
        kd_x = 0.4;
        kp_y = 2.6;
        kd_y = 0.4;
    }

    if (fabs(plannerNode->tarPos.x) >= fabs(last_posx))
    {
        plannerNode->tarPos.x = last_posx;
    }
    if (plannerNode->tarPos.y >= last_posy)
    {
        plannerNode->tarPos.y = last_posy;
    }

    if (fabs(plannerNode->nowPos.x) >= fabs(last_posx))
    {
        velx = 0;
    }
    if (plannerNode->nowPos.y >= last_posy)
    {
        vely = 0;
    }

    if (fabs(plannerNode->nowPos.x - last_posx) <= dif_x)
    {
        if (fabs(last_posx - plannerNode->nowPos.x) <= 400)
        {
            tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 0.2, 0.1, 0, 0).x;
        }
        else
        {
            tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 0.8, 0.1, 0, 0).x;
        }
        // tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, 0, 0, 0.6, 0.1, 0, 0).x;
    }
    else
    {
        tar_velx = velx + PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, kp_x, kd_x, kp_y, kd_y).x;
    }

    omega = YawControl(plannerNode->nowPos.yaw, taryaw, 2.2, 0.4);

    if (step == 0)
    {
        if (fabs(plannerNode->nowPos.y - last_posy) <= 400)
        {
            if (fabs(last_posy - plannerNode->nowPos.y) <= 200)
            {
                tar_vely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 0.6, 0.1).y;
            }
            else
            {
                tar_vely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 1.4, 0.2).y;
            }
        }
        else
        {
            tar_vely = vely + PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, kp_x, kd_x, kp_y, kd_y).y;
        }
    }

    plannerNode->tarVel.SetVel2(tar_velx, tar_vely, omega);

    if (fabs(plannerNode->nowPos.x - last_posx) <= 300 && fabs(plannerNode->nowPos.yaw) <= 3)
    {
        if (plannerNode->lrz.R2.dodgeR1)
        {
            if (plannerNode->R1StarData.R2GoFlag)
            {
                // plannerNode->tarVel.SetVel2(0, 0, 0);
                // plannerNode->Vavle_Upper_Ctrl(0, 0, 1, 0);
                step = 1;
                tar_vely = 1000;
                if (plannerNode->re_area_num == 2)
                {
                    plannerNode->UpperTask = no_upper_task;
                }
                else
                {
                    plannerNode->UpperTask = get_kfs;
                }
                return true;    
            }    
        }
        else
        {
            step = 1;
            tar_vely = 1000;
            if (plannerNode->re_area_num == 2)
            {
                plannerNode->UpperTask = no_upper_task;
            }
            else
            {
                plannerNode->UpperTask = get_kfs;
            }
            return true;
            // plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    }

    return false;
}

bool TrunkPutPiece(void)
{
    trunk.Shoulder.SetTarPosKpKd(4.6, 0.8);
    trunk.Shoulder.SetTarVelKpKi(0.6, 0.4);
    
    plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
    static int flag = 0;
    static int tim_cnt = 0;

    tim_cnt++;
    if (tim_cnt >= 10)
    {
        static float shoulder_pos = 0;
        static float elbow_pos = 0;
        static float wrist_pos = 0;
        static float shoulder_dif[206] = {0,0.00001,0.000119,0.00041,0.000887,0.001551,0.002402,0.00344,0.004666,0.006078,0.007677,0.009464,0.011437,0.013597,0.015944,0.018479,0.0212,0.024099,0.02709,0.03009,0.03309,0.03609,0.03909,0.04209,0.04509,0.04809,0.05109,0.05409,0.05709,0.06009,0.06309,0.06609,0.06909,0.07209,0.07509,0.07809,0.08109,0.08409,0.08709,0.09009,0.09309,0.09609,0.09909,0.10209,0.10509,0.10809,0.11109,0.11409,0.11709,0.12009,0.12309,0.12609,0.12909,0.13209,0.13509,0.13809,0.14109,0.14409,0.14709,0.150081,0.152979,0.155701,0.158235,0.160582,0.162743,0.164716,0.166502,0.168102,0.169514,0.170739,0.171777,0.172628,0.173293,0.17377,0.17406,0.17417,0.174179,0.174179,0.174168,0.174131,0.174053,0.173932,0.173769,0.173564,0.173316,0.173026,0.172694,0.172319,0.171901,0.171441,0.170939,0.170394,0.169806,0.169177,0.168504,0.16779,0.167032,0.166233,0.165391,0.164506,0.163579,0.162614,0.161623,0.160624,0.159624,0.158624,0.157624,0.156624,0.155624,0.154624,0.153624,0.152624,0.151624,0.150624,0.149624,0.148624,0.147624,0.146624,0.145624,0.144624,0.143624,0.142624,0.141624,0.140624,0.139624,0.138624,0.137624,0.136624,0.135624,0.134624,0.133624,0.132624,0.131624,0.130624,0.129624,0.128624,0.127624,0.126624,0.125624,0.124624,0.123624,0.122624,0.121624,0.120624,0.119624,0.118624,0.117624,0.116624,0.115624,0.114624,0.113624,0.112624,0.111624,0.110624,0.109624,0.108624,0.107624,0.106624,0.105624,0.104624,0.103624,0.102624,0.101624,0.100624,0.099624,0.098624,0.097624,0.096624,0.095624,0.094624,0.093624,0.092624,0.091624,0.090624,0.089624,0.088624,0.087624,0.086624,0.085624,0.084624,0.083634,0.082668,0.081741,0.080857,0.080015,0.079215,0.078458,0.077743,0.077071,0.076441,0.075854,0.075309,0.074806,0.074346,0.073929,0.073554,0.073221,0.072931,0.072683,0.072478,0.072315,0.072195,0.072117,0.072079,0.072069,0.072068};
        static float elbow_dif[152] = {0.000011,0.000118,0.000378,0.000792,0.001359,0.00208,0.002955,0.003984,0.005166,0.006502,0.007992,0.009635,0.011433,0.013384,0.015489,0.017747,0.020155,0.022665,0.025165,0.027665,0.030165,0.032665,0.035165,0.037665,0.040165,0.042665,0.045165,0.047665,0.050165,0.052665,0.055165,0.057665,0.060165,0.062665,0.065165,0.067665,0.070165,0.072665,0.075165,0.077665,0.080165,0.082665,0.085165,0.087665,0.090165,0.092665,0.095165,0.097665,0.100165,0.102665,0.105165,0.107665,0.110165,0.112665,0.115165,0.117665,0.120165,0.122665,0.125165,0.127665,0.130165,0.132665,0.135165,0.137665,0.140165,0.142665,0.145165,0.147665,0.150165,0.152665,0.155165,0.157665,0.160165,0.162665,0.165165,0.167665,0.170165,0.172665,0.175165,0.177665,0.180165,0.182665,0.185165,0.187665,0.190165,0.192665,0.195165,0.197665,0.200165,0.202665,0.205165,0.207665,0.210165,0.212665,0.215165,0.217665,0.220165,0.222665,0.225165,0.227665,0.230165,0.232665,0.235165,0.237665,0.240165,0.242665,0.245165,0.247665,0.250165,0.252665,0.255165,0.257665,0.260165,0.262665,0.265165,0.267665,0.270165,0.272665,0.275165,0.277665,0.280165,0.282665,0.285165,0.287665,0.290165,0.292665,0.295165,0.297665,0.300165,0.302665,0.305165,0.307665,0.310165,0.312665,0.315175,0.317582,0.319841,0.321946,0.323897,0.325694,0.327338,0.328827,0.330163,0.331346,0.332374,0.333249,0.33397,0.334537,0.334951,0.335211,0.335319,0.335329,};
        static float wrist_dif[113] = {0.000003,0.00004,0.000135,0.00029,0.000505,0.00078,0.001114,0.001509,0.001964,0.002479,0.003054,0.003688,0.004383,0.005138,0.005953,0.006824,0.007746,0.008706,0.00969,0.010686,0.011686,0.012686,0.013686,0.014686,0.015686,0.016686,0.017686,0.018686,0.019686,0.020686,0.021686,0.022686,0.023686,0.024686,0.025686,0.026686,0.027686,0.028686,0.029686,0.030686,0.031686,0.032686,0.033686,0.034686,0.035686,0.036686,0.037686,0.038686,0.039686,0.040686,0.041686,0.042686,0.043686,0.044686,0.045686,0.046686,0.047686,0.048686,0.049686,0.050686,0.051686,0.052686,0.053686,0.054686,0.055686,0.056686,0.057686,0.058686,0.059686,0.060686,0.061686,0.062686,0.063686,0.064686,0.065686,0.066686,0.067686,0.068686,0.069686,0.070686,0.071686,0.072686,0.073686,0.074686,0.075686,0.076686,0.077686,0.078686,0.079686,0.080686,0.081686,0.082686,0.083686,0.084686,0.085686,0.086686,0.087686,0.088686,0.089686,0.090686,0.091686,0.092686,0.093686,0.094686,0.095686,0.096686,0.097686,0.098686,0.099686,0.100686,0.101686,0.102686,0.103686};
        static bool initinal_flag = 0;
        static int shoulder_cnt = 0;
        static int elbow_cnt = 0;
        static int wrist_cnt = 0;
        shoulder_cnt++;
        elbow_cnt++;
        wrist_cnt++;
        static float shoulder_first_pos = 0;
        static float elbow_first_pos = 0;
        static float wrist_first_pos = 0;
        if (initinal_flag == 0)
        {
            shoulder_first_pos = trunk.Shoulder.GetActPos();
            elbow_first_pos = trunk.Elbow.GetActPos();
            wrist_first_pos = trunk.Wrist.GetActPos();

            initinal_flag = 1;
        }

        if (shoulder_cnt >= 205)
        {
            shoulder_cnt = 205;
        }
        if (elbow_cnt >= 151)
        {
            elbow_cnt = 151;
        }
        if (wrist_cnt >= 112)
        {
            wrist_cnt = 112;
        }

        if (shoulder_cnt >= 205 && elbow_cnt >= 151 && wrist_cnt >= 112)
        {
            if (fabs(trunk.Shoulder.GetActPos() - shoulder_pos) < 0.3
            && fabs(trunk.Elbow.GetActPos() - elbow_pos) < 0.02
            && fabs(trunk.Wrist.GetActPos() - wrist_pos) < 0.02)
            {
                flag = 0;
                shoulder_cnt = 0;
                elbow_cnt = 0;
                wrist_cnt = 0;
                return true;
            }
        }
        else
        {
            shoulder_pos = shoulder_first_pos - shoulder_dif[shoulder_cnt] * 18;
            elbow_pos = elbow_first_pos - elbow_dif[elbow_cnt];
            wrist_pos = wrist_first_pos + wrist_dif[wrist_cnt];
        }

        if (wrist_pos < 0)
        {
            wrist_pos = 0;
        }
        if (elbow_pos > 0)
        {
            elbow_pos = 0;
        }
        if (shoulder_pos < 0.2)
        {
            shoulder_pos = 0.2;
        }

        trunk.Shoulder.SetTarPara(0, shoulder_pos, 0);
        trunk.Elbow.SetTarPara(1.0, elbow_pos, 0);
        trunk.Wrist.SetTarPara(0.5, wrist_pos, 0);
    }
    return false;
}

bool TrunkPutinto(void)
{
    static float shoulder_pos = 0;
    static float elbow_pos = 0;
    static float wrist_pos = 0;
    static float shoulder_dif[85] = {0.000034,0.000148,0.000383,0.000745,0.001236,0.001854,0.0026,0.003474,0.004475,0.005605,0.006862,0.008247,0.00976,0.0114,0.013169,0.015065,0.017089,0.019241,0.02152,0.023916,0.026387,0.028885,0.031385,0.033885,0.036385,0.038885,0.041385,0.043885,0.046385,0.048885,0.051385,0.053885,0.056385,0.058885,0.061385,0.063885,0.066385,0.068885,0.071385,0.073885,0.076385,0.078885,0.081385,0.083885,0.086385,0.088885,0.091385,0.093885,0.096385,0.098885,0.101385,0.103885,0.106385,0.108885,0.111385,0.113885,0.116385,0.118885,0.121385,0.123885,0.126385,0.128885,0.131383,0.133855,0.136251,0.13853,0.140682,0.142706,0.144602,0.14637,0.148011,0.149524,0.150909,0.152166,0.153295,0.154297,0.155171,0.155917,0.156535,0.157025,0.157388,0.157623,0.157737,0.157768,0.157771};
    static float elbow_dif[87] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.000003,0.000045,0.000179,0.000419,0.000766,0.00122,0.001781,0.002449,0.003223,0.004105,0.005093,0.006189,0.007391,0.008701,0.010117,0.01164,0.01327,0.015007,0.016851,0.018795,0.02079,0.02279,0.02479,0.02679,0.02879,0.03079,0.03279,0.03479,0.03679,0.03879,0.04079,0.04279,0.04479,0.04679,0.04879,0.05079,0.05279,0.05479,0.05679,0.05879,0.06079,0.06279,0.06479,0.06679,0.06879,0.07079,0.07279,0.07479,0.07679,0.07879,0.08079,0.082786,0.08473,0.086574,0.088311,0.089941,0.091464,0.09288,0.094189,0.095392,0.096487,0.097476,0.098357,0.099132,0.0998,0.100361,0.100815,0.101162,0.101402,0.101535,0.101577,0.101581};
    static float wrist_dif[87] = {0,0,0,0,0,0,0,0,0.000001,0.000008,0.000038,0.000108,0.00023,0.000408,0.000642,0.000933,0.00128,0.001682,0.002141,0.002656,0.003227,0.003854,0.004538,0.005277,0.006073,0.006924,0.007832,0.008789,0.009778,0.010777,0.011777,0.012777,0.013777,0.014777,0.015777,0.016777,0.017777,0.018777,0.019777,0.020777,0.021777,0.022777,0.023777,0.024777,0.025777,0.026777,0.027777,0.028777,0.029777,0.030777,0.031777,0.032777,0.033777,0.034777,0.035777,0.036777,0.037777,0.038777,0.039777,0.040777,0.041777,0.042777,0.043777,0.044777,0.045777,0.046776,0.047765,0.048722,0.049629,0.050481,0.051277,0.052016,0.052699,0.053326,0.053897,0.054412,0.054871,0.055274,0.055621,0.055911,0.056145,0.056324,0.056446,0.056516,0.056546,0.056553,0.056554};
    static int cnt = 0;
    static int shoulder_cnt = 0;
    static int elbow_cnt = 0;
    static int wrist_cnt = 0;
    static bool initinal_flag = 0;
    cnt++;
    shoulder_cnt++;
    elbow_cnt++;
    wrist_cnt++;
    static float shoulder_first_pos = 0;
    static float elbow_first_pos = 0;
    static float wrist_first_pos = 0;
    if (initinal_flag == 0)
    {
        shoulder_first_pos = trunk.Shoulder.GetActPos();
        elbow_first_pos = trunk.Elbow.GetActPos();
        wrist_first_pos = trunk.Wrist.GetActPos();
        initinal_flag = 1;
    }

    if (shoulder_cnt >= 84)
    {
        shoulder_cnt = 84;
    }

    if (elbow_cnt >= 86)
    {
        elbow_cnt = 86;
    }

    if (wrist_cnt >= 84)
    {
        wrist_cnt = 84;
    }

    if (cnt >= 70)
    {
        plannerNode->Raw_Ctrl(0);
    }

    if (cnt >= 86)
    {
        plannerNode->logger()->info("waiting for into...");

        if (fabs(trunk.Shoulder.GetActPos() - shoulder_pos) <= 0.24
            && fabs(trunk.Elbow.GetActPos() - elbow_pos) <= 0.1)
        {
            plannerNode->Raw_Ctrl(0);
            initinal_flag = 0;
            shoulder_cnt = 0;
            elbow_cnt = 0;
            wrist_cnt = 0;
            cnt = 0;
            return true;
        }
    }
    else
    {
        shoulder_pos = shoulder_first_pos + shoulder_dif[shoulder_cnt] * 18;
        elbow_pos = elbow_first_pos - elbow_dif[elbow_cnt];
        wrist_pos = wrist_first_pos - wrist_dif[wrist_cnt];
    }

    if (wrist_pos < 0)
    {
        wrist_pos = 0;
    }
    if (elbow_pos > 0)
    {
        elbow_pos = 0;
    }
    if (shoulder_pos < 0.2)
    {
        shoulder_pos = 0.2;
    }

    trunk.Shoulder.SetTarPara(0, shoulder_pos, 0);
    trunk.Elbow.SetTarPara(0.35, elbow_pos, 0);
    trunk.Wrist.SetTarPara(0.4, 0.3, 0);

    return false;
}

bool TrunkTakePiece(int i)
{
    float shoulder_last_pos; 
    float elbow_last_pos; 
    float wrist_last_pos; 
    float shoulder_max_vel; 
    float elbow_max_vel; 
    float wrist_max_vel;

    static float shoulder_a1 = 0.1;
    static float shoulder_a2 = 0.2;

    trunk.Shoulder.SetTarPosKpKd(4.4, 0.6);
    trunk.Shoulder.SetTarVelKpKi(0.5, 0.4);

    if (i == 1)
    {
        shoulder_last_pos = TRUNK_LTH0_SHOULDER;
        elbow_last_pos = TRUNK_LTH0_ELBOW;
        wrist_last_pos = TRUNK_LTH0_WRIST;
        shoulder_max_vel = 15;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
    }
    else if (i == 2)
    {
        if (fabs(trunk.Shoulder.GetTarPos() - TRUNK_LTH200_SHOULDER) <= 0.3)
        {
            trunk.Shoulder.SetTarPosKpKd(3.6, 1.6);
            trunk.Shoulder.SetTarVelKpKi(0.6, 0.4);
        }

        shoulder_last_pos = TRUNK_LTH200_SHOULDER;
        elbow_last_pos = TRUNK_LTH200_ELBOW;
        wrist_last_pos = TRUNK_LTH200_WRIST;
        shoulder_max_vel = 14;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
    }
    else if (i == 3)
    {
        shoulder_last_pos = TRUNK_LTH400_SHOULDER;
        elbow_last_pos = TRUNK_LTH400_ELBOW;
        wrist_last_pos = TRUNK_LTH400_WRIST;
        shoulder_max_vel = 15;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
    }
    else if (i == 4)
    {
        plannerNode->myFlags.htl_flag = 1;
        if (fabs(trunk.Shoulder.GetTarPos() - TRUNK_HTL_SHOULDER) <= 0.3)
        {
            trunk.Shoulder.SetTarPosKpKd(4.6, 0.8);
            trunk.Shoulder.SetTarVelKpKi(0.6, 0.4);
        }
        else
        {
            trunk.Shoulder.SetTarPosKpKd(4.4, 0.8);
            trunk.Shoulder.SetTarVelKpKi(0.5, 0.4);
        }
        shoulder_last_pos = TRUNK_HTL_SHOULDER;
        elbow_last_pos = TRUNK_HTL_ELBOW;
        wrist_last_pos = TRUNK_HTL_WRIST;
        shoulder_max_vel = 14;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
        
        if (trunk.Shoulder.GetActPos() > 7.5)
        {
            plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 0);
        }
    }
    else if (i == 5)
    {
        shoulder_last_pos = TRUNK_THROW_SHOULDER;
        elbow_last_pos = TRUNK_THROW_ELBOW;
        wrist_last_pos = TRUNK_THROW_WRIST;
        shoulder_max_vel = 14;
        elbow_max_vel = 1.0;
        wrist_max_vel = 0.6;
    }
    else if (i == 6)
    {
        shoulder_last_pos = 1.067592 * 3;
        elbow_last_pos = -0.336279;
        wrist_last_pos = 0.414482;
        shoulder_max_vel = 8;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
    }
    else if (i == 7)
    {
        shoulder_last_pos = 1.53333 * 3;
        elbow_last_pos = -0.0065054;
        wrist_last_pos = 0.2836076;
        shoulder_max_vel = 10;
        elbow_max_vel = 1.0;
        wrist_max_vel = 1.0;
    }
    else if (i == 8)
    {
        shoulder_last_pos = 1.90567 * 3;
        elbow_last_pos = -0.266473;
        wrist_last_pos = 0.18;
        shoulder_max_vel = 10;
        elbow_max_vel = 0.5;
        wrist_max_vel = 0.8;
    }
    else if (i == 9)
    {
        trunk.Shoulder.SetTarPosKpKd(4.6, 1.4);
        trunk.Shoulder.SetTarVelKpKi(0.6, 0.4);
        shoulder_last_pos = 6.25;
        elbow_last_pos = -0.377667;
        wrist_last_pos = 0.2347;
        shoulder_max_vel = 6;
        shoulder_a1 = 0.3;
        shoulder_a2 = 0.3;
        if (trunk.Shoulder.GetActPos() < 4.5)
        {
            elbow_max_vel = 0.4;
        }
        else
        {
            elbow_max_vel = 0.6;
        }
        wrist_max_vel = 0.6;
    }

    static bool flag = 0;
    static bool raw_flag = 0;
    static int tim_cnt = -1;
    static bool ready_flag = 0;
    static bool clean_flag = 0;

    tim_cnt++;

    if (tim_cnt == 0 && plannerNode->Vavle_Num[9] == 1 && !plannerNode->myFlags.throw_flag)
    {
        raw_flag = 1;
    }

    if (trunk.Shoulder.PlanControl(shoulder_a1, shoulder_a2, shoulder_max_vel, shoulder_last_pos, tim_cnt))
    {
        ready_flag = 1;
    }

    trunk.Wrist.SetTarPara(wrist_max_vel, wrist_last_pos, 0);
    trunk.Elbow.SetTarPara(elbow_max_vel, elbow_last_pos, 0);

    if (i == 5)
    {
        if (ready_flag && fabs(trunk.Wrist.GetActPos() - trunk.Wrist.GetTarPos()) < 0.001 && fabs(trunk.Elbow.GetActPos() - trunk.Elbow.GetTarPos()) < 0.015 && fabs(trunk.Shoulder.GetActVel()) <= 0.11)
        {
            flag = 1;
        } 
    }
    else
    {
        if (ready_flag && fabs(trunk.Wrist.GetActPos() - trunk.Wrist.GetTarPos()) < 0.001 && fabs(trunk.Elbow.GetActPos() - trunk.Elbow.GetTarPos()) < 0.001)
        {
            flag = 1;
        } 
    }

    if (flag)
    {
        plannerNode->myFlags.trunk_flag = 1;

        // if (plannerNode->UpperTask == throw_kfs || raw_flag == 1)
        if (plannerNode->UpperTask == throw_kfs || i == 6 || i == 5 || i == 9)
        {
            raw_flag = 0;
            tim_cnt = -1;
            ready_flag = 0;
            flag = 0;
            plannerNode->myFlags.trunk_flag = 0;
            plannerNode->myFlags.htl_flag = 0;

            shoulder_a1 = 0.1;
            shoulder_a2 = 0.2;

            return true;
        }
        else
        {
            raw_flag = 0;

            static int time_cnt = -1;
            static bool ps_flag = 0;
            
            if (plannerNode->Ps_Num[15])
            {
                time_cnt++;

                if (time_cnt >= 3)
                {
                    ps_flag = 1;
                    plannerNode->Raw_Ctrl(1);
                }
            }
            // else
            // {
            //     time_cnt = 0;
            // }

            if (ps_flag)
            {
                static int cnt = 0;
                cnt++;
                if (plannerNode->ChassisTask == turn || plannerNode->ChassisTask == area2_change_move)
                {
                    if (cnt >= 40)
                    {
                        tim_cnt = -1;
                        ready_flag = 0;
                        flag = 0;
                        time_cnt = -1;
                        ps_flag = 0;
                        cnt = 0;
                        plannerNode->myFlags.trunk_flag = 0;
                        plannerNode->myFlags.htl_flag = 0;

                        shoulder_a1 = 0.1;
                        shoulder_a2 = 0.2;

                        return true;
                    }
                }
                else
                {
                    if (cnt >= 20)
                    {
                        tim_cnt = -1;
                        ready_flag = 0;
                        flag = 0;
                        time_cnt = -1;
                        ps_flag = 0;
                        cnt = 0;
                        plannerNode->myFlags.trunk_flag = 0;
                        plannerNode->myFlags.htl_flag = 0;

                        shoulder_a1 = 0.1;
                        shoulder_a2 = 0.2;

                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool TrunkSave(int i)
{
    static int flag = 0;

    if (flag == 0)
    {
        if (i == 1) // LTHSAVE
        {
            trunk.Shoulder.SetTarPosKpKd(5.0, 0.8);

            static float shoulder_pos = 0;
            static float elbow_pos = 0;
            static float wrist_pos = 0;
            static float shoulder_dif[89] = {0.001091,0.002295,0.003946,0.00604,0.008596,0.011393,0.014166,0.016945,0.019722,0.0225,0.025278,0.028056,0.030833,0.033611,0.036389,0.039167,0.041944,0.044722,0.0475,0.050278,0.053056,0.055833,0.058611,0.061389,0.064167,0.066944,0.069722,0.0725,0.075278,0.078056,0.080833,0.083611,0.086389,0.089167,0.091944,0.094722,0.0975,0.100278,0.103056,0.105833,0.108611,0.111389,0.114167,0.116944,0.119722,0.1225,0.125278,0.128056,0.130833,0.133611,0.136389,0.139167,0.141944,0.144722,0.1475,0.150278,0.153056,0.155833,0.158611,0.161389,0.164167,0.166944,0.169722,0.1725,0.175278,0.178056,0.180833,0.183611,0.186389,0.189167,0.191944,0.194722,0.1975,0.200278,0.203056,0.205833,0.208611,0.211388,0.214168,0.21694,0.219756,0.222355,0.224518,0.226236,0.227506,0.228335,0.228705,0.228753,0.228738};
            static float elbow_dif[112] = {0,0,0,0,0,0,0,0,0,0.02,0.0225,0.025,0.0275,0.03,0.0325,0.035,0.0375,0.04,0.0425,0.045,0.0475,0.05,0.0525,0.055,0.0575,0.06,0.0625,0.065,0.0675,0.07,0.0725,0.075,0.0775,0.08,0.0825,0.085,0.0875,0.09,0.0925,0.095,0.0975,0.1,0.1025,0.105,0.1075,0.11,0.1125,0.115,0.1175,0.12,0.1225,0.125,0.1275,0.13,0.1325,0.135,0.1375,0.14,0.1425,0.145,0.1475,0.15,0.1525,0.155,0.1575,0.16,0.1625,0.165,0.1675,0.17,0.1725,0.175,0.1775,0.18,0.1825,0.185,0.1875,0.19,0.1925,0.195,0.1975,0.2,0.2025,0.205,0.2075,0.21,0.2125,0.215,0.2175,0.22,0.2225,0.225,0.2275,0.23,0.2325,0.235,0.237495,0.239897,0.242129,0.244182,0.246057,0.247753,0.249271,0.25061,0.25177,0.252752,0.253555,0.25418,0.254626,0.254894,0.254992,0.30};
            static float wrist_dif[175] = {0,0,0,0,0,0,0,0,0,0,0,0,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.291992,0.291908,0.291689,0.291331,0.290834,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20,0.20};
            static int cnt = -1;
            static int elbow_cnt = 20;
            static int wrist_cnt = 5;
            static float shoulder_first_pos = 0;
            static float elbow_first_pos = 0;
            static float wrist_first_pos = 0;
            
            elbow_cnt++;
            wrist_cnt++;
            cnt++;
            
            if (cnt == 0)
            {
                shoulder_first_pos = trunk.Shoulder.GetActPos();
                elbow_first_pos = trunk.Elbow.GetActPos();
                wrist_first_pos = trunk.Wrist.GetActPos();
            }

            if (elbow_cnt >= 111)
            {
                elbow_cnt = 111;
            }

            if (cnt >= 88)
            {
                cnt = 88;
            }

            if (wrist_cnt >= 174)
            {
                wrist_cnt = 174;

                if (fabs(trunk.Shoulder.GetActPos() - shoulder_pos) < 0.3
                    && fabs(trunk.Elbow.GetActPos() - elbow_pos) < 0.015
                    && fabs(trunk.Wrist.GetActPos() - wrist_pos) < 0.02)
                {
                    plannerNode->Raw_Ctrl(0);
                    plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
                    static int cnt = 0;
                    cnt++;
                    if(cnt >= 25)
                    {
                        flag = 1;
                    }
                }
            }
            else
            {
                shoulder_pos = shoulder_first_pos - shoulder_dif[cnt] * 18;
                elbow_pos = elbow_first_pos + elbow_dif[elbow_cnt];
                wrist_pos = wrist_first_pos + wrist_dif[wrist_cnt];
            }

            if (elbow_pos > 0)
            {
                elbow_pos = 0;
            }

            if (shoulder_pos < 1.4)
            {
                shoulder_pos = 1.4;
            }

            trunk.Shoulder.SetTarPara(0, shoulder_pos, 0);
            trunk.Elbow.SetTarPara(0.8, elbow_pos, 0);
            trunk.Wrist.SetTarPara(0.7, wrist_pos, 0);
        }
        else if (i == 2)// HTLSAVE
        {
            static float shoulder_pos = 0;
            static float elbow_pos = 0;
            static float wrist_pos = 0;
            static float shoulder_dif[179] = {0.000005,0.000031,0.000103,0.000249,0.000492,0.000838,0.001292,0.001851,0.002517,0.00329,0.004169,0.005153,0.006245,0.007442,0.008746,0.010157,0.011673,0.013296,0.015025,0.016861,0.018802,0.02085,0.023004,0.025262,0.027602,0.029989,0.032388,0.034788,0.037188,0.039588,0.041988,0.044388,0.046788,0.049188,0.051588,0.053988,0.056388,0.058788,0.061188,0.063588,0.065988,0.068388,0.070788,0.073188,0.075588,0.077988,0.080388,0.082788,0.085188,0.087588,0.089988,0.092388,0.094788,0.097188,0.099588,0.101988,0.104388,0.106788,0.109188,0.111588,0.113988,0.116388,0.118788,0.121188,0.123588,0.125988,0.128388,0.130788,0.133188,0.135588,0.137988,0.140388,0.142788,0.145188,0.147588,0.149988,0.152388,0.154788,0.157188,0.159588,0.161988,0.164388,0.166788,0.169188,0.171588,0.173988,0.176388,0.178788,0.181188,0.183588,0.185988,0.188388,0.190788,0.193188,0.195588,0.197988,0.200388,0.202788,0.205188,0.207588,0.209988,0.212388,0.214788,0.217188,0.219588,0.221988,0.224388,0.226788,0.229188,0.231588,0.233988,0.236388,0.238788,0.241188,0.243588,0.245988,0.248388,0.250788,0.253188,0.255588,0.257988,0.260388,0.262788,0.265188,0.267588,0.269988,0.272388,0.274788,0.277188,0.279588,0.281988,0.284388,0.286788,0.289188,0.291588,0.293988,0.296388,0.298788,0.301188,0.303588,0.305988,0.308388,0.310788,0.313188,0.315588,0.317988,0.320388,0.322788,0.325188,0.327588,0.329988,0.332388,0.334786,0.337162,0.339478,0.341703,0.343822,0.345834,0.34774,0.34954,0.351234,0.352821,0.354302,0.355677,0.356945,0.358107,0.359164,0.360113,0.360956,0.361692,0.362324,0.362848,0.363266,0.363578,0.363786,0.363905,0.363959,0.363975,0.363977};
            static float elbow_dif[195] = {0.000391,0.000794,0.001342,0.002032,0.002866,0.003843,0.004964,0.006228,0.007636,0.009187,0.010881,0.012719,0.0147,0.016825,0.019093,0.021501,0.024,0.0265,0.029,0.0315,0.034,0.0365,0.039,0.0415,0.044,0.0465,0.049,0.0515,0.054,0.0565,0.059,0.0615,0.064,0.0665,0.069,0.0715,0.074,0.0765,0.079,0.0815,0.084,0.0865,0.089,0.0915,0.094,0.0965,0.099,0.1015,0.104,0.1065,0.109,0.1115,0.114,0.1165,0.119,0.1215,0.124,0.1265,0.129,0.1315,0.134,0.1365,0.139,0.1415,0.144,0.1465,0.149,0.1515,0.154,0.1565,0.159,0.1615,0.164,0.1665,0.169,0.1715,0.174,0.1765,0.179,0.1815,0.184,0.1865,0.189,0.1915,0.194,0.1965,0.199,0.2015,0.204,0.2065,0.209,0.2115,0.214,0.2165,0.219,0.2215,0.224,0.2265,0.229,0.2315,0.234,0.2365,0.239,0.2415,0.244,0.2465,0.249,0.2515,0.254,0.2565,0.259,0.2615,0.264,0.2665,0.269,0.2715,0.274,0.2765,0.279,0.2815,0.284,0.2865,0.289,0.2915,0.294,0.2965,0.299,0.3015,0.304,0.3065,0.309,0.3115,0.314,0.3165,0.319,0.3215,0.324,0.3265,0.329,0.3315,0.334,0.3365,0.339,0.3415,0.344,0.3465,0.349,0.3515,0.354,0.3565,0.359,0.3615,0.364,0.3665,0.369,0.3715,0.374,0.3765,0.379,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993,0.432993};
            static float wrist_dif[258] = {0.000713,0.001458,0.002469,0.003748,0.005294,0.007106,0.009186,0.011532,0.014146,0.017009,0.02,0.023,0.026,0.029,0.032,0.035,0.038,0.041,0.044,0.047,0.05,0.053,0.056,0.059,0.062,0.065,0.068,0.071,0.074,0.077,0.08,0.083,0.086,0.089,0.092,0.095,0.098,0.101,0.104,0.107,0.11,0.113,0.116,0.119,0.122,0.125,0.128,0.131,0.134,0.137,0.14,0.143,0.146,0.149,0.152,0.155,0.158,0.161,0.164,0.167,0.17,0.173,0.176,0.179,0.182,0.185,0.188,0.191,0.194,0.197,0.2,0.203,0.206,0.209,0.212,0.215,0.218,0.221,0.224,0.227,0.23,0.233,0.236,0.239,0.242,0.245,0.248,0.251,0.254,0.257,0.26,0.262991,0.265854,0.268468,0.270814,0.272894,0.274706,0.276252,0.277531,0.278542,0.279287,0.279765,0.279977,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.28,0.279997,0.279959,0.279856,0.279683,0.27944,0.279128,0.278747,0.278296,0.277776,0.277186,0.276527,0.275798,0.275,0.274133,0.273196,0.27219,0.271114,0.269969,0.268757,0.2675,0.26625,0.265,0.26375,0.2625,0.26125,0.26,0.25875,0.2575,0.25625,0.255,0.25375,0.2525,0.25125,0.25,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18,0.18};
            static int cnt = -1;
            static int wrist_cnt = 0;
            static int elbow_cnt = 12;
            wrist_cnt++;
            elbow_cnt++;
            cnt++;
            static float shoulder_first_pos = 0;
            static float elbow_first_pos = 0;
            static float wrist_first_pos = 0;
            if (cnt == 0)
            {
                shoulder_first_pos = trunk.Shoulder.GetActPos();
                elbow_first_pos = trunk.Elbow.GetActPos();
                wrist_first_pos = trunk.Wrist.GetActPos();
            }

            if (cnt >= 178)
            {
                cnt = 178;
            }
            if (elbow_cnt >= 194)
            {
                elbow_cnt = 194;
            }

            if (wrist_cnt > 257)
            {
                wrist_cnt = 257;
                if (fabs(trunk.Shoulder.GetActPos() - shoulder_pos) < 0.3
                    && fabs(trunk.Elbow.GetActPos() - elbow_pos) < 0.02
                    && fabs(trunk.Wrist.GetActPos() - wrist_pos) < 0.02)
                {
                    plannerNode->Raw_Ctrl(0);
                    plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
                    static int cnt = 0;
                    cnt++;
                    if(cnt >= 25)
                    {
                        flag = 1;
                    }
                }
            }
            else
            {
                shoulder_pos = shoulder_first_pos - shoulder_dif[cnt] * 18;
                elbow_pos = elbow_first_pos + elbow_dif[elbow_cnt];
                wrist_pos = wrist_first_pos + wrist_dif[wrist_cnt];
            }

            if (elbow_pos > 0)
            {
                elbow_pos = 0;
            }
            if (shoulder_pos < 1.4)
            {
                shoulder_pos = 1.4;
            }

            trunk.Shoulder.SetTarPara(0, shoulder_pos, 0);
            trunk.Elbow.SetTarPara(0.9, elbow_pos, 0);
            trunk.Wrist.SetTarPara(1.0, wrist_pos, 0);
        }
        else if (i == 3)
        {
            if(TrunkTakePiece(2))
            {
                return true;
            }
        }
    }
    else
    {
        static int tim_cnt = -1;
        float shoulder_a1 = 0.1;
        float shoulder_a2 = 0.1;
        float shoulder_max_vel = 8;

        static float shoulder_last_pos = 0;
        static float elbow_last_pos = 0;
        if(tim_cnt == -1)
        {
            shoulder_last_pos = trunk.Shoulder.GetActPos() - 0.5;
            elbow_last_pos = TRUNK_LTH0_ELBOW;
        }

        tim_cnt++;

        trunk.Elbow.SetTarPara(1, elbow_last_pos, 0);
        if (trunk.Shoulder.PlanControl(shoulder_a1, shoulder_a2, shoulder_max_vel, shoulder_last_pos, tim_cnt))
        {
            tim_cnt = -1;
            flag = 0;
            return true;
        }
    }

    return false;
}

bool TrunkReceive(float shoulder_max_vel, float elbow_max_vel, float wrist_max_vel)
{
    static int tim_cnt = -1;
    static bool ready_flag = 0;
    static bool next_flag = 0;

    float shoulder_a1 = 0.2;
    float shoulder_a2 = 0.1;

    trunk.Wrist.SetTarPara(wrist_max_vel, 0, 0);
    trunk.Elbow.SetTarPara(elbow_max_vel, -0.027778, 0);

    tim_cnt++;

    if (plannerNode->UpperTask == throw_kfs)
    {
        if (trunk.Shoulder.PlanControl(shoulder_a1, shoulder_a2, shoulder_max_vel, 0.75, tim_cnt))
        {
            ready_flag = 1;
        }
    }
    else
    {
        if (trunk.Shoulder.PlanControl(shoulder_a1, shoulder_a2, shoulder_max_vel, 0.01, tim_cnt))
        {
            ready_flag = 1;
        }
    }

    if (ready_flag && fabs(trunk.Wrist.GetActPos() - trunk.Wrist.GetTarPos()) < 0.001 && fabs(trunk.Elbow.GetActPos() - trunk.Elbow.GetTarPos()) < 0.001)
    {
        tim_cnt = -1;
        ready_flag = 0;
        
        return true;
    }

    return false;
}

bool UpPut(float i)
{
    leg.SetAllCtrlMode(POS_MODE);

    static int tim_cnt = -1;
    float leg_max_vel = 8.0;
    float leg_a1 = 0.1;
    float leg_a2 = 0.1;

    tim_cnt++;

    if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, i, tim_cnt) &
        leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, i, tim_cnt) &
        leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, i, tim_cnt) &
        leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, i, tim_cnt))
    {
        tim_cnt = -1;
        return true;
    }
    return false;
}

bool DownPut(void)
{
    leg.SetAllCtrlMode(POS_MODE);

    static int tim_cnt = -1;
    float leg_max_vel = 8.0;
    float leg_a1 = 0.2;
    float leg_a2 = 0.1;

    tim_cnt++;

    if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
        leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
        leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
        leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt))
    {
        tim_cnt = -1;
        return true;
    }
    return false;
}

bool NoUp(int way)
{
    static int step = 0;
    static float tarvelx = 0;
    static float tarvely = 0;
    static float omega = 0;
    static float tarposx = 0;
    static int LR_flag = 1;
    static int leg_up = 0;
    static bool leg_down_flag = 0;
    static bool step_back = false;
    step_back = 1;

    plannerNode->logger()->info("way:{} ", way);

    if (way == 7 || way == 8 || way == 9)
    {
        static float tar_posx1 = 0;
        static float tar_posx2 = 0;

        if (way == 7)
        {
            tar_posx1 = 4225 * plannerNode->myFlags.RB_flag;
            tar_posx2 = 3025 * plannerNode->myFlags.RB_flag;
        }
        else if (way == 8)
        {
            tar_posx1 = 3025 * plannerNode->myFlags.RB_flag;
            tar_posx2 = 1825 * plannerNode->myFlags.RB_flag;
        }
        else if (way == 9)
        {
            tar_posx1 = 3025 * plannerNode->myFlags.RB_flag;
            tar_posx2 = 4225 * plannerNode->myFlags.RB_flag;
        }

        static bool leg_flag = 0;
        if (!leg_flag)
        {
            if ((way == 8 || way == 9) && plannerNode->UpperTask != get_kfs && leg_up == 4 && plannerNode->UpperTask != save_kfs)
            {
                leg_up = 1;
            }
        }
        else
        {
            if ((way == 8 || way == 9) && plannerNode->UpperTask != get_kfs && leg_up == 2 && trunk.Shoulder.GetActPos() < 4.8)
            {
                leg_up = 3;
            }
        }

        if (step == 0)
        {
            static bool flag = 0;
            if (!flag)
            {
                plannerNode->myFlags.aimFlag = 0;
                leg_up = 3;
                flag = 1;
            }

            if (plannerNode->myFlags.trunk_flag && plannerNode->nowPos.y > 2400)
            {
                if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
                {
                    tarvely = 450;
                }
                else
                {
                    tarvely = 150;
                }
            }
            else
            {
                tarvely = 950;
            }

            if (step_back && plannerNode->Ps_Num[1])
            {
                tarvely = 50;
            }

            if (!plannerNode->Ps_Num[13] && !plannerNode->Ps_Num[14])
            {
                tarvely = 50;
            }

            plannerNode->tarVel.SetVelY(tarvely);

            if (plannerNode->UpperTask != get_kfs)
            {
                step = 1;
            }
        }
        else if (step == 1)
        {
            if (way == 7 || way == 9)
            {
                LR_flag = 1;
            }
            else
            {
                LR_flag = -1;
            }

            if (plannerNode->UpperTask == save_kfs)
            {
                tarvelx = 500 * LR_flag * plannerNode->myFlags.RB_flag;
            }
            else
            {
                tarvelx = 1500 * LR_flag * plannerNode->myFlags.RB_flag;
            }

            tarvely = PosControl(tar_posx1, plannerNode->nowPos.x, 2500, plannerNode->nowPos.y, 1.8, 0.4, 3.6, 0.6).y;
            omega = YawControl(plannerNode->nowPos.yaw, 0, 2.4, 0.6);

            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

            if(fabs(plannerNode->nowPos.x - tar_posx1) <= 600)
            {
                step = 2;
            }
        }
        else if (step == 2)
        {
            if (fabs(plannerNode->nowPos.x - tar_posx1) <= 300)
            {
                tarvelx = PosControl(tar_posx1, plannerNode->nowPos.x, 0, 0, 0.3, 0.1, 0, 0).x;
            }
            else
            {
                tarvelx = PosControl(tar_posx1, plannerNode->nowPos.x, 0, 0, 0.8, 0.2, 0, 0).x;
            }

            tarvely = PosControl(tar_posx2, plannerNode->nowPos.x, 2500, plannerNode->nowPos.y, 1.8, 0.4, 2.4, 0.6).y;
            omega = YawControl(plannerNode->nowPos.yaw, 0, 3.6, 0.8);

            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

            if (fabs(plannerNode->nowPos.x - tar_posx1) <= 200 && plannerNode->myFlags.trunk_flag)
            {
                step = 3;
                plannerNode->myFlags.aimFlag = 0;
            }
        }
        else if (step == 3)
        {
            if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
            {
                tarvely = 450;
            }
            else
            {
                tarvely = 150;
            }

            if (step_back && plannerNode->Ps_Num[1])
            {
                tarvely = 0;
            }

            if (!plannerNode->Ps_Num[13] && !plannerNode->Ps_Num[14])
            {
                tarvely = 50;
            }

            plannerNode->tarVel.SetVelY(tarvely);

            if (plannerNode->UpperTask != get_kfs)
            {
                step = 4;
                leg_flag = 1;
            }
        }
        else if (step == 4)
        {
            if (way == 9)
            {
                LR_flag = 1;
            }
            else
            {
                LR_flag = -1;
            }

            if (plannerNode->UpperTask == save_kfs)
            {
                tarvelx = 500 * LR_flag * plannerNode->myFlags.RB_flag;
            }
            else
            {
                tarvelx = 1500 * LR_flag * plannerNode->myFlags.RB_flag;
            }

            tarvely = PosControl(tar_posx2, plannerNode->nowPos.x, 2500, plannerNode->nowPos.y, 1.8, 0.4, 2.4, 0.4).y;
            omega = YawControl(plannerNode->nowPos.yaw, 0, 3.6, 0.8);

            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

            if(fabs(plannerNode->nowPos.x - tar_posx2) <= 600)
            {
                step = 5;
                if (way == 7)
                {
                    leg_up = 1;
                }
            }
        }
        else if (step == 5)
        {
            if (fabs(plannerNode->nowPos.x - tar_posx2) <= 300)
            {
                tarvelx = PosControl(tar_posx2, plannerNode->nowPos.x, 0, 0, 0.3, 0.1, 0, 0).x;
            }
            else
            {
                tarvelx = PosControl(tar_posx2, plannerNode->nowPos.x, 0, 0, 0.8, 0.2, 0, 0).x;
            }

            tarvely = PosControl(tar_posx2, plannerNode->nowPos.x, 2500, plannerNode->nowPos.y, 1.8, 0.4, 2.4, 0.4).y;
            omega = YawControl(plannerNode->nowPos.yaw, 0, 2.8, 0.6);

            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

            if (fabs(plannerNode->nowPos.x - tar_posx2) <= 200)
            {
                plannerNode->mystep = front_leg_up;
                if (way == 7)
                {
                    if (leg_up == 2)
                    {
                        return true;
                    }
                }
                else
                {
                    if (leg_up == 4)
                    {
                        return true;
                    }
                }
            }
        }
    }
    else
    {
        if (way == 1 || way == 2 || way == 3)
        {
            LR_flag = -1;
        }
        else 
        {
            LR_flag = 1;
        }

        if (way == 1 || way == 3)
        {
            tarposx = 1825 * plannerNode->myFlags.RB_flag;
        }
        else if (way == 2 || way == 6)
        {
            tarposx = 3025 * plannerNode->myFlags.RB_flag; 
        }
        else if (way == 4 || way == 5)
        {
            tarposx = 4225 * plannerNode->myFlags.RB_flag;
        }

        if (way == 1 || way == 5)
        {
            if (step_back)
            {
                static int flag = 0;
                if (trunk.Shoulder.GetActPos() < 4.8 && plannerNode->UpperTask != get_kfs)
                {
                    flag = 1;
                }
                
                if (flag == 0)
                {
                    tarvelx = 1200 * LR_flag * plannerNode->myFlags.RB_flag;
                }
                else
                {
                    tarvelx = 2000 * LR_flag * plannerNode->myFlags.RB_flag;
                }
            }
            else
            {
                tarvelx = 2000 * LR_flag * plannerNode->myFlags.RB_flag;
            }
        }
        else
        {
            static bool flag = 0;

            // if (trunk.Shoulder.GetActPos() < 4.8 && plannerNode->UpperTask != get_kfs)
            // {

            // }
            tarvelx = 1600 * LR_flag * plannerNode->myFlags.RB_flag;
        }

        if (step_back)
        {
            if (leg_up == 0)
            {
                if ((way == 3 || way == 4))
                {
                    leg_up = 1;
                }
                else
                {
                    leg_up = 3;
                }
            }
        }

        if (step == 1 || step == 2)
        {
            if (step_back)
            {
                tarvely = PosControl(0, 0, 2500, plannerNode->nowPos.y, 0, 0, 1.8, 0.4).y;

                if (way == 3 || way == 4)
                {
                    if (leg_up == 2 && plannerNode->UpperTask != save_kfs)
                    {
                        leg_up = 3;
                    }
                }
                else if (way == 2 || way == 6)
                {
                    if (leg_up == 4 && plannerNode->UpperTask != save_kfs && leg_down_flag)
                    {
                        leg_up = 1;
                    }
                }
            }
            else
            {
                tarvely = PosControl(0, 0, 2800, plannerNode->nowPos.y, 0, 0, 1.8, 0.4).y;
            }
        }

        if (!step_back)
        {
            if (step == 0)
            {
                omega = YawControl(plannerNode->nowPos.yaw, 0, 3.6, 0.6);
            }
            else
            {
                omega = 0;
            }
        }
        else
        {
            omega = YawControl(plannerNode->nowPos.yaw, 0, 3.6, 0.6);
        }

        if (step == 0)
        {
            static bool flag = 0;
            if (!flag)
            {
                plannerNode->myFlags.aimFlag = 0;
                flag = 1;
            }

            if (plannerNode->myFlags.trunk_flag && plannerNode->nowPos.y > 2200)
            {
                if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
                {
                    tarvely = 450;
                }
                else
                {
                    tarvely = 150;
                }
            }
            else
            {
                tarvely = 900;
            }

            if (plannerNode->UpperTask != get_kfs)
            {
                step = 1;
            }

            plannerNode->tarVel.SetVelY(tarvely);
        }
        else if (step == 1)
        {
            if (plannerNode->UpperTask == save_kfs)
            {
                plannerNode->tarVel.SetVel2(tarvelx * 0.25, tarvely, omega);    
            }
            else
            {
                plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);
            } 

            if (fabs(plannerNode->nowPos.x - tarposx) <= 600)
            {
                step = 2;
            }
        }
        else if (step == 2)
        {
            if (fabs(plannerNode->nowPos.x - tarposx) <= 350)
            {
                leg_down_flag = 1;
                tarvelx = PosControl(tarposx, plannerNode->nowPos.x, 0, 0, 0.3, 0.1, 0, 0).x;
            }
            else
            {
                tarvelx = PosControl(tarposx, plannerNode->nowPos.x, 0, 0, 0.8, 0.2, 0, 0).x;
            }

            plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

            if (fabs(plannerNode->nowPos.x - tarposx) <= 200)
            {
                if (step_back)
                {
                    plannerNode->myFlags.aimFlag = 0;
                    plannerNode->mystep = front_leg_up;
                    if ((way == 2 || way == 6))
                    {
                        if (leg_up == 2 && (plannerNode->myFlags.trunk_flag || plannerNode->lrz.R2.FirsrR2KFS == 1 || plannerNode->lrz.R2.FirsrR2KFS == 2 || plannerNode->lrz.R2.FirsrR2KFS == 4))
                        {
                            return true;
                        }
                    }
                    else
                    {
                        if (leg_up == 4 && (plannerNode->myFlags.trunk_flag || plannerNode->lrz.R2.FirsrR2KFS == 1 || plannerNode->lrz.R2.FirsrR2KFS == 2 || plannerNode->lrz.R2.FirsrR2KFS == 4))
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    return true;
                }
            }
        }
    }

    if (leg_up == 1)
    {
        leg.SetAllCtrlMode(POS_MODE);

        leg.LegOne.SetTarPosKpKd(3.2, 0.6);
        leg.LegTwo.SetTarPosKpKd(4.0, 0.8);
        leg.LegThree.SetTarPosKpKd(3.6, 0.6);
        leg.LegFour.SetTarPosKpKd(3.0, 0.6);

        static int leg_up_cnt1 = -1;
        float leg_max_vel = 8.0;
        float leg1_a1 = 0.1;
        float leg1_a2 = 0.1;

        leg_up_cnt1++;

        if (leg.LegOne.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegTwo.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegThree.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1) &
            leg.LegFour.PlanControl(leg1_a1, leg1_a2, leg_max_vel, GearUp1, leg_up_cnt1))
        {
            leg_up_cnt1 = -1;
            leg_up = 2;
        }
    }

    if (leg_up == 3)
    {
        leg.SetAllCtrlMode(POS_MODE);

        leg.LegOne.SetTarPosKpKd(3.5, 0.6);
        leg.LegTwo.SetTarPosKpKd(4.2, 0.6);
        leg.LegThree.SetTarPosKpKd(3.6, 0.6);
        leg.LegFour.SetTarPosKpKd(3.2, 0.6);

        static int leg_up_cnt2 = -1;
        float leg_max_vel = 8.0;
        float leg2_a1 = 0.1;
        float leg2_a2 = 0.1;

        leg_up_cnt2++;

        if (leg.LegOne.PlanControl(leg2_a1, leg2_a2, leg_max_vel, GearUp2, leg_up_cnt2) &
            leg.LegTwo.PlanControl(leg2_a1, leg2_a2, leg_max_vel, GearUp2, leg_up_cnt2) &
            leg.LegThree.PlanControl(leg2_a1, leg2_a2, leg_max_vel, GearUp2, leg_up_cnt2) &
            leg.LegFour.PlanControl(leg2_a1, leg2_a2, leg_max_vel, GearUp2, leg_up_cnt2))
        {
            leg_up_cnt2 = -1;
            leg_up = 4;
        }
    }

    return false;
}

bool Up200(void)
{
    switch (plannerNode->mystep)
    {
        case all_leg_down:
        {
            if (plannerNode->myFlags.getKFS_flag || plannerNode->myFlags.htl_flag)
            {
                plannerNode->tarVel.SetVelY(400);
            }

            leg.SetAllCtrlMode(POS_MODE);

            leg.LegOne.SetTarPosKpKd(3.4, 0.8);
            leg.LegTwo.SetTarPosKpKd(4.0, 0.8);
            leg.LegThree.SetTarPosKpKd(4.0, 0.8);
            leg.LegFour.SetTarPosKpKd(3.4, 0.8);

            static int tim_cnt = -1;
            float leg_max_vel = 9.0;
            if (plannerNode->ChassisTask == area2_move3)
            {
                leg_max_vel = 2.0;
            }
            float leg_a1 = 0.1;
            float leg_a2 = 0.1;

            tim_cnt++;

            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp1, tim_cnt))
            {
                tim_cnt = -1;
                plannerNode->myFlags.save_ready_flag = 1;
                plannerNode->Vavle_Chassis_Ctrl(0, 1);
                plannerNode->logger()->info("front_leg_up");
                plannerNode->mystep = front_leg_up;
            }

            break;
        }
        case front_leg_up:
        {
            leg.SetAllCtrlMode(POS_MODE);
            static bool Up_leg_flag = false;
            static int tim_cnt = -1;
            float leg_max_vel = 20.0;
            float leg_a1 = 0.08;
            float leg_a2 = 0.08;
            static bool flag = 0;

            plannerNode->Vavle_Chassis_Ctrl(0, 1);

            if (plannerNode->UpperTask == get_kfs)
            {
                leg.LegOne.SetTarPosKpKd(3.6, 0.8);
                leg.LegTwo.SetTarPosKpKd(4.2, 0.8);
                leg.LegThree.SetTarPosKpKd(4.2, 0.8);
                leg.LegFour.SetTarPosKpKd(3.4, 0.8);
            }
            else
            {
                leg.LegOne.SetTarPosKpKd(3.8, 0.8);
                leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
                leg.LegThree.SetTarPosKpKd(4.0, 0.6);
                leg.LegFour.SetTarPosKpKd(3.8, 0.8);
            }


            if (!flag)
            {
                plannerNode->myFlags.aimFlag = 0;
                flag = 1;
            }

            if (plannerNode->nowPos.y < 2350)
            {
                plannerNode->tarVel.SetVelY(1300);
            }
            else
            {
                plannerNode->tarVel.SetVelY(1050);
            }

            if (plannerNode->myFlags.pre_deceleration)
            {
                plannerNode->tarVel.SetVelY(650);
            }

            if (plannerNode->myFlags.getKFS_flag && plannerNode->nowPos.y >= 2200)
            {
                if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12] && plannerNode->myFlags.trunk_flag)
                {
                    if (plannerNode->myFlags.pre_deceleration)
                    {
                        plannerNode->tarVel.SetVelY(400);
                    }
                    else
                    {
                        plannerNode->tarVel.SetVelY(550);
                    }
                }
                else
                {
                    plannerNode->tarVel.SetVelY(150);
                }
            }

            if (plannerNode->Ps_Num[1] && fabs(plannerNode->nowPos.yaw) <= 3 && plannerNode->myFlags.pre_deceleration)
            {
                if (plannerNode->myFlags.getKFS_flag)
                {
                    if (plannerNode->Vavle_Num[9])
                    {
                        Up_leg_flag = true;
                    }
                }
                else
                {
                    Up_leg_flag = true;
                }
            }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                plannerNode->tarVel.SetVelY(280);

                if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
                    leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt))
                {
                    plannerNode->myFlags.save_start_flag = 1;
                    tim_cnt = -1;
                    flag = 0;
                    Up_leg_flag = false;
                    plannerNode->mystep = hind_leg_up;
                }
            }
            break;
        }
        case hind_leg_up:
        {
            static bool Up_leg_flag = false;
            static int tim_cnt = -1;
            float leg_max_vel = 20.0;
            float leg_a1 = 0.08;
            float leg_a2 = 0.08;

            leg.LegTwo.SetTarPosKpKd(4.5, 0.8);
            leg.LegThree.SetTarPosKpKd(4.5, 0.8);

            plannerNode->tarVel.SetVelY(1200);

            if (plannerNode->Ps_Num[3])
            {
                plannerNode->Vavle_Chassis_Ctrl(1, 0);
                plannerNode->tarVel.SetVelY(820);
                leg.LegOne.SetCtrlMode(DISABLE);
                leg.LegFour.SetCtrlMode(DISABLE);
            }

            if (plannerNode->Ps_Num[7])
            {
                plannerNode->tarVel.SetVelY(400);
                leg.SetAllCtrlMode(POS_MODE);
                leg.LegOne.SetTarPara(0, 0.01, 0);
                leg.LegFour.SetTarPara(0, 0.01, 0);
                Up_leg_flag = true;
            }

            if (plannerNode->Ps_Num[4])
            {
                plannerNode->tarVel.SetVelY(150);
            }

            // if (plannerNode->Ps_Num[5])
            // {
            //     leg.SetAllCtrlMode(POS_MODE);
            //     leg.LegOne.SetTarPara(0, 0.01, 0);
            //     leg.LegFour.SetTarPara(0, 0.01, 0);
            //     if (plannerNode->UpperTask == throw_kfs)
            //     {
            //         plannerNode->tarVel.SetVelY(0);
            //     }
            //     else
            //     {
            //         Up_leg_flag = true;
            //     }
            // }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                    leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
                {
                    tim_cnt = -1;
                    Up_leg_flag = false;
                    plannerNode->mystep = nostep;
                }
            }

            break;
        }
        case nostep:
        {
            plannerNode->tarVel.SetVelY(1000);

            plannerNode->Vavle_Chassis_Ctrl(0, 1);
            
            if (plannerNode->Ps_Num[6])
            {
                leg.LegOne.SetCtrlMode(DISABLE);
                leg.LegFour.SetCtrlMode(DISABLE);
                return true;
            }

            break;
        }
    }

    return false;
}

bool Up400(void)
{
    switch (plannerNode->mystep)
    {
        case all_leg_down:
        {
            leg.SetAllCtrlMode(POS_MODE);
           
            leg.LegOne.SetTarPosKpKd(3.2, 0.6);
            leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
            leg.LegThree.SetTarPosKpKd(3.4, 0.6);
            leg.LegFour.SetTarPosKpKd(3.0, 0.6);

            static int tim_cnt = -1;
            float leg_max_vel = 11.0;
            float leg_a1 = 0.2;
            float leg_a2 = 0.2;

            tim_cnt++;

            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt))
            {
                tim_cnt = -1;
                plannerNode->Vavle_Chassis_Ctrl(0, 1);
                plannerNode->mystep = front_leg_up;
            }

            break;
        }
        case front_leg_up:
        {
            static bool Up_leg_flag = false;
            static int tim_cnt = -1;
            float leg_max_vel = 20.0;
            float leg_a1 = 0.05;
            float leg_a2 = 0.05;
            static bool flag = 0;

            plannerNode->Vavle_Chassis_Ctrl(0, 1);

            if (plannerNode->UpperTask == get_kfs)
            {
                leg.LegOne.SetTarPosKpKd(3.4, 0.8);
                leg.LegTwo.SetTarPosKpKd(4.2, 0.8);
                leg.LegThree.SetTarPosKpKd(4.2, 0.8);
                leg.LegFour.SetTarPosKpKd(3.2, 0.8);
            }
            else
            {
                leg.LegOne.SetTarPosKpKd(3.8, 0.8);
                leg.LegTwo.SetTarPosKpKd(4.0, 0.6);
                leg.LegThree.SetTarPosKpKd(4.0, 0.6);
                leg.LegFour.SetTarPosKpKd(3.8, 0.8);
            }

            if (!flag)
            {
                plannerNode->myFlags.aimFlag = 0;
                flag = 1;
            }

            if (plannerNode->nowPos.y < 2400)
            {
                plannerNode->tarVel.SetVelY(1300);
            }
            else
            {
                plannerNode->tarVel.SetVelY(1000);
            }

            if (plannerNode->myFlags.pre_deceleration)
            {
                plannerNode->tarVel.SetVelY(350);
            }

            if (plannerNode->myFlags.getKFS_flag && plannerNode->nowPos.y >= 2200)
            {
                if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12] && plannerNode->myFlags.trunk_flag)
                {
                    if (plannerNode->myFlags.pre_deceleration)
                    {
                        plannerNode->tarVel.SetVelY(350);
                    }
                    else
                    {
                        plannerNode->tarVel.SetVelY(650);
                    }
                }
                else
                {
                    plannerNode->tarVel.SetVelY(150);
                }
            }

            if (plannerNode->Ps_Num[1] && fabs(plannerNode->nowPos.yaw) <= 3 && plannerNode->myFlags.pre_deceleration)
            {
                if (plannerNode->myFlags.getKFS_flag)
                {
                    if (plannerNode->Vavle_Num[9])
                    {
                        Up_leg_flag = true;
                    }
                }
                else
                {
                    Up_leg_flag = true;
                }
            }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                plannerNode->tarVel.SetVelY(120);
                if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                    leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
                {
                    plannerNode->myFlags.save_start_flag = 1;
                    tim_cnt = -1;
                    flag = 0;
                    Up_leg_flag = false;
                    plannerNode->mystep = hind_leg_up;
                }
            }
            break;
        }
        case hind_leg_up:
        {
            static bool Up_leg_flag = false;
            static int tim_cnt = -1;
            float leg_max_vel = 22.0;
            float leg_a1 = 0.05;
            float leg_a2 = 0.05;

            leg.LegTwo.SetTarPosKpKd(4.2, 0.8);
            leg.LegThree.SetTarPosKpKd(4.2, 0.8);

            plannerNode->tarVel.SetVelY(1250);

            if (plannerNode->Ps_Num[3])
            {
                plannerNode->Vavle_Chassis_Ctrl(1, 0);
                plannerNode->tarVel.SetVelY(680);
                leg.LegOne.SetCtrlMode(DISABLE);
                leg.LegFour.SetCtrlMode(DISABLE);
            }

            if (plannerNode->Ps_Num[7])
            {
                plannerNode->tarVel.SetVelY(120);
                leg.SetAllCtrlMode(POS_MODE);
                leg.LegOne.SetTarPara(0, 0.01, 0);
                leg.LegFour.SetTarPara(0, 0.01, 0);
                Up_leg_flag = true;
            }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                plannerNode->Logger()->info("hind_up_leg_flag");
                plannerNode->tarVel.SetVelY(120);

                if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                    leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
                {
                    tim_cnt = -1;
                    Up_leg_flag = false;
                    plannerNode->mystep = nostep;
                }
            }
            break;
        }
        case nostep:
        {
            plannerNode->Vavle_Chassis_Ctrl(0, 1);

            plannerNode->tarVel.SetVelY(1200);

            if (plannerNode->Ps_Num[6])
            {
                return true;
            }
            break;
        }
    }

    return false;
}

bool Down200(void)
{
    switch (plannerNode->mystep)
    {
    case front_leg_down:
    {
        leg.SetAllCtrlMode(POS_MODE);
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 20.0;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;
        static int flag = 0;
        static int flag2 = 0;
        static bool aim_flag = 0;

        leg.LegOne.SetTarPosKpKd(3.8, 0.6);
        leg.LegFour.SetTarPosKpKd(3.8, 0.6);

        plannerNode->Vavle_Chassis_Ctrl(0, 1);

        if (!aim_flag)
        {
            plannerNode->myFlags.aimFlag = 0;
            aim_flag = 1;
        }

        plannerNode->tarVel.SetVelY(1200);

        if (plannerNode->myFlags.pre_deceleration)
        {
            plannerNode->tarVel.SetVelY(780);
        }

        if (plannerNode->myFlags.getKFS_flag)
        {
            if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12] && plannerNode->myFlags.trunk_flag)
            {
                if (plannerNode->myFlags.pre_deceleration)
                {
                    plannerNode->tarVel.SetVelY(780);
                }
                else
                {
                    plannerNode->tarVel.SetVelY(850);
                }
            }
            else
            {
                plannerNode->tarVel.SetVelY(150);
            }
        }

        if (!plannerNode->Ps_Num[1])
        {
            plannerNode->tarVel.SetVelY(620);
        }

        if (!plannerNode->Ps_Num[2])
        {
            plannerNode->tarVel.SetVelY(350);
        }

        if (plannerNode->myFlags.getKFS_flag)
        {
            if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
            {
                plannerNode->tarVel.SetVelY(150);
            }
        }

        // if (!plannerNode->Ps_Num[3] && !plannerNode->Ps_Num[1] && plannerNode->myFlags.pre_deceleration)
        if (!plannerNode->Ps_Num[3] && !plannerNode->Ps_Num[1])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }

        if (Down_leg_flag)
        {
            plannerNode->tarVel.SetVelY(300);
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown1, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown1, tim_cnt))
            {
                aim_flag = 0;
                tim_cnt = -1;
                Down_leg_flag = false;
                plannerNode->Vavle_Chassis_Ctrl(1, 0);
                plannerNode->mystep = hind_leg_down;
            }
        }
        break;
    }

    case hind_leg_down:
    {
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 20.0;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;

        leg.LegTwo.SetTarPosKpKd(4.2, 0.8);
        leg.LegThree.SetTarPosKpKd(4.2, 0.8);

        if ((((plannerNode->lrz.R2.Action[3] != 0 && plannerNode->nowPos.y <= 6800) || (plannerNode->lrz.R2.Action[4] != 0 && plannerNode->nowPos.y <= 8000)) && plannerNode->nowPos.y >= 5000)
            || (plannerNode->ChassisTask == area2_move3 && plannerNode->lrz.R2.Action[2] != 0 && plannerNode->nowPos.y <= 5600))
        {
            plannerNode->tarVel.SetVelY(680);
        }
        else
        {
            plannerNode->tarVel.SetVelY(1200);
        }

        if (!plannerNode->Ps_Num[6])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }

        if (Down_leg_flag)
        {
            if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown1, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown1, tim_cnt))
            {
                plannerNode->Vavle_Chassis_Ctrl(0, 1);
                tim_cnt = -1;
                Down_leg_flag = false;
                plannerNode->mystep = all_leg_up;
                if(plannerNode->ChassisTask == area2_move3)
                {
                    static int flag = 0;
                    if(flag == 0)
                    {
                        plannerNode->mystep = nostep;
                    }
                    else
                    {
                        plannerNode->mystep = all_leg_up;
                    }
                    flag++;
                }
            }
        }
        break;
    }

    case all_leg_up:
    {
        static bool Up_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 11.0;
        float leg_a1 = 0.1;
        float leg_a2 = 0.1;
        static int cnt = 0;
        static int relay_cnt = 5;

        leg.LegOne.SetTarPosKpKd(3.8, 0.8);
        leg.LegTwo.SetTarPosKpKd(3.4, 0.6);
        leg.LegThree.SetTarPosKpKd(3.4, 0.6);
        leg.LegFour.SetTarPosKpKd(4.0, 0.8);
        
        if (!plannerNode->myFlags.last_platform_flag)
        {
            if (plannerNode->myFlags.pre_deceleration)
            {
                if ((plannerNode->lrz.R2.Action[3] != 0 && plannerNode->nowPos.y <= 6800) || (plannerNode->lrz.R2.Action[4] != 0 && plannerNode->nowPos.y <= 8000))
                {
                    plannerNode->tarVel.SetVelY(380);
                    relay_cnt = 30;
                }
                else
                {
                    plannerNode->tarVel.SetVelY(520);
                    relay_cnt = 10;
                }
            }
            else
            {
                if ((plannerNode->lrz.R2.Action[3] != 0 && plannerNode->nowPos.y <= 6800) || (plannerNode->lrz.R2.Action[4] != 0 && plannerNode->nowPos.y <= 8000))
                {
                    plannerNode->tarVel.SetVelY(380);
                    relay_cnt = 30;
                }
                else
                {
                    if (plannerNode->ChassisTask == area2_move2)
                    {
                        plannerNode->tarVel.SetVelY(800);
                    }
                    else
                    {
                        plannerNode->tarVel.SetVelY(1200);
                    }
                    relay_cnt = 10;
                }
            }
        }
        else
        {
            plannerNode->tarVel.SetVelY(250);
            relay_cnt = 20;
            plannerNode->tarVel.SetVelX(800 * plannerNode->myFlags.RB_flag);
        }

        cnt++;
        if (cnt >= relay_cnt)
        {
            cnt = relay_cnt;
            Up_leg_flag = true;
        }

        if (Up_leg_flag)
        {
            tim_cnt++;
        }

        if (Up_leg_flag)
        {
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
            {
                Up_leg_flag = false;
                tim_cnt = -1;
                cnt = 0;
                plannerNode->mystep = nostep;
            }
        }

        break;
    }

    case nostep:
    {
        return true;

        break;
    }
    }

    return false;
}

bool Down400(void)
{
    switch (plannerNode->mystep)
    {
    case front_leg_down:
    {
        leg.SetAllCtrlMode(POS_MODE);
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;
        static bool flag = 0;

        plannerNode->Vavle_Chassis_Ctrl(0, 1);

        if (!flag)
        {
            plannerNode->myFlags.aimFlag = 0;
            flag = 1;
        }

        plannerNode->tarVel.SetVelY(500);

        if (plannerNode->myFlags.pre_deceleration)
        {
            plannerNode->tarVel.SetVelY(350);
        }

        if (!plannerNode->Ps_Num[1])
        {
            plannerNode->tarVel.SetVelY(250);
        }

        if (!plannerNode->Ps_Num[2])
        {
            plannerNode->tarVel.SetVelY(200);
        }
        
        if (!plannerNode->Ps_Num[3])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }

        if (Down_leg_flag)
        {
            plannerNode->tarVel.SetVelY(200);
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, 22.0, GearDown2, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, 22.0, GearDown2, tim_cnt))
            {
                tim_cnt = -1;
                flag = 0;
                Down_leg_flag = false;
                plannerNode->mystep = hind_leg_down;
                plannerNode->Vavle_Chassis_Ctrl(1, 0);
            }
        }
        break;
    }

    case hind_leg_down:
    {
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 22.0;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;
        
        plannerNode->tarVel.SetVelY(750);

        if (!plannerNode->Ps_Num[6])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }

        if (Down_leg_flag)
        {
            if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt))
            {
                plannerNode->Vavle_Chassis_Ctrl(0, 1);
                tim_cnt = -1;
                Down_leg_flag = false;
                plannerNode->mystep = all_leg_up;
            }
        }
        break;
    }

    case all_leg_up:
    {
        static bool Up_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 11.0;
        float leg_a1 = 0.1;
        float leg_a2 = 0.1;
        static int cnt = 0;

        plannerNode->tarVel.SetVelX(800 * plannerNode->myFlags.RB_flag);
        plannerNode->tarVel.SetVelY(450);

        cnt++;
        if (cnt >= 25)
        {
            cnt = 26;
            Up_leg_flag = true;
        }

        if (Up_leg_flag)
        {
            tim_cnt++;
        }

        if (Up_leg_flag)
        {
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
            leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
            leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
            leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
            {
                Up_leg_flag = false;
                cnt = 0;
                tim_cnt = -1;
                plannerNode->mystep = nostep;
            }
        }

        break;
    }

    case nostep:
    {
        leg.LegOne.SetCtrlMode(DISABLE);
        leg.LegFour.SetCtrlMode(DISABLE);
        return true;
        break;
    }
    }

    return false;
}

bool CarGo2(void)
{
    plannerNode->Vavle_Upper_Ctrl(0, 0, 0, 1);
    leg.LegOne.SetCtrlMode(DISABLE);
    leg.LegFour.SetCtrlMode(DISABLE);

    static float tarvely = 0.0f;
    static float tarvelx = 800 * plannerNode->myFlags.RB_flag;
    static float omega = 0.0f;

    // float omega2 = PitchControl(plannerNode->nowPos.pitch, 0, 0.30, 0.03);
    float omega3 = RollControl(plannerNode->nowPos.roll, 0, 0.30, 0.03);

    float leg2Vel = omega3;
    float leg3Vel = omega3;

    static float leg2Pos = 0.0;
    static float leg3Pos = 0.0;

    static int step = 0;

    static int cnt = 0;
    static float tarposy = 0;
    static int curr = 0;

    if(step == 0)
    {
        if (plannerNode->LastChassisTask == area2_move1)
        {
            if (fabs(plannerNode->nowPos.x) < 3125)
            {
                tarvelx += 25 * plannerNode->myFlags.RB_flag;
                if (fabs(tarvelx) >= 3600)
                {
                    tarvelx = 3600 * plannerNode->myFlags.RB_flag;
                }
            }
            else
            {
                tarvelx -= 25 * plannerNode->myFlags.RB_flag;
                if (fabs(tarvelx) <= 2250)
                {
                    tarvelx = 2250 * plannerNode->myFlags.RB_flag;
                }
            }
        }
        else
        {
            tarvelx += 25 * plannerNode->myFlags.RB_flag;
            if (fabs(tarvelx) >= 2250)
            {
                tarvelx = 2250 * plannerNode->myFlags.RB_flag;
            }
        }

        if(cnt == 0)
        {
            tarposy = plannerNode->nowPos.y - 150;
            cnt = 1;
        }

        tarvely = PosControl(0, 0, tarposy, plannerNode->nowPos.y, 0, 0, 3.0, 0.8).y;

        omega = YawControl(plannerNode->nowPos.yaw, 0, 4.8, 0.8);

        if (fabs(plannerNode->nowPos.x) > 4475)
        {
            tarvely = 0;
            cnt = 0;
            step = 1;
        }
    }
    else if (step == 1)
    {
        leg2Pos += leg2Vel * 0.005;
        leg3Pos += leg3Vel * 0.005;

        if (leg2Pos <= 0.1 && leg2Vel < 0)
        {
            leg2Vel = 0;
            leg2Pos = 0;
        }
        else if (leg2Pos >= 1.305 && leg3Vel > 0)
        {
            leg2Vel = 0;
            leg2Pos = 1.305;
        }

        if (leg3Pos <= 0.1 && leg3Vel < 0)
        {
            leg3Vel = 0;
            leg3Pos = 0;
        }
        else if (leg3Pos >= 1.305 && leg3Vel > 0)
        {
            leg3Vel = 0;
            leg3Pos = 1.305;
        }

        leg.LegTwo.SetTarPara(leg2Vel, leg2Pos, 0);
        leg.LegThree.SetTarPara(leg3Vel, leg3Pos, 0);

        static int flag = 0;
        
        if (plannerNode->nowPos.y <= 9500)
        {
            tarvelx = PosControl(5075 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 1.2, 0.6, 1.0, 0.2).x;    
        }
        else
        {
            tarvelx = PosControl(5075 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, tarposy, plannerNode->nowPos.y, 2.8, 0.6, 1.0, 0.2).x;
        }

        omega = YawControl(plannerNode->nowPos.yaw, 0, 4.8, 0.8);

        if(fabs(plannerNode->nowPos.x) >= 4675 && plannerNode->nowPos.y <= 9500)
        {
            plannerNode->Vavle_Chassis_Ctrl(1, 0);
            tarvely += 25;
            if(tarvely >= 3150)
            {
                tarvely = 3150;
            } 
        }

        if(plannerNode->nowPos.y >= 9500 && plannerNode->nowPos.y <= 10000)
        {
            tarvely -= 20;
            if(tarvely <= 2400)
            {
                tarvely = 2400;
            }
        }

        if(plannerNode->nowPos.y >= 10000 && plannerNode->nowPos.y <= 10500)
        {
            tarvely -= 20;
            if(tarvely <= 2000)
            {
                tarvely = 2000;
            }
        }

        if(plannerNode->nowPos.y >= 10500 && plannerNode->nowPos.y <= 11150)
        {
            tarvely -= 20;
            if(tarvely <= 800)
            {
                tarvely = 800;
            }
        }

        if (plannerNode->nowPos.y >= 11150)
        {
            step = 2;
        }
    }
    else if (step == 2)
    {
        static bool turn_flag = 0;
        plannerNode->lxData.mode = 1;

        leg2Pos += leg2Vel * 0.005;
        leg3Pos += leg3Vel * 0.005;

        if (leg2Pos <= 0.1 && leg2Vel < 0)
        {
            leg2Vel = 0;
            leg2Pos = 0;
        }
        else if (leg2Pos >= 1.305 && leg3Vel > 0)
        {
            leg2Vel = 0;
            leg2Pos = 1.305;
        }

        if (leg3Pos <= 0.1 && leg3Vel < 0)
        {
            leg3Vel = 0;
            leg3Pos = 0;
        }
        else if (leg3Pos >= 1.305 && leg3Vel > 0)
        {
            leg3Vel = 0;
            leg3Pos = 1.305;
        }

        leg.LegTwo.SetTarPara(leg2Vel, leg2Pos, 0);
        leg.LegThree.SetTarPara(leg3Vel, leg3Pos, 0);

        if (fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) <= 30)
        {
            turn_flag = 1;
            omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 4.8, 0.6);
        }

        if (!turn_flag)
        {
            omega = 280 * plannerNode->myFlags.RB_flag;
            tarvelx = -omega * 460 / 2 / 180 * PI - 1000 * plannerNode->myFlags.RB_flag;
            tarvely = PosControl(0, 0, 11290, plannerNode->nowPos.y, 0, 0, 2.0, 0.6).y - omega * 490 / 2 / 180 * PI * plannerNode->myFlags.RB_flag;
        }
        else
        {
            tarvelx = -2500 * plannerNode->myFlags.RB_flag;
            tarvely = PosControl(0, 0, 11290, plannerNode->nowPos.y, 0, 0, 2.4, 0.4).y;
        }

        static bool stop_flag = 0;
        
        if (fabs(plannerNode->nowPos.x) <= 3500 && !stop_flag)
        {
            stop_flag = 1;
        }
        
        if (stop_flag)
        {
            tarvelx = PosControl(3000 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 0, 0, 2.4, 0.4, 0, 0).x;
        }

        if(fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) <= 3 && fabs(plannerNode->nowPos.x) <= 3600)
        {
            return true;
        }
    }

    plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

    return false;
}

bool UpR1(void)
{
    switch (plannerNode->mystep)
    {
        case all_leg_down:
        {
            leg.SetAllCtrlMode(POS_MODE);

            static int tim_cnt = -1;
            float leg_max_vel = 10.0;
            float leg_a1 = 0.2;
            float leg_a2 = 0.2;

            leg.LegOne.SetTarPosKpKd(3.2, 0.6);
            leg.LegTwo.SetTarPosKpKd(4.0, 0.8);
            leg.LegThree.SetTarPosKpKd(3.6, 0.6);
            leg.LegFour.SetTarPosKpKd(3.2, 0.6);

            tim_cnt++;

            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearUp2, tim_cnt))
            {
                tim_cnt = -1;
                plannerNode->logger()->info("front_leg_up");
                plannerNode->Vavle_Chassis_Ctrl(0, 1);
                plannerNode->mystep = front_leg_up;
            }

            break;
        }
        case front_leg_up:
        {
            static bool Up_leg_flag = false;
            static int tim_cnt = -1;
            float leg_max_vel = 18.0;
            float leg_a1 = 0.1;
            float leg_a2 = 0.1;

            if (plannerNode->Ps_Num[1] && !Up_leg_flag)
            {
                Up_leg_flag = true;
            }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                plannerNode->tarVel.SetVel2(-200 * plannerNode->myFlags.RB_flag, 0, 0);
                
                if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                    leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
                {
                    tim_cnt = -1;
                    Up_leg_flag = false;
                    plannerNode->mystep = hind_leg_up;
                }
            }
            break;
        }
        case hind_leg_up:
        {
            static bool Up_leg_flag = false;
            static bool put_flag = false;
            static int put_cnt = 0;
            static int tim_cnt = -1;
            float leg_max_vel = 18.0;
            float leg_a1 = 0.1;
            float leg_a2 = 0.1;

            if (plannerNode->UpperTask != put_kfs_first)
            {
                plannerNode->tarVel.SetVel2(-600 * plannerNode->myFlags.RB_flag, 0, 0);

                if (plannerNode->Ps_Num[3])
                {
                    plannerNode->tarVel.SetVel2(-300 * plannerNode->myFlags.RB_flag, 0, 0);
                    plannerNode->Vavle_Chassis_Ctrl(1, 0);
                }
                
                if (plannerNode->Ps_Num[7])
                {
                    Up_leg_flag = true;
                    leg.SetAllCtrlMode(POS_MODE);
                    leg.LegOne.SetTarPara(0, 0.01, 0);
                    leg.LegFour.SetTarPara(0, 0.01, 0);
                }
            }
            else
            {
                plannerNode->tarVel.SetVel2(-600 * plannerNode->myFlags.RB_flag, 0, 0);

                if (plannerNode->Ps_Num[3])
                {
                    plannerNode->Vavle_Chassis_Ctrl(1, 0);
                    plannerNode->tarVel.SetVel2(-400 * plannerNode->myFlags.RB_flag, 0, 0);
                }

                if (plannerNode->Ps_Num[7])
                {
                    plannerNode->tarVel.SetVel2(-200 * plannerNode->myFlags.RB_flag, 0, 0);
                }
                
                if (plannerNode->Ps_Num[4])
                {
                    plannerNode->tarVel.SetVel2(-200 * plannerNode->myFlags.RB_flag, 0, 0);

                    if (!put_flag && plannerNode->upperwork != ready_put)
                    {
                        plannerNode->upperwork = put_into;
                        put_flag = 1;
                    }

                    leg.SetAllCtrlMode(POS_MODE);
                    leg.LegOne.SetTarPara(0, 0.01, 0);
                    leg.LegFour.SetTarPara(0, 0.01, 0);
                }   

                if (put_flag)
                {
                    put_cnt++;

                    plannerNode->tarVel.SetVel2(0, 0, 0);
                    
                    if (put_cnt >= 50)
                    {
                        Up_leg_flag = true;
                    }
                }
            }

            if (Up_leg_flag)
            {
                tim_cnt++;
            }

            if (Up_leg_flag)
            {
                plannerNode->tarVel.SetVel2(-150 * plannerNode->myFlags.RB_flag, 0, 0);

                if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt) &
                    leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, 0.0, tim_cnt))
                {
                    tim_cnt = -1;
                    put_flag = 0;
                    Up_leg_flag = false;
                    plannerNode->mystep = nostep;
                }
            }

            break;
        }
        case nostep:
        {
            static int cnt = 0;

            plannerNode->tarVel.SetVel2(-400 * plannerNode->myFlags.RB_flag, 0, 0);

            if (!plannerNode->Ps_Num[7] && plannerNode->Ps_Num[5])
            {
                plannerNode->tarVel.SetVel2(0, 0, 0);
                plannerNode->Vavle_Upper_Ctrl(0, 0, 1, 0);
                static float tarwheel1pos = 0;
                static float tarwheel2pos = 0;
                static float tarwheel3pos = 0;
                static float tarwheel4pos = 0;

                leg.LegOne.SetCtrlMode(DISABLE);
                leg.LegFour.SetCtrlMode(DISABLE);

                cnt++;

                if(cnt >= 10)
                {
                    plannerNode->tarVel.SetVel2(-60 * plannerNode->myFlags.RB_flag, 0, 0);
                    
                    return true;
                }
            }
            break;
        }
    }

    return false;
}

bool DownR1(void)
{
    switch (plannerNode->mystep)
    {

    case hind_leg_down:
    {
        leg.SetAllCtrlMode(POS_MODE);
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 20.0;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;

        plannerNode->Vavle_Chassis_Ctrl(1, 0);

        if (!plannerNode->Ps_Num[5] && !plannerNode->Ps_Num[6])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }
        
        if (Down_leg_flag)
        {
            plannerNode->tarVel.SetVel2(150 * plannerNode->myFlags.RB_flag, 0, 0);
            if (leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt) &
                leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt))
            {
                Down_leg_flag = false;
                tim_cnt = -1;
                plannerNode->mystep = front_leg_down;
            }
        }
        break;
    }

    case front_leg_down:
    {
        static int start_flag = 0;
        static bool Down_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 20.0;
        float leg_a1 = 0.08;
        float leg_a2 = 0.08;

        plannerNode->tarVel.SetVel2(450 * plannerNode->myFlags.RB_flag, 0, 0);

        plannerNode->Vavle_Chassis_Ctrl(0, 1);

        if (!plannerNode->Ps_Num[2] && !plannerNode->Ps_Num[3])
        {
            Down_leg_flag = true;
        }

        if (Down_leg_flag)
        {
            tim_cnt++;
        }

        if (Down_leg_flag)
        {
            plannerNode->tarVel.SetVel2(250 * plannerNode->myFlags.RB_flag, 0, 0);
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt) &
                leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, GearDown2, tim_cnt))
            {
                tim_cnt = -1;
                plannerNode->Vavle_Chassis_Ctrl(1, 0);
                Down_leg_flag = false;
                plannerNode->mystep = all_leg_up;
            }
        }

        break;
    }

    case all_leg_up:
    {
        static bool Up_leg_flag = false;
        static int tim_cnt = -1;
        float leg_max_vel = 8.0;
        float leg_a1 = 0.2;
        float leg_a2 = 0.1;

        if(!plannerNode->Ps_Num[1])
        {
            Up_leg_flag = true;
        }

        if (Up_leg_flag)
        {
            tim_cnt++;
        }

        if(Up_leg_flag)
        {
            if (leg.LegOne.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
            leg.LegTwo.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
            leg.LegThree.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt) &
            leg.LegFour.PlanControl(leg_a1, leg_a2, leg_max_vel, Gear0, tim_cnt))
            {
                tim_cnt = -1;
                Up_leg_flag = false;
                plannerNode->mystep = nostep;
            }
        }

        break;
    }

    case nostep:
    {
        return true;

        break;
    }
    }

    return false;
}

int MoveToGetGroundKfs(void)
{
    plannerNode->lxData.mode = 1;

    static float tar_posx = 0;
    static float tar_posy = 0;
    static float tar_angle = 0;
    static float tar_velx = 0;
    static float tar_vely = 0;
    static float tar_omega = 0;
    static float distance = 0;
    static int step = 0;
    float angle1 = 0;
    float tar_posx1 = 0;
    float tar_posy1 = 0;
    float angle2 = 0;   
    float tar_posx2 = 0;
    float tar_posy2 = 0;
    float distance1 = 0;
    float distance2 = 0;

    static bool trunk_flag = 0;
    if (step != 0 && !trunk_flag)
    {
        if (TrunkTakePiece(2))
        {
            trunk_flag = 1;
        }
    }

    if (step == 0)
    {
        if (plannerNode->tlrkfsData.plane_num == 1)
        { 
            angle1 = plannerNode->nowPos.yaw - plannerNode->tlrkfsData.plane1_angle;

            if (angle1 < 0)
            {
                tar_posx1 = 1000 * cos(-plannerNode->tlrkfsData.plane1_angle * PI / 180) + plannerNode->tlrkfsData.plane1_centroid_x;
                tar_posy1 = 1000 * sin(-plannerNode->tlrkfsData.plane1_angle * PI / 180) + plannerNode->tlrkfsData.plane1_centroid_y;
            }
            else
            {
                tar_posx1 = 1000 * cos(plannerNode->tlrkfsData.plane1_angle * PI / 180) + plannerNode->tlrkfsData.plane1_centroid_x;
                tar_posy1 = plannerNode->tlrkfsData.plane1_centroid_y - 1000 * sin(plannerNode->tlrkfsData.plane1_angle * PI / 180);
            }

            tar_posx = tar_posx1;
            tar_posy = tar_posy1;
            tar_angle = angle1;

            plannerNode->logger()->info("tar_posx_step0_num1: {}", tar_posx);
            plannerNode->logger()->info("tar_posy_step0_num1: {}", tar_posy);
            plannerNode->logger()->info("tar_angle_step0_num1: {}", tar_angle);
            step = 1;
        }
        else if (plannerNode->tlrkfsData.plane_num == 2)
        {   
            angle1 = plannerNode->nowPos.yaw - plannerNode->tlrkfsData.plane1_angle;
            angle2 = plannerNode->nowPos.yaw - plannerNode->tlrkfsData.plane2_angle;

            tar_posx1 = 1000 * cos(plannerNode->tlrkfsData.plane1_angle * PI / 180) + plannerNode->tlrkfsData.plane1_centroid_x;
            tar_posy1 = plannerNode->tlrkfsData.plane1_centroid_y - 1000 * sin(plannerNode->tlrkfsData.plane1_angle * PI / 180);

            tar_posx2 = 1000 * cos(-plannerNode->tlrkfsData.plane2_angle * PI / 180) + plannerNode->tlrkfsData.plane2_centroid_x;
            tar_posy2 = 1000 * sin(-plannerNode->tlrkfsData.plane2_angle * PI / 180) + plannerNode->tlrkfsData.plane2_centroid_y; 

            distance1 = sqrt((tar_posx1 - plannerNode->nowPos.x) * (tar_posx1 - plannerNode->nowPos.x) + (tar_posy1 - plannerNode->nowPos.y) * (tar_posy1 - plannerNode->nowPos.y));
            distance2 = sqrt((tar_posx2 - plannerNode->nowPos.x) * (tar_posx2 - plannerNode->nowPos.x) + (tar_posy2 - plannerNode->nowPos.y) * (tar_posy2 - plannerNode->nowPos.y));

            if (distance1 > distance2)
            {
                tar_posx = tar_posx2;
                tar_posy = tar_posy2;
                tar_angle = angle2;
            }
            else
            {
                tar_posx = tar_posx1;
                tar_posy = tar_posy1;
                tar_angle = angle1;
            }

            if (tar_posy1 > 12000 || tar_posy1 < 9500)
            {
                tar_posx = tar_posx2;
                tar_posy = tar_posy2;
                tar_angle = angle2;
            }
            else if (tar_posy2 > 12000 || tar_posy2 < 9500)
            {
                tar_posx = tar_posx1;
                tar_posy = tar_posy1;
                tar_angle = angle1;
            }


            // if (plannerNode->tlrkfsData.plane1_angle >= 0)
            // {
            //     angle1 = plannerNode->nowPos.yaw - plannerNode->tlrkfsData.plane1_angle;   
            //     tar_posx1 = plannerNode->nowPos.x - cos(angle1) * (cos(angle1) * plannerNode->nowPos.x + sin(angle1) * plannerNode->nowPos.y - (cos(angle1) * plannerNode->tlrkfsData.plane1_centroid_x + sin(angle1) * plannerNode->tlrkfsData.plane1_centroid_y));
            //     tar_posy1 = plannerNode->nowPos.y - sin(angle1) * (cos(angle1) * plannerNode->nowPos.x + sin(angle1) * plannerNode->nowPos.y - (cos(angle1) * plannerNode->tlrkfsData.plane1_centroid_x + sin(angle1) * plannerNode->tlrkfsData.plane1_centroid_y));
                
            //     angle2 = -(180 - plannerNode->nowPos.yaw - fabs(plannerNode->tlrkfsData.plane1_angle));   
            //     tar_posx2 = plannerNode->nowPos.x - cos(angle2) * (cos(angle2) * plannerNode->nowPos.x + sin(angle2) * plannerNode->nowPos.y - (cos(angle2) * plannerNode->tlrkfsData.plane2_centroid_x + sin(angle2) * plannerNode->tlrkfsData.plane2_centroid_y));
            //     tar_posy2 = plannerNode->nowPos.y - sin(angle2) * (cos(angle2) * plannerNode->nowPos.x + sin(angle2) * plannerNode->nowPos.y - (cos(angle2) * plannerNode->tlrkfsData.plane2_centroid_x + sin(angle2) * plannerNode->tlrkfsData.plane2_centroid_y));
                
            //     distance1 = sqrt((tar_posy1 - plannerNode->tlrkfsData.plane1_centroid_y) * (tar_posy1 - plannerNode->tlrkfsData.plane1_centroid_y) + (tar_posx1 - plannerNode->tlrkfsData.plane1_centroid_x) * (tar_posx1 - plannerNode->tlrkfsData.plane1_centroid_x));
            //     distance2 = sqrt((tar_posy2 - plannerNode->tlrkfsData.plane2_centroid_y) * (tar_posy2 - plannerNode->tlrkfsData.plane2_centroid_y) + (tar_posx2 - plannerNode->tlrkfsData.plane2_centroid_x) * (tar_posx2 - plannerNode->tlrkfsData.plane2_centroid_x));

            //     if (distance1 <= 750)
            //     {
            //         distance1 = 750;
            //         tar_posx1 = tar_posx1 + distance1 * cos(angle1);
            //         tar_posy1 = tar_posy1 - distance1 * sin(angle1);
            //     }

            //     if (distance2 <= 750)
            //     {
            //         distance2 = 750;
            //         tar_posx2 = tar_posx2 + distance2 * cos(angle2);
            //         tar_posy2 = tar_posy2 - distance2 * sin(angle2);
            //     }

            //     if (distance1 > distance2)
            //     {
            //         tar_posx = tar_posx2;
            //         tar_posy = tar_posy2;
            //         tar_angle = angle2;
            //     }
            //     else
            //     {
            //         tar_posx = tar_posx1;
            //         tar_posy = tar_posy1;
            //         tar_angle = angle1;
            //     }

            //     if (tar_posy1 > 12000 || tar_posy1 < 9500)
            //     {
            //         tar_posx = tar_posx2;
            //         tar_posy = tar_posy2;
            //         tar_angle = angle2;
            //     }
            //     if (tar_posy2 > 12000 || tar_posy2 < 9500)
            //     {
            //         tar_posx = tar_posx1;
            //         tar_posy = tar_posy1;
            //         tar_angle = angle1;
            //     }
            // }
            // else
            // {
            //     angle2 = plannerNode->nowPos.yaw - plannerNode->tlrkfsData.plane1_angle;   
            //     tar_posx2 = plannerNode->nowPos.x - cos(angle2) * (cos(angle2) * plannerNode->nowPos.x + sin(angle2) * plannerNode->nowPos.y - (cos(angle2) * plannerNode->tlrkfsData.plane2_centroid_x + sin(angle2) * plannerNode->tlrkfsData.plane2_centroid_y));
            //     tar_posy2 = plannerNode->nowPos.y - sin(angle2) * (cos(angle2) * plannerNode->nowPos.x + sin(angle2) * plannerNode->nowPos.y - (cos(angle2) * plannerNode->tlrkfsData.plane2_centroid_x + sin(angle2) * plannerNode->tlrkfsData.plane2_centroid_y));
                
            //     angle1 = -(180 - plannerNode->nowPos.yaw - fabs(plannerNode->tlrkfsData.plane1_angle));   
            //     tar_posx1 = plannerNode->nowPos.x - cos(angle1) * (cos(angle1) * plannerNode->nowPos.x + sin(angle1) * plannerNode->nowPos.y - (cos(angle1) * plannerNode->tlrkfsData.plane1_centroid_x + sin(angle1) * plannerNode->tlrkfsData.plane1_centroid_y));
            //     tar_posy1 = plannerNode->nowPos.y - sin(angle1) * (cos(angle1) * plannerNode->nowPos.x + sin(angle1) * plannerNode->nowPos.y - (cos(angle1) * plannerNode->tlrkfsData.plane1_centroid_x + sin(angle1) * plannerNode->tlrkfsData.plane1_centroid_y));

            //     distance1 = sqrt((tar_posy1 - plannerNode->tlrkfsData.plane1_centroid_y) * (tar_posy1 - plannerNode->tlrkfsData.plane1_centroid_y) + (tar_posx1 - plannerNode->tlrkfsData.plane1_centroid_x) * (tar_posx1 - plannerNode->tlrkfsData.plane1_centroid_x));
            //     distance2 = sqrt((tar_posy2 - plannerNode->tlrkfsData.plane2_centroid_y) * (tar_posy2 - plannerNode->tlrkfsData.plane2_centroid_y) + (tar_posx2 - plannerNode->tlrkfsData.plane2_centroid_x) * (tar_posx2 - plannerNode->tlrkfsData.plane2_centroid_x));

            //     if (distance1 <= 750)
            //     {
            //         distance1 = 750;
            //         tar_posx1 = tar_posx1 + distance1 * cos(angle1);
            //         tar_posy1 = tar_posy1 - distance1 * sin(angle1);
            //     }

            //     if (distance2 <= 750)
            //     {
            //         distance2 = 750;
            //         tar_posx2 = tar_posx2 + distance2 * cos(angle2);
            //         tar_posy2 = tar_posy2 - distance2 * sin(angle2);
            //     }

            //     if (distance1 > distance2)
            //     {
            //         tar_posx = tar_posx2;
            //         tar_posy = tar_posy2;
            //         tar_angle = angle2;
            //     }
            //     else
            //     {
            //         tar_posx = tar_posx1;
            //         tar_posy = tar_posy1;
            //         tar_angle = angle1;
            //     }

            //     if (tar_posy1 > 12000 || tar_posy1 < 9500)
            //     {
            //         tar_posx = tar_posx2;
            //         tar_posy = tar_posy2;
            //         tar_angle = angle2;
            //     }
            //     if (tar_posy2 > 12000 || tar_posy2 < 9500)
            //     {
            //         tar_posx = tar_posx1;
            //         tar_posy = tar_posy1;
            //         tar_angle = angle1;
            //     }
            // }
            
            if ((tar_posy1 > 12000 || tar_posy1 < 9500) && (tar_posy2 > 12000 || tar_posy2 < 9500))
            {
                step = 4;   
            }
            else
            {
                step = 1;
            }

            plannerNode->logger()->info("tar_posx1_num2: {}", tar_posx1);
            plannerNode->logger()->info("tar_posy1_num2: {}", tar_posy1);
            plannerNode->logger()->info("tar_angle1_num2: {}", angle1);

            plannerNode->logger()->info("tar_posx2_num2: {}", tar_posx2);
            plannerNode->logger()->info("tar_posy2_num2: {}", tar_posy2);
            plannerNode->logger()->info("tar_angle2_num2: {}", angle2);

            plannerNode->logger()->info("tar_posx_num2: {}", tar_posx);
            plannerNode->logger()->info("tar_posy_num2: {}", tar_posy);
            plannerNode->logger()->info("tar_angle_num2: {}", tar_angle);
        }
    }
    else if (step == 1)
    {
        plannerNode->logger()->info("tar_posx_step1: {}", tar_posx);
        plannerNode->logger()->info("tar_posy_step1: {}", tar_posy);
        plannerNode->logger()->info("tar_angle_step1: {}", tar_angle);

        tar_velx = PosControl(tar_posx, plannerNode->nowPos.x, tar_posy, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).x;
        tar_vely = PosControl(tar_posx, plannerNode->nowPos.x, tar_posy, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).y;

        tar_omega = YawControl(plannerNode->nowPos.yaw, tar_angle, 2.4, 0.4);

        if (fabs(plannerNode->nowPos.yaw - tar_angle) <= 3)
        {
            plannerNode->tarVel.SetVel2(tar_velx, tar_vely, tar_omega);    
        }
        else
        {
            plannerNode->tarVel.SetVel2(0, 0, tar_omega);
        }

        if (fabs(plannerNode->nowPos.x - tar_posx) <= 50 && fabs(plannerNode->nowPos.y - tar_posy) <= 50 && fabs(plannerNode->nowPos.yaw - tar_angle) <= 3)
        {
            tar_velx = 0;
            tar_vely = 0;
            tar_omega = 0;
            step = 2;
        }
    }
    else if (step == 2)
    {
        tar_omega = YawControl(plannerNode->nowPos.yaw, tar_angle, 2.4, 0.4);

        if (!plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
        {
            plannerNode->tarVel.SetVel1(150, tar_angle + 90, tar_omega);
        }
        else if (!plannerNode->Ps_Num[12] && plannerNode->Ps_Num[11])
        {
            plannerNode->tarVel.SetVel1(150, tar_angle - 90, tar_omega);
        }
        else if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
        {
            plannerNode->tarVel.SetVel1(350, tar_angle, tar_omega);
        }
        else if (plannerNode->Ps_Num[11] && plannerNode->Ps_Num[12])
        {
            plannerNode->tarVel.SetVel1(150, tar_angle, tar_omega);
        }
   
        if (trunk_flag)
        {
            step = 3;
        }
    }
    else if (step == 3)
    {
        plannerNode->tarVel.SetVel2(0, 0, 0);
        if (TrunkReceive(7.2, 0.5, 0.6))
        {
            plannerNode->upperwork = ready_put;
            return 1;
        }
    }

    if (step == 4)
    {
        static bool leg_flag = 0;
        if (leg.LegOne.GetTarPos() != 0)
        {
            leg_flag = 1;
        }

        if (leg_flag)
        {
            if (DownPut())
            {
                leg_flag = 0;
            }
        }

        plannerNode->tarPos.y = 10210;
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

        tar_velx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.5, 0.2, 0.5, 0.2).x;
        tar_vely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.5, 0.2, 0.5, 0.2).y;
        tar_omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 1.5, 0.2);

        plannerNode->tarVel.SetVel2(tar_velx, tar_vely, tar_omega);

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 100 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 100 && !leg_flag)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    }

    return 0;
}

bool YesR1(void)
{
    static int step = 0;
    static float tarvely = 0;
    static float tarvelx = 0;
    static float omega = 0;
    int yes[3] = {0};
    static int my_kfs = 0;
    static int other_kfs = 0;
    static bool align_flag = 0;
    static float last_distance = 0;
    static float col_dist = 0;

    if (plannerNode->myFlags.RB_flag == 1)
    {
        my_kfs = 1;
        other_kfs = 2;
    }
    else if (plannerNode->myFlags.RB_flag == -1)
    {
        my_kfs = 2;
        other_kfs = 1;
    }

    plannerNode->lxData.start_test = 2;

    plannerNode->lxData.lx_posz = leg.LegOne.GetActPos() * 30 * PI;

    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.4, 0.4);

    if (step == 0)
    {
        plannerNode->tarPos.y = 10750;
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 400)
        {
            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.4, 0.2, 0.8, 0.2).x;
        }
        else
        {
            tarvelx = 600 * plannerNode->myFlags.RB_flag;
        }
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.4, 0.2, 0.8, 0.2).y;

        if (plannerNode->lxData.start == 1)
        {
            tarvelx = 0;
            tarvely = 0;
            omega = 0;
            step = 1;
        }
    }
    else
    {
        if (plannerNode->myFlags.RB_flag == 1 && plannerNode->R1StarData.r1movedirection == -1 || plannerNode->myFlags.RB_flag == -1 && plannerNode->R1StarData.r1movedirection == 1)
        {
            yes[0] = 0;

            if (plannerNode->lxData.grid[4] == 0)
            {
                yes[1] = 1;

                if (yes[1] == 1)
                {
                    if (   (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == my_kfs)
                        || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == my_kfs)
                        || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == my_kfs))
                    {
                        yes[1] = 4;
                    }
                    else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == other_kfs)
                        || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == other_kfs)
                        || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == other_kfs))
                    {
                        yes[1] = 3;
                    }
                    else if ((plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == 0) 
                        || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == 0)
                        || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == 0))
                    {
                        yes[1] = 2;
                    }
                    else if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == my_kfs)
                        || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == my_kfs)
                        || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == my_kfs))
                    {
                        yes[1] = 1;
                    }
                }

                if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == my_kfs) || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == my_kfs) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == my_kfs))
                {
                    yes[1] = 6;
                }
                else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == other_kfs))
                {
                    yes[1] = 5;
                }
            }
            else if (plannerNode->lxData.grid[4] != 0)
            {
                yes[1] = 0;
            }

            if (plannerNode->lxData.grid[5] == 0)
            {
                yes[2] = 1;

                if(yes[2] == 1)
                {
                    if ((plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == my_kfs))
                    {
                        yes[2] = 4;
                    }
                    else if ((plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[2] == 2 && plannerNode->lxData.grid[8] == 0))
                    {
                        yes[2] = 3;
                    }
                    else if (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == 0)
                    {
                        yes[2] = 2;
                    }
                    else if ((plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[8] == my_kfs) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == other_kfs))
                    {
                        yes[2] = 1;
                    }
                }

                if (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == my_kfs)
                {
                    yes[2] = 7;
                }
                else if (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[8] == other_kfs)
                {
                    yes[2] = 5;
                }
            }
            else if (plannerNode->lxData.grid[5] != 0)
            {
                yes[2] = 0;
            }
        }
        else if (plannerNode->myFlags.RB_flag == 1 && plannerNode->R1StarData.r1movedirection == 1 || plannerNode->myFlags.RB_flag == -1 && plannerNode->R1StarData.r1movedirection == -1)
        {
            if (plannerNode->lxData.grid[3] == 0)
            {
                yes[0] = 1;

                if (yes[0] == 1)
                {
                    if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == my_kfs))
                    {
                        yes[0] = 4;
                    }
                    else if ((plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == 0))
                    {
                        yes[0] = 3;
                    }
                    else if (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == 0)
                    {
                        yes[0] = 2;
                    }
                    else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == my_kfs) || (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == other_kfs))
                    {
                        yes[0] = 1;
                    }
                }

                if (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == my_kfs)
                {
                    yes[0] = 6;
                }
                else if (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == other_kfs)
                {
                    yes[0] = 5;
                }
            }
            else if (plannerNode->lxData.grid[3] != 0)
            {
                yes[0] = 0;
            }

            if (plannerNode->lxData.grid[4] == 0)
            {
                yes[1] = 1;

                if (yes[1] == 1)
                {
                    if (   (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == my_kfs)
                        || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == my_kfs)
                        || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == my_kfs))
                    {
                        yes[1] = 4;
                    }
                    else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == other_kfs)
                        || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == other_kfs)
                        || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == other_kfs))
                    {
                        yes[1] = 3;
                    }
                    else if ((plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == 0) 
                        || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == 0)
                        || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == 0))
                    {
                        yes[1] = 2;
                    }
                    else if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == my_kfs)
                        || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == my_kfs)
                        || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == my_kfs))
                    {
                        yes[1] = 1;
                    }
                }

                if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == my_kfs) || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == my_kfs) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == my_kfs))
                {
                    yes[1] = 6;
                }
                else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == other_kfs))
                {
                    yes[1] = 5;
                }
            }
            else if (plannerNode->lxData.grid[4] != 0)
            {
                yes[1] = 0;
            }

            yes[2] = 0;
        }

        int max_yes = 0;

        for (int i = 1; i <= 2; i++) 
        {
            if (yes[i] > yes[max_yes]) 
            {
                max_yes = i;
            }
        }

        if (max_yes == 0)
        {
            plannerNode->myFlags.col_flag = 1;
        }
        else if (max_yes == 1)
        {
            plannerNode->myFlags.col_flag = 2;
        }
        else if (max_yes == 2)
        {
            plannerNode->myFlags.col_flag = 3;
        }

        if (step == 1)
        {
            if (plannerNode->myFlags.col_flag == 1)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }

            tarvelx = 0;
            tarvely = 0;
            omega = 0;

            step = 2;
        }
        else if (step == 2)
        {
            if (!align_flag)
            {
                if (plannerNode->myFlags.col_flag == 1)
                {
                    col_dist = plannerNode->lxData.col_first;
                    plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (plannerNode->myFlags.RB_flag == 1)
                    {
                        if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                        {
                            plannerNode->tarPos.y = 10210; 
                        }
                    }
                    else if (plannerNode->myFlags.RB_flag == -1)
                    {
                        if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                        {
                            plannerNode->tarPos.y = 11290;
                        }
                    }
                }
                else if (plannerNode->myFlags.col_flag == 2)
                {
                    col_dist = plannerNode->lxData.col_second;
                    plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (fabs(plannerNode->tarPos.y - 10750) >= 500)
                    {
                        plannerNode->tarPos.y = 10750;
                    }
                }
                else if (plannerNode->myFlags.col_flag == 3)
                {
                    col_dist = plannerNode->lxData.col_third;
                    plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (plannerNode->myFlags.RB_flag == 1)
                    {
                        if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                        {
                            plannerNode->tarPos.y = 11290;
                        }
                    }
                    else if (plannerNode->myFlags.RB_flag == -1)
                    {
                        if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                        {
                            plannerNode->tarPos.y = 10210; 
                        }
                    }
                }

                if (fabs(plannerNode->nowPos.x) < 1500)
                {
                    if (plannerNode->lxData.lx_posx != 0)
                    {
                        plannerNode->tarPos.x = plannerNode->nowPos.x - plannerNode->lxData.lx_posx * plannerNode->myFlags.RB_flag + 920 * plannerNode->myFlags.RB_flag;
                    } 
                }
                else
                {
                    plannerNode->tarPos.x = 920 * plannerNode->myFlags.RB_flag;
                }

                if (fabs(plannerNode->tarPos.x) < 600)
                {
                    plannerNode->tarPos.x = 600 * plannerNode->myFlags.RB_flag;
                }
                else if (fabs(plannerNode->tarPos.x) > 1200)
                {
                    plannerNode->tarPos.x = 1200 * plannerNode->myFlags.RB_flag;
                }

                last_distance = plannerNode->lxData.col_first;
                align_flag = 1;
            }

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 400)
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.8, 0.4, 1.5, 0.2).x;
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.5, 0.2).x;
            }

            if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 400)
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.8, 0.8, 1.6, 0.6).y;
            }
            else
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.8, 0.8, 2.6, 0.6).y;
            }

            static float real_vely = 0;
            real_vely = (-wheel.WheelOne.GetActVel() + wheel.WheelTwo.GetActVel() - wheel.WheelThree.GetActVel() + wheel.WheelFour.GetActVel()) * PI * 101.6 / 4.0; 

            static bool col_flag = 0;
            static int col_cnt = 0;

            if (fabs(col_dist) <= 25)
            {
                col_cnt++;

                if (col_cnt >= 9)
                {
                    col_flag = 1;
                }
            }
            else
            {
                col_cnt = 0;
                col_flag = 0;
            }

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 50 && col_flag && fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) < 3 
                && fabs(real_vely) < 30)
            {
                if (plannerNode->upperwork != ready_put)
                {
                    step = 3;
                }
            }
        }
        else if (step == 3)
        {
            static bool put_flag = 0;
            if ((plannerNode->myFlags.col_flag == 1 && plannerNode->lxData.grid[3] == 0) || (plannerNode->myFlags.col_flag == 2 && plannerNode->lxData.grid[4] == 0) || (plannerNode->myFlags.col_flag == 3 && plannerNode->lxData.grid[5] == 0))
            {
                if (!put_flag)
                {
                    plannerNode->upperwork = put_into;
                    put_flag = 1;
                }
            }
            else
            {
                if (!put_flag)
                {
                    plannerNode->tarPos.x = 2500 * plannerNode->myFlags.RB_flag;
                    if (fabs(fabs(plannerNode->nowPos.x) - 2500) <= 150)
                    {
                        step = 1;
                    }   
                }
            }

            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 0.8, 0.2).x;
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 0.8, 0.2).y;

            if (fabs(trunk.Shoulder.GetActPos() < 4.8) && !plannerNode->Vavle_Num[9])
            {
                step = 4;
            }
        }
        else if (step == 4)
        {
            static bool down_flag = 0;
            if (!down_flag)
            {
                if (DownPut())
                {
                    down_flag = 1;
                }
            }

            plannerNode->tarPos.y = 10750;
            plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

            tarvelx = -500 * plannerNode->myFlags.RB_flag;
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).y;

            if (fabs(plannerNode->nowPos.x) >= fabs(plannerNode->tarPos.x))
            {
                tarvelx = 0;
            }

            if (fabs(plannerNode->nowPos.x) >= fabs(plannerNode->tarPos.x) && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 200 && down_flag)
            {
                plannerNode->tarVel.SetVel2(0, 0, 0);
                step = 0;
                return true;
            }
        }
        plannerNode->logger()->info("max_yes: {}", max_yes);
        plannerNode->logger()->info("yes0: {}", yes[0]);
        plannerNode->logger()->info("yes1: {}", yes[1]);
        plannerNode->logger()->info("yes2: {}", yes[2]);
    }

    if (step != 4)
    {
        if (yes[0] == 0 && yes[1] == 0 && yes[2] == 0)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    }

    if (plannerNode->lxData.col_first != last_distance)
    {
        align_flag = 0;
    }

    static bool leg_flag = 0;
    
    if (!leg_flag)
    {
        if (UpPut(GearDown1))
        {
            leg_flag = 1;
        }
    }

    plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

    return false;
}

bool NoR1(void)
{
    static int step = 0;
    static float tarvelx = 0;
    static float tarvely = 0;
    static float omega = 0;
    static int col = 0;
    static int flag = 0;
    static int yes[3] = {0};
    static bool align_flag = 0;
    static float last_distance = 0;
    static float col_dist = 0;
    static int yes_RB = 0;

    plannerNode->lxData.start_test = 2;

    plannerNode->lxData.lx_posz = leg.LegOne.GetActPos() * 30 * PI;

    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 1.5, 0.4);

    if (fabs(plannerNode->nowPos.x) > 1500)
    {
        if (plannerNode->lxData.grid[3] == 0)
        {
            yes[0] = 1;
            
            if (plannerNode->lxData.grid[0] == 0)
            {
                if (plannerNode->lxData.grid[6] != 0)
                {
                    yes[0] = 1;
                }
                else if (plannerNode->lxData.grid[6] == 0)
                {
                    yes[0] = 2;
                }
            }
            else if (plannerNode->lxData.grid[0] != 0)
            {
                if (plannerNode->lxData.grid[6] != 0)
                {
                    yes[0] = 3;
                }
                else if (plannerNode->lxData.grid[6] == 0)
                {
                    yes[0] = 1;
                }
            }
        }
        else if (plannerNode->lxData.grid[3] != 0)
        {
            yes[0] = 0;
        }

        if (plannerNode->lxData.grid[4] == 0)
        {
            yes[1] = 1;

            if (plannerNode->lxData.grid[0] != 0)
            {
                if (plannerNode->lxData.grid[8] != 0)
                {
                    yes[1] = 3;
                }
                else if (plannerNode->lxData.grid[8] == 0)
                {
                    flag++;
                }
            }
            else if (plannerNode->lxData.grid[0] == 0)
            {
                if (plannerNode->lxData.grid[8] != 0)
                {
                    flag++;
                }
            }

            if (plannerNode->lxData.grid[1] != 0)
            {
                if (plannerNode->lxData.grid[7] != 0)
                {
                    yes[1] = 3;
                }
                else if (plannerNode->lxData.grid[7] == 0)
                {
                    flag++;
                }
            }
            else if (plannerNode->lxData.grid[1] == 0)
            {
                if (plannerNode->lxData.grid[7] != 0)
                {
                    flag++;
                }
            }

            if (plannerNode->lxData.grid[2] != 0)
            {
                if (plannerNode->lxData.grid[6] != 0)
                {
                    yes[1] = 3;
                }
                else if (plannerNode->lxData.grid[6] == 0)
                {
                    flag++;
                }
            }
            else if (plannerNode->lxData.grid[2] == 0)
            {
                if (plannerNode->lxData.grid[6] != 0)
                {
                    flag++;
                }
            }

            if (flag == 3)
            {
                yes[1] = 1;
            }
            else
            {
                yes[1] = 3;
            }
            flag = 0;
        }
        else if (plannerNode->lxData.grid[4] != 0)
        {
            yes[1] = 0;
        }

        if (plannerNode->lxData.grid[5] == 0)
        {
            yes[2] = 1;
            if (plannerNode->lxData.grid[2] != 0)
            {
                if (plannerNode->lxData.grid[8] != 0)
                {
                    yes[2] = 3;
                }
                else if (plannerNode->lxData.grid[8] == 0)
                {
                    yes[2] = 1;
                }
            }
            else if (plannerNode->lxData.grid[2] == 0)
            {
                if (plannerNode->lxData.grid[8] != 0)
                {
                    yes[2] = 1;
                }
                else if (plannerNode->lxData.grid[8] == 0)
                {
                    yes[2] = 2;
                }
            }
        }
        else if (plannerNode->lxData.grid[5] != 0)
        {
            yes[2] = 0;
        }
    }

    int max_yes = 0;
    for (int i = 1; i <= 2; i++) 
    {
        if (yes[i] > yes[max_yes]) 
        {
            max_yes = i;
        }
    }

    if (max_yes == 0)
    {
        plannerNode->myFlags.col_flag = 1;
    }
    else if (max_yes == 1)
    {
        plannerNode->myFlags.col_flag = 2;
    }
    else if (max_yes == 2)
    {
        plannerNode->myFlags.col_flag = 3;
    }

    col = plannerNode->myFlags.col_flag;

    if (yes[0] == 0 && yes[1] == 0 && yes[2] == 0)
    {
        step = 3;
    }

    plannerNode->logger()->info("max_yes:{}", max_yes);
    plannerNode->logger()->info("yes0:{}", yes[0]);
    plannerNode->logger()->info("yes1:{}", yes[1]);
    plannerNode->logger()->info("yes2:{}", yes[2]);
    plannerNode->logger()->info("col:{}", col);

    if (step == 0)
    {
        if (!align_flag)
        {
            if (plannerNode->myFlags.col_flag == 1)
            {
                col_dist = plannerNode->lxData.col_first;
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                col_dist = plannerNode->lxData.col_second;
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (fabs(plannerNode->tarPos.y - 10750) >= 500)
                {
                    plannerNode->tarPos.y = 10750;
                }
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                col_dist = plannerNode->lxData.col_third;
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
            }

            if (fabs(plannerNode->nowPos.x) < 1500)
            {
                if (plannerNode->lxData.lx_posx != 0)
                {
                    plannerNode->tarPos.x = plannerNode->nowPos.x - plannerNode->lxData.lx_posx * plannerNode->myFlags.RB_flag + 900 * plannerNode->myFlags.RB_flag;
                } 
            }
            else
            {
                plannerNode->tarPos.x = 900 * plannerNode->myFlags.RB_flag;
            }

            if (fabs(plannerNode->tarPos.x) < 600)
            {
                plannerNode->tarPos.x = 600 * plannerNode->myFlags.RB_flag;
            }
            else if (fabs(plannerNode->tarPos.x) > 1200)
            {
                plannerNode->tarPos.x = 1200 * plannerNode->myFlags.RB_flag;
            }
            
            last_distance = plannerNode->lxData.col_first;
            align_flag = 1;
        }

        static bool leg_flag = 0;
        if (!leg_flag)
        {
            if (UpPut(Gear5))
            {
                leg_flag = 1;
            }
        }

        static bool ready_flag = 0;
        if (!ready_flag)
        {
            plannerNode->UpperTask = put_kfs_first;
            plannerNode->upperwork = ready_put;
            ready_flag = 1;
        }   

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 600)
        {
            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.4, 1.5, 0.2).x;
        }
        else
        {
            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.6, 0.2, 1.5, 0.2).x;
        }

        if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 400)
        {
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.8, 0.4, 2.5, 0.4).y;
        }
        else
        {
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.8, 0.4, 1.6, 0.4).y;
        }

        static float real_vely = 0;
        real_vely = (-wheel.WheelOne.GetActVel() + wheel.WheelTwo.GetActVel() - wheel.WheelThree.GetActVel() + wheel.WheelFour.GetActVel()) * PI * 101.6 / 4.0; 

        static bool col_flag = 0;
        static int col_cnt = 0;

        if (fabs(col_dist) <= 25)
        {
            col_cnt++;
            if (col_cnt >= 9)
            {
                col_flag = 1;
            }
        }
        else
        {
            col_cnt = 0;
            col_flag = 0;
        }

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 50 && col_flag && fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) < 3 && leg_flag && ready_flag
            && fabs(real_vely) <= 50)
        {
            if (plannerNode->upperwork != ready_put)
            {
                // plannerNode->upperwork = put_into;
                col_flag = 0;
                step = 1;
            }
        }
    }
    else if (step == 1)
    {
        static bool put_flag = 0;
        if ((col == 1 && plannerNode->lxData.grid[3] == 0) || (col == 2 && plannerNode->lxData.grid[4] == 0) || (col == 3 && plannerNode->lxData.grid[5] == 0))
        {
            if (!put_flag)
            {
                plannerNode->upperwork = put_into;
                put_flag = 1;
            }
        }
        else
        {
            if (!put_flag)
            {
                plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;
                if (fabs(fabs(plannerNode->nowPos.x) - 3000) <= 150)
                {
                    step = 0;
                }
            }   
        }

        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.0, 0.2, 0.8, 0.2).x;
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.0, 0.2, 0.8, 0.2).y;

        if (fabs(trunk.Shoulder.GetActPos() < 4.8) && !plannerNode->Vavle_Num[9])
        {
            step = 2;
        }
    }
    else if (step == 2)
    {
        static bool down_flag = 0;
        if (!down_flag)
        {
            if (DownPut())
            {
                down_flag = 1;
            }
        }

        plannerNode->tarPos.y = 10210;
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 0.8, 0.2).x;
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 0.8, 0.2).y;

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 100 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 100 && down_flag)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
            return true;
        }
    }

    if (step == 3)
    {
        static bool leg_flag = 0;
        if (leg.LegOne.GetTarPos() != 0)
        {
            leg_flag = 1;
        }

        if (leg_flag)
        {
            if (DownPut())
            {
                leg_flag = 0;
            }
        }

        plannerNode->tarPos.y = 10210;
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.5, 0.2, 0.5, 0.2).x;
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.5, 0.2, 0.5, 0.2).y;

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 100 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 100 && !leg_flag)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
            return true;
        }
    }

    if (plannerNode->lxData.col_first != last_distance)
    {
        align_flag = 0;
    }

    plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

    return false;
}

bool GoToPutSecond(void)
{
    static int step = 0;
    static float tarvely = 0;
    static float tarvelx = 0;
    static float omega = 0;
    int yes[3] = {0};
    static int my_kfs = 0;
    static int other_kfs = 0;
    static bool align_flag = 0;
    static float last_distance = 0;

    if (plannerNode->myFlags.RB_flag == 1)
    {
        my_kfs = 1;
        other_kfs = 2;
    }
    else if (plannerNode->myFlags.RB_flag == -1)
    {
        my_kfs = 2;
        other_kfs = 1;
    }

    plannerNode->lxData.start_test = 2;

    plannerNode->lxData.lx_posz = leg.LegOne.GetActPos() * 30 * PI;

    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 2.4, 0.4);

    if (step == 0)
    {
        plannerNode->tarPos.y = 10750;
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 400)
        {
            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.4, 0.2, 0.8, 0.2).x;
        }
        else
        {
            tarvelx = 600 * plannerNode->myFlags.RB_flag;
        }
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.4, 0.2, 0.8, 0.2).y;

        if (plannerNode->lxData.start == 1)
        {
            tarvelx = 0;
            tarvely = 0;
            omega = 0;
            step = 1;
        }
    }
    else
    {
        if (plannerNode->lxData.grid[3] == 0)
        {
            yes[0] = 1;

            if (yes[0] == 1)
            {
                if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == my_kfs))
                {
                    yes[0] = 4;
                }
                else if ((plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == 0))
                {
                    yes[0] = 3;
                }
                else if (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[6] == 0)
                {
                    yes[0] = 2;
                }
                else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == my_kfs) || (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == other_kfs))
                {
                    yes[0] = 1;
                }
            }

            if (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[6] == my_kfs)
            {
                yes[0] = 6;
            }
            else if (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[6] == other_kfs)
            {
                yes[0] = 5;
            }
        }
        else if (plannerNode->lxData.grid[3] != 0)
        {
            yes[0] = 0;
        }

        if (plannerNode->lxData.grid[4] == 0)
        {
            yes[1] = 1;

            if (yes[1] == 1)
            {
                if (   (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == my_kfs)
                    || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == my_kfs)
                    || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == my_kfs))
                {
                    yes[1] = 4;
                }
                else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == other_kfs)
                    || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == 0) || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == other_kfs)
                    || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == other_kfs))
                {
                    yes[1] = 3;
                }
                else if ((plannerNode->lxData.grid[0] == 0 && plannerNode->lxData.grid[8] == 0) 
                    || (plannerNode->lxData.grid[1] == 0 && plannerNode->lxData.grid[7] == 0)
                    || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[6] == 0))
                {
                    yes[1] = 2;
                }
                else if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == my_kfs)
                    || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == my_kfs)
                    || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == my_kfs))
                {
                    yes[1] = 1;
                }
            }

            if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[8] == my_kfs) || (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[7] == my_kfs) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[6] == my_kfs))
            {
                yes[1] = 6;
            }
            else if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[7] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[6] == other_kfs))
            {
                yes[1] = 5;
            }
        }
        else if (plannerNode->lxData.grid[4] != 0)
        {
            yes[1] = 0;
        }

        if (plannerNode->lxData.grid[5] == 0)
        {
            yes[2] = 1;

            if(yes[2] == 1)
            {
                if ((plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == 0) || (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == my_kfs))
                {
                    yes[2] = 4;
                }
                else if ((plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == other_kfs) || (plannerNode->lxData.grid[2] == 2 && plannerNode->lxData.grid[8] == 0))
                {
                    yes[2] = 3;
                }
                else if (plannerNode->lxData.grid[2] == 0 && plannerNode->lxData.grid[8] == 0)
                {
                    yes[2] = 2;
                }
                else if ((plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[8] == my_kfs) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == other_kfs))
                {
                    yes[2] = 1;
                }
            }

            if (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[8] == my_kfs)
            {
                yes[2] = 6;
            }
            else if (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[8] == other_kfs)
            {
                yes[2] = 5;
            }
        }
        else if (plannerNode->lxData.grid[5] != 0)
        {
            yes[2] = 0;
        }

        int max_yes = 0;
        for (int i = 1; i <= 2; i++) 
        {
            if (yes[i] > yes[max_yes]) 
            {
                max_yes = i;
            }
        }

        if (max_yes == 0)
        {
            // if (yes[0] == yes[1])
            // {
            //     if (fabs(plannerNode->lxData.col_first) < fabs(plannerNode->lxData.col_second))
            //     {
                    plannerNode->myFlags.col_flag = 1;
            //     }
            //     else
            //     {
            //         plannerNode->myFlags.col_flag = 2;
            //     }
            // }
            // else
            // {
            //     plannerNode->myFlags.col_flag = 1;
            // }
        }
        else if (max_yes == 1)
        {
            plannerNode->myFlags.col_flag = 2;
        }
        else if (max_yes == 2)
        {
            // plannerNode->myFlags.col_flag = 3;
            // if (yes[2] == yes[1])
            // {
            //     if (fabs(plannerNode->lxData.col_third) < fabs(plannerNode->lxData.col_second))
            //     {
                    plannerNode->myFlags.col_flag = 3;
            //     }
            //     else
            //     {
            //         plannerNode->myFlags.col_flag = 2;
            //     }
            // }
            // else
            // {
            //     plannerNode->myFlags.col_flag = 3;
            // }
        }

        if (step == 1)
        {
            if (plannerNode->myFlags.col_flag == 1)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }

            tarvelx = 0;
            tarvely = 0;
            omega = 0;

            // if (plannerNode->R1StarData.R2PutFlag)
            // {
            //     step = 2;
            // }
            // else if (plannerNode->tlrData.r1_is_putting && yes[max_yes] == 5)
            // {
            //     step = 2;
            // }
            step = 2;
        }
        else if (step == 2)
        {
            if (!align_flag)
            {
                if (plannerNode->myFlags.col_flag == 1)
                {
                    plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (plannerNode->myFlags.RB_flag == 1)
                    {
                        if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                        {
                            plannerNode->tarPos.y = 10210; 
                        }
                    }
                    else if (plannerNode->myFlags.RB_flag == -1)
                    {
                        if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                        {
                            plannerNode->tarPos.y = 11290;
                        }
                    }
                }
                else if (plannerNode->myFlags.col_flag == 2)
                {
                    plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (fabs(plannerNode->tarPos.y - 10750) >= 500)
                    {
                        plannerNode->tarPos.y = 10750;
                    }
                }
                else if (plannerNode->myFlags.col_flag == 3)
                {
                    plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                    if (plannerNode->myFlags.RB_flag == 1)
                    {
                        if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                        {
                            plannerNode->tarPos.y = 11290;
                        }
                    }
                    else if (plannerNode->myFlags.RB_flag == -1)
                    {
                        if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                        {
                            plannerNode->tarPos.y = 10210; 
                        }
                    }
                }

                if (fabs(plannerNode->nowPos.x) < 1500)
                {
                    if (plannerNode->lxData.lx_posx != 0)
                    {
                        plannerNode->tarPos.x = plannerNode->nowPos.x - plannerNode->lxData.lx_posx * plannerNode->myFlags.RB_flag + 920 * plannerNode->myFlags.RB_flag;
                    } 
                }
                else
                {
                    plannerNode->tarPos.x = 920 * plannerNode->myFlags.RB_flag;
                }

                if (fabs(plannerNode->tarPos.x) < 600)
                {
                    plannerNode->tarPos.x = 600 * plannerNode->myFlags.RB_flag;
                }
                else if (fabs(plannerNode->tarPos.x) > 1200)
                {
                    plannerNode->tarPos.x = 1200 * plannerNode->myFlags.RB_flag;
                }

                last_distance = plannerNode->lxData.col_first;
                align_flag = 1;
            }

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 400)
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.8, 0.4, 1.5, 0.2).x;
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.5, 0.2).x;
            }

            if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 400)
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.8, 0.8, 1.6, 0.6).y;
            }
            else
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 2.8, 0.8, 2.6, 0.6).y;
            }

            static float real_vely = 0;
            real_vely = (-wheel.WheelOne.GetActVel() + wheel.WheelTwo.GetActVel() - wheel.WheelThree.GetActVel() + wheel.WheelFour.GetActVel()) * PI * 101.6 / 4.0; 

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 50 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 30 && fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) < 3 
                && fabs(real_vely) < 30)
            {
                if (plannerNode->upperwork != ready_put)
                {
                    step = 3;
                }
            }
        }
        else if (step == 3)
        {
            static bool put_flag = 0;
            if ((plannerNode->myFlags.col_flag == 1 && plannerNode->lxData.grid[3] == 0) || (plannerNode->myFlags.col_flag == 2 && plannerNode->lxData.grid[4] == 0) || (plannerNode->myFlags.col_flag == 3 && plannerNode->lxData.grid[5] == 0))
            {
                if (!put_flag)
                {
                    plannerNode->upperwork = put_into;
                    put_flag = 1;
                }
            }
            else
            {
                if (!put_flag)
                {
                    plannerNode->tarPos.x = 2500 * plannerNode->myFlags.RB_flag;
                    if (fabs(fabs(plannerNode->nowPos.x) - 2500) <= 150)
                    {
                        step = 1;
                    }   
                }
            }

            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 0.8, 0.2).x;
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.2, 0.8, 0.2).y;

            if (fabs(trunk.Shoulder.GetActPos() < 4.8) && !plannerNode->Vavle_Num[9])
            {
                step = 4;
            }
        }
        else if (step == 4)
        {
            static bool down_flag = 0;
            if (!down_flag)
            {
                if (DownPut())
                {
                    down_flag = 1;
                }
            }

            plannerNode->tarPos.y = 10750;
            plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;

            tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).x;
            tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).y;

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 100 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 100 && down_flag)
            {
                plannerNode->tarVel.SetVel2(0, 0, 0);
                step = 0;
                return true;
            }
        }
        plannerNode->logger()->info("max_yes: {}", max_yes);
        plannerNode->logger()->info("yes0: {}", yes[0]);
        plannerNode->logger()->info("yes1: {}", yes[1]);
        plannerNode->logger()->info("yes2: {}", yes[2]);
    }

    if (step != 4)
    {
        if (yes[0] == 0 && yes[1] == 0 && yes[2] == 0)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    }

    if (plannerNode->lxData.col_first != last_distance)
    {
        align_flag = 0;
    }

    static bool leg_flag = 0;
    
    if (!leg_flag)
    {
        if (UpPut(GearDown1))
        {
            leg_flag = 1;
        }
    }

    plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

    return false;
}

bool MoveToGetR1KFS(void)
{
    static int step = 0;
    static int my_kfs = 0;
    static int other_kfs = 0;
    static float tarvelx = 0;
    static float tarvely = 0;
    static float omega = 0;
    int yes[3] = {0};
    int max_yes = 0;

    plannerNode->lxData.start_test = 2;

    plannerNode->lxData.lx_posz = leg.LegOne.GetActPos() * 30 * PI;

    if (plannerNode->myFlags.RB_flag == 1)
    {
        my_kfs = 1;
        other_kfs = 2;
    }
    else if (plannerNode->myFlags.RB_flag == -1)
    {
        my_kfs = 2;
        other_kfs = 1;
    }

    omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 1.5, 0.4);

    if (step == 0)
    {
        plannerNode->tarPos.x = 3000 * plannerNode->myFlags.RB_flag;
        plannerNode->tarPos.y = 10750;

        tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 0.8, 0.2).x;
        tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 0.8, 0.2).y;

        if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) < 100 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) < 100)
        {
            step = 1;
        }
    }
    else
    {
        if (plannerNode->lxData.grid[6] == my_kfs)
        {
            if (plannerNode->lxData.grid[7] == my_kfs)
            {
                if (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[4] == 0)
                {
                    yes[0] = 1;
                }
            }
            else if (plannerNode->lxData.grid[8] == my_kfs)
            {
                if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[4] == 0) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[5] == 0))
                {
                    yes[0] = 1;
                }
            }

            if ((plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[3] == other_kfs) || (plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[4] == other_kfs))
            {
                yes[0] = 0;
            }
        }

        if (plannerNode->lxData.grid[7] == my_kfs)
        {
            if (plannerNode->lxData.grid[6] == my_kfs)
            {
                if (plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[3] == 0)
                {
                    yes[1] = 1;
                }
            }
            else if (plannerNode->lxData.grid[8] == my_kfs)
            {
                if (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[5] == 0)
                {
                    yes[1] = 1;
                }
            }

            if (plannerNode->lxData.grid[1] == other_kfs && plannerNode->lxData.grid[4] == other_kfs)
            {
                yes[1] = 0;    
            }
        }

        if (plannerNode->lxData.grid[8] == my_kfs)
        {
            if (plannerNode->lxData.grid[7] == my_kfs)
            {
                if (plannerNode->lxData.grid[1] == my_kfs && plannerNode->lxData.grid[4] == 0)
                {
                    yes[2] = 1;
                }
            }
            else if (plannerNode->lxData.grid[6] == my_kfs)
            {
                if ((plannerNode->lxData.grid[0] == my_kfs && plannerNode->lxData.grid[3] == 0) || (plannerNode->lxData.grid[2] == my_kfs && plannerNode->lxData.grid[4] == 0))
                {
                    yes[2] = 1;
                }
            }

            if ((plannerNode->lxData.grid[2] == other_kfs && plannerNode->lxData.grid[5] == other_kfs) || (plannerNode->lxData.grid[0] == other_kfs && plannerNode->lxData.grid[4] == other_kfs))
            {
                yes[2] = 0;
            }
        }

        for (int i = 1; i <= 2; i++) 
        {
            if (yes[i] > yes[max_yes]) 
            {
                max_yes = i;
            }
        }

        if (max_yes == 0)
        {
            plannerNode->myFlags.col_flag = 1;
        }
        else if (max_yes == 1)
        {
            plannerNode->myFlags.col_flag = 2;
        }
        else if (max_yes == 2)
        {
            plannerNode->myFlags.col_flag = 3;
        }

        plannerNode->logger()->info("max_yes:{}", max_yes);
        plannerNode->logger()->info("yes0:{}", yes[0]);
        plannerNode->logger()->info("yes1:{}", yes[1]);
        plannerNode->logger()->info("yes2:{}", yes[2]);

        if (step == 1)
        {
            if (plannerNode->myFlags.col_flag == 1)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
            }

            tarvelx = 0;
            tarvely = 0;
            omega = 0;

            if (fabs(plannerNode->tarPos.y - plannerNode->R1StarData.r1posy) > 800)
            {
                step = 2;
            }
        }
        else if(step == 2)
        {
            if (plannerNode->myFlags.col_flag == 1)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (fabs(plannerNode->tarPos.y - 10750) >= 500)
                {
                    plannerNode->tarPos.y = 10750;
                }
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
            }

            plannerNode->tarPos.x = 1200 * plannerNode->myFlags.RB_flag;

            if (fabs(plannerNode->nowPos.x) < 1500)
            {
                plannerNode->tarPos.x = plannerNode->nowPos.x - plannerNode->lxData.lx_posx * plannerNode->myFlags.RB_flag + 1200 * plannerNode->myFlags.RB_flag; 
            }

            if (fabs(plannerNode->tarPos.x) < 600)
            {
                plannerNode->tarPos.x = 600 * plannerNode->myFlags.RB_flag;
            }
            else if (fabs(plannerNode->tarPos.x) > 1500)
            {
                plannerNode->tarPos.x = 1500 * plannerNode->myFlags.RB_flag;
            }

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 400)
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.5, 0.4, 1.5, 0.2).x;
            }
            else
            {
                tarvelx = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 1.5, 0.2).x;
            }

            if (fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 400)
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 1.0, 0.2).y;
            }
            else
            {
                tarvely = PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 0.8, 0.2, 2.0, 0.4).y;
            }

            if (fabs(plannerNode->nowPos.x - plannerNode->tarPos.x) <= 200 && fabs(plannerNode->nowPos.y - plannerNode->tarPos.y) <= 30 && fabs(plannerNode->nowPos.yaw - 90 * plannerNode->myFlags.RB_flag) < 3)
            {
                step = 3;
                plannerNode->upperwork = get_other_kfs;
            }
        }
        else if (step == 3)
        {
            plannerNode->lxData.start_test = 2;
            
            if (plannerNode->myFlags.col_flag == 1)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_first * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
            }
            else if (plannerNode->myFlags.col_flag == 2)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_second * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (fabs(plannerNode->tarPos.y - 10750) >= 500)
                {
                    plannerNode->tarPos.y = 10750;
                }
            }
            else if (plannerNode->myFlags.col_flag == 3)
            {
                plannerNode->tarPos.y = plannerNode->lxData.col_third * plannerNode->myFlags.RB_flag + plannerNode->nowPos.y;
                if (plannerNode->myFlags.RB_flag == 1)
                {
                    if (fabs(plannerNode->tarPos.y - 11290) >= 500)
                    {
                        plannerNode->tarPos.y = 11290;
                    }
                }
                else if (plannerNode->myFlags.RB_flag == -1)
                {
                    if (fabs(plannerNode->tarPos.y - 10210) >= 500)
                    {
                        plannerNode->tarPos.y = 10210; 
                    }
                }
            }

            if (plannerNode->Ps_Num[11] || plannerNode->Ps_Num[12])
            {
                tarvelx = -100 * plannerNode->myFlags.RB_flag;
            }
            else if (!plannerNode->Ps_Num[11] && !plannerNode->Ps_Num[12])
            {
                tarvelx = -300 * plannerNode->myFlags.RB_flag;
            }

            tarvely = PosControl(0, 0, plannerNode->tarPos.y, plannerNode->nowPos.y, 0, 0, 1.0, 0.2).y;

            if (plannerNode->upperwork != get_other_kfs)
            {
                step = 0;
                return true;
            }
            else if (plannerNode->nowPos.x <= 940)
            {
                step = 4;
            }
        }

        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

        if (yes[0] == 0 && yes[1] == 0 && yes[2] == 0)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    } 

    if (step == 4)
    {
        tarvelx = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).x;
        tarvely = PosControl(2800 * plannerNode->myFlags.RB_flag, plannerNode->nowPos.x, 10750, plannerNode->nowPos.y, 1.0, 0.2, 1.0, 0.2).y;
        omega = YawControl(plannerNode->nowPos.yaw, 90 * plannerNode->myFlags.RB_flag, 3.6, 0.6);
        
        plannerNode->tarVel.SetVel2(tarvelx, tarvely, omega);

        if (fabs(fabs(plannerNode->nowPos.x) - 2800) <= 50 && fabs(plannerNode->nowPos.y - 10750) <= 50)
        {
            plannerNode->tarVel.SetVel2(0, 0, 0);
        }
    }

    static bool leg_flag = 0;
    if (!leg_flag)
    {
        if (UpPut(GearDown1))
        {
            leg_flag = 1;
        }
    }

    return false;
}