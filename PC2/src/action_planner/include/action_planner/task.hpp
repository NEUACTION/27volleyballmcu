/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-30 14:57:21
 * @FilePath: /ROBOCON2026_base/src/action_planner/include/action_planner/task.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _TASK_H
#define _TASK_H

#include <rclcpp/rclcpp.hpp>

enum ctrl_mode
{
    AUTO_MODE,
    CHALLENGE_MODE
};

//计划每个task都为先走后做动作
enum upper_task
{
    uppertest,
    pass_upper,
    error_stop,
	winwinwin,
    get_weapon,
    get_kfs,
    save_kfs,
    recive_kfs,
    throw_kfs,
    put_kfs_first,
    put_kfs_second,
    no_r2_get_kfs,
    no_upper_task,
};

enum chassis_task
{
    pass_chassis,
    chassis_test,

    area1_move,
    area2_move1,
    area2_move2,
    area2_move3,
    area2_change_move,

    area3_move,
    turn,
    move_to_get_ground_kfs,
    move_to_get_r1_kfs,
    no_chassis_task,
};

enum upper_work
{
    noupperwork,
    ready_put,
    put_into,
    get_other_kfs,
};

enum chassis_work
{
    nochassiswork,
    moveweapon,
    dockweapon,
    waitr1,

    noup,
    up200,
    down200,
    up400,
    down400,
    up350,
    down350,

    turn_left,
    turn_right,
    turn_back,

    upr1,
    downr1,
    gomove,
    gotoputsecond,
    carup,
    no_r1,
    yes_r1,
    re_area3,
};

enum step
{
    nostep,
    all_leg_up,
    all_leg_down,
    front_leg_up,
    front_leg_down,
    hind_leg_up,
    hind_leg_down,

    wheel_pos,
    wheel_vel,
};

bool CheckSelf(void);
bool DockWeaponTest(void);
bool DockWeapon(void);
bool Turn_left(void);
bool Turn_right(void);
bool MoveWeapon(void);
bool GetWeapon(void);
bool GoMoveTest(int way);
bool GoMove(int way);
bool WaitR1Test(void);
bool WaitR1(void);
bool TrunkPutPiece(void);
bool TrunkThrowKFS(bool i);
bool TrunkPutinto(void);
bool TrunkTakePiece(int i);
bool TrunkSave(int i);
bool TrunkReceive(float shoulder_max_vel, float elbow_max_vel, float wrist_max_vel);
bool UpPut(float i);
bool DownPut(void);
bool NoUp(int way);
bool YesR1(void);
bool NoR1(void);
bool Up200(void);
bool Up400(void);
bool Down200(void);
bool Down400(void);
bool CarGo2(void);
bool UpR1(void);
bool DownR1(void);
int MoveToGetGroundKfs(void);
bool GoToPutSecond(void);
bool MoveToGetR1KFS(void);

void UpperTask();
void ChassisTask();

void ChallengeTask();
#endif