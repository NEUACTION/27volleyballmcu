#ifndef _SWORD_HPP_
#define _SWORD_HPP_ 

#include "stdint.h"
#include "action_planner/motor.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/esc.hpp"
#include <vector>
#include <cstdio>
#include <fstream>

#define SWORD_FLAT_WRIST            0.27778     //向下
#define SWORD_SAVE_WRIST            0.41308     //向下

// #define SWORD_LTH0_SHOULDER         2.33667     //向下
// #define SWORD_LTH0_ELBOW            -0.23056    //向上
// #define SWORD_LTH0_WRIST            0.11333     //向下

// #define SWORD_LTH0_SHOULDER         7.41849     //向下，低取高不抬腿拿200块
// #define SWORD_LTH0_ELBOW            -0.27219    //向上
// #define SWORD_LTH0_WRIST            0.134749    //向下

// #define SWORD_LTH200_SHOULDER       8.808999     //向下，低取高腿站200或地面取块
// #define SWORD_LTH200_ELBOW          -0.305494    //向上
// #define SWORD_LTH200_WRIST          0.08905    //向下

// #define SWORD_LTH400_SHOULDER       6.06699      //向下，低取高腿不站拿400块 2.02233
// #define SWORD_LTH400_ELBOW          -0.255362    //向上
// #define SWORD_LTH400_WRIST          0.18198      //向下

// #define SWORD_HTL_SHOULDER          10.864993     //向下，高取低
// #define SWORD_HTL_ELBOW             -0.42778 //向上
// #define SWORD_HTL_WRIST             0.09449      //向下

// #define SWORD_THROW_SHOULDER        0.333333
// #define SWORD_THROW_ELBOW           -0.294383
// #define SWORD_THROW_WRIST           0.07501553

// #define SWORD_THROW_SHOULDER        1.6
// #define SWORD_THROW_ELBOW           -0.32877
// #define SWORD_THROW_WRIST           0.11266

// #define SWORD_SAVE_SHOULDER         1.5         //向下
// #define SWORD_SAVE_ELBOW            -0.00194    //向上

// #define SWORD_SAVEMID_SHOULDER      1.9         //向下
// #define SWORD_SAVEMID_ELBOW         -0.20194    //向上

class SWORD
{
public:
    SWORD_MOTOR Slide;
    SWORD_MOTOR Plate;
    SWORD_MOTOR Arm1;
    SWORD_MOTOR Arm2;

public:
    SWORD()
    : Slide(), Plate(), Arm1(), Arm2(){}
};

extern SWORD sword;

#endif