#ifndef _SWORD_HPP_
#define _SWORD_HPP_ 

#include "stdint.h"
#include "action_planner/motor.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/esc.hpp"
#include <vector>
#include <cstdio>
#include <fstream>
//宏，记录数值。下面举例用
#define SWORD_FLAT_WRIST            0.27778     //向下
#define SWORD_SAVE_WRIST            0.41308     //向下


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