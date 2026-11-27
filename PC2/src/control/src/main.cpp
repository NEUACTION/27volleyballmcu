/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-12-01 21:28:09
 * @FilePath: /ROBOCON2026_base/src/control/src/main.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "control/controlnode.hpp"
#include "control/control_timer.hpp"

#define VelLimit 3000.f
#define DRIVERMAGICNUMBER 0.4333333f//现在不用了

std::shared_ptr<controlnode> control_node;

void DataHandler();

/******************************控制节点主函数************************************** */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    //实例化控制节点
    control_node = std::make_shared<controlnode>();

    rclcpp::spin(control_node);

    rclcpp::shutdown();    
    return(0);
}

void controlnode::controlMain(void)
{
    //std::cout << "vel: " << myMovement.velX << " " << myMovement.velY << std::endl;
    myMovement.Resolutionofvelocity(myMovement.velSum, myMovement.velDir, myMovement.velOmega);
}

void DataHandler()
{
    myMovement.SetVelocity(control_node->plannerGoalFromPlanner.target_vel.linear.x,
                           control_node->plannerGoalFromPlanner.target_vel.linear.y,
                           control_node->plannerGoalFromPlanner.target_vel.angular.z,
                           control_node->plannerGoalFromPlanner.cur_pose.orientation.x);
}
/*----------------------END-------------------------------*/
