/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:28:09
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-16 19:36:09
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/main.cpp
 * @Description: 

 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include <cstdio>
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "action_planner/action_planner_node.hpp"
// #include "action_planner/robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

std::shared_ptr<actionPlannerNode> plannerNode;

pathToFollow myPath_R2;

#define RECORD(a,b) recorder1.PushToRecord(a,#b)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 实例化控制节点
  plannerNode = std::make_shared<actionPlannerNode>();

  std::thread action_planner_thread(&actionPlannerNode::mainThread, plannerNode);

  rclcpp::ExecutorOptions executor_options;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 4);
  executor.add_node(plannerNode);
  executor.spin();

  action_planner_thread.join();

  rclcpp::shutdown();  

  return 0;
}

