/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2025-11-09 10:17:56
 * @FilePath: /ROBOCON2026_base/src/control/src/control_timer.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "control/control_timer.hpp"


bool time_5msFlag = 0;
bool time_10msFlag = 0;
bool time_1sFlag = 0;
int time_1msCnt = 0;

void signalHandler(int n)
{
    static uint32_t cnt = 0;
    cnt++;
    time_1msCnt++;
    
    if (cnt % 5 == 0) 
    {
        time_5msFlag = 1;

        //等到跟pps和mcu都通信上再检测
        // if (myRobot.heartBags.mcu < 200 && myRobot.periodCntPC && myRobot.flags.runSucceedFlag) 
        // {
        //     myRobot.heartBags.mcu++;
        // }

        // if (myRobot.heartBags.pps < 200 && myRobot.periodCntPC && myRobot.flags.runSucceedFlag) 
        // {
        //     myRobot.heartBags.pps++;
        // }
        // CheckCom();
    }

    if (cnt % 10 == 0) 
    {
        time_10msFlag = 1;
    }


    if (cnt == 1000) 
    {
      time_1sFlag = 1, cnt = 0;
    }
}

//
bool timer_init(void)
{
    signal(SIGALRM, signalHandler);

    struct itimerval timer_set;

    //启动时间（1s后启动）
    timer_set.it_value.tv_sec = 1;
    timer_set.it_value.tv_usec = 0;

    //间隔定时器间隔：5ms
    timer_set.it_interval.tv_sec = 0;
    timer_set.it_interval.tv_usec = 1000;//5ms一次中断

    if(setitimer(ITIMER_REAL, &timer_set, NULL) < 0)
    {
        return false;
    }

    return true;
}

/// @brief 检测通信是否正常，改变相应标志位
void CheckCom()
{
  // if (myRobot.heartBags.pps > 100)
  // {
  //   //只有第一次断的时候发，避免刷屏
  //   if (!myRobot.flags.ppsErrorFlag) 
  //   {
  //     RCLCPP_FATAL_ONCE(rclcpp::get_logger("pps"),"disconnected!");
  //   }
  //   myRobot.flags.ppsErrorFlag = 1;
  // }
  // else 
  // {
  //   myRobot.flags.ppsErrorFlag = 0;
  // }

  // //与主控通信检测
  // if (myRobot.heartBags.mcu > 20)
  // {
  //   //只有第一次断的时候发，避免刷屏
  //   if (!myRobot.flags.getMcuErrorFlag) 
  //   {
  //     RCLCPP_FATAL_ONCE(rclcpp::get_logger("mcu"),"disconnected!");
  //   }  
  //   myRobot.flags.getMcuErrorFlag = 1;
  // }
  // else 
  // {
  //   myRobot.flags.getMcuErrorFlag = 0;
  // }
}