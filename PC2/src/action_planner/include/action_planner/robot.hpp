/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-25 21:48:36
 * @FilePath: /ROBOCON2026_base/src/action_planner/include/action_planner/robot.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _ROBOT_H
#define _ROBOT_H

#include <cstdio>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <cstring>
#include <time.h>
// #include "action_planner_node.hpp"

using namespace std;

// class HeartBag
// {
//     public:
//         uint8_t mcu; // 小电脑上计数
//         uint8_t pc_to_mcu; // 主控上计数
//         uint8_t wheel1;
//         uint8_t wheel2;
//         uint8_t wheel3;
//         uint8_t tongue1;
//         uint8_t tongue2;
//         uint8_t platform;
//         uint8_t raw;
//         uint8_t connector;
//         uint8_t pps;
//         uint8_t ps;
//         uint8_t psit;
//         uint8_t valve1;
//         uint8_t valve2;
//         uint8_t tongue1turn;
//         uint8_t tongue2turn;
// };

// class Flag
// {
// public:
//     bool startGameFlag;         //比赛开始标志位
//     bool appStartFlag;          //APP开始标志位
//     bool readyFlag;             //初始化全部完成标志位
//     bool runSucceedFlag;        //成功运行程序无错误标志位
//     uint8_t restartFlag;        //can重启次数

//     bool selfCheckFlag;         //自检完成标志位
//     bool selfCheckOutputFlag;   //自检输出标志位
//     bool _24v_Flag;             //电源24标志位

//     bool getPpsFlag;            //收到定位系统标志位
    
//     bool canErrorFlag;          //CAN通信出错标志位
//     bool getMcuErrorFlag;       //通信错误标志位
//     bool mcuErrorFlag;          //mcu出错标志位
//     bool pcErrorFlag;           //pc出错标志位
//     bool ppsErrorFlag;          //定位出错标志位
//     bool disTooBig;             //偏移太远标志位

//     bool WINWINWIN;             //winwinwin
    
//     bool wheelReadyFlag;        //轮向电机转到位标志位

//     bool isDefenseFlag;

//     bool field_color;           //场地颜色标志位，true为红色，false为蓝色

//     bool tongue1_go_flag;
//     bool tongue2_go_flag;
// }; 

/*app数值*/
struct app_data_t
{
    bool stopFlag;      // 停止标志位
    bool disableFlag;

    int appDriveCnt;
    int lastAppDriveCnt;
};

class linear_interpolation{//仅单调递增
public:
	int Allidex = 0;
	float result[200];
	float idex[200];
	void SetData(int _Allidex,float *_result,float *_idex){
		Allidex = _Allidex;
		for(int i = 1;i <= _Allidex; ++i){
			result[i] = _result[i];
			idex[i] = _idex[i];
		}
	}
	float Findidex(float Taridex){
        std::cout << "Taridex " << Taridex << " " << idex[1] << result[1] << std::endl;
		if(Taridex <= idex[1])return result[1];
		if(Taridex >= idex[Allidex])return result[Allidex];
		for(int i = 1;i < Allidex; ++i){
            std::cout << "Result: " << Taridex << " " << i << " " << idex[i] << " " << idex[i + 1] << " " << (result[i] + ((Taridex - idex[i]) / (idex[i + 1] - idex[i]) * (result[i + 1] - result[i]))) << std::endl;
			if((Taridex - idex[i]) * (Taridex - idex[i + 1]) <= 0.f){
                // plannerNode->logger()->info("idex: {} {} {}",Taridex,idex[i],i);
				return (result[i] + ((Taridex - idex[i]) / (idex[i + 1] - idex[i]) * (result[i + 1] - result[i])));
			}
		}
        // return result[Allidex];
	}
};
void LaserData_Init();
extern linear_interpolation Front_Laser;
extern linear_interpolation Right_Laser;
extern linear_interpolation Left_Laser;

extern uint8_t UpdatePosLoopCnt;

#endif