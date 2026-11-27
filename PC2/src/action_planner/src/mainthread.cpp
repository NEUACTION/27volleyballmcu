/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:28:09
 * @LastEditors: NoCarrot06 1543640290@qq.com
 * @LastEditTime: 2025-11-30 22:44:17
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/mainthread.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */

// ==================== 区域：头文件包含 开始 ====================
#include <cstdio>
// 标准输入输出头文件
#include <chrono>
// C++ 时间库，用于休眠和时间相关操作
#include <thread>
// 线程库，用于当前线程休眠
#include "action_planner/action_planner_node.hpp"
// 动作规划节点主头文件
#include "action_planner/robot.hpp"
// 机器人对象定义
#include "action_planner/task.hpp"
// 任务定义
#include "action_planner/leg.hpp"
// 腿部控制定义
#include "action_planner/wheel_motor.hpp"
// 轮电机控制定义
#include "action_planner/trunk.hpp"
// 躯干控制定义
// ==================== 区域：头文件包含 结束 ====================

// ==================== 区域：前置函数声明 开始 ====================
void CopyData();
// 声明反馈数据拷贝函数
void AutoCtl();
// 声明自动控制函数
void ChallengeCtrl();
// 声明挑战赛控制函数，当前文件中未实现
// ==================== 区域：前置函数声明 结束 ====================

// ==================== 区域：全局同步与坐标校正变量 开始 ====================
std::mutex mtx;
// 全局互斥锁，用于保护主线程和反馈数据之间的共享数据

Matrix3f T_corr = Matrix3f::Identity();
// 雷达参考坐标系到 PPS 坐标系的 2D 齐次校正矩阵
Matrix2f R_corr = Matrix2f::Identity();
// 从 T_corr 中提取的 2D 旋转部分
Matrix3f T_corr_switch = Matrix3f::Identity();
// 雷达坐标系到行程开关坐标系的 2D 齐次校正矩阵
Matrix2f R_corr_switch = Matrix2f::Identity();
// 从 T_corr_switch 中提取的 2D 旋转部分
bool is_corr_inited = false;
// 坐标校正矩阵是否已初始化标志
// ==================== 区域：全局同步与坐标校正变量 结束 ====================



void actionPlannerNode::mainThread()
{

  // ==================== 区域：按运行模式初始化任务状态：run 开始 ====================
  if (plannerNode->mode == "run")
  // 如果当前模式为 run
  {
    plannerNode->ctrlMode = AUTO_MODE;
    // 设置控制模式为自动模式
    plannerNode->UpperTask = uppertest;
    // 上层任务设为无上层任务
    // plannerNode->ChassisTask = chassis_test;
    // // 底盘任务设为cs
    // plannerNode->upperwork = noupperwork;
    // // 上层工作状态设为无
    // plannerNode->chassiswork = moveweapon;
    // // 底盘工作状态设为移动武器
    // plannerNode->mystep = all_leg_down;
    // // 腿部步骤设为全腿放下
    static bool start = 0;
    const mode = 4;
  }
  // ==================== 区域：按运行模式初始化任务状态：run 结束 ====================



  // ==================== 区域：腿控制模式与 PID 参数初始化 开始 ====================
  // leg.LegOne.SetTarPosKpKd(1.4, 1.0);
  // // 设置腿 1 位置环 Kp、Kd
  // leg.LegTwo.SetTarPosKpKd(0.5, 0.5);
  // // 设置腿 2 位置环 Kp、Kd
  // leg.LegThree.SetTarPosKpKd(1.2, 0.2);
  // // 设置腿 3 位置环 Kp、Kd
  // leg.LegFour.SetTarPosKpKd(0.4, 0.2);
  // // 设置腿 4 位置环 Kp、Kd

  // leg.LegOne.SetTarVelKpKi(1.2, 0.35);
  // // 设置腿 1 速度环 Kp、Ki
  // leg.LegTwo.SetTarVelKpKi(0.8, 0.5);
  // // 设置腿 2 速度环 Kp、Ki
  // leg.LegThree.SetTarVelKpKi(0.6, 0.7);
  // // 设置腿 3 速度环 Kp、Ki
  // leg.LegFour.SetTarVelKpKi(0.6, 1.5);
  // // 设置腿 4 速度环 Kp、Ki
  // ==================== 区域：腿控制模式与 PID 参数初始化 结束 ====================

  // ==================== 区域：主循环 开始 ====================
  while(1)
  // 主线程无限循环
  {
    // ==================== 区域：5ms 周期触发判断 开始 ====================
    if (time_5msFlag.exchange(false, std::memory_order_acq_rel))
    // 如果 5ms 标志被置位，则进入本周期处理，并原子清除标志
    {
      // ==================== 区域：加锁拷贝反馈数据 开始 ====================
      mtx.lock();
      // 加锁，保护共享反馈数据
      CopyData();
      // 拷贝底层反馈数据到规划器内部对象
      mtx.unlock();
      // 解锁
      // ==================== 区域：加锁拷贝反馈数据 结束 ====================

      // ==================== 区域：等待雷达位置有效 开始 ====================
      // if (abs(plannerNode->nowPos.x) < 1e-5 && abs(plannerNode->nowPos.y) < 1e-5 && abs(plannerNode->nowPos.z) < 1e-5)
      // // 如果当前位置 x、y、z 都近似为 0
      // {
      //   plannerNode->logger()->info("waiting for lidar...");
      //   // 打印等待雷达信息
      //   PublishToLyx();
      //   // 仍然发布给 Lyx
      //   continue;
      //   // 跳过本周期后续逻辑
      // }
      // else
      // // 否则认为雷达位置已有效
      // {
      //   static int wait_cnt = 0;
      //   // 静态等待计数器
      //   wait_cnt++;
      //   // 等待计数加一

      //   if (wait_cnt <= 200)
      //   // 如果等待次数不超过 200
      //   {
      //     continue;
      //     // 继续等待
      //   }
      //   else
      //   // 等待次数超过 200
      //   {
      //     wait_cnt = 201;
      //     // 固定为 201，避免溢出
      //   }
      // }
      // ==================== 区域：等待雷达位置有效 结束 ====================

      // myflag.getPosFlag = 1;
      // // 标记已获取位置

      // PublishToLxChoose();
      // // 发布选择数据给 Lx
      
      SendToCtrl();
      // 发送控制数据到底层
      RecordData();
      // 记录数据

      // ==================== 区域：re_area2 模式等待与初始化 开始 ====================
      if (plannerNode->mode == "re_area2")
      // 如果当前模式为 re_area2
      {
      
      }
      // ==================== 区域：re_area2 模式等待与初始化 结束 ====================

      // ==================== 区域：re_area3 模式等待与初始化 开始 ====================
      if (plannerNode->mode == "re_area3")
      // 如果当前模式为 re_area3
      {
        
      }
      // ==================== 区域：re_area3 模式等待与初始化 结束 ====================


      // ==================== 区域：启动信号状态机变量 开始 ====================
      static int start_flag = 0;
      // 启动状态机当前状态
      static int start_cnt = 0;
      // 启动过程中 PS 信号无效计数
      static bool ps_flag = 0;
      // 当前选择的 PS 启动信号
      // ==================== 区域：启动信号状态机变量 结束 ====================


  

      // ==================== 区域：等待 reset 就绪 开始 ====================
      if (!plannerNode->myFlags.readyFlag)
      // 如果 reset 就绪标志无效
      {
        plannerNode->logger()->info("waiting for reset...");
        // 打印等待 reset 信息
        continue;
        // 等待下一周期
      }
      // ==================== 区域：等待 reset 就绪 结束 ====================

      // ==================== 区域：自动控制执行 开始 ====================
      if (plannerNode->ctrlMode == AUTO_MODE) 
      // 如果控制模式为自动模式
      {
        AutoCtl();
        // 执行自动控制
      }
      // ==================== 区域：自动控制执行 结束 ====================


    }
    else
    // 如果 5ms 标志未触发
    {
      // ==================== 区域：非 5ms 周期休眠 开始 ====================
      // 避免空转占满 CPU，同时降低线程调度抖动
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // 当前线程休眠 1ms
      // ==================== 区域：非 5ms 周期休眠 结束 ====================
    }
  }
  // ==================== 区域：主循环 结束 ====================
}


void CopyData()
{
  // ==================== 区域：拷贝腿电机反馈 开始 ====================
  leg.LegOne.SetActPara(
    plannerNode->feedBackFromCtrl.actleg1.motor_vel,
    plannerNode->feedBackFromCtrl.actleg1.motor_pos,
    plannerNode->feedBackFromCtrl.actleg1.motor_cur
  );
  // 设置腿 1 的实际速度、位置、电流

  leg.LegTwo.SetActPara(
    plannerNode->feedBackFromCtrl.actleg2.motor_vel,
    plannerNode->feedBackFromCtrl.actleg2.motor_pos,
    plannerNode->feedBackFromCtrl.actleg2.motor_cur
  );
  // 设置腿 2 的实际速度、位置、电流

  leg.LegThree.SetActPara(
    plannerNode->feedBackFromCtrl.actleg3.motor_vel,
    plannerNode->feedBackFromCtrl.actleg3.motor_pos,
    plannerNode->feedBackFromCtrl.actleg3.motor_cur
  );
  // 设置腿 3 的实际速度、位置、电流

  leg.LegFour.SetActPara(
    plannerNode->feedBackFromCtrl.actleg4.motor_vel,
    plannerNode->feedBackFromCtrl.actleg4.motor_pos,
    plannerNode->feedBackFromCtrl.actleg4.motor_cur
  );
  // 设置腿 4 的实际速度、位置、电流
  // ==================== 区域：拷贝腿电机反馈 结束 ====================

  // ==================== 区域：拷贝 sword(剑) 电机反馈 开始 ====================
  sword.Slide.SetActPara(
    plannerNode->feedBackFromCtrl.actslide.motor_vel,
    plannerNode->feedBackFromCtrl.actslide.motor_pos,
    plannerNode->feedBackFromCtrl.actslide.motor_cur
  );
  // 设置 Slide 的实际速度、位置、电流

  sword.Plate.SetActPara(
    plannerNode->feedBackFromCtrl.actplate.motor_vel,
    plannerNode->feedBackFromCtrl.actplate.motor_pos,
    plannerNode->feedBackFromCtrl.actplate.motor_cur
  );
  // 设置 Plate 的实际速度、位置、电流

  sword.Arm1.SetActPara(
    plannerNode->feedBackFromCtrl.actarm1.motor_vel,
    plannerNode->feedBackFromCtrl.actarm1.motor_pos,
    plannerNode->feedBackFromCtrl.actarm1.motor_cur
  );
  // 设置 Arm1 的实际速度、位置、电流

  sword.Arm2.SetActPara(
    plannerNode->feedBackFromCtrl.actarm2.motor_vel,
    plannerNode->feedBackFromCtrl.actarm2.motor_pos,
    plannerNode->feedBackFromCtrl.actarm2.motor_cur
  );
  // 设置 Arm2 的实际速度、位置、电流
  // ==================== 区域：拷贝 sword 电机反馈 结束 ====================
  plannerNode->wheel[1].ActRunVel = plannerNode->feedBackFromCtrl.actwheel1.vel;
  plannerNode->wheel[1].ActTurnPos = plannerNode->feedBackFromCtrl.actwheel1.pos;
  plannerNode->wheel[2].ActRunVel = plannerNode->feedBackFromCtrl.actwheel2.vel;
  plannerNode->wheel[2].ActTurnPos = plannerNode->feedBackFromCtrl.actwheel2.pos;
  plannerNode->wheel[3].ActRunVel = plannerNode->feedBackFromCtrl.actwheel3.vel;
  plannerNode->wheel[3].ActTurnPos = plannerNode->feedBackFromCtrl.actwheel3.pos;
  plannerNode->wheel[4].ActRunVel = plannerNode->feedBackFromCtrl.actwheel4.vel;
  plannerNode->wheel[4].ActTurnPos = plannerNode->feedBackFromCtrl.actwheel4.pos;

  plannerNode->wheel[1].TarRunVel = plannerNode->feedBackFromCtrl.tarwheel1.vel;
  plannerNode->wheel[1].TarTurnPos = plannerNode->feedBackFromCtrl.tarwheel1.pos;
  plannerNode->wheel[2].TarRunVel = plannerNode->feedBackFromCtrl.tarwheel2.vel;
  plannerNode->wheel[2].TarTurnPos = plannerNode->feedBackFromCtrl.tarwheel2.pos;
  plannerNode->wheel[3].TarRunVel = plannerNode->feedBackFromCtrl.tarwheel3.vel;
  plannerNode->wheel[3].TarTurnPos = plannerNode->feedBackFromCtrl.tarwheel3.pos;
  plannerNode->wheel[4].TarRunVel = plannerNode->feedBackFromCtrl.tarwheel4.vel;
  plannerNode->wheel[4].TarTurnPos = plannerNode->feedBackFromCtrl.tarwheel4.pos;
  // ==================== 区域：拷贝心跳包数据 开始 ====================
  plannerNode->heartBags.leg1_hb = plannerNode->feedBackFromCtrl.heartbag.leg1_hb;
  // 拷贝腿 1 心跳包
  plannerNode->heartBags.leg2_hb = plannerNode->feedBackFromCtrl.heartbag.leg2_hb;
  // 拷贝腿 2 心跳包
  plannerNode->heartBags.leg3_hb = plannerNode->feedBackFromCtrl.heartbag.leg3_hb;
  // 拷贝腿 3 心跳包
  plannerNode->heartBags.leg4_hb = plannerNode->feedBackFromCtrl.heartbag.leg4_hb;
  // 拷贝腿 4 心跳包

  plannerNode->heartBags.slide_hb = plannerNode->feedBackFromCtrl.heartbag.slide_hb;
  // 拷贝 Slide 心跳包
  plannerNode->heartBags.plate_hb = plannerNode->feedBackFromCtrl.heartbag.plate_hb;
  // 拷贝 Plate 心跳包
  plannerNode->heartBags.arm1_hb = plannerNode->feedBackFromCtrl.heartbag.arm1_hb;
  // 拷贝 Arm1 心跳包
  plannerNode->heartBags.arm2_hb = plannerNode->feedBackFromCtrl.heartbag.arm2_hb;
  // 拷贝 Arm2 心跳包

  plannerNode->heartBags.wheel1_hb = plannerNode->feedBackFromCtrl.heartbag.wheel1_hb;
  plannerNode->heartBags.wheel2_hb = plannerNode->feedBackFromCtrl.heartbag.wheel2_hb;
  plannerNode->heartBags.wheel3_hb = plannerNode->feedBackFromCtrl.heartbag.wheel3_hb;
  plannerNode->heartBags.wheel4_hb = plannerNode->feedBackFromCtrl.heartbag.wheel4_hb;


  plannerNode->heartBags.mcu_hb = plannerNode->feedBackFromCtrl.heartbag.mcu_hb;
  // 拷贝 MCU 心跳包
  plannerNode->heartBags.mcu_to_pc_hb = plannerNode->feedBackFromCtrl.heartbag.mcu_to_pc_hb;
  // 拷贝 MCU 到 PC 心跳包

  plannerNode->heartBags.ps_hb = plannerNode->feedBackFromCtrl.heartbag.ps_hb;
  // 拷贝 PS 心跳包
  // ==================== 区域：拷贝心跳包数据 结束 ====================


  // ==================== 区域：拷贝 PPS 姿态与 reset 就绪标志 开始 ====================
  plannerNode->ppsPos.yaw = plannerNode->posData.yaw;
  // 拷贝 PPS 偏航角

  plannerNode->myFlags.readyFlag = plannerNode->feedBackFromCtrl.mcuflag.reset_flag;

  // 拷贝 reset 就绪标志
  plannerNode->myFlags.only_need_start_flag = plannerNode->feedBackFromCtrl.mcuflag.only_need_start_flag
  // ==================== 区域：拷贝 PPS 姿态与 reset 就绪标志 结束 ====================

  // ==================== 区域：解析 PS_Num 位状态 开始 ====================
  for (int i = 1; i <= 6; ++i)
  // 遍历 1 到 8 位
  {
    plannerNode->Ps_Num[i] = (plannerNode->feedBackFromCtrl.ps_num1 >> (i - 1)) & 0x01;
    // 从 ps_num1 中解析第 i 位
  }
  // ==================== 区域：解析 PS_Num 位状态 结束 ====================
}

void AutoCtl()
{
  // ==================== 区域：自动控制分发：上层任务与底盘任务 开始 ====================
  UpperTask();
  // 执行上层任务控制
  // ChassisTask();
  // 执行底盘任务控制
  // ==================== 区域：自动控制分发：上层任务与底盘任务 结束 ====================
}

// ==================== 区域：构建 2D 齐次变换矩阵 开始 ====================
// 根据 x, y, yaw(度) 构造 2D 齐次变换矩阵
Matrix3f buildTransform2D(double x, double y, double yaw_deg)
{
    float yaw_rad = deg2rad(yaw_deg);
    // 将角度转换为弧度
    float c = cosf(yaw_rad);
    // 计算 cos
    float s = sinf(yaw_rad);
    // 计算 sin

    Matrix3f T;
    // 定义 3x3 齐次变换矩阵
    T << c, -s, x,
         s,  c, y,
         0.0f, 0.0f, 1.0f;
    // 填充旋转和平移部分
    return T;
    // 返回变换矩阵
}
// ==================== 区域：构建 2D 齐次变换矩阵 结束 ====================

// ==================== 区域：用第一组 PPS 数据初始化雷达参考坐标系 开始 ====================
// 用第一组 PPS 数据初始化“雷达参考坐标系”
void correctWithLidar(float pps_x, float pps_y, float pps_yaw,
                      float lidar_x, float lidar_y, float lidar_yaw)
{
    Matrix3f T_pps   = buildTransform2D(pps_x, pps_y, pps_yaw);
    // 构造 PPS 位姿变换矩阵
    Matrix3f T_lidar = buildTransform2D(lidar_x, lidar_y, lidar_yaw);
    // 构造雷达位姿变换矩阵

    T_corr = T_lidar * T_pps.inverse();
    // 计算雷达坐标系到 PPS 坐标系的校正矩阵
    R_corr = T_corr.block<2,2>(0,0);
    // 提取旋转部分
    is_corr_inited = true;
    // 标记校正矩阵已初始化
}
// ==================== 区域：用第一组 PPS 数据初始化雷达参考坐标系 结束 ====================

// ==================== 区域：用行程开关数据更新雷达-开关校正矩阵 开始 ====================
void correctWithSwitch(float lidar_x, float lidar_y, float lidar_yaw,
                      float switch_x, float switch_y, float switch_yaw)
{
    Matrix3f T_switch = buildTransform2D(switch_x, switch_y, switch_yaw);
    // 构造行程开关位姿变换矩阵
    Matrix3f T_lidar  = buildTransform2D(lidar_x, lidar_y, lidar_yaw);
    // 构造雷达位姿变换矩阵

    T_corr_switch = T_lidar * T_switch.inverse();
    // 计算雷达坐标系到行程开关坐标系的校正矩阵
    R_corr_switch = T_corr_switch.block<2,2>(0,0);
    // 提取旋转部分
    is_corr_inited = true;
    // 标记校正矩阵已初始化
}
// ==================== 区域：用行程开关数据更新雷达-开关校正矩阵 结束 ====================

// ==================== 区域：根据行程开关校正雷达位姿 开始 ====================
//根据行程开关更新雷达矩阵，（不矫正的方向给-1，角给绝对值大于360）
void correctLidar(float trueX, float trueY, float trueYaw)
{
  //left_ok_time单位是0.1ms
  //单位统一成0.1ms
  int true_time = ((int)((int64_t)rclcpp::Clock().now().nanoseconds() / 1e6) % 200);
  // 获取当前时间，并统一到 0.1ms 周期内
  int lidar_time = plannerNode->lidar_time % 200;
  // 获取雷达时间，并对 200 取模
  float time_diff = ((float)(true_time - lidar_time));
  // 计算时间差
  if(time_diff < 0)time_diff += 200.f;
  // 如果时间差为负，则加上一个周期
  // float this_vx = plannerNode->nowPos.vx / 1000;
  // float this_vy = plannerNode->nowPos.vy / 1000;
  // float this_omega = plannerNode->nowPos.omega / 1000;
  float this_vx = 0.f;
  // 当前 x 方向速度暂设为 0
  float this_vy = 0.f;
  // 当前 y 方向速度暂设为 0
  float this_omega = 0.f;
  // 当前角速度暂设为 0
  float lidar_x = time_diff * this_vx + plannerNode->nowPos.lidar_x;
  // 根据时间差补偿雷达 x
  float lidar_y = time_diff * this_vy + plannerNode->nowPos.lidar_y;
  // 根据时间差补偿雷达 y
  float lidar_yaw = time_diff * this_omega + plannerNode->nowPos.lidar_yaw;
  // 根据时间差补偿雷达偏航角
  float corr_x = plannerNode->nowPos.x + time_diff * this_vx;
  // 根据时间差补偿当前 x
  float corr_y = plannerNode->nowPos.y + time_diff * this_vy;
  // 根据时间差补偿当前 y
  float corr_yaw = plannerNode->nowPos.yaw + time_diff * this_omega;
  // 根据时间差补偿当前偏航角
  bool correct_x = fabs(trueX) > 10.f;
  // 如果 trueX 绝对值大于 10，则校正 x
  bool correct_y = trueY > 0.f;
  // 如果 trueY 大于 0，则校正 y
  bool correct_yaw = fabs(trueYaw) < 360.f;
  // 如果 trueYaw 绝对值小于 360，则校正偏航角
  if(correct_x)corr_x = trueX;
  // 满足条件时用 trueX 覆盖 corr_x
  if(correct_y)corr_y = trueY;
  // 满足条件时用 trueY 覆盖 corr_y
  if(correct_yaw)corr_yaw = trueYaw;
  // 满足条件时用 trueYaw 覆盖 corr_yaw

  if((correct_x && fabs(lidar_x - corr_x) >= 500.f) || (correct_y && fabs(lidar_y - corr_y) >= 500.f)){
    // 如果校正量与雷达量偏差过大，认为校正异常
    // plannerNode->logger()->info("CorrectLidarError: {} {} {} {}",lidar_x,corr_x,lidar_y,corr_y);
    return;
    // 直接返回，不更新位姿
  }

  plannerNode->nowPos.x = corr_x;
  // 更新当前 x
  plannerNode->nowPos.y = corr_y;
  // 更新当前 y
  plannerNode->nowPos.yaw = corr_yaw;
  // 更新当前偏航角

  correctWithSwitch(lidar_x, lidar_y, lidar_yaw, corr_x, corr_y, corr_yaw);
  // 用行程开关数据更新雷达-开关校正矩阵
  float test_x;
  // 测试用 x
  float test_y;
  // 测试用 y
  float test_yaw;
  // 测试用偏航角
  LidarToSwitchCorrected(lidar_x,lidar_y,lidar_yaw,test_x,test_y,test_yaw);
  // 将雷达坐标转换到行程开关坐标，测试一次
  LidarToSwitchCorrected(plannerNode->nowPos.lidar_x,plannerNode->nowPos.lidar_y,plannerNode->nowPos.lidar_yaw,test_x,test_y,test_yaw);
  // 将当前雷达坐标转换到行程开关坐标，再测试一次
}
// ==================== 区域：根据行程开关校正雷达位姿 结束 ====================

// ==================== 区域：雷达坐标转到行程开关坐标 开始 ====================
void LidarToSwitchCorrected(float lidar_x, float lidar_y, float lidar_yaw,
                         float &switch_x, float &switch_y, float &switch_yaw)
{
    if (!is_corr_inited)
    // 如果校正矩阵尚未初始化
    {
        std::cerr << "Error: correction matrix not initialized!" << std::endl;
        // 输出错误信息
        return;
        // 直接返回
    }

    Matrix3f T_lidar = buildTransform2D(lidar_x, lidar_y, lidar_yaw);
    // 构造雷达位姿变换矩阵
    Matrix3f T_out = T_corr_switch.inverse() * T_lidar;
    // 计算转换到行程开关坐标系的变换矩阵

    switch_x = T_out(0, 2);
    // 提取行程开关 x
    switch_y = T_out(1, 2);
    // 提取行程开关 y

    float yaw_rad = atan2f(T_out(1, 0), T_out(0, 0));
    // 从旋转矩阵提取偏航角弧度
    switch_yaw = normalize_angle_deg(rad2deg(yaw_rad));
    // 转换为角度并归一化
}
// ==================== 区域：雷达坐标转到行程开关坐标 结束 ====================