/* 头文件保护宏：避免任务接口重复定义。 */
#ifndef USER_TASK_H
#define USER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "user_communication.h"
#include "user_FDCAN.h"

/* 外部变量：CMSIS-RTOS 线程 ID；由 User_Task_Init 创建并供其他模块查询。 */
extern osThreadId_t user_task;
/* 外部变量：FreeRTOS 原生任务句柄；当前对应用户任务。 */
extern TaskHandle_t user_task_handle;

/* 函数声明：创建并启动用户任务及相关外设任务；系统初始化时调用。 */
void User_Task_Init(void);
/* 函数声明：用户主控制任务入口；由 RTOS 创建任务时调用，当前任务运行中。 */
void User_Task(void *argument);
/* 函数声明：旧命名的梯形命中位置规划接口；实现保留，当前任务未调用。 */
void gethit_pos(void);
/* 函数声明：按角度域轨迹更新腿部目标；当前任务调用，但源文件存在同名重复定义。 */
void getHitPos(void);
/* 函数声明：按四个给定位置执行腿部位置控制；当前供上层动作逻辑调用。 */
void Leg_PosCtrl(float p1, float p2, float p3, float p4);
/* 函数声明：按四个给定位置执行腿部 PVT 控制；实现保留，目标目录内未检索到调用。 */
void LegPVTCtrl(float p1, float p2, float p3, float p4);

#ifdef __cplusplus
}
#endif

#endif /* USER_TASK_H */
