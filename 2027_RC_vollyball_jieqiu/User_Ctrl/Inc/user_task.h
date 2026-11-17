#ifndef USER_TASK_H
#define USER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "user_communication.h"
#include "user_FDCAN.h"

extern osThreadId_t user_task;
extern TaskHandle_t user_task_handle;

void User_Task_Init(void);
void User_Task(void *argument);
void gethit_pos(void);
void getHitPos(void);
void Leg_PosCtrl(float p1, float p2, float p3, float p4);
void LegPVTCtrl(float p1, float p2, float p3, float p4);

#ifdef __cplusplus
}
#endif

#endif /* USER_TASK_H */
