#ifndef _TIMER_H
#define _TIMER_H


#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstdint>
#include "control/controlnode.hpp"

void signalHandler(int n);

bool timer_init(void);

extern bool time_5msFlag;
extern bool time_10msFlag;
extern bool time_20msFlag;
extern int time_1msCnt;
extern bool time_2sFlag;
extern bool time_1sFlag;

void CheckCom();
#endif