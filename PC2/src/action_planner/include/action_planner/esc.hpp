#ifndef _ESC_HPP_
#define _ESC_HPP_

#include "stdint.h"
#include <vector>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <iostream>

#define PWM_go (1950)       //取块档，95油
#define PWM_keep (1500)    //持块档，50油
#define PWM_off (1010)      //关，最低档

class ESC//电调类
{
public:
    int tarDataEsc;

public:
    ESC()
    :tarDataEsc(1010){};
    
    void SetESCTarPara(int pwm);
    
    int GetTarPWM();
};

extern ESC esc;

#endif