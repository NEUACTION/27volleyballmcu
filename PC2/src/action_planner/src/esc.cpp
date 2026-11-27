#include "action_planner/esc.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>

int ESC::GetTarPWM()
{
    return tarDataEsc;
}

void ESC::SetESCTarPara(int pwm)
{
    tarDataEsc = pwm;
}
