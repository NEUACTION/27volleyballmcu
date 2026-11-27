#主控发给mcu的数，根据需要改
#判断主控断了
Flag pcflag

int32 period_cnt_pc
int32 mode
bool start
#轮子
Wheelmotor wheel1
Wheelmotor wheel2
Wheelmotor wheel3
Wheelmotor wheel4

#蹦蹦
Motor leg1
Motor leg2
Motor leg3
Motor leg4


Motor plate
Motor slide
Motor arm1
Motor arm2


#电调
Esc esc1

char vavle_num1
char vavle_num2

float32 r2posx
float32 r2posy