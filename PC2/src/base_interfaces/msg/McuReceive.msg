#MCU发出的数
Flag mcuflag

#can重启次数
uint8 restart_success_flag

bool mcu_error_flag
bool pps_restart_flag

int32 period_cnt_mcu

#心跳包
HeartBag heartbag

int32 tx_can_cnt
int32 rx_can_cnt

#舵轮
Wheelmotor wheel1
Wheelmotor wheel2
Wheelmotor wheel3
Wheelmotor wheel4

#腿
Motor leg1
Motor leg2
Motor leg3
Motor leg4

#机械臂
Motor slide
Motor plate
Motor arm1
Motor arm2


char ps_num1
char ps_num2
char ps_num3

float32 r1posx
float32 r1posy

int32 the_way2_num
int32[5] action
int32 firstr2kfs
int32 dodger1

bool r1platformflag
bool r1moveflag
bool r2goflag
bool r1area3flag
bool r2putflag
bool r1platehigh
bool r1movedirection
 