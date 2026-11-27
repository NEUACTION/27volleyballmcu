#include "control/chassis.hpp"
#include "control/controlnode.hpp"
#include <iostream>
// CHASSIS Wheel1(83.f);
// CHASSIS Wheel2(-45.f);
// CHASSIS Wheel3(-145.f);
CHASSIS Wheel1(0.f,1);
CHASSIS Wheel2(0.f,2);
CHASSIS Wheel3(0.f,3);
CHASSIS Wheel4(0.f,4);
Robot_Data myRobot;

CHASSIS::CHASSIS(float firstTarPos,int num)
{
    tar_data.turn_motor_pos = firstTarPos;
    tar_data.ChassisNum = num;
    //构造函数
}

void CHASSIS::SetTarRunVel(float Tarvel)// r/s
{
    if(fabs(Tarvel) > tar_data.max_runmotor_vel)
    {
        if(Tarvel > 0)
            Tarvel = tar_data.max_runmotor_vel;
        else
            Tarvel = -tar_data.max_runmotor_vel;
    }
    if(fabs(Tarvel - tar_data.run_motor_vel) > CHASSIS_ACC_MAX)
    {
        if(Tarvel > tar_data.run_motor_vel)
        {
            Tarvel = tar_data.run_motor_vel + CHASSIS_ACC_MAX;
        }
        else{
            Tarvel = tar_data.run_motor_vel - CHASSIS_ACC_MAX;
        }
    }
    tar_data.run_motor_vel = Tarvel;
}

void CHASSIS::SetTarTurnPos(float TarPos)// r
{

    // if(fabs(TarPos - tar_data.turn_motor_pos) > 1.6f && !control_node->plannerGoalFromPlanner.onlyturnwheelflag)//onlyturnwheelflag ???
    {
        if(TarPos > tar_data.turn_motor_pos)                   
        {
            TarPos = tar_data.turn_motor_pos + 1.6f;
        }
        else{
            TarPos = tar_data.turn_motor_pos - 1.6f;
        }
    }
    tar_data.turn_motor_pos = TarPos;
}

void CHASSIS::SetMaxRunVel(float Maxvel)// r/s
{
    tar_data.max_runmotor_vel = Maxvel;
}
float CHASSIS::GetTarRunVel()
{
    return tar_data.run_motor_vel;
}
float CHASSIS::GetTarTurnPos()
{
    return tar_data.turn_motor_pos;
}
float CHASSIS::GetActRunVel()
{
    return act_data.run_motor_vel;
}
float CHASSIS::GetActTurnPos()
{
    return act_data.turn_motor_pos;
}
float CHASSIS::GetActCur()
{
    return act_data.act_cur;
}
float CHASSIS::GetExCur()
{
    return act_data.ex_cur;
}
float CHASSIS::GetMaxRunVel()
{
    return tar_data.max_runmotor_vel;
}


/*
 * @brief 将角度限制在 (-180,180] (deg)
*/
float AngleLimit(float angle)
{
    if (angle > -180.f && angle <= 180.f) return angle;
    else if (angle > 180.f) angle -= 360.f;
    else if (angle <= -180.f) angle += 360.f;
    return angle = AngleLimit(angle);
}

/*
 *@brief 使轮向顺滑函数 同时赋值
 *@param nowWheel 轮子对应的类变量
 *@param tarVel 给轮子的期望转速 r/s
 *@param tarAngle 给轮子的期望角度 degree
*/
void MakeTurnWell(CHASSIS *nowWheel, float tarVel, float tarAngle)
{
    float actPos,actVel;
    actPos = nowWheel->GetActTurnPos(); 
    actVel = nowWheel->GetActRunVel();
    //std::cout << "MakeTurnWell actPos: " << actPos << " actVel: " << actVel << std::endl;
    //如果想要跑 找最近的方向
    if (myRobot.tarVel.sum >= 20.f || fabs(myRobot.tarVel.omega) > 2.f || control_node->plannerGoalFromPlanner.onlyturnwheelflag)
    {
        if (nowWheel->reverseFlag)//看看是否已经反向过了
        {
            tarVel *= -1;
            tarAngle += 180.f;
        } 

        if (actVel >= 3.f)//跑的过程中轮速不能反向,简单处理轮向不能转一整圈即可
        {
            while (fabs(actPos - tarAngle) > 180.f)
            {
                tarAngle += 360.f / fabs(actPos - tarAngle) * (actPos - tarAngle);
            }
        }
        else//如果是起跑可以取反轮速然后找最近的180度
        {
            while (fabs(actPos - tarAngle) > 90.f)
            {                                                    
                tarAngle += 180.f / fabs(actPos - tarAngle) * (actPos - tarAngle);
                tarVel *= -1;//看看是否需要反向
                nowWheel->reverseFlag = !(nowWheel->reverseFlag);
            }
        }
    }
    else
    {
        if (nowWheel->reverseFlag) tarVel *= -1;

        tarAngle = actPos;
    }
    //std::cout << "MakeTurnWell tarAngle: " << tarAngle << " tarVel: " << tarVel << std::endl;
    nowWheel->SetTarRunVel(tarVel);
    nowWheel->SetTarTurnPos(tarAngle);
}

/*
 * @brief 将期望车速度等分解到底盘上并赋值给轮子的类   
 * @param carVel 世界坐标系期望速度 mm/s
 * @param omg 期望角速度 deg/s
 * @param angle 车当前偏航角 deg
 * @note 有爪子那一块为车尾,y轴负半轴
 * @note 左前2号轮，右前3号轮，后轮1号轮
 * @warning 该函数只适合在车跑的时候调用，如果想停车的时候改变轮向，使用OnlyTurnWheelPos()
 */
void ChassisTransform(vel carVel, float omg, float angle)
{
    //改了姿态角0度，直接在原来分解上改个增量角度即可

    angle = AngleLimit(angle);

    vel relativeVel;//车坐标系下速度
    relativeVel.SetVel2(carVel.sum, -AngleLimit(carVel.dir - angle));
    //std::cout << "relativeVel: " << relativeVel.sum << " " << relativeVel.dir << std::endl;
    SteerWheeltran(&Wheel1, relativeVel, omg / 180.f * PI, WHEEL_1_DIS, WHEEL_1_ANGLE1, WHEEL_1_ANGLE);
    SteerWheeltran(&Wheel2, relativeVel, omg / 180.f * PI, WHEEL_2_DIS, WHEEL_2_ANGLE1, WHEEL_2_ANGLE);
    SteerWheeltran(&Wheel3, relativeVel, omg / 180.f * PI, WHEEL_3_DIS, WHEEL_3_ANGLE1, WHEEL_3_ANGLE);
}
/*
 * @brief 前进速度与角速度合成（舵轮）
 * @param *wheel 该轮子对应指针
 * @param aimVel 车坐标系下期望线速度（不包含自转的前进速度）mm/s
 * @param omg 角速度 rad/s
 * @param dis 轮子中心距车中心距离(mm)
 * @param angle 车坐标系下轮子相对于车头0度度数
 * @param angle0 轮子0度位置对应车坐标系的度数
 * @note 最终将之间赋值给对应轮子
 */
vel VelComposition(vel v1, vel v2)
{
    vel v;
    v.SetVel1(v1.x+v2.x, v1.y+v2.y);
    return v;
}
void SteerWheeltran(CHASSIS *wheel, vel aimVel, float omg,float dis, float angle, float angle0)
{
    vel v_omg;//根据车的角速度算出的速度
    v_omg.SetVel2(fabs(omg * dis), omg > 0.f ? AngleLimit(angle + 90.f) : AngleLimit(angle - 90.f));
    vel v = VelComposition(aimVel, v_omg);
    //std::cout << "Wheel Vel: " << v.sum << " " << v.dir << std::endl;

    wheel->world_TarVel = v;
    float wheelTargetVel =  v.sum / PI / WHEEL_DIAMETER;
    float wheelTargetPos = v.dir - angle0;
    MakeTurnWell(wheel, wheelTargetVel, wheelTargetPos);
}

int wheelArriveCnt = 0;
// /*
//  * @brief 用于开跑前转轮向并判断是否到位
//  */
bool OnlyTurnWheelPos(vel carVel, float omg, float angle)
{
    Wheel1.SetTarRunVel(0.f);
    Wheel2.SetTarRunVel(0.f);
    Wheel3.SetTarRunVel(0.f);
    if(fabs(Wheel1.GetActRunVel()) >= 0.05f || fabs(Wheel2.GetActRunVel()) >= 0.05f || fabs(Wheel3.GetActRunVel()) >= 0.05f)return false;
    ChassisTransform(myRobot.tarVel,myRobot.tarVel.omega,myRobot.nowangle);
    Wheel1.SetTarRunVel(0.f);
    Wheel2.SetTarRunVel(0.f);
    Wheel3.SetTarRunVel(0.f);
    // control_node->logger()->info("OnlyTurnWheelPos tarAngle: {} {} {} {} {}",Wheel1.GetTarTurnPos(),Wheel2.GetTarTurnPos(),Wheel3.GetTarTurnPos(),carVel.dir,angle);//????

    //判断是否转到位
    if (fabs(Wheel1.GetActTurnPos() - Wheel1.GetTarTurnPos()) < 0.8f && \
    fabs(Wheel2.GetActTurnPos() - Wheel2.GetTarTurnPos()) < 0.8f && fabs(Wheel3.GetActTurnPos() - Wheel3.GetTarTurnPos()) < 0.8f 
     && fabs(Wheel1.GetActRunVel()) < 0.5f && fabs(Wheel2.GetActRunVel()) < 0.5f && fabs(Wheel3.GetActRunVel()) < 0.5f)
    {
        wheelArriveCnt++;
    }
    else 
    {
        wheelArriveCnt = 0;
    }

    if (wheelArriveCnt >= 5)//可以更改，试试改小好不好用
    {
        return 1;
    }
    else 
    {
       return 0; 
    }
    /*试试看有没有bug了
    可能还得确认一下他们规划给的第一个点给的速度在这个函数里面用会不会有bug
    还有加上（延时？或者 判断上一次跟这一次一不一样？）判断轮向有没有到期望
    */
}

//停车
void StopCar(void)
{
    myRobot.tarVel.SetVel2(0,0);
    myRobot.tarVel.omega= 0;
    Wheel1.SetTarRunVel(0.f);
    Wheel2.SetTarRunVel(0.f);
    Wheel3.SetTarRunVel(0.f);
}

//检测速度小于某一个数
bool VelCheck(float miniVel)
{
    static int cnt = 0;
    if (myRobot.actVel.sum < miniVel)
    {
        cnt++;
    }
    else cnt = 0;

    if (cnt > 2)
    {
        cnt = 0;
        return 1;
    }
    else return 0;
}
/*********************************************************************************
* @name VelCtrlTask
* @brief  速度环
**********************************************************************************/
/**
 * @brief Controls the velocity of the robot based on the actual velocity and direction.
 *
 * This function retrieves the current velocity and direction of the robot from the
 * myRobot object, calculates the total velocity and direction, and then calls the
 * VelControl function to adjust the robot's velocity accordingly.
 *
 * The function does not return any value but directly influences the robot's motion
 * by updating the velocity through the VelControl function.
 */
void VelCtrlTask(void)
{
	vel actVel;
	//获取实际车速
    actVel.SetVel1(myRobot.actVel.x,myRobot.actVel.y);
	VelControl(actVel);
}


/**
 * @brief 速度控制函数，实现PID控制算法，包含斜坡限制和抗饱和处理
 * @param actVel 当前实际速度结构体
 */
void VelControl(vel actVel)
{
    const float dt = 0.01f;                 // 10ms，控制周期
    const float KpX = 1.0f, KpY = 1.0f;    // X和Y方向的比例系数
    const float KiX = 0.1f, KiY = 0.1f;     // X和Y方向的积分系数
    const float KdX = 0.0f, KdY = 0.0f;    // X和Y方向的微分系数

    // ---- 轮加速度 -> 线速度斜坡限制 ----
    const float a_w = 0.144f;                                      // 轮加速度系数
    const float a_v = 3.1415926f * WHEEL_DIAMETER * a_w;   // m/s^2
    const float dv_max = a_v;                         // m/s 每周期最大变化

    static float lastVxErr = 0, lastVyErr = 0;
    static float lastDx = 0, lastDy = 0;
    static float Ix = 0, Iy = 0;
    static float lastVelDir = 0;
    static float lastOutSum = 0;            // 用于 sum 斜坡限制

    // 期望速度
    float expVelX = myRobot.tarVel.x;
    float expVelY = myRobot.tarVel.y;

    // 实际速度
    float actVelX = actVel.x;
    float actVelY = actVel.y;

    // 误差
    float ex = expVelX - actVelX;
    float ey = expVelY - actVelY;

    // -------- D项（带低通，且除以dt）--------
    float dx = (ex - lastVxErr) / dt;
    float dy = (ey - lastVyErr) / dt;

    // 一阶低通：你原来 0.4/0.6 可以保留
    dx = 0.4f * dx + 0.6f * lastDx;
    dy = 0.4f * dy + 0.6f * lastDy;

    lastDx = dx;
    lastDy = dy;
    lastVxErr = ex;
    lastVyErr = ey;

    // -------- PI(D) 控制输出：这里输出的是“修正量”--------
    // 先算不带积分饱和判断的 u
    float ux_p = KpX * ex + KdX * dx;
    float uy_p = KpY * ey + KdY * dy;

    // 先做一个“误差很小就不控”的死区（你们原来 sum<8 的逻辑可保留，但建议按分量做）
    const float dead = 0.01f;
    if (fabsf(ex) < dead) ex = 0;
    if (fabsf(ey) < dead) ey = 0;

    // 预积分
    float Ix_new = Ix + KiX * ex * dt;
    float Iy_new = Iy + KiY * ey * dt;

    // 计算带积分的修正量
    float ux = ux_p + Ix_new;
    float uy = uy_p + Iy_new;

    // -------- 把“修正量”加回期望，得到输出目标速度 --------
    float outX = expVelX + ux;
    float outY = expVelY + uy;

    //std::cout << "expVelX: " << expVelX << " expVelY: " << expVelY << " outX: " << outX << " outY: " << outY << std::endl;
    vel outputVel;
    outputVel.SetVel1(outX, outY);

    // -------- 幅值饱和--------
    if (outputVel.sum >= MAX_TAR_VEL) outputVel.sum = MAX_TAR_VEL;

    // -------- anti-windup：如果输出幅值已经饱和，并且积分还在推向更大，就冻结积分 --------
    // 这里用一个简化判据：饱和且误差方向与输出同向时不更新积分
    // 更严格可以按 outX/outY 分量分别判饱和
    if (outputVel.sum < MAX_TAR_VEL - 1e-3f) {
        Ix = Ix_new;
        Iy = Iy_new;
    } // else: 饱和冻结积分

    // -------- sum 斜坡限制（由轮最大加速度换算而来）--------
    float sumTarget = outputVel.sum;
    float ds = sumTarget - lastOutSum;
    if (ds > dv_max) sumTarget = lastOutSum + dv_max;
    else if (ds < -dv_max) sumTarget = lastOutSum - dv_max;
    outputVel.sum = sumTarget;
    lastOutSum = outputVel.sum;

    // -------- --------
    if (outputVel.sum < 180.0f) {
        float velAdd = AngleLimit(outputVel.dir - lastVelDir);
        if (fabsf(velAdd) > 50.0f) {
            outputVel.dir = (outputVel.dir > lastVelDir) ? (lastVelDir + 50.0f) : (lastVelDir - 50.0f);
        }
    }
    if (outputVel.sum < 1.0f) {
        outputVel.dir = myRobot.tarVel.dir;
    }
    lastVelDir = outputVel.dir;

    // 最终赋值
    myRobot.tarVel.SetVel2(outputVel.sum, outputVel.dir);
}

// 已知xy分速度存入vel
void vel::SetVel1(float vx, float vy)
{
    x = vx;
    y = vy;
    sum = sqrt(vx * vx + vy * vy);
    dir = AngleLimit(atan2(vy,vx) * 180.f / PI);
    if(sum < 1e-6f) dir = this->dir; // 如果速度很小，保持原来的方向不变
}

// 已知速度大小和方向(deg)存入vel
void vel::SetVel2(float vsum, float vdir)
{
    sum = vsum;
    dir = vdir;
    x =  vsum * cosf(vdir / 180.f * PI);
    y =  vsum * sinf(vdir / 180.f * PI);
}