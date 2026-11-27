#ifndef _CHASSIS_H
#define _CHASSIS_H

#define VehicleRadius 415.f // 轮子距车中心距离
#define SquareRootOfTwo  1.414213562373095f // 根号2
#ifndef PI
#define PI 3.14159265358979323846
#endif
//#define CHASSIS_ACC_MAX 0.144 // 轮子加速度极限，单位：转/秒²
#define CHASSIS_ACC_MAX 0.15
#define WHEEL_DIAMETER 52.f //轮子直径 mm
#define WHEEL_1_ANGLE 90.f
#define WHEEL_2_ANGLE -137.5f
#define WHEEL_3_ANGLE -42.5f
#define WHEEL_1_DIS   415.f
#define WHEEL_2_DIS   415.f
#define WHEEL_3_DIS   415.f
#define MAX_TAR_VEL 5000.f
#define WHEEL_1_ANGLE1 90.f
#define WHEEL_2_ANGLE1 -137.5f
#define WHEEL_3_ANGLE1 -42.5f
#include <cmath>
struct Chassis_Data{
    //期望信息
    float run_motor_vel;
    float turn_motor_pos;
    float max_runmotor_vel;

    float act_cur;
    float ex_cur;
    int ChassisNum;
    Chassis_Data():run_motor_vel(0.0f),turn_motor_pos(0.0f),max_runmotor_vel(31.0f),ChassisNum(0){}
};
struct vel{
    float x;
    float y;
    float sum; // mm/s
    float dir; // degree
    float omega; // deg/s

    vel():x(0.0f),y(0.0f),sum(0.0f),dir(0.0f){}
    void SetVel1(float velx, float vely);
    void SetVel2(float velsum, float velDir);
};
struct Robot_Data{
    vel tarVel;
    vel actVel;
    float nowangle;//deg
};
class CHASSIS{
public:
    CHASSIS(float firstTarPos,int num);
    Chassis_Data tar_data;
    Chassis_Data act_data;
    bool reverseFlag;//反向标志位
    vel world_TarVel;

    void SetTarRunVel(float Tarvel);// r/s
    void SetTarTurnPos(float TarPos);// r
    void SetMaxRunVel(float Maxvel);// r/s

    float GetTarRunVel();
    float GetTarTurnPos();
    float GetActRunVel();
    float GetActTurnPos();
    float GetActCur();
    float GetExCur();
    float GetMaxRunVel();

    bool IsArrive()
    {
        return (fabs(this->GetActTurnPos() - this->GetTarTurnPos()) < 0.1f ? 1 : 0);
    }
};
void SteerWheeltran(CHASSIS *wheel, vel aimVel, float omg,float dis, float angle, float angle0);
void ChassisTransform(vel carVel, float omg, float angle);
void StopCar(void);
bool OnlyTurnWheelPos(vel carVel, float omg, float angle);
void VelCtrlTask(void);
void VelControl(vel actVel);

extern CHASSIS Wheel1;
extern CHASSIS Wheel2;
extern CHASSIS Wheel3;
extern CHASSIS Wheel4;
extern Robot_Data myRobot;

#endif