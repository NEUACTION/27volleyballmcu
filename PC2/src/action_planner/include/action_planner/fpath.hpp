#ifndef _FPATH_HPP_
#define _FPATH_HPP_
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdint.h>
#include <string_view>
#include "action_planner/robot.hpp"
#include <rclcpp/rclcpp.hpp>
#define MAX_TAR_VEL (10000.f)
#define PERCENT (1.0f)
struct Point_t
{
	float x;
	float y;
    void SetPoint(const float x, const float y)
    {
        this->x = x;
        this->y = y;
    }
    Point_t() : x(0.f), y(0.f) {}
    Point_t(float x_, float y_) : x(x_), y(y_) {}
    void SetPos(float x_,float y_){
    	x = x_;
    	y = y_;
	} 
};

struct PoseCorrect_t
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
};

struct Path_t
{
    //位置
    Point_t point;
    //角度制 姿态角 速度方向角
    float poseAngle;
    float angle;
    //速度
    float vx;
    float vy;
    float vel;
    //每段到起点的长度
    float length;
    float omega;
};

class vel_data_t
{
public:
    float x;     //mm/s
    float y;     //mm/s
    float sum;   //mm/s
    float dir;   //deg
    float omega; //deg/s 
    bool dirlimitflag = false;

public:

    vel_data_t():x(0.0),y(0.0),sum(0.0),dir(0.0){}
    vel_data_t(float vx, float vy);

    void SetVel(float vsum, float vdir, float omega);
    void SetVel1(float vsum, float vdir, float omega);
    void SetVel2(float vx, float vy, float omega);
    float YawControl(float anglePresent, float angleTarget, float kp, float kd, float nowOmg);
    float PitchControl(float anglePresent, float angleTarget, float kp, float kd);
    float RollControl(float anglePresent, float angleTarget, float kp, float kd);
    void Stop(void);

    float GetVelX();
    float GetVelY();
    float GetVelOmega();
    void TurnDirection(float dir);

    void SetVelX(float vx);
    void SetVelY(float vy);
    void SetOmega(float omega);
    float GetVelSum();
    float GetVelDir();

    vel_data_t operator+ (const vel_data_t & other) const
    {
        vel_data_t result;
        result.SetVel1(this->x + other.x, this->y + other.y, 0.0f);

        return(result);
    }
    vel_data_t operator- (const vel_data_t & other) const
    {
        vel_data_t result;
        result.SetVel1(this->x + other.x, this->y - other.y, 0.0f);
        return(result);
    }

    float operator* (const vel_data_t & other) const
    {
        return this->x * other.x + this->y * other.y;
    }

    vel_data_t operator* (const float multi) const
    {
        vel_data_t multiVel;
        multiVel.SetVel2(this->sum * multi, this->dir, 0.0f);
        return multiVel;
    };

    friend vel_data_t operator*(float multi, const vel_data_t& v)
    {
        return v * multi;
    }
};

// 位姿信息
struct pos_data_t
{
    float x;            // x坐标
    float y;            // y坐标
    float z;            // z坐标 
    float vx;           // x速度
    float vy;           // y速度
    float vz;           // z速度
    float lidar_vx;
    float lidar_vy;
    float lidar_omega;
    float pps_vx;
    float pps_vy;
    float pps_w1;
    float pps_w2;
    float pps_a1;
    float pps_a2;
    float omega;        // 角速度
    float pitch;        // 俯仰角
    float roll;         // 翻滚角
    float yaw;          // 航向角
    float a1;           // 加速度x
    float a2;           // 加速度y
    float a3;           // 加速度z
    float lidar_x = 0.f;
    float lidar_y = 0.f;
    float lidar_yaw = 0.f;
    float pps_x = 0.f;
    float pps_y = 0.f;
    float pps_yaw = 0.f;
    float lidar_corr_x = 0.f;
    float lidar_corr_y = 0.f;
    float lidar_corr_yaw = 0.f;
};

//路径结构体
// struct Path_t
// {
// 	//位置
// 	Point_t point;
// 	//角度制 姿态角 速度方向角
// 	float poseAngle;
// 	float omg;
// 	float angle;
// 	//速度
// 	float vx;
// 	float vy;
// 	float vel;
// 	//每段到起点的长度
// 	float length;

//     void transform(const float cordinalX, const float cordinalY, const float yaw)
//     {
//         float newX, newY, newAngle, newPoseAngle, newVx, newVy;

//         const float YawCos = cosf(yaw * M_PI / 180.f);
//         const float YawSin = sinf(yaw * M_PI / 180.f);

//         newX = YawCos * point.x - YawSin * point.y + cordinalX;
//         newY = YawSin * point.x + YawCos * point.y + cordinalY;

//         newVx = YawCos * vx - YawSin * vy;
//         newVy = YawSin * vx + YawCos * vy;

//         point.SetPoint(newX, newY);

//         poseAngle = poseAngle + yaw;

//         //限制角度
//         angle = fmod(atan2(-1.f * newVx, newVy) * 180.f / M_PI, 360.f);
//         if(angle > 180.f)
//         {
//             angle -= 360.f;
//         }
//         else if(angle <= -180.f)
//         {
//             angle += 360.f;
//         }

//         vx = newVx;
//         vy = newVy;
//         vel = sqrt(vx * vx + vy * vy);
//     }
// };

/**
 * 储存平面坐标变换信息的类
 */
class TransCore
{
public:
    float xVector_x;				//原坐标系x单位向量在新坐标系内的坐标
    float xVector_y;
    float yVector_x;				//原坐标系y单位向量在新坐标系内的坐标
    float yVector_y;
    float base_x;					//新坐标原点在原坐标系的坐标
    float base_y;
};


class DriveSearchPosPoint_T
{
public: 
    std::vector<Path_t>::const_iterator operator()(const Point_t actPoint, const float yaw, const Point_t startPoint,const std::vector<Path_t> &vecPath);
    void Reset();
private:
    std::vector<Path_t>::const_iterator searchBeginIt;
    std::vector<Path_t>::const_iterator it;
    float dis2 = MAXFLOAT;
    float startX;
    float startY;
    float startYaw;
    bool updated;
};

float AngleLimit(float angle);
float CalculateAngleSub(float minuend, float subtrahend);

class pathToFollow
{
public:
    int delNum;
    int VIEW;
    vel_data_t startVel;			

    std::vector<Path_t> tarPath;							//跑的过程中要用的数组
    std::vector<std::vector<Path_t>> taskPaths;				//存放多条路径的向量

public:
    void ChoosePath(uint8_t pathNum);
    explicit pathToFollow();
    void OmgPlanning(float maxAngAcc, float tarOmg);
    void GetAllPath(void);
    void GetAllPath_blue(void);

private:
    void IssGetPath(std::istringstream &iss, std::vector<Path_t> &readPath);
    //第二种读取函数，用来处理不同的路径格式
    void IssGetPath_2(std::istringstream &iss, std::vector<Path_t> &readPath);
};

extern DriveSearchPosPoint_T DriveSearchPosPoint;
extern Point_t stretchPos[5][2][20]; //i 
extern Point_t ProtectPos[5][2][20]; //i = waynum,j = 0左 j = 1右，取的第k个块  
#endif