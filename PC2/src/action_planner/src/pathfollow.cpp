#include "action_planner/pathfollow.hpp"
#define SMALL_VELL (100.f)
#define MAX_TAR_VEL (5000.f)
#define MAX_ADJUST_VEL (850.0f) //500

static uint8_t clearLenFlag = 0;
float lengthWheelTraveled = 0;
bool CalculateLenFlag = 0;//默认初始就开始记录距离
float errDirection = 0.0f;
robotVel_t targetVel = {0.0f};

/*********************************************************************************
* @name 	CalculatePoint2PointDistance
* @brief	计算点到点的距离
* @param	point1 起始点
* @param	point2 结束点
* @retval   返回两点之间的距离
*********************************************************************************/
float CalculatePoint2PointDistance(Point_t point1, Point_t point2)
{
	float dis;
	dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return dis;
}

/// @brief 根据距离起点的路径长度确定所对应的虚拟位置点
/// @param robotLen 距离起点的路径长度  单位 mm
/// @return 虚拟位置点坐标以及该点信息
Path_t SearchVirPosPoint()
{
	int i = 0;
    Point_t actpoint;
    actpoint.x = plannerNode->nowPos.x;
    actpoint.y = plannerNode->nowPos.y;
    float dis = 1E6;
	//从路径点的第二个点开始遍历（索引为1），一直遍历到路径点的末尾。 
	for (i = 1; i < plannerNode->mypaths.tarPath.size(); i++)
	{
		//如果当前计算得到的距离大于之前记录的最短距离dis，则表示找到了距离最近的路径点，跳出循环。
        if (CalculatePoint2PointDistance(actpoint, plannerNode->mypaths.tarPath.at(i).point) > dis)
        {
            break;
        }
		//如果当前计算得到的距离小于或等于之前的最短距离dis，则更新dis为当前的距离，继续循环寻找更近的路径点。
        dis = CalculatePoint2PointDistance(actpoint, plannerNode->mypaths.tarPath.at(i).point);
	}
	//在遍历路径点的循环结束后，索引 i 会指向第一个距离开始变大的路径点。此时需要将 i 减一，回到上一个路径点，因为上一个点才是距离最小的那个点。
    i--;
	//这一部分代码的调整只在路径点的数量大于2时进行，因为如果路径点太少，调整意义不大
    if (plannerNode->mypaths.tarPath.size() > 2)
    {
		//确保选择的路径点是离当前机器人位置最近的
        if (i > 0 && i < plannerNode->mypaths.tarPath.size() - 1)
        {
            if (CalculatePoint2PointDistance(actpoint, plannerNode->mypaths.tarPath.at(i-1).point) > CalculatePoint2PointDistance(actpoint, plannerNode->mypaths.tarPath.at(i+1).point))
            {
                i++;
            }
        }
		//删除已经经过的路径点，只保留当前和后续的路径点。
        plannerNode->mypaths.tarPath.erase(plannerNode->mypaths.tarPath.begin(),plannerNode->mypaths.tarPath.begin() + i - 1);
        plannerNode->mypaths.delNum += (i-1);
    }
	//如果路径点只剩一个，则返回第一个点。否则，返回路径的第二个点。
	if (plannerNode->mypaths.tarPath.size() == 1)
	{
		return plannerNode->mypaths.tarPath.front();
	}
	return plannerNode->mypaths.tarPath.at(1);
}
Path_t GetPosPresent(void)
{
	Path_t pos;
	pos.point.x = plannerNode->nowPos.x;
	pos.point.y = plannerNode->nowPos.y;
	pos.poseAngle = plannerNode->nowPos.yaw;
	pos.vel = plannerNode->actVel.sum;
	return pos;
}
/// @brief 根据距离起点的路径长度及提前量确定所对应的虚拟目标点
/// @param robotLen 距离起点的路径长度  单位 mm
/// @param viewTime 提前量 每一点为5ms
/// @return 虚拟目标点坐标以及该点信息
Path_t SearchVirTarPoint(int viewTime)
{
	Path_t result;
	int i = 1;
    i += viewTime;
	if (i + plannerNode->mypaths.delNum < plannerNode->mypaths.VIEW) //补丁，防止跟回前面的点
	{
		i = plannerNode->mypaths.VIEW - plannerNode->mypaths.delNum;//如果小于则修正为起步时的点
	}
	//如果索引 i 超过了路径点的数量，则返回最后一个路径点 myPath.tarPath.back()
    if (i >= plannerNode->mypaths.tarPath.size())
    {
        result = plannerNode->mypaths.tarPath.back();
    }
    else
    {
	    result = plannerNode->mypaths.tarPath.at(i);
    }

	return result;
}

/*********************************************************************************
* @name 	PathFollowingNew
* @brief	路径跟随函数
* @param	percent 速度的百分比，若为1代表100%所规划的速度运行。范围为0~1,如果超过1，会超过机器人承受速度
* @retval	无
**********************************************************************************/
int PathFollowing(float percent)
{
	int VIEW = 0;							    //提前量
	static vel_data_t carVel;
	float posAngleVT = 0.0f;					//虚拟目标点姿态角
	float robotlen = 0.0f;						//路程长度
	float disRealPos2VirTarget = 0.0f;	        //实际位置到虚拟目标点距离
	float disRealPos2VirPos = 0.0f;			    //实际位置到虚拟位置点距离
	robotVel_t adjustVel = {0.0f};			    //位置环调节速度
	Path_t virPosPoint,virTarPoint;             //虚拟位置点 虚拟目标点

	//虚拟位置点
	//VIEW = 3;
	virPosPoint = SearchVirPosPoint();

	//计算实际位置到虚拟位置点的距离(直线距离)	
	disRealPos2VirPos = CalculatePoint2PointDistance(GetPosPresent().point,virPosPoint.point);

	//虚拟目标点(加上提前量)
	virTarPoint = SearchVirTarPoint(VIEW);
	
	//计算实际位置距离虚拟目标点距离(直线距离)
	disRealPos2VirTarget = CalculatePoint2PointDistance(GetPosPresent().point,virTarPoint.point);

	//修正距离
	float disAdd = (VIEW * 5.0f / 1000 * plannerNode->actVel.sum / 2.0f  - disRealPos2VirPos) - disRealPos2VirTarget;
	std::cout << "disAdd: " << disAdd << std::endl;
	
	// 虚拟目标点姿态角赋值
	posAngleVT = virTarPoint.poseAngle;
	plannerNode->tarPos.yaw = posAngleVT;
	plannerNode->tarPos.x = virTarPoint.point.x;
	plannerNode->tarPos.y = virTarPoint.point.y;
	// omega = (posAngleVT - plannerNode->nowPos.yaw) / 0.005 + AngleControl(plannerNode->nowPos.yaw, posAngleVT, 6.0f, 0.8f);
	carVel.omega = AngleControl(plannerNode->nowPos.yaw, posAngleVT, 8.0f, 0.8f);

	//限制目标速度方向在-180~180
	carVel.dir = AngleLimit(virTarPoint.angle);
	
	//目标速度，并且乘以倍数关系
	carVel.sum = virTarPoint.vel * percent;

	//计算当前点到位置点的位置调节量（速度环用到虚拟位置点）
	adjustVel = GetAdjustVel(GetPosPresent().point, virPosPoint, carVel.sum);

	//将调节速度和规划的速度进行合成，传址
	AdjustVel(&carVel.sum, &carVel.dir, adjustVel);

	//速度，角速度赋值
	carVel.SetVel1(carVel.sum, carVel.dir, carVel.omega);
	carVel.x = carVel.x + PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).x;
	carVel.y = carVel.y + PosControl(plannerNode->tarPos.x, plannerNode->nowPos.x, plannerNode->tarPos.y, plannerNode->nowPos.y, 1.2, 0.2, 1.2, 0.2).y;
	plannerNode->tarVel.SetVel2(carVel.x, carVel.y, carVel.omega);
	// myRobot.VIEW_L = VIEW;							            //提前量
	// myRobot.virtualPosx = virPosPoint.point.x;		            //虚拟位置点
	// myRobot.virtualPosy = virPosPoint.point.y;
	// myRobot.VP_PosAngle = virPosPoint.poseAngle;	            //虚拟位置点姿态角
	// myRobot.virtualTarx = virTarPoint.point.x;		            //虚拟目标点
	// myRobot.virtualTary = virTarPoint.point.y;
	// myRobot.VT_PosAngle = posAngleVT;							//虚拟目标点姿态角
	// myRobot.pathTarVel  = virTarPoint.vel * percent;			//规划速度
	// myRobot.pathTarDir  = virTarPoint.angle;					//规划速度方向
	// myRobot.disRealPos2VirPos = disRealPos2VirPos;				//点到虚拟位置点距离
	// myRobot.disRealPos2VirTarget = disRealPos2VirTarget;	    //点到虚拟目标点距离
	// myRobot.disAdd = disAdd;									//修正距离
	// myRobot.adjustVel.SetVel2(adjustVel.vel,adjustVel.direction);		
	// myRobot.adjustVel_omg = omega;								//角速度

	return 1;
}

/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param    angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget,float kp,float kd)
{
	/****************************普通PD控制*******************************/
	float angleErr = 0.0f, omega = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f, dTermFliter = 0.0f;
	float wzDterm = 0.f;

	//放苗的时候加上ki
	float ki = 0.f;
	static float errSum = 0.f;

	//PD控制器
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	static float lastAngleOutPut = 0.f;
	static uint8_t  timeCnt = 0;
	
	dTerm = (angleErr - lastAngleErr);
	if (fabs(dTerm) > 30.f)
	{
		//低通滤波
		dTermFliter = 0.2f * dTerm + 0.8f * lastAngledTerm;
		omega = angleErr * kp * 0.6f + dTermFliter * kd;
	}
	else
	{
		//低通滤波
		dTermFliter = 0.5f * dTerm + 0.5f * lastAngledTerm;
		omega = angleErr * kp + dTermFliter * kd;
	}

	wzDterm = plannerNode->actVel.omega * 0.001f;

	if (fabs(wzDterm) > fabs(omega) * 1.7f)
	{
		wzDterm = fabs(wzDterm)/wzDterm * fabs(omega) * 1.7f;
	}
	omega += wzDterm;

	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;
	errSum += angleErr;
	if (fabs(errSum) >= 180.f)
	{
		errSum = 180.f * errSum / fabs(errSum);
	}

	//防止输出突变
	if (fabs(omega) > fabs(lastAngleOutPut) * 1.2f && (fabs(lastAngleOutPut) > 20.f || lastAngleOutPut * omega < 0.f))
	{
		omega = fabs(omega)/omega * fabs(lastAngleOutPut) * 1.2f;
	}
	if (fabs(lastAngleOutPut - omega) > 35.f)
	{
		omega = (lastAngleOutPut + omega) / 2.f;
	}
	
	lastAngleOutPut = omega;
	return omega;
}

/*********************************************************************************
* @name 	GetAdjustVel
* @brief	计算调节速度的大小
* @param	robotPos 当前车的状态
* @param    adjustTarget  虚拟位置点的状态
* @param  	vell  当前速度的大小
* @retval	调节速度的大小，方向角速度
**********************************************************************************/
robotVel_t GetAdjustVel(Point_t robotPos,Path_t adjustTarget,float vell)
{
	robotVel_t adjustVel = {0};
	float distance = 0.0f;
	float angle = 0.0f;
	float kp = 1.0f;

	angle = CalculateLineAngle(robotPos,adjustTarget.point);
	
	distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);

    double real_dis = distance;
	
	if (distance <= 0.0f)
	{
		distance = 0.0f;
	}

	//TODO 计算调节速度大小和方向
	if (distance <= 20.0f)
	{
		distance = distance * 0.6f;//0-12
	}
	else if (distance <= 40.0f) 
	{
		distance = distance * 0.8f - 4.0f;//12-28
	}
	else if (distance <= 60.0f)
	{
		distance = distance * 1.0f - 12.f;//28-48
	}
	else if (distance <= 100.0f)
	{
		distance = distance * 1.4f - 36.f;//48-104
	}
	else
	{
		distance = distance * 2.0f - 96.f;
	}	

	adjustVel.vel = distance * 4.0f;
	adjustVel.direction = angle;
	
	//TODO 对调节速度大小进行限制
  	float max_adjustVel; // = MAX_ADJUST_VEL; //

  	if (vell <= 100.f)
  	{
  		max_adjustVel = 50.f;
  	}
  	else if (vell <= 500.f)
  	{
  		max_adjustVel = powf(vell - 100, 2) / 640.f * 3.f + 100.f;//我拟合的曲线
  	}
  	else 
	{
		max_adjustVel = MAX_ADJUST_VEL;
	}


  	if (adjustVel.vel >= max_adjustVel) 
	{
		adjustVel.vel = max_adjustVel;
	}

	return adjustVel;
}

/*********************************************************************************
* @name 	AdjustVel
* @brief	计算合成速度的大小
* @param	carVel 目标速度
* @param    adjustTarget  目标速度角度
* @param  	vell  合成速度的大小
* @retval	调节合成速度的大小，方向角速度
**********************************************************************************/
void AdjustVel(float *carVel, float *velAngle, robotVel_t adjustVel)
{
	float angleErr = 0.0f;
	float projectOnvel = 0.0f;
	vector_t oriVel = {0} , adjust = {0} , result = {0};
	
	oriVel.module = *carVel;
	oriVel.direction = *velAngle;
	
	//计算调节目标速度的大小
	adjust.module = ADJUST_KP * adjustVel.vel;

	//计算速度方向的调节量并进行限制
	angleErr = adjustVel.direction - *velAngle;
	angleErr = angleErr > 180.0f ? angleErr - 360.0f : angleErr; 
	angleErr = angleErr < -180.0f ? angleErr + 360.0f : angleErr;

	angleErr =  ADJUST_KP * angleErr;

	if (angleErr > 180.0f)
	{
		angleErr = 180.0f;
	}
	else if (angleErr <- 180.0f)
	{
		angleErr = -180.0f;
	}
	
	adjust.direction = *velAngle + angleErr;
	
	adjust.direction = adjust.direction > 180.0f ? adjust.direction - 360.0f : adjust.direction; 
	adjust.direction = adjust.direction < -180.0f ? 360.0f + adjust.direction : adjust.direction;

	result = CalculateVectorAdd(oriVel , adjust);
	//对调整结果的速度大小进行限制
	if (result.module >= MAX_TAR_VEL)
	{
		result.module = MAX_TAR_VEL;
	}
	
	*carVel = result.module;
	*velAngle = result.direction;
}

/*********************************************************************************
* @name VelCtrlTask
* @brief  速度环
**********************************************************************************/
void VelCtrlTask(void)
{
	float vel = 0.0f;
	float velDirection = 0.0f;
	vel_data_t actVel;

	//获取实际车速
	actVel.x = plannerNode->actVel.x;
	actVel.y = plannerNode->actVel.y;

	vel = plannerNode->actVel.sum;

	velDirection = plannerNode->actVel.dir;

	VelControl((robotVel_t){vel , velDirection , plannerNode->actVel.omega});
}

/*********************************************************************************
* @name 	VelControl
* @brief	速度控制
**********************************************************************************/
void VelControl(robotVel_t actVel)
{
	robotVel_t velErr = {0.0f};
	robotVel_t outputVel = {0.0f};
	float velXErr , velYErr = 0.0f;
	float velXOutput , velYOutput = 0.0f;
	float KpX = 0.08f, KpY = 0.08f;
	const float KdX = 0.3f, KdY = 0.3f;
	float expVelX , expVelY = 0.0f;
	float actVelX , actVelY = 0.0f;
	float vxDerm = 0, vyDerm = 0;
	static float lastVxDerm = 0, lastVyDerm = 0;
	float vxdTermFilter, vydTermFilter = 0.0;
	static float lastVxErr = 0, lastVyErr = 0, lastVxdTerm = 0, lastVydTerm = 0;
	static float lastVelDir = 0;//last output vel direction

	expVelX = targetVel.vel * cosf(targetVel.direction * CHANGE_TO_RADIAN);
	expVelY = targetVel.vel * sinf(targetVel.direction * CHANGE_TO_RADIAN);
	
	actVelX = actVel.vel * cosf(actVel.direction * CHANGE_TO_RADIAN);
	actVelY = actVel.vel * sinf(actVel.direction * CHANGE_TO_RADIAN);
	
	velXErr = expVelX - actVelX;
	velYErr = expVelY - actVelY;

	if (fabs(GetPath() - plannerNode->mypaths.tarPath.back().length) < 100.f || sqrt(expVelX * expVelX + expVelY * expVelY) < 300.f)
	{	
		KpX = 0.f;
		KpY = 0.f;
	}

	//限幅&死区
	if (fabs(velXErr) >= 700.0f)
	{
		velXErr = velXErr / fabs(velXErr) * 700.0f;
	}
	else
	{
		if (fabs(expVelX) < 10.0f)
		{
			if (fabs(velXErr) <= 8.0f)
			{
				velXErr = 0.0f;
			}
		}
		else if (fabs(velXErr) <= 8.0f)
		{
			velXErr = 0.0f;
		}		
	}

	if (fabs(velYErr) >= 700.0f)
	{
		velYErr = velYErr / fabs(velYErr) * 700.0f;
	}
	else
	{
		if (fabs(expVelY) < 10.0f)
		{
			if (fabs(velYErr) <= 8.0f)
			{
				velYErr = 0.0f;
			}
		}
		else if (fabs(velYErr) <= 8.0f)
		{
			velYErr = 0.0f;
		}		
	}

	velXErr *= KpX;
	velYErr *= KpY;

	vxDerm = velXErr - lastVxErr;
	vyDerm = velYErr - lastVyErr;

	vxdTermFilter = 0.4f * vxDerm + 0.6f * lastVxDerm;
	vydTermFilter = 0.4f * vyDerm + 0.6f * lastVyDerm;

	lastVxDerm = vxDerm;
	lastVyDerm = vyDerm;
	
	lastVxErr = velXErr;
	lastVyErr = velYErr;

	velXErr +=  vxdTermFilter * KdX;
	velYErr +=  vydTermFilter * KdY;

	velErr.vel = sqrtf(velXErr*velXErr + velYErr*velYErr);
	velErr.direction = atan2f(velYErr, velXErr)*CHANGE_TO_ANGLE;

	if (velErr.vel >= 1500.0f)
	{
		velErr.vel = 1500.0f;
	}
	
	if (velErr.vel < 8.0f)
	{
		velErr.vel = 0.0f;
		velErr.direction = targetVel.direction;
	}
	
	velXOutput = velErr.vel * cosf(velErr.direction * CHANGE_TO_RADIAN) + targetVel.vel * cosf(targetVel.direction * CHANGE_TO_RADIAN);
	velYOutput = velErr.vel * sinf(velErr.direction * CHANGE_TO_RADIAN) + targetVel.vel * sinf(targetVel.direction * CHANGE_TO_RADIAN);
	
	outputVel.vel = sqrtf(velXOutput * velXOutput + velYOutput * velYOutput);
	outputVel.direction = atan2f(velYOutput, velXOutput) * CHANGE_TO_ANGLE;

	if (outputVel.vel < 180.0f)
	{
		float velAdd = 0.f;
		velAdd = AngleLimit(outputVel.direction - lastVelDir);
		if (fabs(velAdd) > 50.0f)
		{
			if (outputVel.direction > lastVelDir)
			{
				outputVel.direction = lastVelDir + 50.0f;
			}
			else if (outputVel.direction < lastVelDir)
			{
				outputVel.direction = lastVelDir - 50.0f;
			}
		}
	}
	
	if (outputVel.vel < 1.0f)
	{
		outputVel.direction = targetVel.direction;
	}
	
	if (outputVel.vel >= MAX_TAR_VEL)
	{
		outputVel.vel = MAX_TAR_VEL;
	}

	lastVelDir = outputVel.direction;

	if (lengthWheelTraveled <= 100.f)//小补丁，起步后的速度能大于 SMALL_VELL 就没关系了
	{
		if (outputVel.vel < SMALL_VELL)
		{
			outputVel.vel = SMALL_VELL;
		}
	}
	//最终赋值结果
	vel_data_t nowtarVel;
	nowtarVel.SetVel2(outputVel.vel,outputVel.direction,targetVel.omega);
	plannerNode->tarVel = nowtarVel;
}

float GetPath(void)
{
    return lengthWheelTraveled;
}

float CalculateLineAngle(Point_t pointStart, Point_t pointEnd)
{
    float dx = pointEnd.x - pointStart.x;
    float dy = pointEnd.y - pointStart.y;
    return atan2(dy, dx) * 180.0f / M_PI; // 角度制
}

vector_t CalculateVectorAdd(vector_t vector1, vector_t vector2)
{
    vector_t result;
    result.module = sqrt(vector1.module * vector1.module + vector2.module * vector2.module + 2 * vector1.module * vector2.module * cos((vector1.direction - vector2.direction) * M_PI / 180.0f));
    result.direction = atan2(vector1.module * sin(vector1.direction * M_PI / 180.0f) + vector2.module * sin(vector2.direction * M_PI / 180.0f),
                             vector1.module * cos(vector1.direction * M_PI / 180.0f) + vector2.module * cos(vector2.direction * M_PI / 180.0f)) * 180.0f / M_PI;
    return result;
}
