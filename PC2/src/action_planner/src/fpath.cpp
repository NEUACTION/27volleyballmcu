#include "action_planner/fpath.hpp"
#include "action_planner/frecord.hpp"
#include "action_planner/action_planner_node.hpp"


//方向是正向的轨迹
std::string_view header = "/* -------- minco ------- */ ";	  //注意末尾還有幾個制表符
std::string_view tail = "----end of topp_minco----";

DriveSearchPosPoint_T DriveSearchPosPoint;

const std::vector<std::string_view> attackPaths = 
{

};

pathToFollow::pathToFollow()
{
	delNum = 0;
	VIEW = 0;
	GetAllPath();
}

/// @brief 从读取的一行文本里读取数据并存进路径点容器
/// @param iss 读取的一行路径文件
/// @param readPath 存路径点的容器
void pathToFollow::IssGetPath(std::istringstream &iss, std::vector<Path_t> &readPath)
{
	//坐标，姿态角，速度分量
	float x, y, poseAngle, vx, vy, vel;
	
	float lastx = 0, lasty = 0;		
	float lenSum = 0.f;
			

	if(readPath.size())
	{
		lastx = readPath.back().point.x;
		lasty = readPath.back().point.y;
		lenSum = readPath.back().length;
	}


	//如果不符合预期格式就会报错
	if(!(iss >> x >> y >> vx >> vy >> vel >> poseAngle))
	{
		std::cout << "iss ERROR!" << std::endl;
	}

	Path_t tmpPath;

	//单位换算，从m到mm
	x *= 1000;
	y *= 1000;
	vx *= 1000;
	vy *= 1000;
	tmpPath.point.x = x;
	tmpPath.point.y = y;
	tmpPath.poseAngle = poseAngle;
	tmpPath.vx = vx;
	tmpPath.vy = vy;
	tmpPath.vel = sqrt(vx * vx + vy * vy);
	tmpPath.angle = atan2f(-vx,vy) * 180.f / PI;
	tmpPath.length = sqrtf((x - lastx) * (x - lastx) + (y - lasty) * (y - lasty)) + lenSum;


	if(readPath.size() == 0)
	{
		tmpPath.length = 0;
	}

	readPath.emplace_back(tmpPath);
}

/// @brief 从读取的一行文本里读取数据并存进路径点容器
/// @param iss 读取的一行路径文件
/// @param readPath 存路径点的容器
void pathToFollow::IssGetPath_2(std::istringstream &iss, std::vector<Path_t> &readPath)
{

	float x, y, poseAngle, vx, vy, vsum = 0.f;
	float lastx = 0, lasty = 0;		
	float lenSum = 0.f;
			
	if(readPath.size())
	{
		lastx = readPath.back().point.x;
		lasty = readPath.back().point.y;
		lenSum = readPath.back().length;
	}

	if(!(iss >> x >> y >> vx >> vy >> vsum >> poseAngle))
	{
		std::cout << "iss ERROR!" << std::endl;
	}

	Path_t tmpPath;

	x *= 1000;
	y *= 1000;
	vx *= 1000;
	vy *= 1000;
	vsum *= 1000;
	tmpPath.point.x = x;
	tmpPath.point.y = y;
	tmpPath.poseAngle = poseAngle;
	tmpPath.vx = vx;
	tmpPath.vy = vy;
	tmpPath.vel = vsum;
	tmpPath.angle = atan2f(-vx,vy) * 180.f / PI;
	tmpPath.length = sqrtf((x - lastx) * (x - lastx) + (y - lasty) * (y - lasty)) + lenSum;


	if(readPath.size() == 0)
	{
		tmpPath.length = 0;
	}

	readPath.emplace_back(tmpPath);
}

/**
 * @brief 从目标文件里读取路径点，存进相应类的tarPath向量里面
 */
void pathToFollow::GetAllPath()
{
	{
		std::ostringstream oss;
		//std::cout << "Loading attack paths..." << std::endl;
		oss << "./src/action_planner/path/area1move600.txt";
		std::vector<Path_t> tmp_path_;
		std::fstream pathFile(oss.str());
		std::string pathString;
		std::cout << "Loading path file: " << oss.str() << std::endl;
		bool readFlag = false;
		if(!pathFile)
		{
			std::cout << "FAIL TO OPEN FILE" << oss.str() << std::endl;
			std::cout << "errno" << errno << "describe:" << strerror(errno)<< std::endl;
		}

		while (std::getline(pathFile,pathString))
		{
			std::istringstream iss(pathString);
			IssGetPath(iss, tmp_path_);
			std::cout << pathString << std::endl;
		}
		taskPaths.push_back(tmp_path_);
		std::cout << "Load attack path finished." << std::endl;
		pathFile.close();
	}
	std::cout << "taskPaths Num:" << taskPaths.size() << std::endl;

	
}

/// @brief 选择路径并赋值相应变量
/// @param pathNum 路径编号
void pathToFollow::ChoosePath(uint8_t pathNum)
{
	delNum = 0;
	tarPath.clear();
	tarPath = taskPaths.at(pathNum);
	float dir = 0.f;										//启动速度的方向
	// dir = atan2f(-(tarPath.at(
	// 	 pathNum).point.x - planner.nowPos.x), tarPath.at(pathNum).point.y - planner.nowPos.y);
}


/// @brief 输入当前和路径起始时的位姿信息，以及目标路径，输出路径里的虚拟位置点
/// @param actPoint 实际位置
/// @param startYaw 开始路径时的实际偏航角
/// @param startPoint 开始路径时的位置
/// @param vecPath 路径向量
/// @return 指向路径向量里最近点的迭代器
std::vector<Path_t>::const_iterator DriveSearchPosPoint_T::operator()(const Point_t actPoint, const float startYaw, const Point_t startPoint, const std::vector<Path_t> &vecPath)
{
	if(!updated)
	{
		this->startX = startPoint.x;
		this->startY = startPoint.y;
		this->startYaw = startYaw;
		searchBeginIt = vecPath.begin();
		updated = true;
	}

	Point_t basePoint;
	//坐标变换到原点0初始航向的实际坐标
	basePoint.SetPoint((actPoint.x - startPoint.x) * cosf(startYaw * M_PI / 180.f) + (actPoint.y - startPoint.y) * sinf(startYaw * M_PI / 180.f) , 
						-(actPoint.x - startPoint.x) * sinf(startYaw * M_PI / 180.f) + (actPoint.y - startPoint.y) * cosf(startYaw * M_PI / 180.f));

	it = searchBeginIt;

	dis2 = MAXFLOAT;

	//从路径点的第二个点开始遍历（索引为1），一直遍历到路径点的末尾。 
	for(it; it != vecPath.end(); it++)
	{
		float disToPoint2 = (basePoint.x - it->point.x) * (basePoint.x - it->point.x) + \
							(basePoint.y - it->point.y) * (basePoint.y - it->point.y);
							
		//如果当前计算得到的距离大于之前记录的最短距离dis2，则表示找到了距离最近的路径点，跳出循环。
		if(disToPoint2 > dis2)           
		{
			recorder1.PushToRecord(disToPoint2, "disToPoint2");
			break;
		}

		//如果当前计算得到的距离小于或等于之前的最短距离dis2，则更新dis2为当前的距离，继续循环寻找更近的路径点。
		dis2 = disToPoint2;
	}


	//在遍历路径点的循环结束后，索引 it 会指向第一个距离开始变大的路径点。此时需要将 it 减一，回到上一个路径点，因为上一个点才是距离最小的那个点。
	if (it != vecPath.begin())
    {
        it -= 1;
    }

	//这一部分代码的调整只在路径点的数量大于2时进行，因为如果路径点太少，调整意义不大
	if(vecPath.end() - searchBeginIt > 3 && it != vecPath.end() - 1)
	{
		if(it != vecPath.begin() && it != vecPath.end() - 1)
		{
			//确保选择的路径点是离当前机器人位置最近的
			float dis2Before = (basePoint.x - (it - 1)->point.x) * (basePoint.x - (it - 1)->point.x)+\
								(basePoint.y - (it - 1)->point.y) * (basePoint.y - (it - 1)->point.y);

			float dis2After = (basePoint.x - (it + 1)->point.x) * (basePoint.x - (it + 1)->point.x)+\
								(basePoint.y - (it + 1)->point.y) * (basePoint.y - (it + 1)->point.y);

			if(dis2Before > dis2After)
			{
				it++;
			}
		}
		
	}

	//更新起始迭代器，避免重复搜索
	searchBeginIt = it;

	
	recorder1.PushToRecord(basePoint.x,"basePointx");
	recorder1.PushToRecord(basePoint.y,"basePointy");
	recorder1.PushToRecord(it->point.x,"itX");
	recorder1.PushToRecord(it->point.y,"itY");

	return it;
}


std::vector<Path_t>::const_iterator DriveSearchTarPoint(std::vector<Path_t>::const_iterator posIt, const std::vector<Path_t> &vecPath, int viewTime)
{
	std::vector<Path_t>::const_iterator result;

	for(int i = 1; i <= viewTime; i++)
	{
		posIt++;

		if(posIt == vecPath.end())
		{
			return(posIt - 1);
		}
	}

	return(posIt);
}

void DriveSearchPosPoint_T::Reset()
{
	dis2 = MAXFLOAT;
	updated = false;
}

void vel_data_t::SetVel1(float vsum, float vdir, float omega)
{
	this->sum = vsum;
	this->dir = vdir;
	this->x = -1.f * vsum * sinf(vdir / 180.f * M_PI);
	this->y = vsum * cosf(vdir / 180.f * M_PI);
	this->omega = omega;
}

void vel_data_t::SetVel2(float vx, float vy, float omega)
{
    this->x = vx;
    this->y = vy;
    this->sum = sqrt(vx * vx + vy * vy);
    this->dir = AngleLimit(atan2(-1.f * vx, vy) * 180.f / M_PI);
    this->omega = omega;
}

void vel_data_t::SetVelX(float vx)
{
	float vy = this->y;
	this->x = vx;
	this->sum = sqrt(vx * vx + vy * vy);
    this->dir = AngleLimit(atan2(-1.f * vx, vy) * 180.f / M_PI);
}

void vel_data_t::SetVelY(float vy)
{
	float vx = this->x;
	this->y = vy;
	this->sum = sqrt(vx * vx + vy * vy);
    this->dir = AngleLimit(atan2(-1.f * vx, vy) * 180.f / M_PI);
}

void vel_data_t::SetOmega(float omega)
{
	this->omega = omega;
}

void vel_data_t::Stop(void)
{
	this->sum = 0;
    this->dir = 0;
    this->x = 0;
    this->y = 0;
	this->omega = 0;
}

float vel_data_t::GetVelX()
{
	return this->x;
}

float vel_data_t::GetVelY()
{
	return this->y;
}

float vel_data_t::GetVelOmega()
{
	return this->omega;
}

float vel_data_t::GetVelSum()
{
	return this->sum;
}

float vel_data_t::GetVelDir()
{
	return this->dir;
}

float YawControl(float anglePresent, float angleTarget, float kp, float kd)
{
	/****************************普通PD控制*******************************/
	float angleErr = 0.0f, omega = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f, dTermFliter = 0.0f;
	float wzDterm = 0.f;

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
		omega = angleErr * kp * 1.f + dTermFliter * kd;
	}
	else
	{
		//低通滤波
		dTermFliter = 0.5f * dTerm + 0.5f * lastAngledTerm;
		omega = angleErr * kp + dTermFliter * kd;
	}

	wzDterm = plannerNode->nowPos.omega * 0.005f;

	if (fabs(wzDterm) > fabs(omega) * 1.7f)
	{
		wzDterm = fabs(wzDterm)/wzDterm * fabs(omega) * 1.7f;
	}
	omega += wzDterm;

	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;
	
	//防止输出突变
	if (fabs(omega) > fabs(lastAngleOutPut) * 1.2f && (fabs(lastAngleOutPut) > 20.f || lastAngleOutPut * omega < 0.f))
	{
		omega = fabs(omega)/omega * fabs(lastAngleOutPut) * 1.2f;
	}
	if (fabs(lastAngleOutPut - omega) > 50.f)
	{
		omega = (lastAngleOutPut + omega) / 2.f;
	}
	
	lastAngleOutPut = omega;
	return omega;
}

float PitchControl(float anglePresent, float angleTarget, float kp, float kd)
{
	static float engleErr = 0;
	static float lastAngleErr = 0;
	
	engleErr = anglePresent - angleTarget;

	float adjustVel = 0;
	adjustVel = kp * engleErr + kd * (engleErr - lastAngleErr);

	if(fabs(engleErr) <= 1)
	{
		adjustVel = 0;
	}

	lastAngleErr = engleErr;

	return adjustVel;
}

float RollControl(float anglePresent, float angleTarget, float kp, float kd)
{
	static float engleErr = 0;
	static float lastAngleErr = 0;
	
	engleErr = anglePresent - angleTarget;

	float adjustVel = 0;
	adjustVel = kp * engleErr + kd * (engleErr - lastAngleErr);

	if(fabs(engleErr) <= 1)
	{
		adjustVel = 0;
	}

	lastAngleErr = engleErr;

	return adjustVel;
}

vel_data_t PosControl(float tar_x, float act_x, float tar_y, float act_y, float kp1, float kd1, float kp2, float kd2)
{
    static float errEngle_x = 0;
	static float errEngle_y = 0;
    static float lastErrEngle_x = 0;
	static float lastErrEngle_y = 0;
	vel_data_t adjustVel;

    errEngle_x = tar_x - act_x;
	errEngle_y = tar_y - act_y;

    adjustVel.x = kp1 * errEngle_x + kd1 * (errEngle_x - lastErrEngle_x);
	adjustVel.y = kp2 * errEngle_y + kd2 * (errEngle_y - lastErrEngle_y);
	adjustVel.sum = sqrt(adjustVel.x * adjustVel.x + adjustVel.y * adjustVel.y);
	adjustVel.dir = atan2f(-adjustVel.x, adjustVel.y) * 180.f / M_PI;
	
	lastErrEngle_x = errEngle_x;
	lastErrEngle_y = errEngle_y;

	return adjustVel;
}

float PlanVel(float v_max, float distance, int cnt)
{
	float start_t = cnt * 0.005;
    float total_distance = distance;//总共的路程
    float current_speed = 0.0;//目前速度
	float v = v_max;
	float up_time = v / 1000;

    float const_time = (total_distance - 2 * up_time * v - 2 * up_time * v) / v;//匀速行驶所需的时间

    if (start_t < up_time)
    {      
        current_speed = v * sin(PI * start_t / (2.0 * up_time));//t0时候的速度(0~PI/2)
    }
    else if (start_t >= up_time && start_t <= up_time + const_time)
    {
        current_speed = v;//匀速时的速度
    }
    else if (start_t > up_time + const_time && start_t <= 2 * up_time + const_time)
    {
        current_speed = v * sin(PI / 2 * (1 - (start_t - (up_time + const_time)) / up_time));
    }
    else if (start_t > 2 * up_time + const_time)
    {
        current_speed = 0;
    }

	return current_speed;
}

float AngleLimit(float angle)
{
    angle = fmod(angle, 360.f);

    if(angle > 180.f)
    {
        angle -= 360.f;
    }
    else if(angle <= -180.f)
    {
        angle += 360.f;
    }

    return(angle);
}


/*********************************************************************************
* @name 	CalculateAngleSub
* @brief	对-180,180交界处作处理
* @param	minuend: 被减数;
			subtrahend: 减数 A - B,A为被减数，B为减数;
* @retval	返回计算后的角度值 -180~180
*********************************************************************************/
float CalculateAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;

	result = fmod(result, 360.f);

	if (result >  180.0f)  
	{
		result -= 360.0f;
	}
	if (result < -180.0f)  
	{
		result += 360.0f;
	}
	return result;
}