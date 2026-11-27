#include "action_planner/frecord.hpp"
#include "action_planner/leg.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/trunk.hpp"

Recorder recorder1("mydebug");
Recorder l515("l515_data");


/// @brief 创建输出文件
/// @param foldername 子文件夹名
frecord::frecord(std::string foldername)
{
  std::string stime = this->GetTimeString();

  //生成record文件夹
  if (opendir(FILE_PATH) == NULL) 
  {
    if (mkdir(FILE_PATH, 0777) != 0)
    {
      std::cout << "cannot mkdir " << FILE_PATH << std::endl;
    }
	}

  //生成子文件夹++
  filename = FILE_PATH + foldername + "/";

  if (opendir(filename.c_str()) == NULL) 
  {
    //0777 是权限标志，表示该目录对所有用户（文件所有者、组用户和其他用户）都有读、写、执行权限。
    if (mkdir(filename.c_str(), 0777) != 0)
    {
      std::cout << "cannot mkdir " << filename << std::endl;
    }
	}
  
  //获取debug文件名
  filename = filename + stime + " " + foldername + ".txt";
  
  //以只写模式打开，没有文件就创建
  recordfile.open(filename,std::fstream::out);
}

//关闭文件
frecord::~frecord()
{
    if (recordfile.is_open()) {
        recordfile.close();
    }
}
 

/// @brief 将int类型转化为string类型 并补0
/// @param num int类型数据
/// @param std::stringstream 是 C++ 标准库中的一个类，用于在内存中构建字符串。
/// @param iss 是一个 std::stringstream 类型的对象，用于将整数转换为字符串
/// @return string类型数据
std::string frecord::Convert(int num)
{
  std::stringstream iss;
  if (num < 10)
  {
    //如果 num 小于 10，将数字 0 插入到字符串流 iss 中。这样做的目的是在单个位数的数字前添加一个前导零。
    //因而在文件名称中才会对称
    iss << 0;
  }
  iss << num;
  return iss.str();
}



/// @brief 获取y-m-d h-min 的时间字符串
/// @return 时间字符串
std::string frecord::GetTimeString()
{
    auto now = std::chrono::system_clock::now();
    std::time_t timeReal = std::chrono::system_clock::to_time_t(now);
    std::tm* t = std::gmtime(&timeReal);

    std::ostringstream oss;
    oss << Convert(t->tm_year + 1900) << "-"
        << Convert(t->tm_mon + 1) << "-"
        << Convert(t->tm_mday) << " "
        << Convert(t->tm_hour + 8) << "-"
        << Convert(t->tm_min);

    return oss.str();
}

Recorder::Recorder(std::string dataName)
{
    recordFile = std::make_shared<frecord>(dataName);
    dataOrderVector = std::make_shared<std::vector<std::string>>();
    writer_running_.store(true, std::memory_order_release);
    writer_thread_ = std::thread(&Recorder::WriterLoop, this);
}

Recorder::~Recorder()
{
    writer_running_.store(false, std::memory_order_release);
    write_cv_.notify_one();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    if (this->recordFile && this->recordFile->recordfile.is_open()) {
        this->recordFile->recordfile.flush();
        this->recordFile->recordfile.close();
    }
}

void Recorder::EnqueueLine(std::string line)
{
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.push_back(std::move(line));
    }
    write_cv_.notify_one();
}

void Recorder::WriterLoop()
{
    std::deque<std::string> local_queue;

    while (writer_running_.load(std::memory_order_acquire)) {
        {
            std::unique_lock<std::mutex> lock(write_mutex_);
            write_cv_.wait(lock, [this]() {
                return !writer_running_.load(std::memory_order_acquire) || !write_queue_.empty();
            });
            local_queue.swap(write_queue_);
        }

        while (!local_queue.empty()) {
            this->recordFile->recordfile << local_queue.front();
            local_queue.pop_front();
        }
        this->recordFile->recordfile.flush();
    }

    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        local_queue.swap(write_queue_);
    }

    while (!local_queue.empty()) {
        this->recordFile->recordfile << local_queue.front();
        local_queue.pop_front();
    }
    this->recordFile->recordfile.flush();
}

void Recorder::PrintToFile()
{
    std::ostringstream buffer; // 创建一个字符串流作为缓存

    for (auto it = dataOrderVector->begin(); it != dataOrderVector->end(); it++) 
    {
        const std::string &key = *it;
        const auto &record = dataRecordMap[key];

        if(it != dataOrderVector->begin())
        {
          // 访问dataValue，并根据类型输出到缓存
          buffer << "\t" << key << "\t" ;
        }
        else
        {
          buffer << "\t" << key << "\t" ;
        }

        std::visit(
            [&buffer](auto&& arg) 
            {
                buffer <<  std::setw(12) << arg;
            },
            record);
    }

    buffer << std::endl;

    EnqueueLine(buffer.str());
}

void RecordData(){
  auto time = rclcpp::Clock().now();
  string time_str = std::to_string((int)time.seconds()) + "." + std::to_string((int)((time.seconds() - (int)time.seconds()) * 1e6));
  RECORD(time_str, time);
  RECORD(plannerNode->cnt, cnt);
  RECORD(plannerNode->myFlags.RB_flag, RB_flag);
  RECORD(plannerNode->feedBackFromCtrl.heartbag.mcu_hb, mcuHB);
  RECORD(plannerNode->feedBackFromCtrl.heartbag.mcu_to_pc_hb, mcu_to_pc_hb);
//   RECORD(plannerNode->nowPos.x, x);
//   RECORD(plannerNode->nowPos.y, y);
//   RECORD(plannerNode->nowPos.z, z);
//   RECORD(plannerNode->nowPos.yaw, yaw);
//   RECORD(plannerNode->nowPos.pitch, pitch);
//   RECORD(plannerNode->nowPos.roll, roll);
//   RECORD(plannerNode->nowPos.lidar_omega, omega);
//   RECORD(plannerNode->nowPos.lidar_x, lidar_x);
//   RECORD(plannerNode->nowPos.lidar_y, lidar_y);
//   RECORD(plannerNode->nowPos.lidar_yaw, lidar_yaw);
//   RECORD(plannerNode->nowPos.lidar_vx, vx);
//   RECORD(plannerNode->nowPos.lidar_vy, vy);

//   RECORD(plannerNode->tarPos.x, RobotTarx);
//   RECORD(plannerNode->tarPos.y, RobotTary);
//   RECORD(plannerNode->tarPos.yaw, RobotTaryaw);

//   RECORD(plannerNode->tarVel.GetVelX(),RobotTarVelx);
//   RECORD(plannerNode->tarVel.GetVelY(),RobotTarVely);
//   RECORD(plannerNode->tarVel.GetVelOmega(),RobotTaromega);
//   RECORD(plannerNode->tarVel.GetVelSum(),RobotTarVelSum);

//   RECORD(plannerNode->myFlags.getKFS_flag, get_kfs_flag);
//   RECORD(plannerNode->myFlags.save_ready_flag, save_ready_flag);
//   RECORD(plannerNode->myFlags.save_start_flag, save_start_flag);
  
//   RECORD(plannerNode->R1StarData.R1PlatformFlag, R1PlatformFlag);
//   RECORD(plannerNode->R1StarData.R1MoveFlag, R1MoveFlag);
//   RECORD(plannerNode->myFlags.upr1_flag, upr1_flag);
//   RECORD(plannerNode->lrz.R2.dodgeR1, dodgeR1);
//   RECORD(plannerNode->R1StarData.R2GoFlag, r2go);
//   RECORD(plannerNode->R1StarData.R2PutFlag, R2PutFlag);

  RECORD(plannerNode->myFlags.uppertask, uppertask);
  RECORD(plannerNode->myFlags.chassistask, chassistask);

//   RECORD(plannerNode->R1StarData.r1posx, r1posx);
//   RECORD(plannerNode->R1StarData.r1posy, r1posy);
//   RECORD(plannerNode->lrz.R2.way_num, starway);
//   RECORD(plannerNode->lrz.R2.FirsrR2KFS, the_firstR2_KFS);
//   RECORD(plannerNode->lrz.R2.Action[1], ACTION[1]);
//   RECORD(plannerNode->lrz.R2.Action[2], ACTION[2]);
//   RECORD(plannerNode->lrz.R2.Action[3], ACTION[3]);
//   RECORD(plannerNode->lrz.R2.Action[4], ACTION[4]);

  RECORD(plannerNode->heartBags.leg1, heartbag_leg1);
  RECORD(plannerNode->heartBags.leg2, heartbag_leg2);
  RECORD(plannerNode->heartBags.leg3, heartbag_leg3);
  RECORD(plannerNode->heartBags.leg4, heartbag_leg4);
//   RECORD(plannerNode->heartBags.shoulder, heartbag_shoudler);
//   RECORD(plannerNode->heartBags.elbow, heartbag_elbow);
//   RECORD(plannerNode->heartBags.wrist, heartbag_wrist);
  RECORD(trunk.EscOne.GetTarPWM(), esc);

  RECORD(leg.LegOne.GetTarCtrlMode(), leg1_ctrlmode);
  RECORD(leg.LegOne.GetTarPos(), leg1_tarpos);
  RECORD(leg.LegOne.GetActPos(), leg1_actpos);
  RECORD(leg.LegOne.GetTarVel(), leg1_tarvel);
  RECORD(leg.LegOne.GetActVel(), leg1_actvel);
  RECORD(leg.LegOne.GetActTor(), leg1_acttor);
  RECORD(plannerNode->feedBackFromCtrl.actleg1.error, leg1_error);
  RECORD(plannerNode->feedBackFromCtrl.actleg1.enablestate, leg1_enable);
  RECORD(leg.LegTwo.GetTarCtrlMode(), leg2_ctrlmode);
  RECORD(leg.LegTwo.GetTarPos(), leg2_tarpos);
  RECORD(leg.LegTwo.GetActPos(), leg2_actpos);
  RECORD(leg.LegTwo.GetTarVel(), leg2_tarvel);
  RECORD(leg.LegTwo.GetActVel(), leg2_actvel);
  RECORD(leg.LegTwo.GetActTor(), leg2_acttor);
  RECORD(plannerNode->feedBackFromCtrl.actleg2.error, leg2_error);
  RECORD(plannerNode->feedBackFromCtrl.actleg2.enablestate, leg2_enable);
  RECORD(leg.LegThree.GetTarCtrlMode(), leg3_ctrlmode);
  RECORD(leg.LegThree.GetTarPos(), leg3_tarpos);
  RECORD(leg.LegThree.GetActPos(), leg3_actpos); 
  RECORD(leg.LegThree.GetTarVel(), leg3_tarvel);
  RECORD(leg.LegThree.GetActVel(), leg3_actvel);
  RECORD(leg.LegThree.GetActTor(), leg3_acttor);
  RECORD(plannerNode->feedBackFromCtrl.actleg3.error, leg3_error);
  RECORD(plannerNode->feedBackFromCtrl.actleg3.enablestate, leg3_enable);
  RECORD(leg.LegFour.GetTarCtrlMode(), leg4_ctrlmode);
  RECORD(leg.LegFour.GetTarPos(), leg4_tarpos);
  RECORD(leg.LegFour.GetActPos(), leg4_actpos);
  RECORD(leg.LegFour.GetTarVel(), leg4_tarvel);
  RECORD(leg.LegFour.GetActVel(), leg4_actvel);
  RECORD(leg.LegFour.GetExCur(), leg4_excur);
  RECORD(leg.LegFour.GetActTor(), leg4_acttor);
  RECORD(plannerNode->feedBackFromCtrl.actleg4.error, leg4_error);
  RECORD(plannerNode->feedBackFromCtrl.actleg4.enablestate, leg4_enable);

//   RECORD(trunk.Shoulder.GetTarPos(), trunk_shoulder_tarpos);
//   RECORD(trunk.Shoulder.GetActPos(), trunk_shoulder_actpos);
//   RECORD(trunk.Shoulder.GetTarVel(), trunk_shoulder_tarvel);
//   RECORD(trunk.Shoulder.GetActVel(), trunk_shoulder_actvel);
//   RECORD(trunk.Shoulder.GetExCur(), trunk_shoulder_excur);
//   RECORD(trunk.Shoulder.GetActTor(), trunk_shoulder_acttor);
//   RECORD(plannerNode->feedBackFromCtrl.actshoulder.error, shoulder_error);
//   RECORD(trunk.Elbow.GetTarPos(), trunk_elbow_tarpos);
//   RECORD(trunk.Elbow.GetActPos(), trunk_elbow_actpos);
//   RECORD(trunk.Elbow.GetTarVel(), trunk_elbow_tarvel);
//   RECORD(trunk.Elbow.GetActVel(), trunk_elbow_actvel);
//   RECORD(trunk.Wrist.GetTarPos(), trunk_wrist_tarpos);
//   RECORD(trunk.Wrist.GetActPos(), trunk_wrist_actpos);
//   RECORD(trunk.Wrist.GetTarVel(), trunk_wrist_tarvel);
//   RECORD(trunk.Wrist.GetActVel(), trunk_wrist_actvel);

//   RECORD(plannerNode->plannerGoalToCtrl.tarshoulder.motor_pos, trunk_shoulder_tarpos);
//   RECORD(plannerNode->feedBackFromCtrl.actshoulder.motor_pos, trunk_shoulder_actpos);
//   RECORD(plannerNode->feedBackFromCtrl.actshoulder.motor_vel, trunk_shoulder_actvel);
//   RECORD(plannerNode->feedBackFromCtrl.actshoulder.motor_cur, trunk_shoulder_acttor);
//   RECORD(plannerNode->plannerGoalToCtrl.tarelbow.motor_pos, trunk_elbow_tarpos);
//   RECORD(plannerNode->feedBackFromCtrl.actelbow.motor_pos, trunk_elbow_actpos);
//   RECORD(plannerNode->feedBackFromCtrl.actelbow.motor_vel, trunk_elbow_actvel);
//   RECORD(plannerNode->plannerGoalToCtrl.tarwrist.motor_pos, trunk_wrist_tarpos);
//   RECORD(plannerNode->feedBackFromCtrl.actwrist.motor_pos, trunk_wrist_actpos);
//   RECORD(plannerNode->feedBackFromCtrl.actwrist.motor_vel, trunk_wrist_actvel);

  RECORD(plannerNode->heartBags.wheel1, heartbag_wheel1);
  RECORD(plannerNode->heartBags.wheel2, heartbag_wheel2);
  RECORD(plannerNode->heartBags.wheel3, heartbag_wheel3);
  RECORD(plannerNode->heartBags.wheel4, heartbag_wheel4);

//   cout << "Tarvel: " << plannerNode->feedBackFromCtrl.tarwheel1.motor_vel << " " << plannerNode->feedBackFromCtrl.tarwheel2.motor_vel << " " <<
//                         plannerNode->feedBackFromCtrl.tarwheel3.motor_vel << " " << plannerNode->feedBackFromCtrl.tarwheel4.motor_vel << endl;
//   RECORD(plannerNode->feedBackFromCtrl.tarwheel1.motor_vel,wheel1TarVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel1.motor_vel,wheel1ActVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel1.motor_ex_cur,wheel1excur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel1.motor_cur,wheel1cur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel1.error, wheel1_error);
//   RECORD(plannerNode->feedBackFromCtrl.tarwheel2.motor_vel,wheel2TarVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel2.motor_vel,wheel2ActVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel2.motor_ex_cur,wheel2excur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel2.motor_cur,wheel2cur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel2.error, wheel2_error);
//   RECORD(plannerNode->feedBackFromCtrl.tarwheel3.motor_vel,wheel3TarVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel3.motor_vel,wheel3ActVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel3.motor_ex_cur,wheel3excur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel3.motor_cur,wheel3cur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel3.error, wheel3_error);
//   RECORD(plannerNode->feedBackFromCtrl.tarwheel4.motor_vel,wheel4TarVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel4.motor_vel,wheel4ActVel);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel4.motor_ex_cur,wheel4excur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel4.motor_cur,wheel4cur);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel4.error, wheel4_error);
  RECORD(plannerNode->heartBags.ps, heartbag_ps);

//   RECORD(plannerNode->plannerGoalToCtrl.tarwheel1.motor_pos, wheel1TarPos);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel1.motor_pos, wheel1ActPos);
//   RECORD(plannerNode->plannerGoalToCtrl.tarwheel2.motor_pos, wheel2TarPos);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel2.motor_pos, wheel2ActPos);
//   RECORD(plannerNode->plannerGoalToCtrl.tarwheel3.motor_pos, wheel3TarPos);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel3.motor_pos, wheel3ActPos);
//   RECORD(plannerNode->plannerGoalToCtrl.tarwheel4.motor_pos, wheel4TarPos);
//   RECORD(plannerNode->feedBackFromCtrl.actwheel4.motor_pos, wheel4ActPos);

  RECORD(plannerNode->Ps_Num[1],PS_NUM1);//????
  RECORD(plannerNode->Ps_Num[2],PS_NUM2);
  RECORD(plannerNode->Ps_Num[3],PS_NUM3);
  RECORD(plannerNode->Ps_Num[4],PS_NUM4);
  RECORD(plannerNode->Ps_Num[5],PS_NUM5);
  RECORD(plannerNode->Ps_Num[6],PS_NUM6);
  RECORD(plannerNode->Ps_Num[0],PS_NUM0);


//   RECORD(plannerNode->lrz.R2.way_num, thebestway);
//   RECORD(plannerNode->lrz.trunk_num, trunk_num);
//   RECORD(plannerNode->myFlags.saveflag, saveflag);
//   RECORD(plannerNode->lrz.trunk_flag[0][0], trunk00);
 

//   RECORD(plannerNode->myFlags.getKFS_flag, plannerNode_myFlags_getKFS_flag);
//   RECORD(plannerNode->myFlags.trunk_flag, plannerNode_myFlags_trunk_flag);
  RECORD(plannerNode->myFlags.pre_deceleration, plannerNode_myFlags_pre_deceleration);

//   RECORD(plannerNode->grid.data[0][0], zhu00);


//   RECORD(plannerNode->lxData.dist_y, lx_disty);
//   RECORD(plannerNode->lxData.weapon_state, lx_weapon_state);
//   RECORD(plannerNode->dataFromLx.dist_y, lx_disty_msg);
//   RECORD(plannerNode->lxData.mode, lx_mode);
//   RECORD(plannerNode->lxData.start_test, start_test);
//   RECORD(plannerNode->lxData.ok_assemble, lx_ok);
//   RECORD(plannerNode->lxData.task, lxtask);

//   RECORD(plannerNode->lxData.grid[0] , lx_grid0);
 

//   RECORD(plannerNode->lxData.start, lx_start);

//   RECORD(plannerNode->lxData.col_first, col_1);
//   RECORD(plannerNode->lxData.col_second, col_2);
//   RECORD(plannerNode->lxData.col_third, col_3);

//   RECORD(plannerNode->lxData.lx_posx, lx_posx);

//   RECORD(plannerNode->dataFromLxKfs.kfs_dist, kfs_dist);
//   RECORD(plannerNode->lxData.kfs_dist, kfs_dist_msg);

//   RECORD(plannerNode->tlrData.distance, tlr_disty);
//   RECORD(plannerNode->tlrData.depth, tlr_distx);
//   RECORD(plannerNode->tlrData.angle, tlr_angle);
//   RECORD(plannerNode->tlrData.should_climb, tlr_shoule_climb);
//   RECORD(plannerNode->tlrData.start, tlr_start);
//   RECORD(plannerNode->tlrData.r1_is_putting, r1_is_putting);

//   RECORD(plannerNode->tlrData.kfs_start, tlr_kfs_start);
//   RECORD(plannerNode->tlrkfsData.depth, tlrkfs_depth);
//   RECORD(plannerNode->tlrkfsData.distance, tlrkfs_distance);
//   RECORD(plannerNode->tlrkfsData.plane_num, plane_num);
//   RECORD(plannerNode->tlrkfsData.plane1_angle, plane1_angle);
//   RECORD(plannerNode->tlrkfsData.plane1_centroid_x, plane1_centroid_x);
//   RECORD(plannerNode->tlrkfsData.plane1_centroid_y, plane1_centroid_y);
//   RECORD(plannerNode->tlrkfsData.plane1_centroid_z, plane1_centroid_z);
//   RECORD(plannerNode->tlrkfsData.plane1_pointnum, plane1_pointnum);
//   RECORD(plannerNode->tlrkfsData.plane2_angle, plane2_angle);
//   RECORD(plannerNode->tlrkfsData.plane2_centroid_x, plane2_centroid_x);
//   RECORD(plannerNode->tlrkfsData.plane2_centroid_y, plane2_centroid_y);
//   RECORD(plannerNode->tlrkfsData.plane2_centroid_z, plane2_centroid_z);
//   RECORD(plannerNode->tlrkfsData.plane2_pointnum, plane2_pointnum);
  

  RECORD(plannerNode->feedBackFromCtrl.mcucnt, mcucnt);

  recorder1.PrintToFile();
}