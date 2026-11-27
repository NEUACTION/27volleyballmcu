/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:35:14
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2026-01-20 23:25:51
 * @FilePath: /ROBOCON2026_base/src/action_planner/src/action_planner_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "action_planner/action_planner_node.hpp"
#include "common_logger/common_logger.hpp"
#include "action_planner/leg.hpp"
#include "action_planner/wheel_motor.hpp"
#include "action_planner/trunk.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <eigen3/Eigen/Dense>
#include <iomanip>

actionPlannerNode::actionPlannerNode(): Node("actionPlannerNode")
{
    logger_ = common_logger::Logger::getInstance("action_planner_node");
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pps_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    odin1_record_ = std::make_shared<frecord>("odin1");
    
    lx_task_publisher = this->create_publisher<base_interfaces::msg::CameraTask>("/lx_task", 10);
    lx_msg_subscriber = this->create_subscription<base_interfaces::msg::Camera>("/weapon_camera_miao",10, 
                std::bind(&actionPlannerNode::LxCallback, this, std::placeholders::_1));
    kfs_msg_subscriber = this->create_subscription<base_interfaces::msg::CameraKfs>("/kfs_distance",10, 
                std::bind(&actionPlannerNode::KfsCallback, this, std::placeholders::_1));
    grid_lx_msg_subscriber = this->create_subscription<base_interfaces::msg::GridState>("d455/gridstate",10, 
                std::bind(&actionPlannerNode::Grid_lx_Callback, this, std::placeholders::_1));
    grid_distances_subscriber = this->create_subscription<base_interfaces::msg::GridDistances>("d455/griddist",10, 
                std::bind(&actionPlannerNode::GridDistancesCallback, this, std::placeholders::_1));
    tlr_msg_subscriber = this->create_subscription<base_interfaces::msg::Align>("d455/align",10, 
                std::bind(&actionPlannerNode::TlrCallback, this, std::placeholders::_1));
    tlrkfs_msg_subscriber = this->create_subscription<base_interfaces::msg::Pick>("d455/pick",10, 
                std::bind(&actionPlannerNode::TlrKFSCallback, this, std::placeholders::_1));
    imu_msg_subscriber = this->create_subscription<base_interfaces::msg::PosData>("/pps_data",10, 
                std::bind(&actionPlannerNode::PosCallback, this, std::placeholders::_1));
    grid_zhu_msg_subscriber = this->create_subscription<std_msgs::msg::Int32>("/encoded_grid_pythonqqq", 10, 
                std::bind(&actionPlannerNode::Grid_zhu_Callback, this, std::placeholders::_1));//10864712
    kfs_num_msg_subscriber = this->create_subscription<std_msgs::msg::Int32>("/selected_value", 10, 
                std::bind(&actionPlannerNode::KfsNumCallback, this, std::placeholders::_1));
    encoded_grid_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/encoded_grid", 10);
    lx_choose_publisher = this->create_publisher<base_interfaces::msg::CameraChoose>("/camera_mode", 10);
    lx_kfs_publisher = this->create_publisher<base_interfaces::msg::D455Task>("/d455task", 10);
    yyy_change_publisher = this->create_publisher<base_interfaces::msg::PoseCorrection>("/pose_correction", 10);
    lx_posz_publisher = this->create_publisher<base_interfaces::msg::D455Posz>("/d455posz", 10);
    lyx_publisher = this->create_publisher<base_interfaces::msg::LidarState>("/site", 10);
    
    this->declare_parameter<std::string>("mode", "run");

    // 读取参数
    mode = this->get_parameter("mode").as_string();

    assert(mode == "run" || mode == "self-check" || mode == "re_area2" || mode == "re_area3" || mode == "test");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&actionPlannerNode::timer_callback, this),
        timer_callback_group_
    );
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // TfTimer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(5),
    //         std::bind(&actionPlannerNode::Tf_Callback, this));
    // // 里程计订阅（订阅 /map_to_base_link，回调为类成员）

    // 控制/校准用的是融合后的里程计位姿，不是原始传感器流；保留可靠传输，只保留最新一帧避免积压旧位姿。
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/map_to_base_link", odom_qos,
        std::bind(&actionPlannerNode::odom_callback, this, std::placeholders::_1),
        odom_options);

    rclcpp::QoS qos(50);
    qos.best_effort();

    odom_highfreq_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odin1/odometry_highfreq",
        qos,
        std::bind(&actionPlannerNode::odom_highfreq_callback, this, std::placeholders::_1));

    action_plan_client_ = rclcpp_action::create_client<base_interfaces::action::ActionPlan>(
        this,
        "action_plan"
    );
    // 等待 Action Server
    if (!action_plan_client_->wait_for_action_server(std::chrono::seconds(5))) {
        // logger_->error("Action server not available!");
    } 
    else 
    {
        // logger_->info("Connected to action server");
    }
}

void actionPlannerNode::odom_highfreq_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    const auto &p = msg->pose.pose.position;
    const auto &o = msg->pose.pose.orientation;

    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

    static double last_stamp_sec = -1.0;
    if (last_stamp_sec >= 0.0) {
        const double dt = stamp_sec - last_stamp_sec;
        if (dt > 0.1) {
            RCLCPP_WARN(this->get_logger(), "Drop odom_highfreq: stamp gap %.6fs (>0.1s) last=%.6f cur=%.6f", dt, last_stamp_sec, stamp_sec);
            return;
        }
    }
    last_stamp_sec = stamp_sec;

    const std::string odom_frame = msg->header.frame_id.empty() ? std::string("odom") : msg->header.frame_id;
    geometry_msgs::msg::TransformStamped transformStamped;

    // 只保留：把里程计 odom 下位姿通过一个 TF 转到 map 下
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg->header;
    pose_in.header.frame_id = odom_frame;
    pose_in.pose.position = p;
    pose_in.pose.orientation = o;

    geometry_msgs::msg::PoseStamped pose_in_map;
    try {
        // 尽量用消息时间，避免 TimePointZero 带来的时序误差
        transformStamped = tf_buffer_->lookupTransform("map", odom_frame, msg->header.stamp);
        tf2::doTransform(pose_in, pose_in_map, transformStamped);
        // RCLCPP_INFO(this->get_logger(), "TF lookup succeeded (map<- %s @ stamp)", odom_frame.c_str());
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed (map<- %s @ stamp): %s; fallback to latest", odom_frame.c_str(), ex.what());
        try {
            transformStamped = tf_buffer_->lookupTransform("map", odom_frame, tf2::TimePointZero);
            tf2::doTransform(pose_in, pose_in_map, transformStamped);
        } catch (tf2::TransformException &ex2) {
            // RCLCPP_ERROR(this->get_logger(), "TF failed (map<- %s latest): %s", odom_frame.c_str(), ex2.what());
            return;
        }
    }

    const Eigen::Quaterniond q_map(
        pose_in_map.pose.orientation.w,
        pose_in_map.pose.orientation.x,
        pose_in_map.pose.orientation.y,
        pose_in_map.pose.orientation.z);
    const Eigen::Matrix3d rot = q_map.toRotationMatrix();
    // T lidar_init->lidar
    Eigen::Matrix4d T_lidar_init_lidar = Eigen::Matrix4d::Identity();
    T_lidar_init_lidar.block<3,3>(0,0) = rot;
    T_lidar_init_lidar(0,3) = pose_in_map.pose.position.x;
    T_lidar_init_lidar(1,3) = pose_in_map.pose.position.y;
    T_lidar_init_lidar(2,3) = pose_in_map.pose.position.z;
    // T car->lidar
    Eigen::Matrix4d T_car_lidar = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_car_lidar = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_car_lidar.block<3,3>(0,0) = R_car_lidar;
    T_car_lidar(0,3) = -0.02901;
    T_car_lidar(1,3) = -0.32853;
    // T lidar->car
    const Eigen::Matrix4d T_lidar_car = T_car_lidar.inverse();
    // T lidar_init->car = T lidar_init->lidar * T lidar->car
    const Eigen::Matrix4d T_lidar_init_car = T_lidar_init_lidar * T_lidar_car;
    const Eigen::Matrix3d R_lidar_init_car = T_lidar_init_car.block<3,3>(0,0);
    const Eigen::Vector3d t_lidar_init_car = T_lidar_init_car.block<3,1>(0,3);
    // R map->lidar_init
    const Eigen::Matrix3d R_map_lidar_init = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // t map->lidar_init
    const Eigen::Vector3d t_map_lidar_init = Eigen::Vector3d(0.38551, 9.42003, 0.0);
    // t map->car = R map->lidar_init * t lidar_init->car + t map->lidar_init
    const Eigen::Vector3d t_map_car = R_map_lidar_init * t_lidar_init_car + t_map_lidar_init;
    const Eigen::Matrix3d R_map_car = R_map_lidar_init * R_lidar_init_car;
    
    double x_center = t_map_car.x();
    double y_center = t_map_car.y();

    const Eigen::Vector3d ypr_map_car = R_map_car.eulerAngles(2, 1, 0);
    const double yaw_map_car = atan2(R_map_car(1,0), R_map_car(0,0));
    const double pitch_map_car = asin(-R_map_car(2,0));
    const double roll_map_car = atan2(R_map_car(2,1), R_map_car(2,2));
    const float yaw_deg = static_cast<float>(yaw_map_car / M_PI * 180.0); // 航向角


    // RCLCPP_INFO(this->get_logger(), "High-freq center in map: (%.2f, %.2f)", pose_in_map.pose.position.x, pose_in_map.pose.position.y);

    // if (odin1_record_ && odin1_record_->recordfile.is_open()) {
    //     odin1_record_->recordfile << std::fixed << std::setprecision(2)
    //                               << "High-freq center in map: (" << pose_in_map.pose.position.x << ", "
    //                               << pose_in_map.pose.position.y << ")\n";
    // }

    const float x_mm = static_cast<float>(x_center * 1000.0);
    const float y_mm = static_cast<float>(y_center * 1000.0);
    const float z_mm = static_cast<float>(pose_in_map.pose.position.z * 1000.0);

    {
        std::lock_guard<std::mutex> lock(odom_pose_mtx_);
        odom_nowPos_.yaw = yaw_deg;
        odom_nowPos_.x = x_mm;
        odom_nowPos_.y = y_mm;
        odom_nowPos_.z = z_mm;
    }

    // RCLCPP_INFO(this->get_logger(), "High-freq center: (%.2f, %.2f) yaw=%.1f stamp=%.6f", x_mm, y_mm, yaw_deg, stamp_sec);

    if (odin1_record_ && odin1_record_->recordfile.is_open()) {
        odin1_record_->recordfile << std::fixed << std::setprecision(6)
                                  << "High-freq origin: (" << pose_in_map.pose.position.x << ", " << pose_in_map.pose.position.y << ")\t"
                                  << "trans: (" << x_center << ", " << y_center << ")\t"
                                  << std::setprecision(1) << "yaw=" << yaw_deg << " "
                                  << std::setprecision(6) << "stamp=" << stamp_sec << "\n";
    }
}

void actionPlannerNode::timer_callback()
{
    time_5msFlag.store(true, std::memory_order_release);
}

void actionPlannerNode::PublishToYyy(float x, float y, float z, float roll, float pitch, float yaw, bool need_correction)
{
    dataToYyy.pose.position.x = x / 1000.0; 
    dataToYyy.pose.position.y = y / 1000.0; 
    dataToYyy.pose.position.z = z / 1000.0; 
    dataToYyy.need_correction = need_correction;
    dataToYyy.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(1,0,0), roll * M_PI / 180.0) *
                                            tf2::Quaternion(tf2::Vector3(0,1,0), pitch * M_PI / 180.0) *
                                            tf2::Quaternion(tf2::Vector3(0,0,1), yaw * M_PI / 180.0));

    this->yyy_change_publisher->publish(dataToYyy);
}

void actionPlannerNode::PublishToLyx()
{
    mtx.lock();
    dataToLyx.field_color = plannerNode->myFlags.RB_flag;
    dataToLyx.field_num = plannerNode->myFlags.field_num;
    mtx.unlock();
    this->lyx_publisher->publish(dataToLyx);
}

void actionPlannerNode::PublishToLx()
{
    mtx.lock();
    dataToLx.task = plannerNode->lxData.task;
    mtx.unlock();
    this->lx_task_publisher->publish(dataToLx);
}

void actionPlannerNode::PublishToLxChoose()
{
    mtx.lock();
    dataToLxChoose.camera = plannerNode->lxData.mode;
    mtx.unlock();
    this->lx_choose_publisher->publish(dataToLxChoose);
}

void actionPlannerNode::PublishToLxKfs()
{
    // this->logger_->info("Send kfs data");
    mtx.lock();
    dataToLxKfs.start_test = plannerNode->lxData.start_test;
    mtx.unlock();
    this->lx_kfs_publisher->publish(dataToLxKfs);
}

void actionPlannerNode::PublishToLxPosz()
{
    // this->logger_->info("Send posz data");
    mtx.lock();
    dataTolxposz.z = plannerNode->lxData.lx_posz;
    mtx.unlock();
    this->lx_posz_publisher->publish(dataTolxposz);
}

void actionPlannerNode::publish_encoded_grid(int32_t encoded_value) 
{
    auto msg = std_msgs::msg::Int32();
    msg.data = encoded_value;
    encoded_grid_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published encoded grid: %d", encoded_value);
}

void actionPlannerNode::send_goal(const base_interfaces::action::ActionPlan::Goal& goal)
{
    // this->logger_->info("Send goal");
    auto goal_msg = base_interfaces::action::ActionPlan::Goal();
    // 填充消息
    goal_msg = goal;

    // 注册回调
    rclcpp_action::Client<base_interfaces::action::ActionPlan>::SendGoalOptions options;
    options.goal_response_callback = 
        std::bind(&actionPlannerNode::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = 
        std::bind(&actionPlannerNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = 
        std::bind(&actionPlannerNode::result_callback, this, std::placeholders::_1);

    // logger_->info("Sending goal: {} {}", goal_msg.target_pose.position.x, goal_msg.target_pose.position.y);
    action_plan_client_->async_send_goal(goal_msg, options);
}

void actionPlannerNode::goal_response_callback(const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        logger_->error("Goal was rejected");
        return;
    }

    // logger_->info("Goal accepted");
}

void actionPlannerNode::feedback_callback(
    const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::SharedPtr& goal_handle,
    const std::shared_ptr<const base_interfaces::action::ActionPlan::Feedback> feedback)
{
    // logger_->info("Received feedback");
    (void)goal_handle;
    (void)feedback;
    // 处理反馈
}

void actionPlannerNode::LxCallback(const base_interfaces::msg::Camera::SharedPtr msg)
{
    // logger_->info("Received Lx data");
    mtx.lock();
    plannerNode->dataFromLx = *msg;
    mtx.unlock();
}

void actionPlannerNode::KfsCallback(const base_interfaces::msg::CameraKfs::SharedPtr msg)
{
    // logger_->info("Received Kfs data");
    mtx.lock();
    plannerNode->dataFromLxKfs = *msg;
    mtx.unlock();
}

void actionPlannerNode::Grid_lx_Callback(const base_interfaces::msg::GridState::SharedPtr msg)
{
    // logger_->info("Received grid_lx data");
    mtx.lock();
    plannerNode->dataFromLxstate = *msg;
    mtx.unlock();
}

void actionPlannerNode::GridDistancesCallback(const base_interfaces::msg::GridDistances::SharedPtr msg)
{
    // logger_->info("Received grid_distances data");
    mtx.lock();
    plannerNode->dataFromGridDistances = *msg;
    mtx.unlock();
}

void actionPlannerNode::Grid_zhu_Callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    // logger_->info("Received grid_zhu data");
    mtx.lock();
    Grid decoded_result = decode_grid(msg->data);
    for(int i = 0; i < 4; i++) 
    {
        for(int j = 0; j < 3; j++) 
        {
            plannerNode->grid.data[i][j] = decoded_result.data[i][j];
        }
    }
    for (int i = 0; i < 4 ; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            KFS[i * 3 + j] = grid.data[i][j];
        }
    }
    mtx.unlock();
}

void actionPlannerNode::KfsNumCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    mtx.lock();
    plannerNode->re_area_num = msg->data;
    mtx.unlock();
}

void actionPlannerNode::TlrCallback(const base_interfaces::msg::Align::SharedPtr msg)
{
    // logger_->info("Received tlr data");
    mtx.lock();
    plannerNode->dataFromtlr= *msg;
    mtx.unlock();
}

void actionPlannerNode::TlrKFSCallback(const base_interfaces::msg::Pick::SharedPtr msg)
{
    // logger_->info("Received tlrkfs data");
    mtx.lock();
    plannerNode->dataFromtlrkfs= *msg;
    mtx.unlock();
}

void actionPlannerNode::result_callback(const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::WrappedResult& result)
{
    cnt++;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        // logger_->info("Goal succeeded");
    } else {
        logger_->error("Goal failed");
    }

    // Use Result snapshot as the state source (replaces previous feedBackFromCtrl-from-Feedback)
    if (result.result) {
        std::lock_guard<std::mutex> lock(mtx);
        feedBackFromCtrl = result.result->final_state;
    }
}
void actionPlannerNode::LoadToCtrlData()
{
    // ===== 腿 =====
    plannerGoalToCtrl.tarleg1.motor_vel = leg.LegOne.GetTarVel();
    plannerGoalToCtrl.tarleg1.motor_pos = leg.LegOne.GetTarPos();
    plannerGoalToCtrl.tarleg1.motor_pos_kp = leg.LegOne.GetTarPos_Kp();
    plannerGoalToCtrl.tarleg1.motor_pos_kd = leg.LegOne.GetTarPos_Kd();
    plannerGoalToCtrl.tarleg1.motor_vel_kp = leg.LegOne.GetTarVel_Kp();
    plannerGoalToCtrl.tarleg1.motor_vel_ki = leg.LegOne.GetTarVel_Ki();

    plannerGoalToCtrl.tarleg2.motor_vel = leg.LegTwo.GetTarVel();
    plannerGoalToCtrl.tarleg2.motor_pos = leg.LegTwo.GetTarPos();
    plannerGoalToCtrl.tarleg2.motor_pos_kp = leg.LegTwo.GetTarPos_Kp();
    plannerGoalToCtrl.tarleg2.motor_pos_kd = leg.LegTwo.GetTarPos_Kd();
    plannerGoalToCtrl.tarleg2.motor_vel_kp = leg.LegTwo.GetTarVel_Kp();
    plannerGoalToCtrl.tarleg2.motor_vel_ki = leg.LegTwo.GetTarVel_Ki();

    plannerGoalToCtrl.tarleg3.motor_vel = leg.LegThree.GetTarVel();
    plannerGoalToCtrl.tarleg3.motor_pos = leg.LegThree.GetTarPos();
    plannerGoalToCtrl.tarleg3.motor_pos_kp = leg.LegThree.GetTarPos_Kp();
    plannerGoalToCtrl.tarleg3.motor_pos_kd = leg.LegThree.GetTarPos_Kd();
    plannerGoalToCtrl.tarleg3.motor_vel_kp = leg.LegThree.GetTarVel_Kp();
    plannerGoalToCtrl.tarleg3.motor_vel_ki = leg.LegThree.GetTarVel_Ki();

    plannerGoalToCtrl.tarleg4.motor_vel = leg.LegFour.GetTarVel();
    plannerGoalToCtrl.tarleg4.motor_pos = leg.LegFour.GetTarPos();
    plannerGoalToCtrl.tarleg4.motor_pos_kp = leg.LegFour.GetTarPos_Kp();
    plannerGoalToCtrl.tarleg4.motor_pos_kd = leg.LegFour.GetTarPos_Kd();
    plannerGoalToCtrl.tarleg4.motor_vel_kp = leg.LegFour.GetTarVel_Kp();
    plannerGoalToCtrl.tarleg4.motor_vel_ki = leg.LegFour.GetTarVel_Ki();

    // ===== sword =====
    plannerGoalToCtrl.tarslide.motor_pos = sword.Slide.GetTarPos();

    plannerGoalToCtrl.tarplate.motor_vel = sword.Plate.GetTarVel();
    plannerGoalToCtrl.tarplate.motor_cur = sword.Plate.GetTarTor();
    plannerGoalToCtrl.tarplate.motor_pos_kp = sword.Plate.GetTarPos_Kp();
    plannerGoalToCtrl.tarplate.motor_pos_kd = sword.Plate.GetTarPos_Kd();

    plannerGoalToCtrl.tararm1.motor_pos = sword.Arm1.GetTarPos();
    plannerGoalToCtrl.tararm1.motor_pos_kp = sword.Arm1.GetTarPos_Kp();
    plannerGoalToCtrl.tararm1.motor_pos_kd = sword.Arm1.GetTarPos_Kd();
    plannerGoalToCtrl.tararm1.motor_vel_kp = sword.Arm1.GetTarVel_Kp();
    plannerGoalToCtrl.tararm1.motor_vel_ki = sword.Arm1.GetTarVel_Ki();

    plannerGoalToCtrl.tararm2.motor_pos = sword.Arm2.GetTarPos();
    plannerGoalToCtrl.tararm2.motor_pos_kp = sword.Arm2.GetTarPos_Kp();
    plannerGoalToCtrl.tararm2.motor_pos_kd = sword.Arm2.GetTarPos_Kd();
    plannerGoalToCtrl.tararm2.motor_vel_kp = sword.Arm2.GetTarVel_Kp();
    plannerGoalToCtrl.tararm2.motor_vel_ki = sword.Arm2.GetTarVel_Ki();
    plannerGoalToCtrl.mode = mode;
    plannerGoalToCtrl.start = start;
}

    // // ===== pcflag  =====
    // plannerGoalToCtrl.pcflag.run_successed_flag = myflag.getPosFlag;;



/// @brief 收到定位系统的数后进行储存
/// @param msg 
void actionPlannerNode::PosCallback(const base_interfaces::msg::PosData::SharedPtr msg)
{
    // logger_->info("Received PPS data");
    mtx.lock();
    plannerNode->posData = *msg;
    mtx.unlock();
}

void actionPlannerNode::SendToCtrl()
{
    LoadToCtrlData();
    send_goal(plannerGoalToCtrl);
}

void actionPlannerNode::Tf_Callback()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        nowPos.x = transformStamped.transform.translation.x * 1000;
        nowPos.y = transformStamped.transform.translation.y * 1000;
        nowPos.z = transformStamped.transform.translation.z * 1000;
        std::cout << "Position: (" << nowPos.x << ", " << nowPos.y << ", " << nowPos.z << ", " << transformStamped.transform.rotation.w << ")\n";
        // 提取旋转角度（假设只需要绕Z轴的旋转）
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        static float last_yaw = 0.0;
        nowPos.yaw = yaw / M_PI * 180.f; // 航向角
        nowPos.omega = (yaw - last_yaw) / 0.005;
        last_yaw = yaw;
        nowPos.pitch = pitch / M_PI * 180.f;
        nowPos.roll  = roll / M_PI * 180.f;
        cout << "rpy: " << nowPos.roll << " " << nowPos.pitch << " " <<nowPos.yaw << endl;
    }
    catch (tf2::TransformException &ex) {
        logger_->error("Could not transform odom to base_link: {}", ex.what());
    }
}
// 里程计回调（类成员）
void actionPlannerNode::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    static rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

    const auto &p = msg->pose.pose.position;
    const auto &o = msg->pose.pose.orientation;

    nowPos.lidar_x = static_cast<float>(p.x * 1000.0f);  // m -> mm
    nowPos.lidar_y = static_cast<float>(p.y * 1000.0f);
    nowPos.z = static_cast<float>(p.z * 1000.0f);

    static bool has_last_odom = false;
    static float last_lidar_x = 0.0f;
    static float last_lidar_y = 0.0f;
    static float last_lidar_yaw = 0.0f;
    static double last_odom_time = 0.0;
    const double odom_time = rclcpp::Time(msg->header.stamp).seconds();
    double dt = odom_time - last_odom_time;
    if (!has_last_odom || dt <= 1e-6 || !std::isfinite(dt)) {
        dt = 0.005;
    }

    // 优先使用消息中的线速度，如果不可用则回退到雷达原始位置差分法。
    const auto &tw = msg->twist.twist;
    if (std::isfinite(tw.linear.x) && std::isfinite(tw.linear.y)) {
        nowPos.lidar_vx = static_cast<float>(tw.linear.x * 1000.0);
        nowPos.lidar_vy = static_cast<float>(tw.linear.y * 1000.0);
    } else {
        nowPos.lidar_vx = has_last_odom ? static_cast<float>((nowPos.lidar_x - last_lidar_x) / dt) : 0.0f;
        nowPos.lidar_vy = has_last_odom ? static_cast<float>((nowPos.lidar_y - last_lidar_y) / dt) : 0.0f;
    }

    // 计算旋转角（从四元数 -> RPY）
    tf2::Quaternion q(o.x, o.y, o.z, o.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 保存到缓存（度）
    nowPos.roll  = static_cast<float>(roll  / M_PI * 180.0);
    nowPos.pitch = static_cast<float>(pitch / M_PI * 180.0);
    nowPos.lidar_yaw  = static_cast<float>(yaw   / M_PI * 180.0);
    //nowPos.yaw = -0.0;
    // 优先使用 odom 角速度，如果不可用则对 lidar_yaw 差值做归一化差分。
    if (std::isfinite(tw.angular.z)) {
        nowPos.lidar_omega = static_cast<float>(tw.angular.z * 180.0 / M_PI);
    } else {
        const float dy = normalize_angle_deg(nowPos.lidar_yaw - last_lidar_yaw);
        nowPos.lidar_omega = has_last_odom ? static_cast<float>(dy / dt) : 0.0f;
    }
    last_lidar_x = nowPos.lidar_x;
    last_lidar_y = nowPos.lidar_y;
    last_lidar_yaw = nowPos.lidar_yaw;
    last_odom_time = odom_time;
    has_last_odom = true;
    static int lidar_err_cnt = 0;
    if(fabs(nowPos.vx) >= 50000.0 || fabs(nowPos.vy) >= 50000)
    {
        if(lidar_err_cnt < 1){
            lidar_err_cnt ++;
        }else
            plannerNode->myFlags.ppsErrorFlag = true;
    }
    plannerNode->lidar_time = msg->header.stamp.nanosec / 1e6f; // 转换为毫秒
    static bool first_odom = true;
    if (first_odom) {
        first_odom = false;
        correctWithSwitch(nowPos.lidar_x, nowPos.lidar_y, nowPos.lidar_yaw, nowPos.lidar_x, nowPos.lidar_y, nowPos.lidar_yaw);
    }
    LidarToSwitchCorrected(nowPos.lidar_x, nowPos.lidar_y, nowPos.lidar_yaw,nowPos.x, nowPos.y, nowPos.yaw);

    // if(plannerNode->correct_pps_flag){
    //     plannerNode->correct_pps_flag = false;
    //     correctWithLidar(nowPos.pps_x, nowPos.pps_y, nowPos.pps_yaw, nowPos.lidar_corr_x, nowPos.lidar_corr_y, nowPos.lidar_corr_yaw);
    // }
    // 记录时间差（当前时间与消息时间）
    // rclcpp::Time time_now = this->get_clock()->now();
    // double t_now = time_now.seconds();
    // double t_msg = rclcpp::Time(msg->header.stamp).seconds();
    // double dt_ms = (t_now - t_msg) * 1000.0;

}

void actionPlannerNode::Vavle_Upper_Ctrl(bool vavle1, bool vavle2, bool vavle3, bool vavle4)
{
    this->Vavle_Num[0] = vavle1;//倒
    this->Vavle_Num[1] = vavle2;//伸
    this->Vavle_Num[2] = vavle3;//夹
    this->Vavle_Num[3] = vavle4;//压块
}

void actionPlannerNode::Vavle_Chassis_Ctrl(bool vavle5, bool vavle6)
{
    this->Vavle_Num[4] = vavle5;//缓冲
    this->Vavle_Num[6] = vavle6;//从动
}

void actionPlannerNode::Led_Ctrl(bool vavle8, bool vavle9)
{
    this->Vavle_Num[7] = vavle8;
    this->Vavle_Num[8] = vavle9;
}

void actionPlannerNode::Raw_Ctrl(bool vavle10)
{
    this->Vavle_Num[9] = vavle10;
}
