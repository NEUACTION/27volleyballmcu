/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:30:27
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2026-01-20 23:31:10
 * @FilePath: /ROBOCON2026_base/src/action_planner/include/action_planner/action_planner_node.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _ACTION_PLANNER_NODE_H_
#define _ACTION_PLANNER_NODE_H_

#include <atomic>
#include <mutex>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "base_interfaces/action/action_plan.hpp"
#include "base_interfaces/msg/action_plan_state.hpp"
#include "base_interfaces/msg/camera.hpp"
#include "base_interfaces/msg/camera_kfs.hpp"
#include "base_interfaces/msg/d455_task.hpp"
#include "base_interfaces/msg/choose_weapon.hpp"
#include "base_interfaces/msg/pos_data.hpp"
#include "base_interfaces/msg/camera_task.hpp"
#include "base_interfaces/msg/align.hpp"
#include "base_interfaces/msg/camera_choose.hpp"
#include "base_interfaces/msg/pose_correction.hpp"
#include "base_interfaces/msg/grid_state.hpp"
#include "base_interfaces/msg/pick.hpp"
#include "base_interfaces/msg/grid_distances.hpp"
#include "base_interfaces/msg/d455_posz.hpp"
#include "base_interfaces/msg/lidar_state.hpp"
#include "common_logger/common_logger.hpp"
#include "action_planner/fpath.hpp"
#include "action_planner/task.hpp"
#include "action_planner/frecord.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/int32.hpp"
#include "Eigen/Dense"

using namespace Eigen;

#define YES_R1
// #define NO_R1
extern bool start;

class actionPlannerNode: public rclcpp::Node 
{
public:
    actionPlannerNode();
    ~actionPlannerNode() noexcept override = default;

    std::shared_ptr<common_logger::Logger> logger() const { return logger_; }
    
    std::atomic_bool time_5msFlag{false};
    bool Ps_Num[22];
    bool Vavle_Num[10];
    bool throw_flag = 0;
    int KFS[12] = {0};
    int re_area_num = -1;

    int lidar_time = 0;

    upper_task UpperTask = pass_upper;
    chassis_task ChassisTask = pass_chassis;
    chassis_task LastChassisTask = pass_chassis;
    upper_work upperwork = noupperwork;
    chassis_work chassiswork = nochassiswork;
    chassis_work lastchassiswork = nochassiswork;
    step mystep = nostep;
    ctrl_mode ctrlMode = AUTO_MODE;
    vel_data_t tarVel;
    vel_data_t actVel;
    pos_data_t nowPos;
    pos_data_t tarPos;
    pos_data_t ppsPos;
    base_interfaces::msg::HeartBag heartBags;
    base_interfaces::msg::Flag myFlags;
    base_interfaces::msg::Camera dataFromLx;
    base_interfaces::msg::CameraKfs dataFromLxKfs;
    base_interfaces::msg::GridState dataFromLxstate;
    base_interfaces::msg::Align dataFromtlr;
    base_interfaces::msg::Pick dataFromtlrkfs;
    // HeartBag heartBags;
    // Lrz lrz;
    // Flag myFlags;
    pathToFollow mypaths;
    // lx_data_t lxData;
    // zhu_data_t zhuData;
    // Grid grid;
    // tlr_data_t tlrData;
    // tlrkfs_data_t tlrkfsData;
    // R1StarData_t R1StarData;

    std::string mode;

    base_interfaces::action::ActionPlan::Goal plannerGoalToCtrl;
    base_interfaces::msg::ActionPlanState feedBackFromCtrl;
    // base_interfaces::msg::Camera dataFromLx;
    // base_interfaces::msg::CameraKfs dataFromLxKfs;
    // base_interfaces::msg::GridState dataFromLxstate;
    base_interfaces::msg::GridDistances dataFromGridDistances;
    // base_interfaces::msg::Align dataFromtlr;
    // base_interfaces::msg::CameraTask dataToLx;
    // base_interfaces::msg::CameraChoose dataToLxChoose;
    // base_interfaces::msg::D455Task dataToLxKfs;
    base_interfaces::msg::PosData posData;
    // base_interfaces::msg::PoseCorrection dataToYyy;
    // base_interfaces::msg::Pick dataFromtlrkfs;
    // base_interfaces::msg::D455Posz dataTolxposz;
    // base_interfaces::msg::LidarState dataToLyx;

    void mainThread();
    void LoadToCtrlData();
    void SendToCtrl();
    void Tf_Callback();
    void Vavle_Upper_Ctrl(bool vavle1, bool vavle2, bool vavle3, bool vavle4);
    void Vavle_Chassis_Ctrl(bool vavle5, bool vavle6);
    void Led_Ctrl(bool vavle8, bool vavle9);
    void Raw_Ctrl(bool vavle10);
    void PublishToLxChoose();
    void PublishToLxKfs();
    void PublishToYyy(float x, float y, float z, float roll, float pitch, float yaw, bool need_correction);
    void PublishToLxPosz();
    void publish_encoded_grid(int32_t encoded_value);

    std::shared_ptr<common_logger::Logger> Logger() { return logger_; }

    //里程计消息订阅（在构造函数中创建）
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_highfreq_subscriber_;
    void odom_highfreq_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    unsigned long long cnt = 0;

private:

    rclcpp::TimerBase::SharedPtr TfTimer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr pps_callback_group_;
    
    std::shared_ptr<common_logger::Logger> logger_;

    std::shared_ptr<frecord> odin1_record_;

    std::mutex odom_pose_mtx_;
    pos_data_t odom_nowPos_{};

    rclcpp_action::Client<base_interfaces::action::ActionPlan>::SharedPtr action_plan_client_;
    rclcpp::Subscription<base_interfaces::msg::Camera>::SharedPtr lx_msg_subscriber;
    rclcpp::Subscription<base_interfaces::msg::CameraKfs>::SharedPtr kfs_msg_subscriber;
    rclcpp::Subscription<base_interfaces::msg::GridState>::SharedPtr grid_lx_msg_subscriber;
    rclcpp::Subscription<base_interfaces::msg::GridDistances>::SharedPtr grid_distances_subscriber;
    rclcpp::Subscription<base_interfaces::msg::Align>::SharedPtr tlr_msg_subscriber;
    rclcpp::Subscription<base_interfaces::msg::Pick>::SharedPtr tlrkfs_msg_subscriber;
    rclcpp::Subscription<base_interfaces::msg::PosData>::SharedPtr imu_msg_subscriber;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grid_zhu_msg_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_num_msg_subscriber;
    rclcpp::Publisher<base_interfaces::msg::CameraTask>::SharedPtr lx_task_publisher;
    rclcpp::Publisher<base_interfaces::msg::CameraChoose>::SharedPtr lx_choose_publisher;
    rclcpp::Publisher<base_interfaces::msg::D455Task>::SharedPtr lx_kfs_publisher;
    rclcpp::Publisher<base_interfaces::msg::PoseCorrection>::SharedPtr yyy_change_publisher;
    rclcpp::Publisher<base_interfaces::msg::D455Posz>::SharedPtr lx_posz_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoded_grid_publisher_;
    rclcpp::Publisher<base_interfaces::msg::LidarState>::SharedPtr lyx_publisher;

    void send_goal(const base_interfaces::action::ActionPlan::Goal& goal);
    void PublishToLx();
    void PublishToLyx();

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::SharedPtr& goal_handle);
    void feedback_callback(const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::SharedPtr& goal_handle,
                           const std::shared_ptr<const base_interfaces::action::ActionPlan::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<base_interfaces::action::ActionPlan>::WrappedResult& result);
    void LxCallback(const base_interfaces::msg::Camera::SharedPtr msg);
    void KfsCallback(const base_interfaces::msg::CameraKfs::SharedPtr msg);
    void Grid_lx_Callback(const base_interfaces::msg::GridState::SharedPtr msg);
    // void Grid_zhu_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    void KfsNumCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void GridDistancesCallback(const base_interfaces::msg::GridDistances::SharedPtr msg);

    void TlrCallback(const base_interfaces::msg::Align::SharedPtr msg);
    void TlrKFSCallback(const base_interfaces::msg::Pick::SharedPtr msg);
    void PosCallback(const base_interfaces::msg::PosData::SharedPtr msg);

    
    void timer_callback();
};

extern std::shared_ptr<actionPlannerNode> plannerNode;
extern std::mutex mtx;

// 角度 -> 弧度
static inline float deg2rad(float deg)
{
    return deg * (float)M_PI / 180.0f;
}

// 弧度 -> 角度
static inline float rad2deg(float rad)
{
    return rad * 180.0f / (float)M_PI;
}

// 角度归一化到 [-180, 180]
static inline float normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void correctWithLidar(float pps_x, float pps_y, float pps_yaw,
                      float lidar_x, float lidar_y, float lidar_yaw);
void correctWithSwitch(float lidar_x, float lidar_y, float lidar_yaw,
                      float switch_x, float switch_y, float switch_yaw);
void ppsToLidarCorrected(float pps_x, float pps_y, float pps_yaw_deg,
                float &lidar_x, float &lidar_y, float &lidar_z, float &lidar_yaw_deg);
void ppsVelToLidarCorrected(float pps_vx, float pps_vy, float pps_wz,
                            float &lidar_vx, float &lidar_vy, float &lidar_wz);
void LidarToSwitchCorrected(float lidar_x, float lidar_y, float lidar_yaw,
                         float &switch_x, float &switch_y, float &switch_yaw);
void correctLidar(float trueX,float trueY,float trueYaw);

#endif