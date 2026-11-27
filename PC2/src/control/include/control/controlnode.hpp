/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-03 22:04:43
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2025-11-11 20:14:49
 * @FilePath: /ROBOCON2026_base/src/control/include/control/controlnode.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef _CONTROL_NODE_H_
#define _CONTROL_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "math.h"
#include "base_interfaces/msg/mcu_receive.hpp"
#include "base_interfaces/msg/mcu_transmit.hpp"
#include "base_interfaces/msg/pos_data.hpp"
#include "base_interfaces/msg/action_plan_state.hpp"
#include "base_interfaces/action/action_plan.hpp"
#include "common_logger/common_logger.hpp"
#include "control/ppsCom.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "control/movement.hpp"


class controlnode : public rclcpp::Node {
public:
    controlnode();

    ~controlnode();

    void controlMain(void);

    base_interfaces::msg::McuTransmit dataToMcu;
    base_interfaces::msg::McuReceive  dataFromMcu;
    base_interfaces::action::ActionPlan::Goal plannerGoalFromPlanner;
    base_interfaces::msg::ActionPlanState feedBackToPlanner;

    uint64_t ppsPeriod;
    Pos_t* pps_data;

    int mode;
    
    bool GetActiveFlag();
    void Publish_control(bool ppsresetflag, bool course, bool zeroflag, bool restartflag);
    void PublishToMCU();
    void LoadDataToMcu();
    void LoadDataFeedBack();

private:
    rclcpp_action::Server<base_interfaces::action::ActionPlan>::SharedPtr action_plan_server_;

    rclcpp::Subscription<base_interfaces::msg::McuReceive>::SharedPtr mcu_msg_subscriber;
    rclcpp::Publisher<base_interfaces::msg::McuTransmit>::SharedPtr control_msg;
    rclcpp::Subscription<base_interfaces::msg::PosData>::SharedPtr pps_msg_subscriber;
    rclcpp::TimerBase::SharedPtr timer_;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const base_interfaces::action::ActionPlan::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle);
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle);
    
    void McuListenCallback(const base_interfaces::msg::McuReceive::SharedPtr);
    // void WheelPPSListenCallback(const base_interfaces::msg::PosData::SharedPtr msg);
    bool activeFlag = true;
    Pos_t* last_pos_data;
    //std::unique_ptr<ParamsConfig> ParamsConfigPtr_;

    std::shared_ptr<common_logger::Logger> logger_;

    // 状态跟踪
    std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> current_goal_handle_;
    std::mutex goal_handle_mutex_;
};

extern std::shared_ptr<controlnode> control_node;

#endif