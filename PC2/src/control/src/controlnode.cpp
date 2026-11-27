#include "control/controlnode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "base_interfaces/msg/action_plan_state.hpp"
#include "control/chassis.hpp"

using namespace std::chrono_literals;


controlnode::controlnode(): Node("controlNode") , activeFlag(true)
{
    logger_ = common_logger::Logger::getInstance("controlnode");

    action_plan_server_ = rclcpp_action::create_server<base_interfaces::action::ActionPlan>(
        this,
        "action_plan",
        std::bind(&controlnode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&controlnode::handle_cancel, this, std::placeholders::_1),
        std::bind(&controlnode::handle_accepted, this, std::placeholders::_1)
    );
    
    //创建一个从mcu节点收取mcu数据的订阅
    mcu_msg_subscriber = this->create_subscription<base_interfaces::msg::McuReceive>("/mcu/receive", 10, \
                std::bind(&controlnode::McuListenCallback, this, std::placeholders::_1));

    //创建一个发布控制数据到MCU的发布者
    control_msg = this->create_publisher<base_interfaces::msg::McuTransmit>("/mcu/transmit", 10);

    //创建一个从wheel_position收数的订阅目前放于MCU节点。
    // pps_msg_subscriber = this->create_subscription<base_interfaces::msg::PosData>("/ops_data", 10, \
    //             std::bind(&controlnode::WheelPPSListenCallback, this, std::placeholders::_1));

    
    //创建一个pos_t
    pps_data = new Pos_t();
    last_pos_data = new Pos_t();

    rcl_interfaces::msg::ParameterDescriptor descriptor1;
    rcl_interfaces::msg::FloatingPointRange para1Range;

    std::cout << "control node initialized!" << std::endl;
}




void DataHandler();
void controlnode::LoadDataFeedBack()
{
    // ===== 轮子 =====
    feedBackToPlanner.tarwheel1.pos = Wheel1.GetTarTurnPos();
    feedBackToPlanner.actwheel1.pos = dataFromMcu.wheel1.pos;
    feedBackToPlanner.tarwheel1.vel = Wheel1.GetTarRunVel();
    feedBackToPlanner.actwheel1.vel = dataFromMcu.wheel1.vel;
    feedBackToPlanner.actwheel1.pos_cur = dataFromMcu.wheel1.pos_cur;
    feedBackToPlanner.actwheel1.vel_cur = dataFromMcu.wheel1.vel_cur;

    feedBackToPlanner.tarwheel2.pos = Wheel2.GetTarTurnPos();
    feedBackToPlanner.actwheel2.pos = dataFromMcu.wheel2.pos;
    feedBackToPlanner.tarwheel2.vel = Wheel2.GetTarRunVel();
    feedBackToPlanner.actwheel2.vel = dataFromMcu.wheel2.vel;
    feedBackToPlanner.actwheel2.pos_cur = dataFromMcu.wheel2.pos_cur;
    feedBackToPlanner.actwheel2.vel_cur = dataFromMcu.wheel2.vel_cur;

    feedBackToPlanner.tarwheel3.pos = Wheel3.GetTarTurnPos();
    feedBackToPlanner.actwheel3.pos = dataFromMcu.wheel3.pos;
    feedBackToPlanner.tarwheel3.vel = Wheel3.GetTarRunVel();
    feedBackToPlanner.actwheel3.vel = dataFromMcu.wheel3.vel;
    feedBackToPlanner.actwheel3.pos_cur = dataFromMcu.wheel3.pos_cur;
    feedBackToPlanner.actwheel3.vel_cur = dataFromMcu.wheel3.vel_cur;

    feedBackToPlanner.tarwheel4.pos = Wheel4.GetTarTurnPos();
    feedBackToPlanner.actwheel4.pos = dataFromMcu.wheel4.pos;
    feedBackToPlanner.tarwheel4.vel = Wheel4.GetTarRunVel();
    feedBackToPlanner.actwheel4.vel = dataFromMcu.wheel4.vel;
    feedBackToPlanner.actwheel4.pos_cur = dataFromMcu.wheel4.pos_cur;
    feedBackToPlanner.actwheel4.vel_cur = dataFromMcu.wheel4.vel_cur;



    // ===== 腿 =====
    feedBackToPlanner.actleg1.motor_pos = dataFromMcu.leg1.motor_pos;
    feedBackToPlanner.actleg1.motor_vel = dataFromMcu.leg1.motor_vel;
    feedBackToPlanner.actleg1.motor_cur = dataFromMcu.leg1.motor_cur;

    feedBackToPlanner.actleg2.motor_pos = dataFromMcu.leg2.motor_pos;
    feedBackToPlanner.actleg2.motor_vel = dataFromMcu.leg2.motor_vel;
    feedBackToPlanner.actleg2.motor_cur = dataFromMcu.leg2.motor_cur;

    feedBackToPlanner.actleg3.motor_pos = dataFromMcu.leg3.motor_pos;
    feedBackToPlanner.actleg3.motor_vel = dataFromMcu.leg3.motor_vel;
    feedBackToPlanner.actleg3.motor_cur = dataFromMcu.leg3.motor_cur;

    feedBackToPlanner.actleg4.motor_pos = dataFromMcu.leg4.motor_pos;
    feedBackToPlanner.actleg4.motor_vel = dataFromMcu.leg4.motor_vel;
    feedBackToPlanner.actleg4.motor_cur = dataFromMcu.leg4.motor_cur;

    // ===== 手臂：slide / plate / arm1 / arm2 =====
    feedBackToPlanner.actslide.motor_pos = dataFromMcu.slide.motor_pos;
    feedBackToPlanner.actslide.motor_vel = dataFromMcu.slide.motor_vel;
    feedBackToPlanner.actslide.motor_cur = dataFromMcu.slide.motor_cur;

    feedBackToPlanner.actplate.motor_pos = dataFromMcu.plate.motor_pos;
    feedBackToPlanner.actplate.motor_vel = dataFromMcu.plate.motor_vel;
    feedBackToPlanner.actplate.motor_cur = dataFromMcu.plate.motor_cur;

    feedBackToPlanner.actarm1.motor_pos = dataFromMcu.arm1.motor_pos;
    feedBackToPlanner.actarm1.motor_vel = dataFromMcu.arm1.motor_vel;
    feedBackToPlanner.actarm1.motor_cur = dataFromMcu.arm1.motor_cur;

    feedBackToPlanner.actarm2.motor_pos = dataFromMcu.arm2.motor_pos;
    feedBackToPlanner.actarm2.motor_vel = dataFromMcu.arm2.motor_vel;
    feedBackToPlanner.actarm2.motor_cur = dataFromMcu.arm2.motor_cur;

    // ===== heartbag =====
    feedBackToPlanner.heartbag.mcu_hb = dataFromMcu.heartbag.mcu_hb;

    feedBackToPlanner.heartbag.mcu_to_pc_hb = dataFromMcu.heartbag.mcu_to_pc_hb;

    feedBackToPlanner.heartbag.leg1_hb = dataFromMcu.heartbag.leg1_hb;
    feedBackToPlanner.heartbag.leg2_hb = dataFromMcu.heartbag.leg2_hb;
    feedBackToPlanner.heartbag.leg3_hb = dataFromMcu.heartbag.leg3_hb;
    feedBackToPlanner.heartbag.leg4_hb = dataFromMcu.heartbag.leg4_hb;

    feedBackToPlanner.heartbag.wheel1_hb = dataFromMcu.heartbag.wheel1_hb;
    feedBackToPlanner.heartbag.wheel2_hb = dataFromMcu.heartbag.wheel2_hb;
    feedBackToPlanner.heartbag.wheel3_hb = dataFromMcu.heartbag.wheel3_hb;
    feedBackToPlanner.heartbag.wheel4_hb = dataFromMcu.heartbag.wheel4_hb;

    feedBackToPlanner.heartbag.plate_hb = dataFromMcu.heartbag.plate_hb;
    feedBackToPlanner.heartbag.slide_hb = dataFromMcu.heartbag.slide_hb;
    feedBackToPlanner.heartbag.arm1_hb = dataFromMcu.heartbag.arm1_hb;
    feedBackToPlanner.heartbag.arm2_hb = dataFromMcu.heartbag.arm2_hb;

    // ===== ps / 计数 / flag =====
    feedBackToPlanner.ps_num1 = dataFromMcu.ps_num1;
    feedBackToPlanner.ps_num2 = dataFromMcu.ps_num2;
    feedBackToPlanner.ps_num3 = dataFromMcu.ps_num3;
    feedBackToPlanner.mcucnt = dataFromMcu.period_cnt_mcu;

    feedBackToPlanner.mcuflag.reset_flag = dataFromMcu.mcuflag.reset_flag;

    // ===== error =====
    feedBackToPlanner.actleg1.error = dataFromMcu.leg1.error;
    feedBackToPlanner.actleg2.error = dataFromMcu.leg2.error;
    feedBackToPlanner.actleg3.error = dataFromMcu.leg3.error;
    feedBackToPlanner.actleg4.error = dataFromMcu.leg4.error;

    feedBackToPlanner.actslide.error = dataFromMcu.slide.error;
    feedBackToPlanner.actplate.error = dataFromMcu.plate.error;
    feedBackToPlanner.actarm1.error = dataFromMcu.arm1.error;
    feedBackToPlanner.actarm2.error = dataFromMcu.arm2.error;

    // ===== 位置 / 标志 =====
    feedBackToPlanner.r1posx = dataFromMcu.r1posx;
    feedBackToPlanner.r1posy = dataFromMcu.r1posy;
    
    feedBackToPlanner.mcuflag.only_need_start_flag = dataFromMcu.mcuflag.only_need_start_flag;
    // feedBackToPlanner.r1platformflag = dataFromMcu.r1platformflag;
    // feedBackToPlanner.r1moveflag = dataFromMcu.r1moveflag;
    // feedBackToPlanner.r2goflag = dataFromMcu.r2goflag;
    // feedBackToPlanner.r1area3flag = dataFromMcu.r1area3flag;
    // feedBackToPlanner.r1platehigh = dataFromMcu.r1platehigh;
    // feedBackToPlanner.r1movedirection = dataFromMcu.r1movedirection;

    // feedBackToPlanner.the_way2_num = dataFromMcu.the_way2_num;
    // feedBackToPlanner.firstr2kfs = dataFromMcu.firstr2kfs;
    // feedBackToPlanner.dodger1 = dataFromMcu.dodger1;
    // feedBackToPlanner.r2putflag = dataFromMcu.r2putflag;

    // for (int i = 0; i < 5; i++)
    // {
    //     feedBackToPlanner.action[i] = dataFromMcu.action[i];
    // }
}

/// @brief 收到MCU的数后存到dataFromMcu里
/// @param msg 从MCU节点收到的数的指针
void controlnode::McuListenCallback(const base_interfaces::msg::McuReceive::SharedPtr msg)
{
    this->dataFromMcu = *msg;
}



bool controlnode::GetActiveFlag()
{
    return(activeFlag);
}



// action
rclcpp_action::GoalResponse controlnode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const base_interfaces::action::ActionPlan::Goal> goal)
{
    (void)uuid;
    // 检查是否已有正在执行的目标
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
        logger_->warn("Rejecting goal: another goal is already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // 可以添加更多的目标验证逻辑
    logger_->info("Accepting new goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}



rclcpp_action::CancelResponse controlnode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle)
{
    logger_->warn("Received request to cancel motion control goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}



void controlnode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle)
{
    // 在新的线程中执行目标
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    current_goal_handle_ = goal_handle;
    logger_->info("Starting execution of new goal");

    // 在新线程中执行，避免阻塞
    std::thread{
        std::bind(&controlnode::execute, this, std::placeholders::_1), 
        goal_handle
    }.detach();
}




void controlnode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces::action::ActionPlan>> goal_handle)
{
    logger_->info("Executing motion control goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<base_interfaces::action::ActionPlan::Feedback>();
    auto result = std::make_shared<base_interfaces::action::ActionPlan::Result>();


    plannerGoalFromPlanner = *goal;
    std::cout << "Executing goal: " << plannerGoalFromPlanner.tarleg1.motor_pos << std::endl;
    DataHandler();
    control_node->controlMain();

    LoadDataFeedBack();
    feedback->mcucnt = feedBackToPlanner.mcucnt;
    feedback->ps_num1 = feedBackToPlanner.ps_num1;

    goal_handle->publish_feedback(feedback);
    control_node->PublishToMCU();

    // 任务成功完成
    if (rclcpp::ok()) 
    {
        result->success = true;
        result->message = "Motion control completed";
        result->final_state = feedBackToPlanner;
        goal_handle->succeed(result);
        logger_->info("Motion control succeeded: {}", result->message);
    }
    
    // 清除当前目标句柄
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    current_goal_handle_.reset();
}




controlnode::~controlnode()
{
    delete pps_data;
    delete last_pos_data;
}



/// @brief 把要发给mcu的数装进dataToMcu里面
void controlnode::LoadDataToMcu()
{
    // ===== 腿 =====
    dataToMcu.leg1.motor_vel = plannerGoalFromPlanner.tarleg1.motor_vel;
    dataToMcu.leg1.motor_pos = plannerGoalFromPlanner.tarleg1.motor_pos;
    dataToMcu.leg1.motor_pos_kp = plannerGoalFromPlanner.tarleg1.motor_pos_kp;
    dataToMcu.leg1.motor_pos_kd = plannerGoalFromPlanner.tarleg1.motor_pos_kd;
    dataToMcu.leg1.motor_vel_kp = plannerGoalFromPlanner.tarleg1.motor_vel_kp;
    dataToMcu.leg1.motor_vel_ki = plannerGoalFromPlanner.tarleg1.motor_vel_ki;

    dataToMcu.leg2.motor_vel = plannerGoalFromPlanner.tarleg2.motor_vel;
    dataToMcu.leg2.motor_pos = plannerGoalFromPlanner.tarleg2.motor_pos;
    dataToMcu.leg2.motor_pos_kp = plannerGoalFromPlanner.tarleg2.motor_pos_kp;
    dataToMcu.leg2.motor_pos_kd = plannerGoalFromPlanner.tarleg2.motor_pos_kd;
    dataToMcu.leg2.motor_vel_kp = plannerGoalFromPlanner.tarleg2.motor_vel_kp;
    dataToMcu.leg2.motor_vel_ki = plannerGoalFromPlanner.tarleg2.motor_vel_ki;

    dataToMcu.leg3.motor_vel = plannerGoalFromPlanner.tarleg3.motor_vel;
    dataToMcu.leg3.motor_pos = plannerGoalFromPlanner.tarleg3.motor_pos;
    dataToMcu.leg3.motor_pos_kp = plannerGoalFromPlanner.tarleg3.motor_pos_kp;
    dataToMcu.leg3.motor_pos_kd = plannerGoalFromPlanner.tarleg3.motor_pos_kd;
    dataToMcu.leg3.motor_vel_kp = plannerGoalFromPlanner.tarleg3.motor_vel_kp;
    dataToMcu.leg3.motor_vel_ki = plannerGoalFromPlanner.tarleg3.motor_vel_ki;

    dataToMcu.leg4.motor_vel = plannerGoalFromPlanner.tarleg4.motor_vel;
    dataToMcu.leg4.motor_pos = plannerGoalFromPlanner.tarleg4.motor_pos;
    dataToMcu.leg4.motor_pos_kp = plannerGoalFromPlanner.tarleg4.motor_pos_kp;
    dataToMcu.leg4.motor_pos_kd = plannerGoalFromPlanner.tarleg4.motor_pos_kd;
    dataToMcu.leg4.motor_vel_kp = plannerGoalFromPlanner.tarleg4.motor_vel_kp;
    dataToMcu.leg4.motor_vel_ki = plannerGoalFromPlanner.tarleg4.motor_vel_ki;

    // ===== 机械臂 =====

    dataToMcu.slide.motor_pos = plannerGoalFromPlanner.tarslide.motor_pos;


    dataToMcu.plate.motor_vel = plannerGoalFromPlanner.tarplate.motor_vel;
    dataToMcu.plate.motor_cur = plannerGoalFromPlanner.tarplate.motor_cur;
    dataToMcu.plate.motor_pos_kp = plannerGoalFromPlanner.tarplate.motor_pos_kp;
    dataToMcu.plate.motor_pos_kd = plannerGoalFromPlanner.tarplate.motor_pos_kd;


    dataToMcu.arm1.motor_pos = plannerGoalFromPlanner.tararm1.motor_pos;
    dataToMcu.arm1.motor_pos_kp = plannerGoalFromPlanner.tararm1.motor_pos_kp;
    dataToMcu.arm1.motor_pos_kd = plannerGoalFromPlanner.tararm1.motor_pos_kd;
    dataToMcu.arm1.motor_vel_kp = plannerGoalFromPlanner.tararm1.motor_vel_kp;
    dataToMcu.arm1.motor_vel_ki = plannerGoalFromPlanner.tararm1.motor_vel_ki;

    dataToMcu.arm2.motor_pos = plannerGoalFromPlanner.tararm2.motor_pos;
    dataToMcu.arm2.motor_pos_kp = plannerGoalFromPlanner.tararm2.motor_pos_kp;
    dataToMcu.arm2.motor_pos_kd = plannerGoalFromPlanner.tararm2.motor_pos_kd;
    dataToMcu.arm2.motor_vel_kp = plannerGoalFromPlanner.tararm2.motor_vel_kp;
    dataToMcu.arm2.motor_vel_ki = plannerGoalFromPlanner.tararm2.motor_vel_ki;



    // ===== 舵轮（Wheelmotor）=====
    dataToMcu.wheel1.vel = Wheel1.GetTarRunVel();
    dataToMcu.wheel2.vel = Wheel2.GetTarRunVel();
    dataToMcu.wheel3.vel = Wheel3.GetTarRunVel();
    dataToMcu.wheel4.vel = Wheel4.GetTarRunVel();
    dataToMcu.wheel1.pos = Wheel1.GetTarTurnPos();
    dataToMcu.wheel2.pos = Wheel2.GetTarTurnPos();
    dataToMcu.wheel3.pos = Wheel3.GetTarTurnPos();
    dataToMcu.wheel4.pos = Wheel4.GetTarTurnPos();

    // ===== pcflag / r2pos =====
    // dataToMcu.pcflag.run_successed_flag = plannerGoalFromPlanner.tarflag.run_successed_flag;
    dataToMcu.mode = plannerGoalFromPlanner.mode;
    dataToMcu.start = plannerGoalFromPlanner.start;

 
}

/// @brief 向发送数组填数并发送
void controlnode::PublishToMCU()
{
    LoadDataToMcu();
    this->control_msg->publish(dataToMcu);
}

