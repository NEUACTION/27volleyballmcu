/* 文件作用：ROS 2在线动作调度节点，按类别/子模式白名单逐周期调用既有规划库，输出八轴目标。
 * 用同号复位事务ACK、所选reset位和真实ready位区分机械准备与模式配置；复位准备完成但ready未齐时重发真实首步。
 * 位置以圈、速度以圈/秒、电流以A对外发送；数学接口的rad/rad/s在对应分支显式转换，增益保留原始编码。
 * 不缓存未来轨迹、不合并八轴反馈，不负责MCU硬件复位和电机闭环；PVT另需显式未验证实验许可。 */
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/mcu_receive.hpp"
#include "base_interfaces/msg/mcu_transmit.hpp"
#include "planner/catch/angle_five2.hpp"
#include "planner/catch/height_profile.hpp"
#include "planner/serve/arm_trajectory.hpp"
#include "planner/serve/arm_legacy.hpp"
#include "planner/serve/plate_distance.hpp"
#include "planner/serve/plate_time.hpp"
#include "planner/serve/sequence.hpp"
#include "planner/slide/position.hpp"

namespace action_planner
{
    // kPeriodSeconds为接球离散采样步长（秒）；每次回调最多推进一步，不补算丢失调度周期。
    constexpr double kPeriodSeconds = 0.005;
    // kArmStartTurns为一般挥臂映射使用的双臂准备圈位置（圈），不自动用于其余历史分支。
    constexpr float kArmStartTurns = 1.75f;
    // PC协议模式码（无量纲）：kDisabled=失能，kVelocity=速度，kPosition=位置，kPvt=PVT。
    // PVT仅在显式实验许可和运行门控同时打开且复位事务完成时发送配置/目标。
    constexpr std::uint8_t kDisabled = 0, kVelocity = 1, kPosition = 2, kPvt = 3;

    // 三大类互斥；同一表项集中描述子模式、内部规划身份和实际参与机构。
    // test_way0内的legacy_way1_*仅为隔离重放历史算法，不能同时启动组合发球。
    struct ModeSpec
    {
        const char * category; // 面向操作者的大类参数字符串，无数值单位。
        const char * submode; // 大类内唯一子模式参数字符串，无数值单位。
        const char * planner_mode; // 分派到内部规划器的身份字符串，不代表电机控制模式码。
        std::uint8_t axes; // 本模式参与轴的无量纲位掩码：bit0/1双臂、bit2盘、bit3丝杆、bit4..7四腿。
    }
    ;
    // kModes为类别/子模式到内部规划身份及参与轴的只读白名单；掩码不表示硬件CAN ID。
    constexpr ModeSpec kModes[] =
    {
        {"idle", "", "idle", 0x00},
        // 舵轮位置环测试：四个轮子同步跟随虚拟主轮，参与轴为槽8..11。
        {"test_way0", "wheel_virtual", "wheel_virtual", 0x0f00},
        {"catch", "catch_t", "catch_t", 0xf0},
        {"catch", "catch_five1", "catch_five1", 0xf0}, {"catch", "catch_five2", "catch_five2", 0xf0},
        {"catch", "catch_five2_pvt_experimental", "catch_five2_pvt_experimental", 0xf0},
        {"serve_way1", "legacy_way1_high", "serve_combined_legacy_way1_high", 0x07},
        {"test_way0", "row_legacy_way1_high", "row_legacy_way1_high", 0x03},
        {"test_way0", "row_prepare_legacy_way1_high", "row_prepare_legacy_way1_high", 0x03},
        {"serve_way1", "legacy_way1_high_down", "serve_combined_legacy_way1_high_down", 0x07},
        {"test_way0", "row_legacy_way1_high_down", "row_legacy_way1_high_down", 0x03},
        {"test_way0", "row_prepare_legacy_way1_high_down", "row_prepare_legacy_way1_high_down", 0x03},
        {"serve_way1", "legacy_way1_high_down117", "serve_combined_legacy_way1_high_down117", 0x07},
        {"test_way0", "row_legacy_way1_high_down117", "row_legacy_way1_high_down117", 0x03},
        {"test_way0", "row_prepare_legacy_way1_high_down117", "row_prepare_legacy_way1_high_down117", 0x03},
        {"serve_way1", "legacy_way1_high_down1110", "serve_combined_legacy_way1_high_down1110", 0x07},
        {"test_way0", "row_legacy_way1_high_down1110", "row_legacy_way1_high_down1110", 0x03},
        {"test_way0", "row_prepare_legacy_way1_high_down1110", "row_prepare_legacy_way1_high_down1110", 0x03},
        {"serve_way1", "legacy_way1_low118a", "serve_combined_legacy_way1_low118a", 0x07},
        {"test_way0", "row_legacy_way1_low118a", "row_legacy_way1_low118a", 0x03},
        {"test_way0", "row_prepare_legacy_way1_low118a", "row_prepare_legacy_way1_low118a", 0x03},
        {"serve_way1", "legacy_way1_low118b", "serve_combined_legacy_way1_low118b", 0x07},
        {"test_way0", "row_legacy_way1_low118b", "row_legacy_way1_low118b", 0x03},
        {"test_way0", "row_prepare_legacy_way1_low118b", "row_prepare_legacy_way1_low118b", 0x03},
        {"serve_way1", "legacy_way1_low119", "serve_combined_legacy_way1_low119", 0x07},
        {"test_way0", "row_legacy_way1_low119", "row_legacy_way1_low119", 0x03},
        {"test_way0", "row_prepare_legacy_way1_low119", "row_prepare_legacy_way1_low119", 0x03},
        {"serve_way1", "legacy_way1_five1_high_down1110", "serve_combined_legacy_way1_five1_high_down1110", 0x07},
        {"test_way0", "row_legacy_way1_five1_high_down1110", "row_legacy_way1_five1_high_down1110", 0x03},
        {"test_way0", "row_prepare_legacy_way1_five1_high_down1110", "row_prepare_legacy_way1_five1_high_down1110", 0x03},
        {"test_way0", "plate_time", "plate_time", 0x04},
        {"test_way0", "plate_distance_way0_high", "plate_distance_way0_high", 0x04},
        {"test_way0", "plate_distance_way0_low", "plate_distance_way0_low", 0x04},
        {"test_way0", "plate_distance_way0_try", "plate_distance_way0_try", 0x04},
        {"test_way0", "plate_distance_way1_high", "plate_distance_way1_high", 0x04},
        {"test_way0", "plate_distance_way1_low", "plate_distance_way1_low", 0x04},
        {"test_way0", "plate_distance_way1_high_down", "plate_distance_way1_high_down", 0x04},
        {"test_way0", "plate_distance_way1_high_down117", "plate_distance_way1_high_down117", 0x04},
        {"test_way0", "plate_distance_way1_high_down1110", "plate_distance_way1_high_down1110", 0x04},
        {"test_way0", "plate_distance_way1_low118a", "plate_distance_way1_low118a", 0x04},
        {"test_way0", "plate_distance_way1_low118b", "plate_distance_way1_low118b", 0x04},
        {"test_way0", "plate_distance_way1_low119", "plate_distance_way1_low119", 0x04},
        {"test_way0", "slide_legacy_way0_high", "slide_legacy_way0_high", 0x08},
        {"test_way0", "slide_legacy_way0_low", "slide_legacy_way0_low", 0x08},
        {"test_way0", "slide_legacy_way1_high", "slide_legacy_way1_high", 0x08},
        {"test_way0", "slide_legacy_way1_low", "slide_legacy_way1_low", 0x08},
        {"test_way0", "slide_legacy_way1_high_down", "slide_legacy_way1_high_down", 0x08},
        {"test_way0", "row_general", "row_general", 0x03},
        {"test_way0", "row_legacy_way0_five", "row_legacy_way0_five", 0x03},
    }
    ;

    // 在线动作节点：只保存规划实例和上次增益，不缓存轨迹点列。
    class actionPlannerNode : public rclcpp::Node
    {
        public:
        /* 中文名：建立在线动作节点。参数：无；返回：无。
         * 副作用：声明模式/运行/实验许可和显式几何增益参数，建立ROS订阅/发布及5ms定时器；
         * 不预生成轨迹点。反馈和定时器使用下方main的单线程执行器顺序调用。 */
        actionPlannerNode() : Node("actionPlannerNode")
        {
            declare_parameter<std::string>("category", "idle");
            declare_parameter<std::string>("submode", "");
            declare_parameter<bool>("run_successed", false);
            
            declare_parameter<bool>("allow_unvalidated_pvt", false);// 用户批准笔误修正的PVT仍未实测，必须额外显式开启实验许可。
            declare_parameter<double>("plate_previous_kp", std::numeric_limits<double>::quiet_NaN());
            declare_parameter<double>("plate_previous_ki", std::numeric_limits<double>::quiet_NaN());
            // 历史丝杆位置增益没有统一实机确认值，要求操作者显式提供。
            declare_parameter<double>("slide_pos_kp", std::numeric_limits<double>::quiet_NaN());
            declare_parameter<double>("slide_pos_kd", std::numeric_limits<double>::quiet_NaN());
            // FIVE明确输入，全部NaN默认；不把历史几何/限幅偷偷变成当前机构默认配置。
            for (const char * name :
                {
                    "row_five_home_rad", "row_five_hit_rad", "row_five_end_rad",
                    "row_five_hit_velocity_rad_s", "row_five_max_speed_rad_s", "row_five_max_acceleration_rad_s2",
                    "row_five_minimum_lead_s"
                }
                )
            {
                // name为逐个待声明的参数名（无数值单位）；对应值的单位由名称_rad/_rad_s/_rad_s2/_s注明。
                declare_parameter<double>(name, std::numeric_limits<double>::quiet_NaN());
            }
            // 舵轮：一个虚拟主轮给出位置期望，四个轮子运动同步；但每轮的增益和前馈电流各自独立。
            declare_parameter<double>("wheel_virtual_turns", std::numeric_limits<double>::quiet_NaN());
            for (int wheel = 1; wheel <= 4; ++wheel)
            {
                // 参数名按轮编号生成：wheelN_pos_kp / _pos_kd / _vel_kp / _vel_ki / _ff_current。
                const std::string n = std::to_string(wheel);
                declare_parameter<double>("wheel" + n + "_pos_kp", std::numeric_limits<double>::quiet_NaN());
                declare_parameter<double>("wheel" + n + "_pos_kd", std::numeric_limits<double>::quiet_NaN());
                declare_parameter<double>("wheel" + n + "_vel_kp", std::numeric_limits<double>::quiet_NaN());
                declare_parameter<double>("wheel" + n + "_vel_ki", std::numeric_limits<double>::quiet_NaN());
                declare_parameter<double>("wheel" + n + "_ff_current", 0.0);
            }
            publisher_ = create_publisher<base_interfaces::msg::McuTransmit>("/mcu/transmit", 1);
            subscription_ = create_subscription<base_interfaces::msg::McuReceive>(
                "/mcu/receive", 1,
                /* 中文名：保存最新反馈回调。参数：feedback为MCU反馈消息共享指针；返回：无。
                 * 副作用：覆盖feedback_全部字段，不校验反馈年龄；位置圈、速度圈/秒、电流A，状态为位掩码/整数。 */
                [this](const base_interfaces::msg::McuReceive::SharedPtr feedback)
                {
                    // 保留整帧八轴实测值、复位ACK、reset/ready掩码及ps_num3，不只保留盘位置，不换算编码计数。
                    feedback_ = *feedback;
                }
                );
            // 中文名：规划定时回调；无参数/返回；副作用：调用Tick构造并发布当前目标，可能推进规划状态。
            timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]()
                {
                    Tick();
                }
                );
        }

        private:
        /* 中文名：选择盘路程曲线（DistanceProfile）。参数：mode为内部模式字符串。
         * 返回：十一种已知路程曲线对应的枚举，未知字符串为空；副作用：无。 */
        static std::optional<planner::ServeDistanceProfile> DistanceProfile(const std::string & mode)
        {
            if (mode == "plate_distance_way0_high") return planner::ServeDistanceProfile::Way0High;
            if (mode == "plate_distance_way0_low") return planner::ServeDistanceProfile::Way0Low;
            if (mode == "plate_distance_way0_try") return planner::ServeDistanceProfile::Way0Try;
            if (mode == "plate_distance_way1_high") return planner::ServeDistanceProfile::Way1High;
            if (mode == "plate_distance_way1_low") return planner::ServeDistanceProfile::Way1Low;
            if (mode == "plate_distance_way1_high_down") return planner::ServeDistanceProfile::Way1HighDown;
            if (mode == "plate_distance_way1_high_down117") return planner::ServeDistanceProfile::Way1HighDown117;
            if (mode == "plate_distance_way1_high_down1110") return planner::ServeDistanceProfile::Way1HighDown1110;
            if (mode == "plate_distance_way1_low118a") return planner::ServeDistanceProfile::Way1Low118A;
            if (mode == "plate_distance_way1_low118b") return planner::ServeDistanceProfile::Way1Low118B;
            if (mode == "plate_distance_way1_low119") return planner::ServeDistanceProfile::Way1Low119;
            return std::nullopt;
        }

        /* 中文名：选择历史丝杆曲线（SlideProfile）。参数：mode为内部模式字符串。
         * 返回：五个历史WAY/函数身份的枚举，未知为空；副作用：无，不推断上手/下手等业务含义。 */
        static std::optional<planner::HistoricalSlideProfile> SlideProfile(const std::string & mode)
        {
            if (mode == "slide_legacy_way0_high") return planner::HistoricalSlideProfile::Way0High;
            if (mode == "slide_legacy_way0_low") return planner::HistoricalSlideProfile::Way0Low;
            if (mode == "slide_legacy_way1_high") return planner::HistoricalSlideProfile::Way1High;
            if (mode == "slide_legacy_way1_low") return planner::HistoricalSlideProfile::Way1Low;
            if (mode == "slide_legacy_way1_high_down") return planner::HistoricalSlideProfile::Way1HighDown;
            return std::nullopt;
        }

        /* 中文名：选择历史挥臂执行身份（LegacyRowMode）。参数：mode为内部模式字符串。
         * 返回：已知WAY1执行整数码（无量纲），未知为空；副作用：无。
         * 数字12对应库内原FIVE1分派，不混同WAY0 FIVE。 */
        static std::optional<int> LegacyRowMode(const std::string & mode)
        {
            if (mode == "row_legacy_way1_high") return planner::legacy_way1::HIGH;
            if (mode == "row_legacy_way1_high_down") return planner::legacy_way1::HIGHDOWN;
            if (mode == "row_legacy_way1_high_down117") return planner::legacy_way1::HIGHDOWN117;
            if (mode == "row_legacy_way1_high_down1110") return planner::legacy_way1::HIGHDOWN1110;
            if (mode == "row_legacy_way1_low118a") return planner::legacy_way1::LOW118A;
            if (mode == "row_legacy_way1_low118b") return planner::legacy_way1::LOW118B;
            if (mode == "row_legacy_way1_low119") return planner::legacy_way1::LOW119;
            if (mode == "row_legacy_way1_five1_high_down1110") return planner::legacy_way1::FHIGHDOWN1110;
            return std::nullopt;
        }

        /* 中文名：解析独立准备身份（PrepareRowMode）。参数：mode为内部模式字符串。
         * 返回：以row_prepare_开头且后缀匹配的WAY1整数执行码，否则空；副作用：无，不启动挥击。 */
        static std::optional<int> PrepareRowMode(const std::string & mode)
        {
            const std::string prefix = "row_prepare_"; // 准备模式的明确前缀。
            if (mode.rfind(prefix, 0) != 0) return std::nullopt;
            return LegacyRowMode("row_" + mode.substr(prefix.size()));
        }

        /* 中文名：解析组合发球身份（CombinedRowMode）。参数：mode为内部模式字符串。
         * 返回：以serve_combined_开头且后缀匹配的八个WAY1整数码之一，否则空；副作用：无。 */
        static std::optional<int> CombinedRowMode(const std::string & mode)
        {
            const std::string prefix = "serve_combined_"; // 组合实验的明确模式前缀。
            if (mode.rfind(prefix, 0) != 0) return std::nullopt;
            return LegacyRowMode("row_" + mode.substr(prefix.size()));
        }

        /* 中文名：重建动作起点（ResetMode）。参数：mode为欲使用的内部模式字符串；返回：无。
         * 副作用：保存mode_，重置接球时钟、盘/挥臂/组合/丝杆实例、准备状态、失能位置和失败锁存；
         * 重新读取FIVE几何及限幅、前次盘增益、丝杆增益，重建接球曲线并重算白名单支持标志。
         * 不清feedback_、复位事务号/序列/待确认标志，也不改运行边沿记忆和发帧计数。
         * Tick在模式切换/运行上升沿/PVT重新许可时调用；ready未齐且未锁存双臂失能时还在首步求值前后调用，
         * 从而反复发送真实首步配置但不保留首步产生的时间进度；本函数不请求机械复位或发送消息。 */
        void ResetMode(const std::string & mode)
        {
            mode_ = mode;
            catch_seconds_ = 0.0;
            plate_time_.Reset();
            row_.Reset();
            row_variants_.reset();
            serve_sequence_.reset();
            combined_position_ok_ = false;
            combined_prepare_target_.reset();
            row_prepare_ready_ = false;
            row_disabled_positions_.reset();
            row_five_loop_count_ = 0;
            row_five_ready_ = false;
            row_five_failed_ = false;
            row_five_plan_ = {};
            row_five_plan_.homePositionRadians = static_cast<float>(get_parameter("row_five_home_rad").as_double());
            row_five_plan_.hitPositionRadians = static_cast<float>(get_parameter("row_five_hit_rad").as_double());
            row_five_plan_.endPositionRadians = static_cast<float>(get_parameter("row_five_end_rad").as_double());
            row_five_plan_.hitAngularVelocityRadPerSecond = static_cast<float>(get_parameter("row_five_hit_velocity_rad_s").as_double());
            row_five_max_speed_ = static_cast<float>(get_parameter("row_five_max_speed_rad_s").as_double());
            row_five_max_acceleration_ = static_cast<float>(get_parameter("row_five_max_acceleration_rad_s2").as_double());
            row_five_minimum_lead_ = static_cast<float>(get_parameter("row_five_minimum_lead_s").as_double());
            catch_t_.InitThetaProfile();
            catch_five1_.InitThetaProfile();
            catch_five2_.InitThetaProfile();
            previous_gains_ =
            {
                static_cast<float>(get_parameter("plate_previous_kp").as_double()),
                static_cast<float>(get_parameter("plate_previous_ki").as_double())
            }
            ;
            // 每次模式切换创建对应历史函数的独立实例，从软件零点重新累计。
            slide_.reset();
            if (const auto profile = SlideProfile(mode)) slide_.emplace(*profile); // profile为当前显式历史丝杆枚举，可空、无量纲。
            slide_pos_kp_ = static_cast<float>(get_parameter("slide_pos_kp").as_double());
            slide_pos_kd_ = static_cast<float>(get_parameter("slide_pos_kd").as_double());
            mode_supported_ = mode == "idle" || mode == "plate_time" || DistanceProfile(mode).has_value() ||
            SlideProfile(mode).has_value() || LegacyRowMode(mode).has_value() || PrepareRowMode(mode).has_value() ||
            CombinedRowMode(mode).has_value() || mode == "row_legacy_way0_five" ||
            mode == "catch_t" || mode == "catch_five1" || mode == "catch_five2" ||
            mode == "catch_five2_pvt_experimental" || mode == "row_general";
            if (!mode_supported_)
            {
                RCLCPP_ERROR(get_logger(), "Rejected unsupported mode '%s': publishing disabled commands", mode.c_str());
            }
        }

        /* 中文名：填写单轴位置控制（SetPosition）。参数：motor为待改写消息轴，position为位置（圈），
         * kp/kd为该轴驱动约定的原始位置比例/微分增益（不统一换算量纲）。
         * 返回：无；副作用：仅写motor的模式、位置及位置增益，不生成速度/电流前馈或PVT数据。 */
        static void SetPosition(base_interfaces::msg::Motor & motor, float position, float kp, float kd)
        {
            motor.controlmode = kPosition;
            motor.motor_pos = position;
            motor.motor_pos_kp = kp;
            motor.motor_pos_kd = kd;
        }

        /* 中文名：求四腿位置控制首/后续一步（CatchStep）。模板CatchPlanner为提供接球曲线与时序的类型。
         * 参数：planner为所选接球规划实例，command为本周期待填目标；返回：无。
         * 副作用：先按catch_seconds_采样位置，再最多推进一个5ms步长，最后按推进后时刻选增益；
         * 写四腿独立方向的位置（圈）和原始增益，不在此函数验证位置是否有限。 */
        template<typename CatchPlanner>
        void CatchStep(CatchPlanner & planner, base_interfaces::msg::McuTransmit & command)
        {
            const float position = static_cast<float>(planner.GetMotorRevsThetaPlan(catch_seconds_)); // 当前采样位置，原始电机圈。
            if (catch_seconds_ < planner.timing.total_seconds) catch_seconds_ += kPeriodSeconds;
            const std::array<float, 4> acceleration_kp
            {
                12.0f, 11.8f, 10.8f, 11.5f
            }
            ; // acceleration_kp为leg1..leg4加速段原始位置比例增益，各项不作量纲缩放。
            const std::array<float, 4> constant_kp
            {
                10.5f, 9.8f, 10.0f, 10.5f
            }
            ; // constant_kp为leg1..leg4匀速段原始位置比例增益，各项不作量纲缩放。
            const std::array<float, 4> deceleration_kp
            {
                9.0f, 8.8f, 9.0f, 9.5f
            }
            ; // deceleration_kp为leg1..leg4减速/结束保持段原始位置比例增益。
            const auto & kp = catch_seconds_ <= planner.timing.acceleration_seconds ? acceleration_kp :
            (catch_seconds_ <= planner.timing.acceleration_seconds + planner.timing.constant_seconds ?
                constant_kp : deceleration_kp); // kp为按推进后时刻选出的四腿原始增益表引用。
            SetPosition(command.leg1, -position, kp[0], 0.0005f);
            SetPosition(command.leg2, position, kp[1], 0.0005f);
            SetPosition(command.leg3, -position, kp[2], 0.0005f);
            SetPosition(command.leg4, position, kp[3], 0.0005f);
        }

        /* 中文名：求实验接球PVT一步（CatchPvtStep）。参数：command为本周期待填目标。
         * 返回：额外实验许可开启且位置/速度有限时true，否则false；不是实机验证结果。
         * 副作用：有效时采样同一时刻位置（圈）/速度（圈/秒）、推进接球时钟并写四腿PVT字段；
         * 无许可时节流报错；按批准修正原pos误写为vel，电流前馈为0A，未验证实验不自动放行。 */
        bool CatchPvtStep(base_interfaces::msg::McuTransmit & command)
        {
            if (!get_parameter("allow_unvalidated_pvt").as_bool())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Experimental catch PVT blocked: allow_unvalidated_pvt must be explicitly enabled");
                return false;
            }
            const float position = static_cast<float>(catch_five2_.GetMotorRevsThetaPlan(catch_seconds_)); // 当前采样位置，圈。
            const float velocity = static_cast<float>(catch_five2_.GetMotorVelAtTime(catch_seconds_)); // 同一时刻采样速度，圈/秒。
            if (!std::isfinite(position) || !std::isfinite(velocity)) return false;
            if (catch_seconds_ < catch_five2_.timing.total_seconds) catch_seconds_ += kPeriodSeconds;
            const std::array<base_interfaces::msg::Motor *, 4> motors
            {
                &command.leg1, &command.leg2, &command.leg3, &command.leg4
            }
            ; // motors为leg1..leg4本帧输出字段地址表，无数值单位。
            const std::array<float, 4> signs
            {
                -1.0f, 1.0f, -1.0f, 1.0f
            }
            ; // signs为leg1..leg4位置/速度方向因子，无量纲；不推断腿在现场的空间方位。
            for (std::size_t axis = 0; axis < motors.size(); ++axis)
            {
                // axis为motors/signs中当前腿的0起始索引（0..3），不是硬件编号。
                auto & motor = *motors[axis]; // 当前腿的消息目标。
                motor.controlmode = kPvt;
                motor.motor_pos = signs[axis] * position;
                motor.motor_vel = signs[axis] * velocity;
                motor.motor_cur = signs[axis] * ;
                // PVT的pos_kp/kd槽位承载mit_kp/kd原值，不采用PosCtrl的增益缩放。
                motor.motor_pos_kp = 1.0f;
                motor.motor_pos_kd = 1.0f;
                // 原tor没有赋值：0只表示无额外前馈电流，不构造不存在的历史力矩目标。
                
            }
            return true;
        }

        /* 中文名：求盘速度控制一步（PlateStep）。参数：command为本周期目标；profile为可选的组合试验
         * 显式路程曲线，空时由mode_选择时间曲线或对应路程曲线。返回：取得有效目标为true，否则false。
         * 副作用：时间分支推进plate_time_；路程分支使用反馈圈位置即时求值并保留返回增益；
         * 写盘速度（圈/秒）、原始速度kp/ki及速度模式，缺少反馈/显式增益时节流报错。 */
        bool PlateStep(base_interfaces::msg::McuTransmit & command,
            std::optional<planner::ServeDistanceProfile> profile = std::nullopt) // 组合试验明确指定的盘分支。
        {
            if (mode_ == "plate_time" && !profile)
            {
                const auto target = plate_time_.Step(); // target为本周期盘速度（圈/秒）及驱动原始kp/ki增益。
                command.plate.motor_vel = static_cast<float>(target.velocity_turns_per_second);
                command.plate.motor_vel_kp = static_cast<float>(target.velocity_kp);
                command.plate.motor_vel_ki = static_cast<float>(target.velocity_ki);
            }
            else
            {
                // 路程直接使用 MCU 反馈圈位置，沿用源 plateMsg.pos/4096 的零点语义。
                if (!feedback_ || !std::isfinite(feedback_->plate.motor_pos) ||
                    !std::isfinite(previous_gains_.kp) || !std::isfinite(previous_gains_.ki))
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Distance mode needs finite MCU plate position and explicit plate_previous_kp/ki (re-enter mode after configuring)");
                    return false;
                }
                const auto target = planner::EvaluateServeDistance(
                    profile ? *profile : *DistanceProfile(mode_), feedback_->plate.motor_pos, previous_gains_); // target为按反馈圈位置即时求得的可选速度（圈/秒）和原始kp/ki增益，空表示求值失败。
                if (!target) return false;
                previous_gains_ = target->gains;
                command.plate.motor_vel = target->velocity_turns_per_second;
                command.plate.motor_vel_kp = target->gains.kp;
                command.plate.motor_vel_ki = target->gains.ki;
            }
            command.plate.controlmode = kVelocity;
            return true;
        }

        /* 中文名：求一般挥臂一步（RowStep）。参数：command为本周期目标；返回：已填位置或失能目标为true。
         * 副作用：已有失能锁存时重发；否则推进row_并将逻辑圈位置映射成双臂独立圈位置/原始增益，
         * 库要求失能或完成时锁存双臂0圈失能位置，不再次使能。 */
        bool RowStep(base_interfaces::msg::McuTransmit & command)
        {
            if (row_disabled_positions_) return DisableRow(command, (*row_disabled_positions_)[0], (*row_disabled_positions_)[1]);
            const auto reference = row_.Step(); // reference为本周期逻辑圈位置、独立原始增益、阶段和失能意图。
            if (reference.disable_requested || reference.stage == planner::RowMotionStage::Complete) return DisableRow(command, 0.0f, 0.0f);
            const auto targets = planner::MapGeneralRowTurns(reference.position_turns, kArmStartTurns,
                reference.arm1_gains, reference.arm2_gains); // targets为按准备圈位置和双臂符号映射后的独立圈位置与原始增益。
            SetPosition(command.arm1, targets.arm1_position_turns, targets.arm1_gains.kp, targets.arm1_gains.kd);
            SetPosition(command.arm2, targets.arm2_position_turns, targets.arm2_gains.kp, targets.arm2_gains.kd);
            return true;
        }

        /* 中文名：锁存并填写双臂失能（DisableRow）。参数：command为待改消息，arm1/arm2分别为双臂
         * 失能帧仍携带的位置（圈）；返回：固定true，表示成功填帧，不代表硬件已确认失能。
         * 副作用：保存row_disabled_positions_，将双臂模式置失能并写位置，直至ResetMode清除锁存。 */
        bool DisableRow(base_interfaces::msg::McuTransmit & command, float arm1, float arm2)
        {
            row_disabled_positions_ = std::array<float, 2>
            {
                arm1, arm2
            }
            ;
            command.arm1.controlmode = command.arm2.controlmode = kDisabled;
            command.arm1.motor_pos = arm1;
            command.arm2.motor_pos = arm2;
            return true;
        }

        /* 中文名：求历史WAY1挥臂一步（LegacyRowStep）。参数：command为本周期目标，selected_mode为
         * 可选WAY1整数身份（无量纲），空时从mode_解析；返回：有效位置或失能意图已写入为true，否则false。
         * 副作用：推进row_variants_；失能意图优先并锁存，位置意图写双臂独立圈位置/原始增益；
         * 已锁存失能时只重发，不推进轨迹；失败时节流报错。 */
        bool LegacyRowStep(base_interfaces::msg::McuTransmit & command,
            std::optional<int> selected_mode = std::nullopt) // 组合序列明确选取的原WAY1执行身份。
        {
            if (row_disabled_positions_) return DisableRow(command, (*row_disabled_positions_)[0], (*row_disabled_positions_)[1]);
            const auto target = row_variants_.ControlArmPosition2(selected_mode ? *selected_mode : *LegacyRowMode(mode_)); // target为所选历史轨迹/FIVE1本步输出：双臂圈位置、原始增益及位置/失能/失败意图。
            if (target.failed)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Legacy WAY1 row trajectory failed");
                return false;
            }
            // 用户明确批准结束保持失能：双意图时失能优先并锁存，不执行原停止周期随后的PosCtrl。
            if (target.disableIntent) return DisableRow(command, target.disablePosition1, target.disablePosition2);
            if (!target.positionIntent) return false;
            SetPosition(command.arm1, target.arm1.positionRevolutions, target.arm1.pos_kp, target.arm1.pos_kd);
            SetPosition(command.arm2, target.arm2.positionRevolutions, target.arm2.pos_kp, target.arm2.pos_kd);
            return true;
        }

        /* 中文名：求独立双臂准备一步（PrepareRowStep）。参数：command为本周期目标。
         * 返回：有有限臂1反馈且库给出纯位置意图为true，否则false。副作用：按臂1实测圈位置
         * 推进历史累计非连续误差计数，写库给出的双臂准备圈位置/原始增益，更新就位日志状态；
         * 使用库的无-0.009镜像，不统一重映射；就位不自动起挥。 */
        bool PrepareRowStep(base_interfaces::msg::McuTransmit & command)
        {
            if (!feedback_ || !std::isfinite(feedback_->arm1.motor_pos))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Row preparation needs finite MCU arm1 position in turns");
                return false;
            }
            const auto target = row_variants_.ControlArmPosition1(*PrepareRowMode(mode_), feedback_->arm1.motor_pos); // target为准备圈位置、原始增益、原累计非连续就位次数（个）及就位意图。
            if (!target.positionIntent || target.disableIntent) return false;
            SetPosition(command.arm1, target.arm1.positionRevolutions, target.arm1.pos_kp, target.arm1.pos_kd);
            SetPosition(command.arm2, target.arm2.positionRevolutions, target.arm2.pos_kp, target.arm2.pos_kd);
            if (target.positionOk && !row_prepare_ready_)
            {
                RCLCPP_INFO(get_logger(), "Legacy row preparation ready: accumulated count=%d; no automatic swing", target.readyCount);
            }
            row_prepare_ready_ = target.positionOk;
            return true;
        }

        /* 中文名：求WAY1组合发球一步（CombinedServeStep）。参数：command为本周期目标。
         * 返回：取得准备保持、盘意图或挥臂意图且相关求值成功为true，否则false。
         * 副作用：使用有限臂1/盘圈位置和显式盘增益，先做准备获得就位后状态，再把准备前/后状态及
         * ps_num3原始字节交给serve_sequence_推进；等待时保持最近准备目标，盘先于臂求值。
         * 不翻转光电电平或猜测传感器编号；失败可已推进部分规划实例，由Tick选择是否发布禁用帧。 */
        bool CombinedServeStep(base_interfaces::msg::McuTransmit & command)
        {
            if (!feedback_ || !std::isfinite(feedback_->arm1.motor_pos) ||
                !std::isfinite(feedback_->plate.motor_pos) || !std::isfinite(previous_gains_.kp) ||
                !std::isfinite(previous_gains_.ki))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Combined WAY1 experiment needs finite arm1/plate feedback and explicit plate_previous_kp/ki");
                return false;
            }
            const int selected_mode = *CombinedRowMode(mode_); // selected_mode为所选WAY1整数执行码，无量纲，不代表传感器编号。
            const bool before = combined_position_ok_; // 当前周期准备调用前的就位标记。
            if (!before)
            {
                const auto preparation = row_variants_.ControlArmPosition1(selected_mode, feedback_->arm1.motor_pos); // preparation为本周期准备输出：双臂圈位置、原始增益、就位标志和计数。
                if (!preparation.positionIntent || preparation.disableIntent) return false;
                combined_prepare_target_ = preparation;
                combined_position_ok_ = preparation.positionOk;
            }
            const auto intent = serve_sequence_.step(selected_mode, before, combined_position_ok_, feedback_->ps_num3); // intent为组合序列本步盘/臂执行意图，输入ps_num3为未翻转的原始字节。
            if (combined_prepare_target_ && !intent.rowIntent)
            {
                const auto & held = *combined_prepare_target_; // held为最近一次求得的准备输出引用，圈位置/原始增益等待中保持；不保证先前帧已实际发送。
                SetPosition(command.arm1, held.arm1.positionRevolutions, held.arm1.pos_kp, held.arm1.pos_kd);
                SetPosition(command.arm2, held.arm2.positionRevolutions, held.arm2.pos_kp, held.arm2.pos_kd);
            }
            if (intent.plateIntent)
            {
                std::optional<planner::ServeDistanceProfile> profile; // profile为按WAY1执行身份选择的盘路程曲线枚举，可空、无量纲。
                switch (selected_mode)
                {
                    case planner::legacy_way1::HIGH: profile = planner::ServeDistanceProfile::Way1High; break;
                    case planner::legacy_way1::HIGHDOWN: profile = planner::ServeDistanceProfile::Way1HighDown; break;
                    case planner::legacy_way1::HIGHDOWN117: profile = planner::ServeDistanceProfile::Way1HighDown117; break;
                    case planner::legacy_way1::HIGHDOWN1110:
                    case planner::legacy_way1::FHIGHDOWN1110: profile = planner::ServeDistanceProfile::Way1HighDown1110; break;
                    case planner::legacy_way1::LOW118A: profile = planner::ServeDistanceProfile::Way1Low118A; break;
                    case planner::legacy_way1::LOW118B: profile = planner::ServeDistanceProfile::Way1Low118B; break;
                    case planner::legacy_way1::LOW119: profile = planner::ServeDistanceProfile::Way1Low119; break;
                    default: return false;
                }
                if (!PlateStep(command, profile)) return false;
            }
            if (intent.rowIntent && !LegacyRowStep(command, selected_mode)) return false;
            return combined_prepare_target_.has_value() || intent.plateIntent || intent.rowIntent;
        }

        /* 中文名：求WAY0 FIVE挥臂一步（RowFiveStep）。参数：command为本周期目标。
         * 返回：位置或失能意图有效为true，规划/求值失败为false。副作用：递增5ms调用计数，
         * 首次按显式几何/速度/限幅构造数学计划；按当前时刻求值，将FOLLOW覆盖为恒速收杆。
         * 恒速终点提前失能并锁存；失能位置保留4096计数截断语义，失败锁存直到ResetMode。 */
        bool RowFiveStep(base_interfaces::msg::McuTransmit & command)
        {
            constexpr float two_pi = 6.28318530718f; // 每圈弧度数，单位rad/圈，用于弧度转电机圈。
            if (row_disabled_positions_) return DisableRow(command, (*row_disabled_positions_)[0], (*row_disabled_positions_)[1]);
            if (row_five_failed_) return false;
            if (row_five_loop_count_ < std::numeric_limits<std::uint32_t>::max()) ++row_five_loop_count_;
            const float now = static_cast<float>(row_five_loop_count_) * 0.005f; // 本次FIVE软件时刻，秒；先加调用次数再换算。
            if (!row_five_ready_)
            {
                const float hit_velocity = row_five_plan_.hitAngularVelocityRadPerSecond; // 显式击球角速度，rad/s。
                const float follow_duration = (row_five_plan_.endPositionRadians - row_five_plan_.hitPositionRadians) / hit_velocity; // 恒速收杆时长，秒。
                row_five_plan_.hitTimeSeconds = now +
                2.0f * std::fabs(row_five_plan_.hitPositionRadians - row_five_plan_.homePositionRadians) /
                std::fabs(hit_velocity) + row_five_minimum_lead_ + 0.005f;
                if (!std::isfinite(follow_duration) || follow_duration <= 0.0f ||
                    !planner::PlanRowPositionTrajectory(&row_five_plan_, now, row_five_max_speed_,
                    row_five_max_acceleration_, row_five_minimum_lead_))
                {
                    row_five_failed_ = true;
                    RCLCPP_ERROR(get_logger(), "WAY0 FIVE requires finite explicit geometry/speed/limits and a valid plan");
                    return false;
                }
                row_five_ready_ = true;
            }
            const float disable_time = row_five_plan_.hitTimeSeconds +
            (row_five_plan_.endPositionRadians - row_five_plan_.hitPositionRadians) /
            row_five_plan_.hitAngularVelocityRadPerSecond; // disable_time为按恒速行程计算的失能软件时刻（秒），不等待数学五次FOLLOW结束。
            if (now >= disable_time)
            {
                const auto targets = planner::MapLogicalRowTurns(row_five_plan_.endPositionRadians / two_pi,
                    {
                        0.0f, 0.0f
                    }
                    ,
                    {
                        0.0f, 0.0f
                    }
                    ); // targets为失能双臂圈位置映射，保留源终点与库内-0.009偏置；此处增益为0。
                // 原失能接口先截断为整数编码计数，再以圈数装入本地消息，保留正常范围的截断语义。
                return DisableRow(command, std::trunc(targets.arm1_position_turns * 4096.0f) / 4096.0f,
                    std::trunc(targets.arm2_position_turns * 4096.0f) / 4096.0f);
            }
            float position = 0.0f; // 当下位置参考，rad。
            float velocity = 0.0f; // 求值器速度参考，rad/s；只用于保持原求值接口。
            const auto stage = planner::GetRowPositionReference(&row_five_plan_, now, &position, &velocity); // stage为当前求值阶段枚举，无量纲。
            if (stage == planner::POS_ROW_ERROR)
            {
                row_five_failed_ = true;
                RCLCPP_ERROR(get_logger(), "WAY0 FIVE evaluation failed");
                return false;
            }
            const bool follow = stage == planner::POS_ROW_FOLLOW; // follow为是否处于FOLLOW阶段的布尔标记，此阶段不发送数学核心的减速多项式目标。
            if (follow) position = row_five_plan_.hitPositionRadians +
                row_five_plan_.hitAngularVelocityRadPerSecond * (now - row_five_plan_.hitTimeSeconds);
            const auto targets = planner::MapLogicalRowTurns(position / two_pi,
                {
                    follow ? 3.6814f : 3.327241f, 0.00005f
                }
                ,
                {
                    3.5f, 0.00005f
                }
                ); // targets为本帧双臂圈位置及各自原始增益，按阶段独立保存，不统一缩放增益。
            SetPosition(command.arm1, targets.arm1_position_turns, targets.arm1_gains.kp, targets.arm1_gains.kd);
            SetPosition(command.arm2, targets.arm2_position_turns, targets.arm2_gains.kp, targets.arm2_gains.kd);
            return true;
        }

        /* 中文名：求历史丝杆一步（SlideStep）。参数：command为本周期目标；返回：实例/显式位置增益
         * 及本步位置均有效为true，否则false。副作用：推进丝杆软件累计圈位置并写位置模式/原始增益；
         * 首次Step即增加一步，未提供有效增益时节流报错，不生成未来点缓存。 */
        bool SlideStep(base_interfaces::msg::McuTransmit & command)
        {
            if (!slide_ || !std::isfinite(slide_pos_kp_) || !std::isfinite(slide_pos_kd_))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Historical slide mode needs explicit finite slide_pos_kp/kd; re-enter mode or restart run after configuring");
                return false;
            }
            const auto position = slide_->Step(); // 当前5ms软件累计位置，单位圈，首次即增加一步。
            if (!position || !std::isfinite(*position)) return false;
            SetPosition(command.slide, *position, slide_pos_kp_, slide_pos_kd_);
            return true;
        }

        /* 中文名：把虚拟主轮位置分发到四个舵轮（DispatchVirtualWheel）。
         * 作用：把上层给出的一个虚拟主轮位置（圈）换算成四个舵轮各自的位置目标（圈，
         * 元素0对应wheel1）。当前四个轮子运动同步，返回四个相同值；
         * 将来若改成差速或转向角几何，只需替换本函数内部，调用方不变。
         * 参数：virtual_turns为虚拟主轮位置（圈）。返回：四轮位置目标（圈）数组。 */
        static std::array<float, 4> DispatchVirtualWheel(float virtual_turns)
        {
            return {virtual_turns, virtual_turns, virtual_turns, virtual_turns};
        }

        /* 中文名：求舵轮位置环一步（WheelStep）。
         * 作用：读取虚拟主轮位置和每轮独立的位置/速度/前馈电流参数，分发后写入四个舵轮。
         * 任意一项参数不是有限值就拒绝输出并节流报错。
         * 参数：command为本周期待填目标。返回：参数齐全并写入为true，否则false。 */
        bool WheelStep(base_interfaces::msg::McuTransmit & command)
        {
            const double virtual_turns = get_parameter("wheel_virtual_turns").as_double(); // 虚拟主轮位置期望，圈。
            if (!std::isfinite(virtual_turns))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "wheel_virtual_turns must be a finite value before running wheel_virtual");
                return false;
            }
            const auto positions = DispatchVirtualWheel(static_cast<float>(virtual_turns)); // 四轮位置目标，当前同步。
            std::array<base_interfaces::msg::Motor *, 4> motors // 四轮ROS字段地址，顺序wheel1..wheel4。
            {
                &command.wheel1, &command.wheel2, &command.wheel3, &command.wheel4
            };
            for (std::size_t i = 0; i < motors.size(); ++i)
            {
                // 每一轮读取自己那一组参数；缺任何一项就整体拒绝，避免部分轮用旧值运动。
                const std::string n = std::to_string(i + 1); // 参数后缀1..4。
                const double pos_kp = get_parameter("wheel" + n + "_pos_kp").as_double();
                const double pos_kd = get_parameter("wheel" + n + "_pos_kd").as_double();
                const double vel_kp = get_parameter("wheel" + n + "_vel_kp").as_double();
                const double vel_ki = get_parameter("wheel" + n + "_vel_ki").as_double();
                const double ff_current = get_parameter("wheel" + n + "_ff_current").as_double();
                if (!std::isfinite(pos_kp) || !std::isfinite(pos_kd) ||
                    !std::isfinite(vel_kp) || !std::isfinite(vel_ki) || !std::isfinite(ff_current))
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "wheel%zu_* gains must all be finite before running wheel_virtual", i + 1);
                    return false;
                }
                auto & motor = *motors[i]; // 当前舵轮目标。
                SetPosition(motor, positions[i], static_cast<float>(pos_kp), static_cast<float>(pos_kd));
                // 位置模式下速度环增益单独下发，驱动器内部按串级结构使用。
                motor.motor_vel_kp = static_cast<float>(vel_kp);
                motor.motor_vel_ki = static_cast<float>(vel_ki);
                motor.motor_cur = static_cast<float>(ff_current); // 每轮独立的前馈电流，A。
            }
            return true;
        }

        /* 中文名：执行一次模式门控与在线求值（Tick）。参数：无；返回：无。
         * 副作用：读运行参数、检测模式/运行/PVT许可边沿并重建动作，创建或重发所选轴复位事务，
         * 检查最新反馈的同号ACK+reset位与ready位，求真实首步配置或后续目标，递增帧计数并发布。
         * reset_ok只证明本事务复位准备完成；ready_ok还要求所选轴真实模式/使能确认，不能互相替代。
         * ready未齐且未锁存双臂失能时允许发布首步配置帧，但前后ResetMode不保留规划进度；ACK未齐不求值。
         * 无效/禁止/求值失败时目标清零禁用，复位事务字段随后重新补上；反馈无超时/年龄校验。 */
        void Tick()
        {
            const auto category = get_parameter("category").as_string(); // 当前大类参数字符串，无数值单位。
            const auto submode = get_parameter("submode").as_string(); // 当前大类内子模式参数字符串，无数值单位。
            const ModeSpec * selected = nullptr; // 只有一个类别/子模式表项能取得执行权。
            // entry为逐个扫描的白名单表项引用，无数值单位。
            for (const auto & entry : kModes)
            {
                if (category == entry.category && submode == entry.submode)
                {
                    selected = &entry; break;
                }
            }
            const std::string requested_mode = selected ? selected->planner_mode : "__invalid_category_mode__"; // 映射后的请求身份，非法组合使用拒绝哨兵。
            const bool run_requested = get_parameter("run_successed").as_bool(); // 当前显式运行门控。
            const bool allow_pvt = get_parameter("allow_unvalidated_pvt").as_bool(); // 未验证PVT的额外显式实验许可。
            // 运行上升沿重新确定本次动作采样原点；PVT实验许可重新开启也从起点开始，无预测缓冲。
            const bool valid_category_mode = requested_mode != "__invalid_category_mode__"; // 类别/子模式组合是否命中白名单。
            const bool mode_changed = requested_mode != mode_; // 内部规划身份是否与上周期不同。
            const bool run_rising = run_requested && !previous_run_requested_; // 显式运行false到true边沿。
            if ((mode_changed || run_rising ||
                (requested_mode == "catch_five2_pvt_experimental" && allow_pvt && !previous_allow_pvt_)))
            {
                ResetMode(requested_mode);
                // 先等首条主控反馈再分配事务号，避免PC重启后误用主控旧ACK。
                reset_request_id_ = 0;
                reset_transaction_pending_ = true;
            }
            if (run_requested && reset_transaction_pending_ && reset_request_id_ == 0 && feedback_)
            {
                // 首次以MCU已有ACK作种子，再递增跳过0及当前ACK；uint32回绕仍不得把旧ACK当新事务。
                if (reset_sequence_counter_ == 0) reset_sequence_counter_ = feedback_->reset_request_ack;
                do
                {
                    ++reset_sequence_counter_;
                }
                while (reset_sequence_counter_ == 0 || reset_sequence_counter_ == feedback_->reset_request_ack);
                reset_request_id_ = reset_sequence_counter_;
            }
            previous_run_requested_ = run_requested;
            previous_allow_pvt_ = allow_pvt;
            base_interfaces::msg::McuTransmit command{}; // command为本次新建目标消息：位置圈、速度圈/秒、电流A、原始增益；不复用其它模式遗留目标。
            command.arm1.controlmode = command.arm2.controlmode = command.plate.controlmode = kDisabled;
            command.slide.controlmode = command.leg1.controlmode = command.leg2.controlmode = kDisabled;
            command.leg3.controlmode = command.leg4.controlmode = kDisabled;
            command.wheel1.controlmode = command.wheel2.controlmode = kDisabled;
            command.wheel3.controlmode = command.wheel4.controlmode = kDisabled;
            command.wheel_reset_mask = 0; // 舵轮复位位单独传递；默认不请求任何舵轮复位。
            const std::uint8_t required_mask = selected ? selected->axes : 0; // 本模式所需协议轴位掩码；非法或idle为0。
            // 轴掩码分两段：机构轴用低8位，舵轮用bit8..11；分别核对feedback的两个复位/就绪掩码。
            const std::uint8_t required_mech_mask = static_cast<std::uint8_t>(required_mask & 0xffU);
            const std::uint8_t required_wheel_mask = static_cast<std::uint8_t>((required_mask >> 8) & 0x0fU);
            // mode_allowed为身份受支持、实际选轴非空且PVT额外许可满足的布尔门控。
            const bool mode_allowed = valid_category_mode && mode_supported_ && required_mask != 0 &&
            (mode_ != "catch_five2_pvt_experimental" || allow_pvt);
            bool valid = run_requested && mode_allowed; // 当前帧可发布运行/配置目标的意图，后续ACK或求值失败可清除。
            const bool reset_ack_ok = reset_request_id_ != 0 && feedback_ && feedback_->reset_request_ack == reset_request_id_; // 当前非零事务号已同号回显。
            // reset_ok还要求所有参与轴复位准备位齐；双臂bit1按共享协议特例解释，不等于ARM2独立复位回报。
            const bool reset_ok = reset_ack_ok && feedback_ &&
            (feedback_->reset_mask & required_mech_mask) == required_mech_mask &&
            (feedback_->wheel_reset_mask & required_wheel_mask) == required_wheel_mask;
            if (reset_ok) reset_transaction_pending_ = false;
            command.reset_request_id = reset_request_id_;
            command.reset_request_mask = run_requested && mode_allowed && reset_transaction_pending_ ? required_mech_mask : 0;
            command.wheel_reset_mask = run_requested && mode_allowed && reset_transaction_pending_ ? required_wheel_mask : 0;
            // 用户批准的结束失能不能因ready清位而重新使能，已完成臂无需重新等待enable位。
            // ready所需机构位；已完成失能双臂移除bit0/1。
            const std::uint8_t ready_required = row_disabled_positions_ ? static_cast<std::uint8_t>(required_mech_mask & ~0x03U) : required_mech_mask;
            // 机构位与舵轮位都要齐；未检查反馈年龄。
            const bool ready_ok = feedback_ && (feedback_->ready_mask & ready_required) == ready_required &&
            (feedback_->wheel_ready_mask & required_wheel_mask) == required_wheel_mask;
            const bool configuring = valid && reset_ok && !ready_ok; // 事务复位准备已完成但真实配置/使能未齐：只重发首步，不保留规划进度。
            if (valid && !reset_ok)
            {
                valid = false;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting for reset request ack id=%u and selected reset_mask=0x%02x", reset_request_id_, required_mask);
            }
            if (configuring && !row_disabled_positions_) ResetMode(mode_);
            if (valid && configuring && row_disabled_positions_)
            {
                // 已结束双臂保持禁用；仅组合盘配置丢失时重新给盘零速配置，不重启已结束轨迹。
                DisableRow(command, (*row_disabled_positions_)[0], (*row_disabled_positions_)[1]);
                if (CombinedRowMode(mode_) && std::isfinite(previous_gains_.kp) && std::isfinite(previous_gains_.ki))
                {
                    command.plate.controlmode = kVelocity;
                    command.plate.motor_vel_kp = previous_gains_.kp;
                    command.plate.motor_vel_ki = previous_gains_.ki;
                }
                else valid = false;
            }
            else if (valid)
            {
                if (mode_ == "plate_time" || DistanceProfile(mode_)) valid = PlateStep(command);
                else if (mode_ == "catch_t") CatchStep(catch_t_, command);
                else if (mode_ == "catch_five1") CatchStep(catch_five1_, command);
                else if (mode_ == "catch_five2") CatchStep(catch_five2_, command);
                else if (mode_ == "catch_five2_pvt_experimental") valid = CatchPvtStep(command);
                else if (mode_ == "row_general") valid = RowStep(command);
                else if (mode_ == "row_legacy_way0_five") valid = RowFiveStep(command);
                else if (LegacyRowMode(mode_)) valid = LegacyRowStep(command);
                else if (PrepareRowMode(mode_)) valid = PrepareRowStep(command);
                else if (CombinedRowMode(mode_)) valid = CombinedServeStep(command);
                else if (SlideProfile(mode_)) valid = SlideStep(command);
                else if (mode_ == "wheel_virtual") valid = WheelStep(command);
            }
            if (valid && CombinedRowMode(mode_) && command.plate.controlmode == kDisabled)
            {
                // 组合等待触发时盘持续零速使能，避免ready位清除后反复重置组合计数。
                command.plate.controlmode = kVelocity;
                command.plate.motor_vel = 0.0f;
                command.plate.motor_vel_kp = previous_gains_.kp;
                command.plate.motor_vel_ki = previous_gains_.ki;
            }
            // 反复发送各模式真实首步，不冒充数学t=0，也不把机械0当安全准备角；ready齐后仍从同一首步开始。
            if (configuring && !row_disabled_positions_) ResetMode(mode_);
            if (!valid) command = base_interfaces::msg::McuTransmit{};
            command.reset_request_id = reset_request_id_;
            command.reset_request_mask = run_requested && mode_allowed && reset_transaction_pending_ ? required_mech_mask : 0;
            command.wheel_reset_mask = run_requested && mode_allowed && reset_transaction_pending_ ? required_wheel_mask : 0;
            command.pcflag.run_successed_flag = valid;
            // 用无符号计数并限制到int32正数范围，避免长期运行的有符号溢出。
            period_count_ = (period_count_ + 1U) & 0x7fffffffU;
            command.period_cnt_pc = static_cast<std::int32_t>(period_count_);
            publisher_->publish(command);
        }

        std::uint32_t reset_sequence_counter_ = 0; // reset_sequence_counter_为本进程连续uint32事务序号（无量纲）；首发从已见ACK后递增，跳过0和当前ACK。
        std::uint32_t reset_request_id_ = 0; // reset_request_id_为当前非零动作复位事务号（无量纲）；0表示等首条反馈分配，ResetMode不清此值。
        bool reset_transaction_pending_ = false; // reset_transaction_pending_为本动作是否仍等待同号ACK及所选reset位的布尔标志；不表示ready状态。
        std::string mode_ = "__uninitialized__"; // mode_为当前内部规划身份字符串，初始哨兵保证首次Tick重置。
        bool mode_supported_ = false; // mode_supported_为内部身份是否受支持的布尔标记，未知模式明确拒绝运动。
        bool previous_run_requested_ = false; // previous_run_requested_为上周期运行布尔参数，用于检测false到true并重置采样原点。
        bool previous_allow_pvt_ = false; // previous_allow_pvt_为上周期PVT实验许可布尔值，重新开启时重置本实验时间。
        double catch_seconds_ = 0.0; // catch_seconds_为本实例接球离散采样时钟（秒），每次有效求值最多增加5ms。
        std::uint32_t period_count_ = 0; // period_count_为本节点发布消息计数（无量纲、31位循环），独立于串口节点构帧计数。
        planner::PlateTimePlanner plate_time_; // 盘时间模式实例。
        planner::GeneralRowPlanner row_; // 一般挥臂模式实例。
        planner::legacy_way1::RowVariants row_variants_; // WAY1各独立轨迹的原实例状态。
        planner::legacy_way1::ServeSequence serve_sequence_; // 原1000/1400/22调用周期组合调度实例，每次Tick最多推进一次。
        bool combined_position_ok_ = false; // combined_position_ok_为组合准备就位布尔锁存，供序列判断本周期准备调用前/后状态。
        std::optional<planner::legacy_way1::RowOutput> combined_prepare_target_; // 最近一次求得的双臂准备圈位置/原始增益输出，等待中保持，不保证已实际发送。
        bool row_prepare_ready_ = false; // row_prepare_ready_为准备就位日志的上周期布尔状态，不作为自动起挥门控。
        std::optional<std::array<float, 2>> row_disabled_positions_; // 双臂失能圈位置锁存：元素0为臂1、1为臂2，空表示尚未锁存。
        planner::PosRowPlan_t row_five_plan_{}; // row_five_plan_为WAY0 FIVE输入及数学计划：角度rad、角速度rad/s、时刻秒，不是预采样点列。
        std::uint32_t row_five_loop_count_ = 0; // row_five_loop_count_为FIVE求值调用次数（个），每次对应5ms；配置等待会被ResetMode清零，不保留累计进度。
        bool row_five_ready_ = false; // row_five_ready_为FIVE当前显式输入已完成一次数学规划的布尔标记。
        bool row_five_failed_ = false; // row_five_failed_为FIVE规划/求值失败布尔锁存，直到ResetMode（含配置等待中的重置）才清除。
        float row_five_max_speed_ = 0.0f; // row_five_max_speed_为FIVE显式角速度上限，rad/s。
        float row_five_max_acceleration_ = 0.0f; // row_five_max_acceleration_为FIVE显式角加速度上限，rad/s²。
        float row_five_minimum_lead_ = 0.0f; // row_five_minimum_lead_为FIVE显式起挥预留时间，秒。
        std::optional<planner::HistoricalSlidePlanner> slide_; // 当前明确选择的历史丝杆实例，不保存点列。
        float slide_pos_kp_ = std::numeric_limits<float>::quiet_NaN(); // slide_pos_kp_为操作者确认的丝杆位置比例增益，采用驱动原始量纲，不在节点内缩放。
        float slide_pos_kd_ = std::numeric_limits<float>::quiet_NaN(); // slide_pos_kd_为操作者确认的丝杆位置微分增益或驱动哨兵，采用驱动原始编码，不推断统一单位。
        planner::CatchFive2Planner catch_five2_; // 角度域五次接球实例。
        planner::CatchHeightPlanner catch_t_ = planner::CatchHeightPlanner(planner::CatchHeightMode::Trapezoidal); // 高度梯形接球实例。
        planner::CatchHeightPlanner catch_five1_ = planner::CatchHeightPlanner(planner::CatchHeightMode::Quintic); // 高度五次接球实例。
        planner::DistanceVelocityGains previous_gains_{}; // previous_gains_为路程分支逐周期保留的kp/ki，使用库/驱动原始量纲，由显式参数初始化。
        std::optional<base_interfaces::msg::McuReceive> feedback_; // 最新八轴反馈：圈、圈/秒、A和状态/事务整数；空表示尚未收到，不检查年龄。
        rclcpp::Publisher<base_interfaces::msg::McuTransmit>::SharedPtr publisher_; // MCU目标消息发布器。
        rclcpp::Subscription<base_interfaces::msg::McuReceive>::SharedPtr subscription_; // MCU实测反馈订阅器。
        rclcpp::TimerBase::SharedPtr timer_; // 5ms墙钟规划定时器。
    }
    ;
}
// namespace action_planner

/* 中文名：动作节点进程入口（main）。参数：argc为命令行项数（个），argv为参数字符串数组。
 * 返回：正常结束为0；副作用：初始化ROS 2、创建节点并阻塞spin，退出后关闭ROS上下文。
 * 使用单线程执行器，参数、反馈和规划回调顺序执行，无跨线程规划实例竞争。 */
int main(int argc, char ** argv) // argc/argv 为ROS命令行参数。
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<action_planner::actionPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
