/* 文件作用：把ROS目标编码成SP下发帧，把QR反馈字节流拆成八轴ROS实测消息。
 * 负责短写续发和接收流重同步，不执行运动规划；复位/ready含义以共享协议定义为准。 */
#include "mcu/mcuNode.hpp"
#include "usb_codec.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>

namespace mcu
{
    namespace
    {
        // 下发帧固定标记，均为单字节字符值；反馈帧头另用QR，尾标记共用CRLF。
        constexpr uint8_t kHeader0 = 'S'; // 下发帧头第0字节。
        constexpr uint8_t kHeader1 = 'P'; // 下发帧头第1字节。
        constexpr uint8_t kTail0 = '\r'; // 帧尾倒数第2字节：回车。
        constexpr uint8_t kTail1 = '\n'; // 帧尾最后1字节：换行。

        /* 中文名：写发送浮点槽。参数：frame为发送帧，slot为数据区从0起算的槽序号（个），
         * value为float32原值（单位沿用位置圈/速度圈每秒/电流A或原始增益字段）。
         * 返回：无；副作用：改写frame中该槽的4字节，不作槽边界检查。 */
        void PutFloat(std::array<uint8_t, TX_DATA_LEN_MCU> &frame, std::size_t slot, float value)
        {
            VbWriteFloat(frame.data() + VB_COMMAND_DATA_OFFSET + slot * sizeof(float), value);
        }

        /* 中文名：写发送整数槽。参数：frame为发送帧，slot为数据槽序号（个），value为无量纲整数计数。
         * 返回：无；副作用：改写frame对应4字节，不经浮点转换、不作槽边界检查。 */
        void PutUint32(std::array<uint8_t, TX_DATA_LEN_MCU> &frame, std::size_t slot, uint32_t value)
        {
            VbWriteU32(frame.data() + VB_COMMAND_DATA_OFFSET + slot * sizeof(value), value);
        }
    }
    // namespace

    /* 中文名：初始化串口ROS桥。参数：无；返回：无；副作用：读取设备路径与波特率参数、
     * 尝试打开串口，创建深度1的目标订阅/反馈发布及5ms通信定时器；失败只记录错误。 */
    mcuNode::mcuNode()
        : Node("mcuNode")
    {
        const auto device = declare_parameter<std::string>("mcu_serial", "/dev/mcu"); // MCU设备路径，无数值单位。
        const auto baud = declare_parameter<int>("baudrate", 921600); // 请求串口波特率，bit/s。
        mcuComSerial_ = std::make_unique<Serial>(device, static_cast<uint32_t>(baud));
        if (!mcuComSerial_->SerialInit())
        {
            RCLCPP_ERROR(get_logger(), "Unable to open MCU serial device: %s", device.c_str());
        }
        // 在线控制只保留最新一帧，避免消息积压变成过时目标队列。
        mcuPublisher = create_publisher<base_interfaces::msg::McuReceive>("/mcu/receive", 1);
        controlListner = create_subscription<base_interfaces::msg::McuTransmit>(
            "/mcu/transmit", 1,
            std::bind(&mcuNode::ControlListnerCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&mcuNode::timer_callback, this));
    }

    /* 中文名：缓存最新控制目标。参数：msg为ROS目标消息共享指针；返回：无。
     * 副作用：复制覆盖command_；不会改写尚在续发的txDataMcu_，也不立即执行串口I/O。 */
    void mcuNode::ControlListnerCallback(const base_interfaces::msg::McuTransmit::SharedPtr msg)
    {
        command_ = *msg;
    }

    /* 中文名：执行通信周期。参数：无；返回：无；副作用：调用发送与接收，
     * 可推进短写偏移、消费串口输入并发布反馈；不按返回值中止后续接收。 */
    void mcuNode::timer_callback()
    {
        // 无论当前发送成功与否都消费反馈，避免发送异常阻塞接收。
        Send2MCU();
        ReadFromMCU();
    }

    /* 中文名：构帧并续发十二轴目标。参数：无；返回：本次已发完整帧为true，短写/写失败为false。
     * 副作用：新帧递增period_cnt_pc_并从command_编码；短写继续原帧，更新txOffset_/txPending_。
     * 位置圈、速度圈/秒、电流A及原始增益直接编码；不计算轨迹、不等待硬件执行完成。 */
    bool mcuNode::Send2MCU()
    {
        // 完整帧构造后保持不变，直到所有字节成功写出。
        if (!txPending_)
        {
            period_cnt_pc_ = (period_cnt_pc_ + 1U) & 0x7FFFFFFFU;
            txDataMcu_.fill(0);
            txDataMcu_[0] = kHeader0;
            txDataMcu_[1] = kHeader1;
            txDataMcu_[TX_DATA_LEN_MCU - 2] = kTail0;
            txDataMcu_[TX_DATA_LEN_MCU - 1] = kTail1;
            txDataMcu_[2] = command_.pcflag.run_successed_flag ? 1U : 0U;
            PutUint32(txDataMcu_, 0, period_cnt_pc_);
            txDataMcu_[VB_RESET_REQUEST_BYTE] = command_.reset_request_mask;
            VbWriteU32(txDataMcu_.data() + VB_RESET_ID_BYTE, command_.reset_request_id);

            // motors为本帧十二轴目标副本，顺序与协议槽一致：双臂、盘、丝杆、四腿、四舵轮；不推断CAN硬件ID。
            // 舵轮槽8..11挂在CAN1，运动同步但复位各自独立。
            const std::array<base_interfaces::msg::Motor, 12> motors
            {
                command_.arm1, command_.arm2, command_.plate, command_.slide,
                command_.leg1, command_.leg2, command_.leg3, command_.leg4,
                command_.wheel1, command_.wheel2, command_.wheel3, command_.wheel4
            }
            ;
            // 舵轮复位掩码由action节点放进wheel_reset_mask；机构掩码仍走reset_request_mask。
            txDataMcu_[VB_WHEEL_RESET_REQUEST_BYTE] = command_.wheel_reset_mask;
            // i为协议轴序号（0..11，无量纲），非硬件CAN标识符。
            for (std::size_t i = 0; i < motors.size(); ++i)
            {
                const auto &motor = motors[i]; // 当前协议轴目标引用；字段单位见函数注释。
                const std::size_t base = 1 + i * 7; // 当前轴首数据槽序号（个），槽0保留整数计数。
                PutFloat(txDataMcu_, base, motor.motor_pos);
                PutFloat(txDataMcu_, base + 1, motor.motor_vel);
                PutFloat(txDataMcu_, base + 2, motor.motor_cur);
                PutFloat(txDataMcu_, base + 3, motor.motor_pos_kp);
                PutFloat(txDataMcu_, base + 4, motor.motor_pos_kd);
                PutFloat(txDataMcu_, base + 5, motor.motor_vel_kp);
                PutFloat(txDataMcu_, base + 6, motor.motor_vel_ki);
                if (i < 8)
                {
                    // 机构轴：模式两字节、clean一字节，覆盖轴槽0..7。
                    const std::size_t modeByte = VB_MODE_BYTE + i / 4;
                    txDataMcu_[modeByte] |= static_cast<uint8_t>((motor.controlmode & 0x03U) << ((i % 4) * 2));
                    if (motor.clean) txDataMcu_[VB_CLEAN_BYTE + i / 8] |= static_cast<uint8_t>(1U << (i % 8));
                }
                else
                {
                    // 舵轮：模式和clean各用一个专用字节，轮1占最低两位/最低位。
                    const std::size_t wheel = i - 8;
                    txDataMcu_[VB_WHEEL_MODE_BYTE] |=
                        static_cast<uint8_t>((motor.controlmode & 0x03U) << (wheel * 2));
                    if (motor.clean) txDataMcu_[VB_WHEEL_CLEAN_BYTE] |= static_cast<uint8_t>(1U << wheel);
                }
            }
            txOffset_ = 0;
            txPending_ = true;
        }
        // written为本次实际提交的字节数；0保留当前帧等待下周期，负数表示写错误。
        const int written = mcuComSerial_->SerialWrite(
            txDataMcu_.data() + txOffset_, txDataMcu_.size() - txOffset_);
        if (written < 0) return false;
        txOffset_ += static_cast<std::size_t>(written);
        if (txOffset_ != txDataMcu_.size()) return false;
        txPending_ = false;
        return true;
    }

    /* 中文名：拆解串口反馈流。参数：无；返回：本次至少成功解析一帧为true，否则false。
     * 副作用：读取到rxDataMcu_并累计rxCache_，逐字节寻找QR/CRLF对齐；仅检查固定头尾，无CRC。
     * 完整帧更新十二轴反馈和事务/ready位并发布/mcu/receive，再从缓存移除该帧。 */
    bool mcuNode::ReadFromMCU()
    {
        const int count = mcuComSerial_->SerialRead(rxDataMcu_.data(), rxDataMcu_.size()); // 本次接收字节数；0无输入，负数为错误。
        if (count > 0) rxCache_.insert(rxCache_.end(), rxDataMcu_.begin(), rxDataMcu_.begin() + count);
        bool decoded = false; // 本次是否已经解析至少一帧完整反馈。
        while (rxCache_.size() >= RX_DATA_LEN_MCU)
        {
            if (rxCache_[0] != 'Q' || rxCache_[1] != 'R' ||
                rxCache_[RX_DATA_LEN_MCU - 2] != kTail0 ||
                rxCache_[RX_DATA_LEN_MCU - 1] != kTail1)
            {
                rxCache_.erase(rxCache_.begin());
                continue;
            }
            // 状态区2:心跳；3..10:各轴错误；11:使能；12:复位准备位（双臂bit1有协议特例）；其余索引见共享头。
            receivedData_.heartbag.mcu_to_pc_hb = rxCache_[VB_HEARTBEAT_BYTE];
            receivedData_.reset_mask = rxCache_[VB_RESET_BYTE];
            receivedData_.ready_mask = rxCache_[VB_READY_BYTE];
            receivedData_.ps_num3 = rxCache_[VB_PS3_BYTE];
            receivedData_.reset_request_ack = VbReadU32(rxCache_.data() + VB_RESET_ACK_BYTE);
            receivedData_.mcuflag.reset_flag = receivedData_.reset_mask == 0xFFU;
            const std::size_t payload = VB_FEEDBACK_DATA_OFFSET; // 数据区起点，单位字节。
            // ROS沿用int32计数接口，屏蔽最高位后在非负31位范围循环（包含0）。
            receivedData_.period_cnt_mcu = static_cast<int32_t>(VbReadU32(rxCache_.data() + payload) & 0x7FFFFFFFU);
            // motors为十二轴反馈字段地址表：双臂、盘、丝杆、四腿、四舵轮；保持协议顺序，不合并实测轴。
            std::array<base_interfaces::msg::Motor *, 12> motors
            {
                &receivedData_.arm1, &receivedData_.arm2, &receivedData_.plate, &receivedData_.slide,
                &receivedData_.leg1, &receivedData_.leg2, &receivedData_.leg3, &receivedData_.leg4,
                &receivedData_.wheel1, &receivedData_.wheel2, &receivedData_.wheel3, &receivedData_.wheel4
            }
            ;
            // 舵轮状态位回到wheel_reset_mask/wheel_ready_mask，机构轴仍用reset_mask/ready_mask。
            receivedData_.wheel_reset_mask = rxCache_[VB_WHEEL_RESET_BYTE];
            receivedData_.wheel_ready_mask = rxCache_[VB_WHEEL_READY_BYTE];
            // i为协议轴序号（0..11，无量纲），不是推断出的硬件编号。
            for (std::size_t i = 0; i < motors.size(); ++i)
            {
                auto &motor = *motors[i]; // 当前轴的ROS反馈字段：位置圈、速度圈/秒、电流A。
                const std::size_t base = payload + (1 + 3 * i) * 4; // 当前轴位置槽在帧内的绝对字节偏移。
                motor.motor_pos = VbReadFloat(rxCache_.data() + base);
                motor.motor_vel = VbReadFloat(rxCache_.data() + base + 4);
                motor.motor_cur = VbReadFloat(rxCache_.data() + base + 8);
                if (i < 8)
                {
                    motor.error = rxCache_[VB_ERROR_BYTE + i];
                    motor.enablestate = (rxCache_[VB_ENABLE_BYTE] >> i) & 1U;
                }
                else
                {
                    const std::size_t wheel = i - 8;
                    motor.error = rxCache_[VB_WHEEL_ERROR_BYTE + wheel];
                    motor.enablestate = (rxCache_[VB_WHEEL_ENABLE_BYTE] >> wheel) & 1U;
                }
            }
            mcuPublisher->publish(receivedData_);
            rxCache_.erase(rxCache_.begin(), rxCache_.begin() + RX_DATA_LEN_MCU);
            decoded = true;
        }
        return decoded;
    }

}
// namespace mcu
