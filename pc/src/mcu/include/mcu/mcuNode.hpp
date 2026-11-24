/* 文件作用：声明ROS 2与MCU串口之间的八轴协议桥接节点、固定帧尺寸和跨周期收发状态。
 * 本节点只转发目标/反馈，不负责运动规划、机械复位或电机闭环计算。 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "base_interfaces/msg/mcu_receive.hpp"
#include "base_interfaces/msg/mcu_transmit.hpp"
#include "mcu/serial.hpp"
#include "usb_protocol.h"

namespace mcu
{

    // 帧由2字节帧头、控制区、4字节数据槽及2字节CRLF构成；以下长度均为字节数或槽数。
    // 线上传十二个轴：八个机构电机在CAN3，四个舵轮在CAN1；槽序与共享协议一致。
    constexpr std::size_t TX_UINT8_NUM_MCU = VB_TX_CONTROL_BYTES; // 发送控制区长度，字节。
    constexpr std::size_t TX_FLOAT_NUM_MCU = VB_COMMAND_SLOTS; // 发送槽数（个），槽0为整数计数，其余十二轴各七字段。
    constexpr std::size_t RX_UINT8_NUM_MCU = VB_RX_CONTROL_BYTES; // 接收控制区长度，字节。
    constexpr std::size_t RX_FLOAT_NUM_MCU = VB_FEEDBACK_SLOTS; // 接收槽数（个），槽0为整数计数，其余十二轴各三反馈。
    constexpr std::size_t TX_DATA_LEN_MCU = 4 + TX_UINT8_NUM_MCU + 4 * TX_FLOAT_NUM_MCU; // 完整下发帧长，字节。
    constexpr std::size_t RX_DATA_LEN_MCU = 4 + RX_UINT8_NUM_MCU + 4 * RX_FLOAT_NUM_MCU; // 完整反馈帧长，字节。
    static_assert(TX_DATA_LEN_MCU == 373, "Command frame size changed; update PROTOCOL.md and MCU firmware together");
    static_assert(RX_DATA_LEN_MCU == 181, "Feedback frame size changed; update PROTOCOL.md and MCU firmware together");

    // mcuNode将ROS目标与八轴串口帧互转，保留短写帧与接收流，不计算轨迹。
    class mcuNode final : public rclcpp::Node
    {
        public:
        /* 中文名：建立MCU桥接节点。参数：无；返回：无；副作用：声明串口参数、
         * 打开设备，注册目标订阅、反馈发布和5ms定时器；打开失败记录日志。 */
        mcuNode();

        private:
        /* 中文名：保存最新目标回调。参数：msg为接收的目标消息共享指针；返回：无。
         * 副作用：覆盖command_，不立即发送，未完成的旧串口帧仍先续发。 */
        void ControlListnerCallback(const base_interfaces::msg::McuTransmit::SharedPtr msg);
        /* 中文名：通信周期回调。参数：无；返回：无；副作用：先尝试发送再消费反馈，
         * 即使发送失败仍接收；可能更新缓存、计数和发布ROS消息。 */
        void timer_callback();
        /* 中文名：发送或续发目标帧。参数：无；返回：本次补全整帧为true，短写或错误为false。
         * 副作用：空闲时按command_构帧并递增序号，否则续发原帧；更新发送偏移。 */
        bool Send2MCU();
        /* 中文名：接收并拆解反馈流。参数：无；返回：本次解析至少一帧为true。
         * 副作用：累计非阻塞输入、按帧头尾同步，更新receivedData_并逐帧发布反馈。 */
        bool ReadFromMCU();

        // 以下ROS句柄及串口资源由本节点持有，无物理量单位。
        std::unique_ptr<Serial> mcuComSerial_; // MCU串口的独占资源句柄。
        rclcpp::Publisher<base_interfaces::msg::McuReceive>::SharedPtr mcuPublisher; // /mcu/receive反馈发布器。
        rclcpp::Subscription<base_interfaces::msg::McuTransmit>::SharedPtr controlListner; // /mcu/transmit最新目标订阅器。
        rclcpp::TimerBase::SharedPtr timer_; // 5ms通信墙钟定时器。

        // txDataMcu_为完整待发送帧（字节数组），短写期间保持不变；rxDataMcu_为单次read暂存字节区，未必包含整帧。
        std::array<uint8_t, TX_DATA_LEN_MCU> txDataMcu_{};
        std::array<uint8_t, RX_DATA_LEN_MCU> rxDataMcu_{};
        // command_为最新ROS目标快照：位置圈、速度圈/秒、电流A、增益保持驱动原始量纲。
        base_interfaces::msg::McuTransmit command_{};
        // receivedData_为最近成功拆解的反馈：每轴位置圈、速度圈/秒、电流A及状态位。
        base_interfaces::msg::McuReceive receivedData_{};
        uint32_t period_cnt_pc_ = 0; // period_cnt_pc_为构造新发送帧时递增的无量纲序号，限制到31位并按整数编码，不代表已完成发送。
        std::vector<uint8_t> rxCache_; // 接收字节流缓存，长度单位字节：USB/UART单次read不保证完整一帧。
        std::size_t txOffset_ = 0; // txOffset_为当前帧已发送字节数/下次续发偏移，单位字节，禁止与新帧交叉。
        bool txPending_ = false; // txPending_为当前帧是否仍有字节等待发送的布尔标志。
    }
    ;

}
// namespace mcu
