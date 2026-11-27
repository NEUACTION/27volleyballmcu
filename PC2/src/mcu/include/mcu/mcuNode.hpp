#pragma once
#ifndef _MCU_NODE_H_
#define _MCU_NODE_H_

#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/mcu_receive.hpp"
#include "base_interfaces/msg/mcu_transmit.hpp"
#include "base_interfaces/msg/pos_data.hpp"
#include "common_logger/common_logger.hpp"
#include "mcu/serial.hpp"
#include <unistd.h>

#define MAX_GET_LEN_MCU  (4000)
#define MAX_GET_LEN_STAR (4000)

#define TX_UINT8_NUM_MCU (10)
#define TX_FLOAT_NUM_MCU (73)//77
#define RX_UINT8_NUM_MCU (25)
#define RX_FLOAT_NUM_MCU (41)//43

#define TX_UINT8_NUM_STAR (0)
#define TX_FLOAT_NUM_STAR (2)
#define TX_DATA_LEN_STAR  (2 + 2 + TX_UINT8_NUM_STAR + TX_FLOAT_NUM_STAR * 4 + 2)
#define RX_UINT8_NUM_STAR (4)
#define RX_FLOAT_NUM_STAR (2)
#define RX_DATA_LEN_STAR  (2 + 2 + RX_UINT8_NUM_STAR + RX_FLOAT_NUM_STAR * 4 + 2)//2帧头+2标志位+数据位+2帧尾

#define TX_DATA_LEN_MCU (4 + TX_UINT8_NUM_MCU + 4 * TX_FLOAT_NUM_MCU)
#define RX_DATA_LEN_MCU (4 + RX_UINT8_NUM_MCU + 4 * RX_FLOAT_NUM_MCU)

using namespace std::chrono_literals;

union trans_T {
    float     dataf[TX_FLOAT_NUM_MCU];
    int       dataint[TX_FLOAT_NUM_MCU];
    uint32_t  dataint32[TX_FLOAT_NUM_MCU];
    char      datac[TX_FLOAT_NUM_MCU * 4];
};

union trans_R {
    float     dataf[RX_FLOAT_NUM_MCU];
    int       dataint[RX_FLOAT_NUM_MCU];
    uint32_t  dataint32[RX_FLOAT_NUM_MCU];
    char      datac[RX_FLOAT_NUM_MCU * 4];
};

class mcuCom {
public:
    explicit mcuCom();
    ~mcuCom();

    Serial*  mcuComSerial = nullptr;
    Serial*  starComSerial = nullptr;

    uint8_t  txDataMcu[TX_DATA_LEN_MCU]{};
    uint8_t  rxDataMcu[MAX_GET_LEN_MCU]{};
    uint8_t  getDataMcu[RX_DATA_LEN_MCU - 4]{};

    uint8_t  rxDataStar[MAX_GET_LEN_STAR]{};
    uint8_t  getDataStar[RX_DATA_LEN_STAR - 4]{};
    uint8_t  txDataStar[TX_DATA_LEN_STAR]{};

    int      lenRead = 0;
    int      StarlenRead = 0;
    bool     receivedFlag = false;

    const char frame_header_read1  = 'Q';
    const char frame_header_read2  = 'R';
    const char frame_tail_read1    = '\r';
    const char frame_tail_read2    = '\n';

    const char frame_header_write1 = 'S';
    const char frame_header_write2 = 'P';
    const char frame_tail_write1   = '\r';
    const char frame_tail_write2   = '\n';

    bool Send2MCU();
    bool Send2R1();
    bool CheckRxData(int len);
    bool CheckStarRxData(int len);

    std::mutex tx_mutex;
};

class mcuNode : public rclcpp::Node {
public:
    mcuNode();
    ~mcuNode() override;

    rclcpp::Publisher<base_interfaces::msg::McuReceive>::SharedPtr      mcuPublisher;
    rclcpp::Subscription<base_interfaces::msg::McuTransmit>::SharedPtr  controlListner;
    rclcpp::Subscription<base_interfaces::msg::PosData>::SharedPtr      pps_msg_subscriber;
    rclcpp::TimerBase::SharedPtr                                        timer_;

    base_interfaces::msg::McuReceive receivedData{};
    float myX = 0.f;
    float myY = 0.f;
    float myYaw = 0.f;
    bool  activeFlag = true;
    bool  getMcuErrorFlag = false;

    std::shared_ptr<common_logger::Logger> logger;

    void timer_callback();
    void ControlListnerCallback(const base_interfaces::msg::McuTransmit::SharedPtr msg);
    void WheelPPSListenCallback(const base_interfaces::msg::PosData::SharedPtr msg);
    void AnalyzeData();
    void AnalyzeStarData();
    bool GetActiveFlag();

private:
    std::thread mcu_rx_thread_;
    std::thread star_rx_thread_;
    std::atomic<bool> running_{false};
    std::mutex data_mutex_;

    void mcuRxLoop();
    void starRxLoop();
};

extern trans_T trans_t;
extern trans_R trans_r;
extern mcuCom comMCU;
extern std::shared_ptr<mcuNode> mcu_node;

#endif