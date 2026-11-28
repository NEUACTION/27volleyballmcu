#include "mcu/mcuNode.hpp"
#include <cmath>
#include <cstdio>

using namespace std;

trans_T trans_t;
trans_R trans_r;
// trans_T R1Star_t;
// trans_R R1Star_r;
mcuCom comMCU;
bool ControlDataComFlag;
int MapNum[12];
int R1PosData[12] = {0};
float R2PosX;
float R2PosY;

mcuNode::mcuNode(): Node("mcuNode")
{
    logger = common_logger::Logger::getInstance("mcuNode");
    logger->info("mcuNode init...");

    mcuPublisher = create_publisher<base_interfaces::msg::McuReceive>("/mcu/receive", 10);
    controlListner = create_subscription<base_interfaces::msg::McuTransmit>(
        "/mcu/transmit", 10,
        std::bind(&mcuNode::ControlListnerCallback, this, std::placeholders::_1));
         
    pps_msg_subscriber = create_subscription<base_interfaces::msg::PosData>(
        "/pps_data", 10,
        std::bind(&mcuNode::WheelPPSListenCallback, this, std::placeholders::_1));//定位系统
 
    timer_ = create_wall_timer(5ms, std::bind(&mcuNode::timer_callback, this));

    running_ = true;
    mcu_rx_thread_  = std::thread(&mcuNode::mcuRxLoop,  this);
    // star_rx_thread_ = std::thread(&mcuNode::starRxLoop, this);

    logger->info("mcuNode started.");
}

// ===== 线程循环 =====
void mcuNode::mcuRxLoop() {
    while (rclcpp::ok() && running_) 
    {
        comMCU.lenRead = comMCU.mcuComSerial ?
                         comMCU.mcuComSerial->SerialRead(comMCU.rxDataMcu, RX_DATA_LEN_MCU) : 0;
        if (comMCU.lenRead > 0 && comMCU.CheckRxData(comMCU.lenRead)) 
        {
            this->logger->info("MCU Data Received");
            std::lock_guard<std::mutex> lk(data_mutex_);

            
            AnalyzeData();
        } 
        comMCU.lenRead = 0;
        std::memset(comMCU.rxDataMcu, 0, sizeof(comMCU.rxDataMcu));
    }
}
 
// void mcuNode::starRxLoop() {
//     while (rclcpp::ok() && running_) {
//         comMCU.StarlenRead = comMCU.starComSerial ?
//                           comMCU.starComSerial->SerialRead(comMCU.rxDataStar, RX_DATA_LEN_STAR) : 0;
//         if (comMCU.StarlenRead > 0 && comMCU.CheckStarRxData(comMCU.StarlenRead)) {
//             this->logger->info("Star Data Received");
//             AnalyzeStarData();
//         }
//         comMCU.StarlenRead = 0;
//         std::memset(comMCU.rxDataStar, 0, sizeof(comMCU.rxDataStar));
//     }
// }
 
/*??*/
void mcuNode::WheelPPSListenCallback(const base_interfaces::msg::PosData::SharedPtr msg)
{
    myX = msg->x;
    myY = msg->y;
    myYaw = msg->yaw;
}

//构造函数，初始化一个串口类
mcuCom::mcuCom()
{
    std::string Mcu_Serial_Name = "/dev/mcu";
    int baudrate = B921600;
    // std::string Star_Serial_Name = "/dev/star";
    mcuComSerial = new Serial(Mcu_Serial_Name.c_str(), baudrate);
    mcuComSerial -> SerialInit();
     
    // starComSerial = new Serial(Star_Serial_Name.c_str(), baudrate);
    // starComSerial -> SerialInit();
     
}

//析构函数，释放该串口类
mcuCom::~mcuCom(void)
{
    delete mcuComSerial;
    // delete starComSerial;
}

void mcuNode::AnalyzeData()//匹配主控返回来的数据
{
    //用来占位的消息类型，根据实际需要到messagetype/msg/McuReceived.msg改
    receivedData.heartbag.mcu_hb = 0;
    
    receivedData.heartbag.mcu_to_pc_hb = comMCU.getDataMcu[0];
    receivedData.heartbag.leg1_hb = comMCU.getDataMcu[1];
    receivedData.heartbag.leg2_hb = comMCU.getDataMcu[2];
    receivedData.heartbag.leg3_hb = comMCU.getDataMcu[3];
    receivedData.heartbag.leg4_hb = comMCU.getDataMcu[4];

    receivedData.heartbag.wheel1_hb = comMCU.getDataMcu[5];
    receivedData.heartbag.wheel2_hb = comMCU.getDataMcu[6];
    receivedData.heartbag.wheel3_hb = comMCU.getDataMcu[7];
    receivedData.heartbag.wheel4_hb = comMCU.getDataMcu[8];

    receivedData.heartbag.plate_hb = comMCU.getDataMcu[9];
    receivedData.heartbag.slide_hb = comMCU.getDataMcu[10];
    receivedData.heartbag.arm1_hb = comMCU.getDataMcu[11];
    receivedData.heartbag.arm2_hb = comMCU.getDataMcu[12];


    receivedData.ps_num1 = comMCU.getDataMcu[13];


    receivedData.mcuflag.reset_flag = comMCU.getDataMcu[14];

    receivedData.leg1.error = comMCU.getDataMcu[15];
    receivedData.leg2.error = comMCU.getDataMcu[16];
    receivedData.leg3.error = comMCU.getDataMcu[17];
    receivedData.leg4.error = comMCU.getDataMcu[18];

    receivedData.plate.error = comMCU.getDataMcu[19];
    receivedData.slide.error = comMCU.getDataMcu[20];
    receivedData.arm1.error = comMCU.getDataMcu[21];
    receivedData.arm2.error = comMCU.getDataMcu[22];

    receivedData.mcuflag.only_need_start_flag = comMCU.getDataMcu[23];

    // receivedData.leg1.enablestate = comMCU.getDataMcu[25];
    // receivedData.leg2.enablestate = comMCU.getDataMcu[26];
    // receivedData.leg3.enablestate = comMCU.getDataMcu[27];
    // receivedData.leg4.enablestate = comMCU.getDataMcu[28];

    //std::cout << "ps_num: " << (int)receivedData.ps_num << std::endl;
    std::cout << "ps_num:" ;
    for(int  i = 0 ;i < 8; ++i){
        cout << ((receivedData.ps_num1 >> (i)) & 0x01) << " "; 
    }
    cout << endl;

    for (int i = RX_UINT8_NUM_MCU; i < RX_DATA_LEN_MCU - 4; i++)
    {
        trans_r.datac[i - RX_UINT8_NUM_MCU] = comMCU.getDataMcu[i];
    }

    this->logger->info("leg1Pos: {}", receivedData.leg1.motor_pos);

    receivedData.leg1.motor_pos = trans_r.dataf[1];
    receivedData.leg1.motor_vel = trans_r.dataf[2];
    receivedData.leg1.motor_cur = trans_r.dataf[3];

    receivedData.leg2.motor_pos = -trans_r.dataf[4];
    receivedData.leg2.motor_vel = -trans_r.dataf[5];
    receivedData.leg2.motor_cur = -trans_r.dataf[6];

    receivedData.leg3.motor_pos = trans_r.dataf[7];
    receivedData.leg3.motor_vel = trans_r.dataf[8];
    receivedData.leg3.motor_cur = trans_r.dataf[9];

    receivedData.leg4.motor_pos = -trans_r.dataf[10];
    receivedData.leg4.motor_vel = -trans_r.dataf[11];
    receivedData.leg4.motor_cur = -trans_r.dataf[12];

    receivedData.wheel1.pos = trans_r.dataf[13];
    receivedData.wheel1.vel = trans_r.dataf[14];
    receivedData.wheel1.pos_cur = trans_r.dataf[15];
    receivedData.wheel1.vel_cur = trans_r.dataf[16];

    receivedData.wheel2.pos = trans_r.dataf[17];
    receivedData.wheel2.vel = trans_r.dataf[18];
    receivedData.wheel2.pos_cur = trans_r.dataf[19];
    receivedData.wheel2.vel_cur = trans_r.dataf[20];

    receivedData.wheel3.pos = trans_r.dataf[21];
    receivedData.wheel3.vel = trans_r.dataf[22];
    receivedData.wheel3.pos_cur = trans_r.dataf[23];
    receivedData.wheel3.vel_cur = trans_r.dataf[24];

    receivedData.wheel4.pos = trans_r.dataf[25];
    receivedData.wheel4.vel = trans_r.dataf[26];
    receivedData.wheel4.pos_cur = trans_r.dataf[27];
    receivedData.wheel4.vel_cur = trans_r.dataf[28];

    receivedData.plate.motor_pos = trans_r.dataf[29];
    receivedData.plate.motor_vel = trans_r.dataf[30];
    receivedData.plate.motor_cur = trans_r.dataf[31];

    receivedData.slide.motor_pos = trans_r.dataf[32];
    receivedData.slide.motor_vel = trans_r.dataf[33];
    receivedData.slide.motor_cur = trans_r.dataf[34];

    receivedData.arm1.motor_pos = trans_r.dataf[35];
    receivedData.arm1.motor_vel = trans_r.dataf[36];
    receivedData.arm1.motor_cur = trans_r.dataf[37];

    receivedData.arm2.motor_pos = -trans_r.dataf[38];
    receivedData.arm2.motor_vel = -trans_r.dataf[39];
    receivedData.arm2.motor_cur = -trans_r.dataf[40];
}


// void mcuNode::AnalyzeStarData()
// {
//     this->logger->info("ReceiveR1Data: {} {}",(char)R1PosData[0],(char)R1PosData[1]);
    
//     // if (R1PosData[0] == 'T' && R1PosData[0] == 'E')
//     // {
//     //     comMCU.txDataStar[0] = 'R';
//     //     comMCU.txDataStar[1] = 'C';
//     //     comMCU.txDataStar[2] = 'T';
//     //     comMCU.txDataStar[3] = 'E';
//     //     comMCU.txDataStar[4] = 'R';
//     //     comMCU.txDataStar[5] = '2';
//     //     comMCU.txDataStar[6] = 'O';
//     //     comMCU.txDataStar[7] = 'K';
//     //     comMCU.txDataStar[12] = '\r';
//     //     comMCU.txDataStar[13] = '\n';

//     //     comMCU.Send2R1();
//     // }

//     if(R1PosData[0] == 'S' && R1PosData[1] == 'M')
//     {
//         bool R1PlatformFlag = (R1PosData[2]) & 1;
//         bool R1MoveFlag = (R1PosData[3]) & 1;
//         bool R2PutFlag = (R1PosData[4]) & 1;
//         bool R2GoFlag = (R1PosData[5]) & 1;
//         bool R1Area3Flag = (R1PosData[6]) & 1;
//         bool R1PlateHigh = (R1PosData[7]) & 1;
//         char R1MoveDirection = R1PosData[8];
//         if (R1MoveDirection == 'I')
//         {
//             receivedData.r1movedirection = -1;
//         }
//         else if (R1MoveDirection == 'O')
//         {
//             receivedData.r1movedirection = 1;
//         }
//         else
//         {
//             receivedData.r1movedirection = 0;
//         }

//         receivedData.r1platformflag = R1PlatformFlag;
//         receivedData.r1moveflag = R1MoveFlag;
//         receivedData.r2putflag = R2PutFlag;
//         receivedData.r2goflag = R2GoFlag;
//         receivedData.r1area3flag = R1Area3Flag;
//         receivedData.r1platehigh = R1PlateHigh;

//         for (int i = RX_UINT8_NUM_STAR; i < RX_DATA_LEN_STAR - 6; i++)
//         {
//             R1Star_r.datac[i - RX_UINT8_NUM_STAR] = R1PosData[i + 2];
//         }

//         this->logger->info("ReceiveR1Platfrom: {}", R1PlatformFlag);
//         this->logger->info("R2PutFlag {}", R2PutFlag);
//         this->logger->info("R2GoFlag {}", R2GoFlag);
//         this->logger->info("R1Area3Flag {}", R1Area3Flag);
//     } 

//     if(R1PosData[0] == 'G' && R1PosData[1] == 'D')
//     {
//         int the_way2_num;
//         int the_Action[5];
//         int the_firstR2_KFS;
//         int dodgeR1;
//         the_way2_num = R1PosData[2];
//         the_Action[1] = R1PosData[3];
//         the_Action[2] = R1PosData[4];
//         the_Action[3] = R1PosData[5];
//         the_Action[4] = R1PosData[6];
//         the_firstR2_KFS = R1PosData[7];
//         dodgeR1 = R1PosData[8] - '0';

//         this->logger->info("the_way2_num: {}",the_way2_num);

//         receivedData.the_way2_num = the_way2_num - '0';
//         receivedData.firstr2kfs = the_firstR2_KFS - '0';
//         receivedData.dodger1 = dodgeR1;
//         this->logger->info("dodgeR1:{}", receivedData.dodger1);

//         for(int i = 1; i < 5; i++)
//         {
//             receivedData.action[i] = the_Action[i] - '0';
//         }

//     }
// }

void mcuNode::timer_callback()
{
    static int CtlMcu_HB = 0;

    receivedData.heartbag.mcu_hb++;

    if (receivedData.heartbag.mcu_hb > 200)
    {
        receivedData.heartbag.mcu_hb = 200;
    }

    this->logger->info("MCU Heartbeat: {}", receivedData.heartbag.mcu_hb);

    if (!ControlDataComFlag)
    {
        if(CtlMcu_HB <= 200)
            CtlMcu_HB++;
    }
    else CtlMcu_HB = 0;
    ControlDataComFlag = 0;
    if (comMCU.Send2MCU())
    {
        this->logger->info("Send to MCU Successed");
    }
    else
    {
        this->logger->error("Send to MCU Failed");
    }
    // if (comMCU.Send2R1())
    // {
    //     this->logger->info("Send to Star Successed");
    // }
    // else
    // {
    //     this->logger->error("Send to Star Failed");
    // }

    base_interfaces::msg::McuReceive snap;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        snap = receivedData;
    }
    mcuPublisher->publish(snap);
}

void mcuNode::ControlListnerCallback(const base_interfaces::msg::McuTransmit::SharedPtr controlMsg)
//从control节点收到数据之后，发给主控
{
    cout << "publishToMcu" << endl;
    ControlDataComFlag = 1;
    this->getMcuErrorFlag = controlMsg->pcflag.get_mcu_error_flag;
    //获取要向mcu发的数后，向发送数组填数
    //赋值帧头帧尾
    comMCU.txDataMcu[0] = comMCU.frame_header_write1;
    comMCU.txDataMcu[1] = comMCU.frame_header_write2;
    comMCU.txDataMcu[TX_DATA_LEN_MCU-2] = comMCU.frame_tail_write1;
    comMCU.txDataMcu[TX_DATA_LEN_MCU-1] = comMCU.frame_tail_write2;

    /* uint8_t型数据 */
    comMCU.txDataMcu[2] = controlMsg->pcflag.run_successed_flag;
    
    // comMCU.txDataMcu[3] =
    //     ((controlMsg->leg1.controlmode  & 0x03) << 0) |
    //     ((controlMsg->leg2.controlmode  & 0x03) << 2) |
    //     ((controlMsg->leg3.controlmode  & 0x03) << 4) |
    //     ((controlMsg->leg4.controlmode  & 0x03) << 6);

    // comMCU.txDataMcu[4] =
    //     ((controlMsg->wheel1.controlmode & 0x03) << 0) |
    //     ((controlMsg->wheel2.controlmode & 0x03) << 2) |
    //     ((controlMsg->wheel3.controlmode & 0x03) << 4) |
    //     ((controlMsg->wheel4.controlmode & 0x03) << 6); 
        
    // comMCU.txDataMcu[5] = controlMsg->vavle_num1;

    // comMCU.txDataMcu[6] =
    //     ((controlMsg->shoulder.controlmode  & 0x03) << 0) |
    //     ((controlMsg->elbow.controlmode  & 0x03) << 2) |
    //     ((controlMsg->wrist.controlmode  & 0x03) << 4);
    
    // comMCU.txDataMcu[7] = controlMsg->vavle_num2;
    
    trans_t.dataint32[0] = controlMsg->period_cnt_pc;


    /* float型数据 */
    trans_t.dataf[1]     = controlMsg->leg1.motor_pos;
    trans_t.dataf[2]     = controlMsg->leg1.motor_vel;
    trans_t.dataf[3]     = controlMsg->leg1.motor_cur;
    trans_t.dataf[4]     = controlMsg->leg1.motor_pos_kp;
    trans_t.dataf[5]     = controlMsg->leg1.motor_pos_kd;
    trans_t.dataf[6]     = controlMsg->leg1.motor_vel_kp;
    trans_t.dataf[7]     = controlMsg->leg1.motor_vel_ki;

    trans_t.dataf[8]     = -controlMsg->leg2.motor_pos;
    trans_t.dataf[9]     = -controlMsg->leg2.motor_vel;
    trans_t.dataf[10]    = -controlMsg->leg2.motor_cur;
    trans_t.dataf[11]    = controlMsg->leg2.motor_pos_kp;
    trans_t.dataf[12]    = controlMsg->leg2.motor_pos_kd;
    trans_t.dataf[13]    = controlMsg->leg2.motor_vel_kp;
    trans_t.dataf[14]    = controlMsg->leg2.motor_vel_ki;

    trans_t.dataf[15]    = controlMsg->leg3.motor_pos;
    trans_t.dataf[16]    = controlMsg->leg3.motor_vel;
    trans_t.dataf[17]    = controlMsg->leg3.motor_cur;
    trans_t.dataf[18]    = controlMsg->leg3.motor_pos_kp;
    trans_t.dataf[19]    = controlMsg->leg3.motor_pos_kd;
    trans_t.dataf[20]    = controlMsg->leg3.motor_vel_kp;
    trans_t.dataf[21]    = controlMsg->leg3.motor_vel_ki;

    trans_t.dataf[22]    = -controlMsg->leg4.motor_pos;
    trans_t.dataf[23]    = -controlMsg->leg4.motor_vel;
    trans_t.dataf[24]    = -controlMsg->leg4.motor_cur;
    trans_t.dataf[25]    = controlMsg->leg4.motor_pos_kp;
    trans_t.dataf[26]    = controlMsg->leg4.motor_pos_kd;
    trans_t.dataf[27]    = controlMsg->leg4.motor_vel_kp;
    trans_t.dataf[28]    = controlMsg->leg4.motor_vel_ki;

    trans_t.dataf[29]    = controlMsg->plate.motor_vel;
    trans_t.dataf[30]    = controlMsg->plate.motor_cur;
    trans_t.dataf[31]    = controlMsg->plate.motor_vel_kp;
    trans_t.dataf[32]    = controlMsg->plate.motor_vel_ki;

    trans_t.dataf[33]    = controlMsg->arm1.motor_pos;
    trans_t.dataf[34]    = controlMsg->arm1.motor_pos_kp;
    trans_t.dataf[35]    = controlMsg->arm1.motor_pos_kd;
    trans_t.dataf[36]    = controlMsg->arm1.motor_vel_kp;
    trans_t.dataf[37]    = controlMsg->arm1.motor_vel_ki;

    trans_t.dataf[38]    = -controlMsg->arm2.motor_pos;
    trans_t.dataf[39]    = controlMsg->arm2.motor_pos_kp;
    trans_t.dataf[40]    = controlMsg->arm2.motor_pos_kd;
    trans_t.dataf[41]    = controlMsg->arm2.motor_vel_kp;
    trans_t.dataf[42]    = controlMsg->arm2.motor_vel_ki;

    trans_t.dataf[43]    = controlMsg->slide.motor_pos;

    trans_t.dataf[44]    = controlMsg->wheel1.pos;
    trans_t.dataf[45]    = controlMsg->wheel1.vel;

    trans_t.dataf[46]    = controlMsg->wheel2.pos;
    trans_t.dataf[47]    = controlMsg->wheel2.vel;

    trans_t.dataf[48]    = controlMsg->wheel3.pos;
    trans_t.dataf[49]    = controlMsg->wheel3.vel;

    trans_t.dataf[50]    = controlMsg->wheel4.pos;
    trans_t.dataf[51]    = controlMsg->wheel4.vel;

    trans_t.dataf[52]    = controlMsg->leg1.mit_pos;
    trans_t.dataf[53]    = controlMsg->leg1.mit_vel;
    trans_t.dataf[54]    = controlMsg->leg1.mit_pos_kd;
    trans_t.dataf[55]    = controlMsg->leg1.mit_vel_kp;
    trans_t.dataf[56]    = controlMsg->leg1.mit_cur;

    trans_t.dataf[57]    = controlMsg->leg2.mit_pos;
    trans_t.dataf[58]    = controlMsg->leg2.mit_vel;
    trans_t.dataf[59]    = controlMsg->leg2.mit_pos_kd;
    trans_t.dataf[60]    = controlMsg->leg2.mit_vel_kp;
    trans_t.dataf[61]    = controlMsg->leg2.mit_cur;

    trans_t.dataf[62]    = controlMsg->leg3.mit_pos;
    trans_t.dataf[63]    = controlMsg->leg3.mit_vel;
    trans_t.dataf[64]    = controlMsg->leg3.mit_pos_kd;
    trans_t.dataf[65]    = controlMsg->leg3.mit_vel_kp;
    trans_t.dataf[66]    = controlMsg->leg3.mit_cur;

    trans_t.dataf[67]    = controlMsg->leg4.mit_pos;
    trans_t.dataf[68]    = controlMsg->leg4.mit_vel;
    trans_t.dataf[69]    = controlMsg->leg4.mit_pos_kd;
    trans_t.dataf[70]    = controlMsg->leg4.mit_vel_kp;
    trans_t.dataf[71]    = controlMsg->leg4.mit_cur;

    trans_t.dataf[72]    = controlMsg->mode;
    trans_t.dataf[73]    = controlMsg->start;
    logger_->info("plvel:{}", controlMsg->plate.motor_vel);
    // logger_->info("Clean:{}", controlMsg->wheel1.clean);

    for (int i = 0; i < TX_FLOAT_NUM_MCU * 4; i++)
    {
        comMCU.txDataMcu[2 + TX_UINT8_NUM_MCU + i] = trans_t.datac[i];
    }

    // comMCU.txDataStar[0] = 'R';
    // comMCU.txDataStar[1] = 'C';
    // comMCU.txDataStar[2] = 'S';
    // comMCU.txDataStar[3] = controlMsg->pcflag.upr1_flag;
    // R1Star_t.dataf[0] = controlMsg->r2posx;
    // R1Star_t.dataf[1] = controlMsg->r2posy;
    // comMCU.txDataStar[12] = '\r';
    // comMCU.txDataStar[13] = '\n';
    for (int i = 2 + TX_UINT8_NUM_STAR; i < RX_FLOAT_NUM_STAR * 4 + 2; i++)
    {
        // comMCU.txDataStar[2 + i] = R1Star_t.datac[i - 2];
    }

    this->logger->info("upr1_flag:{}", controlMsg->pcflag.upr1_flag);
}

bool mcuNode::GetActiveFlag()
{
    return(activeFlag);
}

//与主控通信???
bool mcuCom::Send2MCU()
{
    //获取时间戳以查看通信用时间
    std::cout << txDataMcu[0] << "\t" << txDataMcu[1] << "\t" << (int)txDataMcu[14] << "\t" << (int)txDataMcu[15]
        << "\t" << (int)txDataMcu[16] << "\t" << (int)txDataMcu[17] << std::endl;
    if (mcuComSerial->SerialWrite(txDataMcu, TX_DATA_LEN_MCU) == -1) // 发送 并判断是否成功
    {
        return false;
    }
    else
    {
        return true;
    }

}
// bool mcuCom::Send2R1(){
//     if (starComSerial->SerialWrite(txDataStar, TX_DATA_LEN_STAR) == -1) // 发送 并判断是否成功
//     {
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// }
/*
 * @brief 遍历从主控接收到的数据 找到帧头帧尾
 * @return true--成功分析并赋值给数组 false--帧头帧尾错误，对不上
 */
bool mcuCom::CheckRxData(int len)
{
    for (int i = len - RX_DATA_LEN_MCU; i >= 0; i--)//匹配帧头帧尾 从后往前找，以使用最新数据  0～len-LEN
    {
        if (rxDataMcu[i] == frame_header_read1 && rxDataMcu[i+1] == frame_header_read2 &&\
            rxDataMcu[i + RX_DATA_LEN_MCU - 2] == frame_tail_read1 &&\
            rxDataMcu[i + RX_DATA_LEN_MCU - 1] == frame_tail_read2)
        {
            for (int j = 0; j < RX_DATA_LEN_MCU - 4; j++)
            {
                getDataMcu[j] = rxDataMcu[i + 2 + j];
            }
            return true;
        }

    }
    return false;
}

// bool mcuCom::CheckStarRxData(int len)
// {
//     for (int i = len - RX_DATA_LEN_STAR; i >= 0; i--)//匹配帧头帧尾 从后往前找，以使用最新数据  0～len-LEN
//     {
//         // std::cout << "Star: " << rxDataStar[i] << " " << rxDataStar[i+1] << std::endl;
//         if (rxDataStar[i] == 'R' && rxDataStar[i+1] == 'C' && rxDataStar[i + RX_DATA_LEN_STAR - 2] == '\r' && rxDataStar[i + RX_DATA_LEN_STAR - 1] == '\n')
//         {
//             for (int j = 0; j < RX_DATA_LEN_STAR - 4; j++)
//             {
//                 R1PosData[j] = rxDataStar[i + 2 + j];
//             }
//             return true;
//         }

//     }
//     return false;
// }

mcuNode::~mcuNode()
{
    running_ = false;
    if (mcu_rx_thread_.joinable())  mcu_rx_thread_.join();
    // if (star_rx_thread_.joinable()) star_rx_thread_.join();
}
