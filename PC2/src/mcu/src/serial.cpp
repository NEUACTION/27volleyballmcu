#include "mcu/serial.hpp"

Serial::Serial(const char *port, uint32_t baudrate)
{
    this_port = port;
    this_baudrate = baudrate;
    std::cout << "port: " <<port<< " baud rate: " << baudrate << std::endl;
}

void Serial::SerialInit()
{
    //打开串口并获取句柄 O_RDWR可读写 O_NOCTTY该设备不作为控制终端 O_NONBLOCK读取不到数据或是写入缓冲区已满会马上return，而不会阻塞等待
    handle = open(this_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    std::cout << "handle:" << handle << std::endl;

    if (handle == -1)//如果打开失败，循环，持续打开
    {
        perror("open_port: Unable to open serial");//输出该句并在后面输出错误信息
        return;
    }

    /* 串口相关配置 */
    tcgetattr(handle, &termAttr);//获取与终端相关的参数

    cfsetispeed(&termAttr, this_baudrate);//设置输入波特率
    cfsetospeed(&termAttr, this_baudrate);//设置输出波特率

    /*  c_iflag：输入模式标志，控制终端输入方式
        c_oflag：输出模式标志，控制终端输出方式
        c_cflag：控制模式标志，指定终端硬件控制信息
    */
    termAttr.c_cflag &= ~PARENB;    //不使用奇偶校验
    termAttr.c_cflag &= ~CSTOPB;    //禁用两个停止位 即设置一位停止位
    termAttr.c_cflag &= ~CSIZE;     //字符长度清空 
    termAttr.c_cflag |= CS8;        //字符长度设置CS8
    termAttr.c_cflag |= (CLOCAL | CREAD);   //忽略调制解调器线路状态 | 使用接收器

    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//禁用 规范输入 进行回送 可见擦除字符 终端产生的信号 
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);//禁用 软件流控制 
    termAttr.c_oflag &= ~OPOST;//禁用 输出处理
    termAttr.c_oflag &= ~(ONLCR | OCRNL); //禁用 ONLCR:将'\n'转换成'\r'和'\n' OCRNL:将'\r'转换成'\n'
    termAttr.c_iflag &= ~(INLCR | IGNCR | ICRNL); //禁用 INLCR：将'\n'转换成'\r' IGNCR：忽略输入的回车 ICRNL：将'\r'转换成'\n'
    termAttr.c_cc[VMIN] = 0;//需读取的最小字节数 为0
    termAttr.c_cc[VTIME] = 1;//限定的传输或等待的最长时间 +
    tcflush(handle, TCIOFLUSH);//清空所有正在发生的IO数据
    tcsetattr(handle, TCSANOW, &termAttr);//应用上述配置

    std::cout << "Serial configured....\n";
}

//串口读取数据
int Serial::SerialRead(uint8_t *rxbuf, uint32_t len) 
{
    int lenR = read(handle, rxbuf, len);

    return lenR;
}

//串口发送数据
int Serial::SerialWrite(uint8_t *txbuf, uint32_t len)
{
    int lenW = write(handle, txbuf, len);
    std::cout << "tx_send:"<<lenW << "\t"<<std::endl;
    return lenW;
}
