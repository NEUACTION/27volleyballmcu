/* 文件作用：声明POSIX串口资源封装与非阻塞字节读写接口，不负责协议拆帧或重连。 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Serial独占一个POSIX串口描述符；采用非阻塞读写，返回实际传输字节数，不保证整帧。
class Serial
{
    public:
    /* 中文名：串口对象构造。参数：port为设备路径，baudrate为波特率（bit/s）；
     * 返回：无；副作用：仅保存配置，不打开设备。 */
    Serial(std::string port, uint32_t baudrate);
    /* 中文名：串口对象析构。参数：无；返回：无；副作用：关闭已打开的描述符。 */
    ~Serial();
    // 禁止拷贝构造：未命名参数为源对象引用；无可调用实现、返回值或运行副作用，避免重复关闭串口。
    Serial(const Serial &) = delete;
    // 禁止拷贝赋值：未命名参数为源对象引用；返回类型为对象引用但该操作不可调用，无运行副作用。
    Serial &operator = (const Serial &) = delete;

    /* 中文名：打开并配置串口。参数：无；返回：打开且完成配置为true，否则false。
     * 副作用：打开port_，配置原始8N1及非阻塞收发；失败时关闭本次已打开的描述符。 */
    bool SerialInit();
    /* 中文名：非阻塞读串口。参数：buffer为可写接收区，length为其最大接收字节数。
     * 返回：实际字节数，暂不可读/被信号中断为0，参数无效或其他错误为-1。
     * 副作用：消费串口输入并写入buffer的已读部分，不补齐协议帧。 */
    int SerialRead(uint8_t *buffer, std::size_t length);
    /* 中文名：非阻塞写串口。参数：buffer为待发字节区，length为请求发送字节数。
     * 返回：实际字节数，暂不可写/被信号中断为0，参数无效或其他错误为-1。
     * 副作用：向串口提交至多length字节，可能短写，由调用者续发。 */
    int SerialWrite(const uint8_t *buffer, std::size_t length);

    private:
    std::string port_;       // 串口设备路径字符串，由ROS参数提供，无数值单位。
    uint32_t baudrate_;      // 请求波特率，单位bit/s；实现仅接受列出的三档速率。
    int handle_ = -1;         // handle_为POSIX文件描述符（无量纲），-1表示未连接。
}
;
