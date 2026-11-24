/* 文件作用：实现Linux/POSIX串口的打开、8N1配置、非阻塞读写和析构关闭；不实现协议校验。 */
#include "mcu/serial.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

/* 中文名：保存串口配置。参数：port为设备路径，baudrate为波特率（bit/s）；
 * 返回：无；副作用：移动保存路径并保存速率，不执行设备I/O。 */
Serial::Serial(std::string port, uint32_t baudrate)
    : port_(std::move(port)), baudrate_(baudrate)
{
}

/* 中文名：释放串口。参数：无；返回：无；副作用：若handle_有效则关闭它，忽略close返回值。 */
Serial::~Serial()
{
    if (handle_ >= 0)
    {
            ::close(handle_);
    }
}

/* 中文名：初始化非阻塞串口。参数：无；返回：全部系统调用配置成功为true，否则false。
 * 副作用：打开设备并设置原始8N1、无硬件流控；配置失败关闭设备并将handle_置-1。 */
bool Serial::SerialInit()
{
    handle_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (handle_ < 0)
    {
        return false;
    }

    // settings为当前终端属性的本地副本；后续修改后整体提交，无统一数值单位。
    termios settings{};
    if (::tcgetattr(handle_, &settings) != 0)
    {
            ::close(handle_);
        handle_ = -1;
        return false;
    }
        ::cfmakeraw(&settings);
    speed_t speed = 0; // termios速率常量编码，不是以bit/s表示的原始整数。
    switch (baudrate_)
    {
        case 921600: speed = B921600; break;
        case 460800: speed = B460800; break;
        case 115200: speed = B115200; break;
        default:
            ::close(handle_);
        handle_ = -1;
        return false;
    }
    if (::cfsetispeed(&settings, speed) != 0 || ::cfsetospeed(&settings, speed) != 0)
    {
            ::close(handle_);
        handle_ = -1;
        return false;
    }
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CSTOPB;
    // 参考链路为三线8N1，不继承终端上次留下的RTS/CTS硬件流控。
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::tcsetattr(handle_, TCSANOW, &settings) != 0)
    {
            ::close(handle_);
        handle_ = -1;
        return false;
    }
    return true;
}

/* 中文名：读取当前可用串口字节。参数：buffer为可写缓冲区，length为容量（字节）。
 * 返回：读入字节数；EAGAIN/EWOULDBLOCK/EINTR映射为0；无效参数或其他错误为-1。
 * 副作用：读设备，成功时修改buffer前返回值个字节；不阻塞等待整帧。 */
int Serial::SerialRead(uint8_t *buffer, std::size_t length)
{
    if (handle_ < 0 || buffer == nullptr || length == 0) return -1;
    const ssize_t count = ::read(handle_, buffer, length); // 本次系统读操作的字节数，-1表示系统错误。
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    return static_cast<int>(count);
}

/* 中文名：提交当前串口发送字节。参数：buffer为只读数据区，length为请求字节数。
 * 返回：已写字节数；EAGAIN/EWOULDBLOCK/EINTR映射为0；无效参数或其他错误为-1。
 * 副作用：向设备发送数据，可能短写；不修改buffer，不自行续发。 */
int Serial::SerialWrite(const uint8_t *buffer, std::size_t length)
{
    if (handle_ < 0 || buffer == nullptr || length == 0) return -1;
    const ssize_t count = ::write(handle_, buffer, length); // 本次系统写操作的字节数，-1表示系统错误。
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    return static_cast<int>(count);
}
