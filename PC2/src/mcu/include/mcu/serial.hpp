#ifndef _SERIAL_H
#define _SERIAL_H

#include <cstdint>
#include <iostream>
#include <termios.h>
#include <fcntl.h> //包含open的库
#include <unistd.h>//包含read write的库
#include <cstring>

class Serial
{
    public:
        Serial(const char *port, uint32_t baudrat);
        void SerialInit();
        int handle;
        int SerialRead(uint8_t *rxbuf, uint32_t len);
        int SerialWrite(uint8_t *txbuf, uint32_t len);
    private:
        const char *this_port;
        uint32_t this_baudrate;
        struct termios termAttr;//修改终端参数
        
};

#endif