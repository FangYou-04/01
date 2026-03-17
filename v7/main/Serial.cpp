#include "Serial.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

Serial::Serial() : fd(-1) {}

Serial::~Serial() 
{
    close();    
}

bool Serial::open(const char* device, speed_t baudrate) 
{
    fd = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd < 0) 
    {
        std::cout << "打开串口失败 " << strerror(errno) << std::endl;
        return false;
    }
    
    if (!configure(baudrate))
    {
        close();
        return false;
    }

    return true;
}

bool Serial::configure(speed_t baudrate) 
{
    struct termios options;
    if (tcgetattr(fd, &options) < 0) 
    {
        std::cout << "获取串口属性失败 " << strerror(errno) << std::endl;
        return false;
    }

    // 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收器
    options.c_cflag &= ~PARENB; // 无奇偶校验
    options.c_cflag &= ~CSTOPB; // 1停止位
    options.c_cflag &= ~CSIZE; // 清除数据位设置
    options.c_cflag |= CS8; // 8数据位
    options.c_cflag &= ~CRTSCTS; // 禁用硬件流控制

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 本地模式，原始输入

    // 输入模式设置
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    options.c_iflag &= ~(INLCR | ICRNL); // 禁止将换行转换为回车 

    options.c_oflag &= ~OPOST; // 原始输出

    // 设置 读取超时
    options.c_cc[VMIN] = 0; // 最小读取字符数
    options.c_cc[VTIME] = 10; // 读取超时，单位为0.1秒

    // 清空输入输出缓冲区
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) < 0) 
    {
        std::cout << "设置串口属性失败 " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

ssize_t Serial::write(const void* data, size_t len)
{
    return ::write(fd, data, len);
}

ssize_t Serial::read(void* buffer, size_t len)
{
    return ::read(fd, buffer, len);
}

void Serial::close() 
{
    if (fd >= 0) 
    {
        ::close(fd);
        fd = -1;
    }
}

bool Serial::is_open() const 
{
    return fd != -1;
}