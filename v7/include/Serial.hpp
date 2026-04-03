#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <termios.h>
#include <cstddef>
#include <cstdint>

class Serial
{
public:
    Serial();
    ~Serial();

    // 打开串口
    bool open(const char* device, speed_t baudrate);

    // 发送数据
    ssize_t write(const void* data, size_t len);

    // 接收数据
    ssize_t read(void* buffer, size_t len);

    // 关闭串口
    void close();

    // 检查串口是否打开
    bool is_open() const; 

private:
    int fd; // 文件描述符
    
    bool configure(speed_t baudrate);
};

#endif // SERIAL_HPP   

