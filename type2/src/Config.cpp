#include "Config.hpp"

// 静态成员变量的定义（只定义一次）
Config* Config::instance_ = nullptr;
std::mutex Config::mutex_;