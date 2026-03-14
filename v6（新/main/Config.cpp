#include "Congfig.hpp"

// 初始化静态成员
Config* Config::instance_ = nullptr;
std::mutex Config::mutex_;