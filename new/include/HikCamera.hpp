#ifndef HIK_CAMERA_HPP
#define HIK_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>  // 海康MVS SDK头文件

// 海康相机封装类，提供初始化、取流、释放等功能
class HikCamera
{
public:
    HikCamera();
    ~HikCamera();

    // 初始化
    bool init();

    // 获取一帧图像
    bool getFrame(cv::Mat& frame);

    // 释放资源
    void release();

private:
    void* m_hCamera; // 海康相机句柄
    MV_CC_DEVICE_INFO_LIST stDeviceList; // 设备列表
};

#endif // HIK_CAMERA_HPP