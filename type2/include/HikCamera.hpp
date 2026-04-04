#ifndef HIK_CAMERA_HPP
#define HIK_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>

class HikCamera {
public:
    HikCamera();
    ~HikCamera();
    bool init();
    bool getFrame(cv::Mat& frame);
    void release();

private:
    void* m_hCamera;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
};

#endif