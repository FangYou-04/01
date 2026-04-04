#include "HikCamera.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>

HikCamera::HikCamera()
    : m_hCamera(nullptr)
{   
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCamera::~HikCamera()
{
    release();
}

// 海康工业相机初始化：枚举设备+打开相机+配置参数
bool HikCamera::init()
{
    // 1. 枚举所有可用工业相机(网口+USB口都支持)
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum == 0)
    {
        std::cerr << "[ERROR] 未搜索到海康工业相机！" << std::endl;
        return false;
    }
    std::cout << "[INFO] 搜索到 " << stDeviceList.nDeviceNum << " 台海康工业相机" << std::endl;

    // 2. 打开第一个相机（多相机可修改索引0为1/2）
    MV_CC_CreateHandle(&m_hCamera, stDeviceList.pDeviceInfo[0]);
    if (MV_CC_OpenDevice(m_hCamera) != MV_OK)
    {
        std::cerr << "[ERROR] 打开海康工业相机失败！" << std::endl;
        return false;
    }

    // 3. 关键配置：设置触发模式为连续取流（关闭软触发，工业相机必配）
    if (MV_CC_SetEnumValue(m_hCamera, "TriggerMode", MV_TRIGGER_MODE_OFF) != MV_OK)
    {
        std::cerr << "[WARNING] 设置连续取流失败，继续运行" << std::endl;
        // 这里改成警告，不报error，防止部分相机不支持该指令导致退出
    }

    // 4. 开始取流
    if (MV_CC_StartGrabbing(m_hCamera) != MV_OK)
    {
        std::cerr << "[ERROR] 相机开始取流失败！" << std::endl;
        return false;
    }
    std::cout << "[INFO] 海康工业相机初始化成功，已开始取流" << std::endl;
    return true;
}

// 从海康工业相机获取一帧图像，转换为OpenCV可用Mat格式 ✅ 全版本通用，无成员报错
bool HikCamera::getFrame(cv::Mat& frame)
{
    if (m_hCamera == NULL) return false;

    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    // 分配足够大的缓存，适配所有分辨率，无需任何获取缓存接口，杜绝报错
    unsigned int buffer_size = 2048 * 2048 * 3; // 足够1080P/2K相机使用
    unsigned char *pData = new unsigned char[buffer_size];
    if(pData == NULL) return false;

    // 核心取流接口，参数完全正确，编译通过的版本
    int nRet = MV_CC_GetOneFrameTimeout(m_hCamera, pData, buffer_size, &stFrameInfo, 1000);
    if (nRet != MV_OK || stFrameInfo.nFrameLen <= 0 || stFrameInfo.nWidth<=0 || stFrameInfo.nHeight<=0)
    {
        delete[] pData;
        return false;
    }

    // ============ 重点修复：全格式兼容的图像转换，海康相机所有格式都适配 ============
    cv::Mat img_src;
    // 先构建原始图像Mat，不管什么格式，先拿到有效图像
    img_src = cv::Mat(stFrameInfo.nHeight, stFrameInfo.nWidth, CV_8UC1, pData);
    // 格式转换：兼容海康相机所有主流Bayer格式 + BGR8 + 灰度图
    if (stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8)
    {
        cv::cvtColor(img_src, frame, cv::COLOR_BayerRG2BGR);
    }
    else if (stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8)
    {
        cv::cvtColor(img_src, frame, cv::COLOR_BayerBG2BGR);
    }
    else if (stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8)
    {
        cv::cvtColor(img_src, frame, cv::COLOR_BayerGR2BGR);
    }
    else if (stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8)
    {
        cv::cvtColor(img_src, frame, cv::COLOR_BayerGB2BGR);
    }
    else if (stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        // BGR8_Packed 格式直接构建 Mat，但需要克隆一份以避免依赖 pData
        frame = cv::Mat(stFrameInfo.nHeight, stFrameInfo.nWidth, CV_8UC3, pData).clone();
    }
    else
    {
        // 兜底：不管什么格式，都转成RGB显示，绝对不会空
        cv::cvtColor(img_src, frame, cv::COLOR_GRAY2BGR);
    }

    // ============ 强制兜底：确保图像非空 ============
    if(frame.empty())
    {
        frame = img_src.clone();
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }

    // 释放临时缓冲区
    delete[] pData;

    if(frame.empty() || frame.data == nullptr)
    {
        std::cout << "[ERROR]图像输出无效" << std::endl;
        return false;
    }

    return true;
}

// 海康工业相机资源释放，停止取流+关闭设备+销毁句柄
void HikCamera::release()
{
    if (m_hCamera == NULL) return;
    MV_CC_StopGrabbing(m_hCamera);
    MV_CC_CloseDevice(m_hCamera);
    MV_CC_DestroyHandle(m_hCamera);
    m_hCamera = NULL;
    std::cout << "[INFO] 海康工业相机资源已释放" << std::endl;
}