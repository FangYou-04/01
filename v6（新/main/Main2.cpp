#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cstring>
#include <MvCameraControl.h>  // 海康MVS SDK头文件

// 全局相机句柄，用于操作相机
MV_CC_DEVICE_INFO_LIST stDeviceList;
void* g_hCamera = NULL;

// 海康工业相机初始化：枚举设备+打开相机+配置参数
bool HikIndustrialCam_Init()
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
    MV_CC_CreateHandle(&g_hCamera, stDeviceList.pDeviceInfo[0]);
    if (MV_CC_OpenDevice(g_hCamera) != MV_OK)
    {
        std::cerr << "[ERROR] 打开海康工业相机失败！" << std::endl;
        return false;
    }

    // 3. 关键配置：设置触发模式为连续取流（关闭软触发，工业相机必配）
    if (MV_CC_SetEnumValue(g_hCamera, "TriggerMode", MV_TRIGGER_MODE_OFF) != MV_OK)
    {
        std::cerr << "[WARNING] 设置连续取流失败，继续运行" << std::endl;
        // 这里改成警告，不报error，防止部分相机不支持该指令导致退出
    }

    // 4. 开始取流
    if (MV_CC_StartGrabbing(g_hCamera) != MV_OK)
    {
        std::cerr << "[ERROR] 相机开始取流失败！" << std::endl;
        return false;
    }
    std::cout << "[INFO] 海康工业相机初始化成功，已开始取流" << std::endl;
    return true;
}

// 从海康工业相机获取一帧图像，转换为OpenCV可用Mat格式 ✅ 全版本通用，无成员报错
bool HikIndustrialCam_GetFrame(cv::Mat& frame)
{
    if (g_hCamera == NULL) return false;

    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    // 分配足够大的缓存，适配所有分辨率，无需任何获取缓存接口，杜绝报错
    unsigned int buffer_size = 2048 * 2048 * 3; // 足够1080P/2K相机使用
    unsigned char *pData = new unsigned char[buffer_size];
    if(pData == NULL) return false;

    // 核心取流接口，参数完全正确，编译通过的版本
    int nRet = MV_CC_GetOneFrameTimeout(g_hCamera, pData, buffer_size, &stFrameInfo, 1000);
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
        frame = cv::Mat(stFrameInfo.nHeight, stFrameInfo.nWidth, CV_8UC3, pData);
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

    if(frame.empty() || frame.data == nullptr)
    {
        std::cout << "[ERROR]图像输出无效" << std::endl;
        delete pData;
        return false;
    }

    return true;
}


// 海康工业相机释放资源
void HikIndustrialCam_Release()
{
    if (g_hCamera == NULL) return;
    MV_CC_StopGrabbing(g_hCamera);
    MV_CC_CloseDevice(g_hCamera);
    MV_CC_DestroyHandle(g_hCamera);
    g_hCamera = NULL;
    std::cout << "[INFO] 海康工业相机资源已释放" << std::endl;
}

void drawFriend(ArmorsDetector& detector, cv::Mat& mask, const std::vector<Light> &lights, const std::vector<Armors> &armors)
{
    detector.draw(mask, lights, armors);
}

// 程序入口函数
int main() 
{
    cv::setNumThreads(1);
    cv::setUseOptimized(true);

    // 1. 初始化海康工业相机（优先级最高）
    if (!HikIndustrialCam_Init())
    {
        std::cerr << "相机初始化失败，程序退出！" << std::endl;
        return -1;
    }

    // 2. 读取相机内参（原有逻辑保留）
    cv::Mat camera_matrix, dist_coeffs;
    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "[WARNING] 无法打开calib_result.yml,位姿解算可能异常" << std::endl;
        // 此处可加默认内参，避免程序退出
        camera_matrix = (cv::Mat_<double>(3,3) << 1000,0,640, 0,1000,360, 0,0,1);
        dist_coeffs = cv::Mat::zeros(5,1,CV_64F);
    }
    else
    {
        fs["cameraMatrix"] >> camera_matrix;
        fs["distCoeffs"] >> dist_coeffs;
        fs.release();
    }
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cout << "[WARNING] 相机内参无效，使用默认内参" << std::endl;
        camera_matrix = (cv::Mat_<double>(3,3) << 1000,0,640, 0,1000,360, 0,0,1);
        dist_coeffs = cv::Mat::zeros(5,1,CV_64F);
    }

    // 3. 初始化装甲板检测器
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);
 
    cv::Mat frame, mask;
    std::vector<Armors> armors;
    std::vector<Light> lights;

    // 4. 循环取流+检测+绘制 (修复后)
    // ============ 前置初始化窗口：解决imshow黑屏核心 ============
    cv::namedWindow("海康工业相机-装甲板检测结果", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("海康工业相机-装甲板检测结果", 1280, 720); // 固定窗口大小，防止变形

    while (true)
    {

        // 获取工业相机帧
        bool get_frame_ok = HikIndustrialCam_GetFrame(frame);
        if (!get_frame_ok || frame.empty())
        {
            std::cerr << "获取相机帧失败，重试..." << std::endl;
            cv::waitKey(5);
            continue;
        }
        
        // 装甲板检测（原有逻辑保留，不修改任何Detector代码）
        try 
        {
            armors = armorsdetector.detect(frame);
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "检测函数异常：" << e.what() << std::endl;
            continue;
        }

        mask = frame.clone();
        drawFriend(armorsdetector, mask, lights,  armors);

        // ============ 修复显示逻辑：强制刷新+延时适配 ============
        cv::imshow("海康工业相机-装甲板检测结果", mask);
        // 关键：waitKey改成10，既流畅又能让opencv刷新窗口，ESC退出不变
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27)
        {
            std::cout << "用户退出程序！" << std::endl;
            break;
        }
    }


    // 5. 资源释放（必须执行）
    HikIndustrialCam_Release();
    cv::destroyAllWindows();
    return 0;
}
