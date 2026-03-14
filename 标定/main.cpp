#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <vector>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;

// 全局相机句柄
MV_CC_DEVICE_INFO_LIST stDeviceList;
void* g_hCamera = NULL;
unsigned int g_nPixelFormat = PixelType_Gvsp_Mono8;

// 读取标定参数（适配上级路径）
bool loadCalibParams(const string& calibPath, Mat& cameraMatrix, Mat& distCoeffs) {
    // 优先查找上级路径的标定文件
    string absPath = "../" + calibPath;
    FileStorage fs(absPath, FileStorage::READ);
    if (!fs.isOpened()) {
        // 若上级路径无文件，再查找当前路径
        fs.open(calibPath, FileStorage::READ);
        if (!fs.isOpened()) {
            cerr << "[ERROR] 标定文件打开失败：" << calibPath << " 和 " << absPath << endl;
            return false;
        }
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    return true;
}

// 海康相机初始化
bool HikCameraInit() {
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum <= 0) {
        cerr << "[ERROR] 枚举相机失败，错误码：" << nRet << endl;
        return false;
    }
    cout << "[INFO] 找到" << stDeviceList.nDeviceNum << "台海康相机" << endl;

    MV_CC_DEVICE_INFO* pstDevInfo = stDeviceList.pDeviceInfo[0];
    nRet = MV_CC_CreateHandle(&g_hCamera, pstDevInfo);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 创建句柄失败，错误码：" << nRet << endl;
        return false;
    }

    nRet = MV_CC_OpenDevice(g_hCamera);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 打开相机失败，错误码：" << nRet << endl;
        MV_CC_DestroyHandle(g_hCamera);
        g_hCamera = NULL;
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_hCamera, "TriggerMode", 0);
    if (MV_OK != nRet) {
        cerr << "[WARNING] 设置触发模式失败，错误码：" << nRet << endl;
    }

    MVCC_ENUMVALUE_EX stPixelInfo;
    nRet = MV_CC_GetEnumValueEx(g_hCamera, "PixelFormat", &stPixelInfo);
    if (MV_OK == nRet) {
        g_nPixelFormat = stPixelInfo.nCurValue;
        cout << "[INFO] 相机像素格式数值：" << g_nPixelFormat << endl;
    } else {
        cerr << "[WARNING] 获取像素格式失败,默认使用Mono8" << endl;
        g_nPixelFormat = PixelType_Gvsp_Mono8;
    }

    return true;
}

// 曝光/增益调节
bool HikCameraSetParam(float exposureUs, float gainDb) {
    if (g_hCamera == NULL) {
        cerr << "[ERROR] 相机未初始化" << endl;
        return false;
    }
    int nRet = 0;

    nRet = MV_CC_SetEnumValue(g_hCamera, "ExposureAuto", 0);
    if (MV_OK != nRet) cerr << "[WARNING] 关闭自动曝光失败：" << nRet << endl;
    nRet = MV_CC_SetFloatValue(g_hCamera, "ExposureTime", exposureUs);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 设置曝光失败：" << nRet << endl;
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_hCamera, "GainAuto", 0);
    if (MV_OK != nRet) cerr << "[WARNING] 关闭自动增益失败：" << nRet << endl;
    nRet = MV_CC_SetFloatValue(g_hCamera, "Gain", gainDb);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 设置增益失败：" << nRet << endl;
        return false;
    }

    cout << "[INFO] 曝光：" << exposureUs << "μs,增益：" << gainDb << "dB" << endl;
    return true;
}

bool HikCameraGetFrame(Mat& frame) {
    if (g_hCamera == NULL) {
        cerr << "[ERROR] 相机未初始化" << endl;
        return false;
    }

    // 让SDK自动分配缓冲区，初始化为NULL
    unsigned char* pData = NULL;
    unsigned int nBufSize = 0;
    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    // 读取相机实际分辨率
    MVCC_INTVALUE stWidth, stHeight;
    MV_CC_GetIntValue(g_hCamera, "Width", &stWidth);
    MV_CC_GetIntValue(g_hCamera, "Height", &stHeight);
    int nWidth = stWidth.nCurValue;
    int nHeight = stHeight.nCurValue;

    // 强制设置为Mono8格式，避免格式不兼容
    MV_CC_SetEnumValue(g_hCamera, "PixelFormat", PixelType_Gvsp_Mono8);
    g_nPixelFormat = PixelType_Gvsp_Mono8;

    // 启动采集
    int nRectStart = MV_CC_StartGrabbing(g_hCamera);
    if (MV_OK != nRectStart)
    {
        cerr << "[ERROR] 启动采集失败，错误码:" << nRectStart << endl;
        return false;
    }

    // 获取一帧图像（SDK会自动填充pData和nBufSize）
    int nRet = MV_CC_GetOneFrameTimeout(g_hCamera, pData, nBufSize, &stFrameInfo, 1000);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 获取帧失败，错误码：" << nRet << endl;
        return false;
    }

    // ********** 关键修改：使用pData作为缓冲区地址 **********
    if (pData == NULL) {
        cerr << "[ERROR] 图像缓冲区为空" << endl;
        return false;
    }

    //停止采集
    MV_CC_StopGrabbing(g_hCamera);

    // 转换为OpenCV Mat
    frame = Mat(nHeight, nWidth, CV_8UC1, pData).clone();
    return true;
}

// 关闭相机资源
void HikCameraClose() {
    if (g_hCamera) {
        MV_CC_StopGrabbing(g_hCamera);
        MV_CC_CloseDevice(g_hCamera);
        MV_CC_DestroyHandle(g_hCamera);
        g_hCamera = NULL;
        cout << "[INFO] 相机资源已释放" << endl;
    }
}

// BMP标定图处理（修改为上级路径）
bool calibrateWithTIF(const string& calibDir, Size boardSize, Mat& cameraMatrix, Mat& distCoeffs) {
    vector<String> imgPaths;
    // 关键修改：添加../指向项目根目录的calib_imgs
    string absCalibDir = "../" + calibDir;
    glob(absCalibDir + "/*.bmp", imgPaths);
    
    if (imgPaths.empty()) {
        cerr << "[ERROR] 无标定图：" << absCalibDir << endl;
        return false;
    }
    cout << "[INFO] 找到" << imgPaths.size() << "张标定图" << endl;

    vector<vector<Point2f>> imgPoints;
    vector<vector<Point3f>> objPoints;
    vector<Point3f> objP;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objP.push_back(Point3f(j * 10.0f, i * 10.0f, 0.0f));
        }
    }

    Size imgSize;
    for (auto& path : imgPaths) {
        Mat img = imread(path, IMREAD_UNCHANGED);
        if (img.empty()) {
            cerr << "[WARNING] 跳过无效图：" << path << endl;
            continue;
        }
        if (img.depth() == CV_16U) {
            normalize(img, img, 0, 255, NORM_MINMAX);
            img.convertTo(img, CV_8U);
        }
        imgSize = img.size();

        Mat gray;
        if (img.channels() == 3) cvtColor(img, gray, COLOR_BGR2GRAY);
        else gray = img.clone();

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners, 
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), 
                TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
            imgPoints.push_back(corners);
            objPoints.push_back(objP);
            drawChessboardCorners(img, boardSize, corners, found);
            imshow("标定角点", img);
            waitKey(200);
        }
    }
    destroyWindow("标定角点");

    if (imgPoints.empty()) {
        cerr << "[ERROR] 无有效角点，标定失败" << endl;
        return false;
    }

    vector<Mat> rvecs, tvecs;
    calibrateCamera(objPoints, imgPoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    // 标定参数保存到上级路径（项目根目录）
    FileStorage fs("../calib_result.yml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs.release();
    cout << "[INFO] 标定完成，参数保存至../calib_result.yml" << endl;

    return true;
}

int main() {
    Size boardSize(11, 8);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    // 传入相对目录名，函数内部自动拼接上级路径
    if (!calibrateWithTIF("calib_imgs", boardSize, cameraMatrix, distCoeffs)) {
        cerr << "[WARNING] 标定失败，使用默认内参继续运行" << endl;
        // 标定失败时使用默认内参
        cameraMatrix = (Mat_<double>(3,3) << 1200, 0, 960, 0, 1200, 540, 0, 0, 1);
        distCoeffs = Mat::zeros(5, 1, CV_64F);
    }

    if (!HikCameraInit()) {
        return -1;
    }
    HikCameraSetParam(500.0f, 1.0f);

    Mat frame, undistFrame, map1, map2;
    if (!HikCameraGetFrame(frame)) {
        HikCameraClose();
        return -1;
    }
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), 
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frame.size(), 1, frame.size()), 
        frame.size(), CV_16SC2, map1, map2);

    int nRet = MV_CC_StartGrabbing(g_hCamera);
    if (MV_OK != nRet) {
        cerr << "[ERROR] 开始采集失败，错误码：" << nRet << endl;
        HikCameraClose();
        return -1;
    }
    cout << "[INFO] 实时采集中,按ESC退出..." << endl;

    while (true) {
        if (HikCameraGetFrame(frame)) {
            remap(frame, undistFrame, map1, map2, INTER_LINEAR);
            resize(frame, frame, Size(frame.cols/2, frame.rows/2));
            resize(undistFrame, undistFrame, Size(undistFrame.cols/2, undistFrame.rows/2));
            imshow("原始帧", frame);
            imshow("校正后帧", undistFrame);
        }
        if (waitKey(1) == 27) break;
    }

    HikCameraClose();
    destroyAllWindows();
    return 0;
}