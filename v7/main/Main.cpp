#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include <cstdint>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

void drawFriend(ArmorsDetector& detector, cv::Mat& mask, const std::vector<Light> &lights, const std::vector<Armors> &armors)
{
    detector.draw(mask, lights, armors);
}

// 程序入口函数（必须有）
int main() 
{
    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    //读取文件
    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "[ERROR] 无法打开calib_result.yml" << std::endl;
        return -1;
    }

    // 读取内参与畸变系数
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;
    fs.release();

    // 验证相机内参有效性，避免在空或非3x3矩阵上调用 OpenCV 的函数导致异常终止
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cout << "PnPSolver::solve: 无效的相机内参，跳过位姿求解" << std::endl;
        return -1;
    }

    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);
 
    cv::VideoCapture cap("src/red1.mp4");
    // cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "视频打开失败！"; 
        return -1;
    }
    
    cv::Mat frame;

    while (cap.read(frame))
    {
        // cap >> frame;

        if (frame.empty())
        {
            std::cout << "视频播放完毕！";
            break;
        }
        
        if (frame.rows == 0 || frame.cols == 0)
        {
            std::cout << "无效的视频帧" << std::endl;
            continue;
        }


        std::vector<Armors> armors;
        std::vector<Light> lights;
        try 
        {
            armors = armorsdetector.detect(frame); 
        } 

        catch (const std::exception& e) 
        {
            std::cerr << "检测函数异常：" << e.what() << std::endl;
            continue;
        }

        // 为了确保 imshow 能正确可视化，统一转换为 CV_8U 单通道图像
        // cv::Mat mask_show;
        // if (mask.empty()) {
        //     mask_show = cv::Mat::zeros(frame.size(), CV_8U);
        // } else if (mask.type() == CV_32F || mask.type() == CV_64F) {
        //     mask.convertTo(mask_show, CV_8U, 255.0);
        // } else if (mask.channels() == 3) {
        //     cv::cvtColor(mask, mask_show, cv::COLOR_BGR2GRAY);
        // } else {
        //     mask_show = mask;
        // }

        cv::Mat mask = frame.clone();

        drawFriend(armorsdetector, mask, lights, armors);

        cv::imshow("结果", mask);
        if (cv::waitKey(50)==27)
            {
                std::cout << "退出！";
                break;
            }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}