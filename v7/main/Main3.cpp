#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "Kalman.hpp"
#include "DrawTrack.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "HikCamera.hpp"

int main()
{
    cv::setNumThreads(0);
    cv::setUseOptimized(false);
    
    //读取文件

    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        // 修复：cout << 正确写法
        std::cout << "[ERROR] 无法打开calib_result.yml" << std::endl;
        return -1;
    }
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cout << "PnPSolver::solve: 无效的相机内参，跳过位姿求解" << std::endl;
        return -1;
    }
    
    // 检测器
    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);

    // 卡尔曼追踪
    Kalman tracker(0.033, 7.33);
    bool inited = false;

    cv::Point3f last_valid_position; 
    double last_valid_yaw;           
    cv::Mat last_valid_rvec;         

    // 视频.ver
    cv::VideoCapture cap("src/red1.mp4");
    if (!cap.isOpened())
    { 
        std::cerr << "视频打开失败" << std::endl;
        return -1;
    }
    
    // // 海康工业相机.ver
    // HikCamera cam;
    // if (!cam.init())
    // {
    //     return -1;
    // }

    cv::namedWindow("Armor Tracker", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Armor Tracker", 1280, 720); // 固定窗口大小
    
    cv::Mat frame;
    while (true)
    {
        // 【视频取流】
        if (!cap.read(frame))
        {
            break;
        }
        
        // // 海康工业取流
        // if (!cam.getFrame(frame))
        // {
        //     continue;
        // }
        
        if (frame.empty())
        {
            continue;
        }
        
        std::vector<Armors> armors = armorsdetector.detect(frame);

        if (!armors.empty())
        {
            const Armors& best = armors[0];

            cv::Point3f pos(
                best.tvec.at<double>(0),
                best.tvec.at<double>(1),
                best.tvec.at<double>(2)
            );

            double yaw = best.yaw;

            last_valid_position = pos;
            last_valid_yaw = yaw;
            last_valid_rvec = best.rvec.clone(); 

            if (!inited)
            {
                tracker.init(pos, yaw);
                inited = true;
            }
            else
            {
                tracker.predict();
                tracker.update(pos, yaw);
            }

            cv::Point3f pred_pos = tracker.getPosition();
            double pred_yaw = tracker.getYaw();

            // 修正：传入检测到的装甲板，而不是空对象
            drawTrack(frame, best, pred_pos, pred_yaw, 
                      last_valid_rvec, camera_matrix, dist_coeffs);
        } 
        else
        {
            if (inited)
            {
                tracker.predict();
                cv::Point3f pred_pos = tracker.getPosition();
                double pred_yaw = tracker.getYaw();

                drawTrack(frame, Armors(), pred_pos, pred_yaw, 
                          last_valid_rvec, camera_matrix, dist_coeffs);
            }
        }
        
        cv::imshow("Armor Tracker", frame);
        if (cv::waitKey(30) == 27)
        {
            break;
        }
    }

    cap.release();
    // cam.release();
    cv::destroyAllWindows();
    return 0;
}