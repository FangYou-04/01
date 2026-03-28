#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "KalmanTracker.hpp"
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
    
    // 读取相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
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

    // 卡尔曼追踪（使用 KalmanTracker）
    KalmanTracker tracker;          // 无参构造，参数从配置文件加载
    bool inited = false;

    cv::Point3f last_valid_position; 
    double last_valid_yaw;           
    cv::Mat last_valid_rvec;         

    // 视频读取
    cv::VideoCapture cap("src/red1.mp4");
    if (!cap.isOpened())
    {
        std::cerr << "视频打开失败" << std::endl;
        return -1;
    }
    
    // // 海康工业相机
    // HikCamera cam;
    // if (!cam.init())
    // {
    //     return -1;
    // }

    cv::namedWindow("Armor Tracker", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Armor Tracker", 1280, 720);
    
    cv::Mat frame;
    double timestamp = 0.0;      // 模拟时间戳，可根据实际帧率或系统时间设置
    const double dt = 0.033;     // 假设30fps

    while (true)
    {
        // 视频取流
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
        
        timestamp += dt;   // 模拟时间戳递增
        
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
                tracker.init(pos, timestamp);   // 使用时间戳初始化
                inited = true;
            }
            else
            {
                tracker.predicted(timestamp);   // 预测
                tracker.update(pos, timestamp); // 更新
            }

            cv::Point3f est_pos = tracker.getEstimatedPosition(); // 获取滤波后位置
            double pred_yaw = last_valid_yaw;   // 注意：KalmanTracker 未处理 yaw，此处沿用检测值

            drawTrack(frame, best, est_pos, pred_yaw, 
                      last_valid_rvec, camera_matrix, dist_coeffs);
        } 
        else
        {
            if (inited)
            {
                cv::Point3f est_pos = tracker.predicted(timestamp); // 仅预测

                drawTrack(frame, Armors(), est_pos, last_valid_yaw, 
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