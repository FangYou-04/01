#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "ExtendedKalman.hpp"
#include "Serial.hpp"
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
    
    // 帧计数器（每隔5帧发送一次）
    int frameCounter = 0;
    const int SEND_INTERVAL = 5;

    // 定义相机内参和畸变系数
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

    // 卡尔曼追踪
    ExtendedKalman tracker;
    bool inited = false;

    cv::Point3f last_valid_position; 
    double last_valid_yaw;
    double last_valid_pitch;           
    cv::Mat last_valid_rvec;     
    
    // 打开串口（改为 ttyS0）
    Serial serial;
    if (!serial.open("/dev/ttyS0", B115200))
    {
        std::cerr << "无法打开串口" << std::endl;
        return -1;
    }

    // 视频源
    cv::VideoCapture cap("src/red1.mp4");
    if (!cap.isOpened())
    { 
        std::cerr << "视频打开失败" << std::endl;
        return -1;
    }
    
    // // 海康工业相机（备选）
    // HikCamera cam;
    // if (!cam.init())
    // {
    //     return -1;
    // }

    cv::namedWindow("Armor Tracker", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Armor Tracker", 1280, 720);
    
    cv::Mat frame;

    double timeStamp = 0.0;
    const double dt = 0.033;

    while (true)
    {
        // 获取一帧图像
        if (!cap.read(frame))
        {
            break;
        }
        
        // 若使用海康相机，取消注释以下代码
        // if (!cam.getFrame(frame))
        // {
        //     continue;
        // }
        
        if (frame.empty())
        {
            continue;
        }
        
        frameCounter++;  // 每帧递增计数器

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
            double pitch = best.pitch;

            // 保存上一次有效值（用于绘制及 pitch 预测占位）
            last_valid_position = pos;
            last_valid_yaw = yaw;
            last_valid_pitch = pitch;
            last_valid_rvec = best.rvec.clone();

            // ------------------ 卡尔曼更新 ------------------
            if (!inited)
            {
                tracker.init(pos, yaw, timeStamp);
                inited = true;
            }
            else
            {
                tracker.update(pos, yaw, timeStamp);
            }

            // 获取卡尔曼预测值（估计位置和预测偏航角）
            cv::Point3f est_pos = tracker.getEstimatedPosition();
            double pred_yaw_deg = tracker.getPredictedYawDeg();
            double pred_dist = sqrt(est_pos.x*est_pos.x + est_pos.y*est_pos.y + est_pos.z*est_pos.z);

            // ------------------ 串口发送（每5帧，仅检测到装甲板时）------------------
            if (frameCounter % SEND_INTERVAL == 0 && serial.is_open())
            {
                char buffer[256];
                snprintf(buffer, sizeof(buffer),
                         "(yaw: %.3f, pitch: %.3f, dist: %.3f, x: %.3f, y: %.3f, z: %.3f)\n",
                         pred_yaw_deg, last_valid_pitch, pred_dist,
                         est_pos.x, est_pos.y, est_pos.z);
                
                if (!serial.writeString(buffer, false))  // false: 不自动添加换行（已包含\n）
                {
                    std::cout << "[ERROR] 串口发送失败" << std::endl;
                }
            }

            // 绘制跟踪结果
            drawTrack(frame, best, est_pos, pred_yaw_deg, 
                      last_valid_rvec, camera_matrix, dist_coeffs);
        } 
        else
        {
            // 未检测到装甲板：不发送任何串口数据，只进行卡尔曼预测和绘制
            if (inited)
            {
                tracker.predict(timeStamp);
                cv::Point3f est_pos = tracker.getEstimatedPosition();
                double pred_yaw_deg = tracker.getPredictedYawDeg();

                drawTrack(frame, Armors(), est_pos, pred_yaw_deg, 
                          last_valid_rvec, camera_matrix, dist_coeffs);
            }
        }
        
        cv::imshow("Armor Tracker", frame);
        if (cv::waitKey(30) == 27)
        {
            break;
        }

        timeStamp += dt;
    }

    cap.release();
    // cam.release();
    cv::destroyAllWindows();
    serial.close();
    return 0;
}