#include "Config.hpp"  
#include "Struct.hpp"
#include "PoseSolve.hpp"  
#include "ExtendedKalman.hpp"   // 替换原来的 Kalman.hpp
#include "Yolo.hpp"              // 新增 YOLO 类
#include <cstdint>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <chrono>                // 用于时间戳

// 获取当前时间戳（秒）
double getTimestamp() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ========== 以下是您原有的 drawTrack 函数（完全保留，一字不改） ==========
// 可视化跟踪结果（检测框+预测框+坐标信息）
void drawTrack(cv::Mat& img, const Armors& detected_armor, const cv::Point3f& predict_position, double predict_yaw)
{
    if (img.empty()) {
        std::cerr << "[drawTrack] 输入图像为空，跳过绘制" << std::endl;
        return;
    }

    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    // 读取标定文件
    std::vector<std::string> calib_paths = {
    // 1. ROS2包安装后的配置路径（推荐）
    //ament_index_cpp::get_package_share_directory("armor_pose_solver") + "/config/calib_result.yml",
    // 2. 源码目录路径（开发阶段）
    "/home/hh/ros2_ws/src/armor_pose_solver/config/armor_params.yaml",
    // 3. 当前工作目录（兜底）
    "config/armor_params.yaml",
    "armor_params.yaml"
};
    bool calib_loaded = false;
    for (const auto& path : calib_paths) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> camera_matrix;
            fs["distCoeffs"] >> dist_coeffs;
            fs.release();
            calib_loaded = true;
            break;
        }
    }
    if (!calib_loaded) {
        std::cerr << "[ERROR] 无法打开相机标定文件" << std::endl;
        return;
    }

    // 校验相机内参有效性
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参无效（非3x3矩阵）" << std::endl;
        return;
    }

    // 1. 绘制检测到的装甲板轮廓 (红色)
    if (!detected_armor.corners.empty()) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : detected_armor.corners) {
            pts.emplace_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 2); // 红色轮廓
    }

    // 2. 校验预测参数有效性
    if (std::isnan(predict_position.x) || std::isnan(predict_position.y) || std::isnan(predict_position.z) ||
        std::isnan(predict_yaw) || predict_position.z <= 0) {
        std::cerr << "[drawTrack] 预测参数无效（NaN/深度≤0），跳过预测框绘制" << std::endl;
        
        // 仅显示错误提示
        cv::putText(img, "Predict: Invalid Data", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 3. 定义装甲板三维参考点 (固定尺寸)
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        {-armorWidth/2, -armorHeight/2, 0.0f}
    };

    // 4. 构造预测的旋转向量（yaw绕Z轴旋转，弧度）
    double yaw_rad = predict_yaw * CV_PI / 180.0;
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad),  cos(yaw_rad), 0,
        0,             0,            1);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << predict_position.x, predict_position.y, predict_position.z);

    // 5. 投影预测的3D点到2D图像平面
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);

    // 6. 绘制预测的装甲板轮廓 (绿色)
    if (predict_image_points.size() == 4) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> pts;
        for (const auto& p : predict_image_points) {
            // 限制点在图像范围内
            int x = std::clamp(static_cast<int>(p.x), 0, img.cols - 1);
            int y = std::clamp(static_cast<int>(p.y), 0, img.rows - 1);
            pts.emplace_back(x, y);
        }
        contours.push_back(pts);
        cv::drawContours(img, contours, 0, cv::Scalar(0, 255, 0), 2); // 绿色轮廓
    } else {
        std::cerr << "[drawTrack] 投影点数量异常：" << predict_image_points.size() << std::endl;
    }
    
    // 7. 显示预测坐标和角度
    char text[100];
    snprintf(text, sizeof(text), "Predict: (%.2f, %.2f, %.2f) Yaw: %.1f°", 
             predict_position.x, predict_position.y, predict_position.z, predict_yaw);
    cv::putText(img, text, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}
// ========== drawTrack 函数结束 ==========

int main()
{
    try {
        // ========== 新增：加载全局配置（替代原有的硬编码路径） ==========
        Config* cfg = Config::getInstance();
        const AppConfig& app_cfg = cfg->getConfig();

        // 1. 初始化 YOLO 检测器（使用配置中的模型路径和类别文件）
        std::string labels_path = "/home/hh/ros2_ws/src/armor_pose_solver/config/armor.names"; // 可改为配置项
        YOLOSolver yolo(app_cfg.yolo_config.model_path, labels_path);

        // 2. 加载相机标定参数（优先使用配置文件中的内参，若没有则回退到文件读取）
        cv::Mat camera_matrix, dist_coeffs;
        bool calib_from_config = false;
        // 检查配置中是否有有效的相机内参
        if (app_cfg.armor_params.fx > 0 && app_cfg.armor_params.fy > 0) {
            camera_matrix = (cv::Mat_<double>(3,3) <<
                app_cfg.armor_params.fx, 0, app_cfg.armor_params.cx,
                0, app_cfg.armor_params.fy, app_cfg.armor_params.cy,
                0, 0, 1);
            dist_coeffs = (cv::Mat_<double>(1,5) <<
                app_cfg.armor_params.k1, app_cfg.armor_params.k2,
                app_cfg.armor_params.p1, app_cfg.armor_params.p2,
                app_cfg.armor_params.k3);
            calib_from_config = true;
            std::cout << "[INFO] 使用配置文件中的相机内参" << std::endl;
        } else {
            // 原有的标定文件读取逻辑（保留）
            std::vector<std::string> calib_paths = {
                "src/calib_result.yml",
                "../src/calib_result.yml",
                "../../src/calib_result.yml",
                "calib_result.yml"
            };
            bool calib_loaded = false;
            for (const auto& path : calib_paths) {
                cv::FileStorage fs(path, cv::FileStorage::READ);
                if (fs.isOpened()) {
                    fs["cameraMatrix"] >> camera_matrix;
                    fs["distCoeffs"] >> dist_coeffs;
                    fs.release();
                    calib_loaded = true;
                    break;
                }
            }
            if (!calib_loaded) {
                std::cerr << "[ERROR] 无法加载相机标定文件！" << std::endl;
                return -1;
            }
        }
        if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
            std::cerr << "[ERROR] 相机内参无效！" << std::endl;
            return -1;
        }

        // 3. 初始化位姿解算器（从 YOLO 框解算装甲板位姿）
        ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);

        // 4. 初始化扩展卡尔曼滤波器（替换原来的 Kalman）
        ExtendedKalman tracker;   // 注意：ExtendedKalman 不需要传入 dt 和角速度
        bool tracker_inited = false;
        cv::Point3f last_valid_position;
        double last_valid_yaw = 0.0;

        // 5. 打开视频/摄像头（保留原有逻辑）
        cv::VideoCapture cap;
        // 可选1：打开视频文件
        if (!cap.open("/home/hh/ros2_ws/src/armor_pose_solver/config/armor5blue.mp4")) {
            std::cerr << "[WARNING] 视频文件打开失败，尝试打开摄像头..." << std::endl;
            // 可选2：打开摄像头
            if (!cap.open(0)) {
                std::cerr << "[ERROR] 摄像头打开失败！" << std::endl;
                return -1;
            }
        }

        // 6. 主循环
        cv::Mat frame;
        while (cap.read(frame)) {
            if (frame.empty()) {
                std::cout << "视频播放完毕/无有效帧！" << std::endl;
                break;
            }

            // ===================== 核心流程 =====================
            // Step 1: YOLO检测（使用 YOLOSolver 类）
            std::vector<DetectionResult> detections = yolo.detect(frame);
            
            // 转换为原有的格式（boxes, confs, cls_ids）
            std::vector<cv::Rect> yolo_boxes;
            std::vector<float> yolo_confs;
            std::vector<int> yolo_cls_ids;
            for (const auto& det : detections) {
                yolo_boxes.push_back(det.bbox);
                yolo_confs.push_back(det.confidence);
                yolo_cls_ids.push_back(det.class_id);
            }

            // Step 2: 位姿解算（从YOLO框解算装甲板位姿）
            std::vector<Armors> armors = armorsdetector.detect(frame, yolo_boxes, yolo_confs, yolo_cls_ids);

            // Step 3: 扩展卡尔曼滤波跟踪（替换原来的 Kalman）
            double timestamp = getTimestamp();  // 获取当前时间戳
            if (!armors.empty()) {
                // 取置信度最高的装甲板
                Armors best_armor = armors[0];

                // 校验位姿有效性
                bool pose_valid = !best_armor.tvec.empty() && cv::checkRange(best_armor.tvec) && 
                                  std::isfinite(best_armor.yaw) && best_armor.distance > 0;

                if (pose_valid) {
                    // 位姿有效：构造3D位置
                    cv::Point3f detect_point(
                        static_cast<float>(best_armor.tvec.at<double>(0)),
                        static_cast<float>(best_armor.tvec.at<double>(1)),
                        static_cast<float>(best_armor.tvec.at<double>(2))
                    );
                    double detect_yaw_rad = best_armor.yaw * CV_PI / 180.0;  // 转弧度

                    // 初始化/更新扩展卡尔曼滤波器
                    if (!tracker_inited) {
                        tracker.init(detect_point, detect_yaw_rad, timestamp);
                        tracker_inited = true;
                        last_valid_position = detect_point;
                        last_valid_yaw = best_armor.yaw;
                    } else {
                        tracker.predict(timestamp);
                        tracker.update(detect_point, detect_yaw_rad, timestamp);
                        last_valid_position = detect_point;
                        last_valid_yaw = best_armor.yaw;
                    }

                    // 获取预测结果（位置和 yaw 角，注意 EKF 返回的是弧度）
                    cv::Point3f predict_point = tracker.getEstimatedPosition();
                    double predict_yaw_deg = tracker.getPredictedYawDeg();  // 转为度
                    drawTrack(frame, best_armor, predict_point, predict_yaw_deg);
                } else {
                    // 位姿无效：仅预测
                    std::cerr << "[WARNING] 装甲板位姿无效，仅执行预测" << std::endl;
                    if (tracker_inited) {
                        tracker.predict(timestamp);
                        cv::Point3f predict_point = tracker.getEstimatedPosition();
                        double predict_yaw_deg = tracker.getPredictedYawDeg();
                        drawTrack(frame, best_armor, predict_point, predict_yaw_deg);
                    }
                }
            } else {
                // 未检测到装甲板：仅预测（如果已初始化）
                if (tracker_inited) {
                    tracker.predict(timestamp);
                    cv::Point3f predict_point = tracker.getEstimatedPosition();
                    double predict_yaw_deg = tracker.getPredictedYawDeg();
                    drawTrack(frame, Armors(), predict_point, predict_yaw_deg);
                }
            }

            // ===================== 可视化 =====================
            // 可选：同时绘制 YOLO 原始检测框（调用 yolo.drawResults）
            yolo.drawResults(frame, detections);  // 保留原有 YOLO 框绘制，与 drawTrack 不冲突

            cv::imshow("Armor Tracker (YOLO+EKF)", frame);
            int key = cv::waitKey(30); // 30ms ≈ 30帧/秒
            if (key == 113) { // q键退出
                std::cout << "用户退出程序！" << std::endl;
                break;
            }
        }

        // 释放资源
        cap.release();
        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return -1;
    }
    return 0;
}