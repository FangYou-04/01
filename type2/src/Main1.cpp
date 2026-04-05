#include "Config.hpp"  
#include "Struct.hpp"
#include "PoseSolve.hpp"  
#include "ExtendedKalman.hpp"
#include "Yolo.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>

double getTimestamp() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ========== 修改后的 drawTrack（接收相机内参作为参数） ==========
void drawTrack(cv::Mat& img, const Armors& detected_armor,
               const cv::Point3f& predict_position, double predict_yaw,
               const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    if (img.empty()) {
        std::cerr << "[drawTrack] 输入图像为空，跳过绘制" << std::endl;
        return;
    }

    // 校验相机内参有效性
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[drawTrack] 相机内参无效" << std::endl;
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
        cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 2);
    }

    // 2. 校验预测参数有效性
    if (std::isnan(predict_position.x) || std::isnan(predict_position.y) || std::isnan(predict_position.z) ||
        std::isnan(predict_yaw) || predict_position.z <= 0) {
        cv::putText(img, "Predict: Invalid Data", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 3. 定义装甲板三维参考点
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points = {
        {-armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2,  armorHeight/2, 0.0f},
        { armorWidth/2, -armorHeight/2, 0.0f},
        {-armorWidth/2, -armorHeight/2, 0.0f}
    };

    // 4. 构造预测的旋转向量
    double yaw_rad = predict_yaw * CV_PI / 180.0;
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad),  cos(yaw_rad), 0,
        0,             0,            1);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << predict_position.x, predict_position.y, predict_position.z);

    // 5. 投影
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);

    // 6. 绘制绿色预测轮廓
    if (predict_image_points.size() == 4) {
        std::vector<cv::Point> pts;
        for (const auto& p : predict_image_points) {
            int x = std::clamp(static_cast<int>(p.x), 0, img.cols - 1);
            int y = std::clamp(static_cast<int>(p.y), 0, img.rows - 1);
            pts.emplace_back(x, y);
        }
        cv::polylines(img, pts, true, cv::Scalar(0, 255, 0), 2);
    } else {
        std::cerr << "[drawTrack] 投影点数量异常：" << predict_image_points.size() << std::endl;
    }

    // 7. 显示预测文本
    char text[100];
    snprintf(text, sizeof(text), "Predict: (%.2f, %.2f, %.2f) Yaw: %.1f°",
             predict_position.x, predict_position.y, predict_position.z, predict_yaw);
    cv::putText(img, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

int main() {
    try {
        Config* cfg = Config::getInstance();
        const AppConfig& app_cfg = cfg->getConfig();

        // 1. 初始化 YOLO（注意：类别文件应为 .names，而非 .yaml）
        std::string labels_path = "config/armor.names";
        YOLOSolver yolo(app_cfg.yolo_config.model_path, labels_path);

        // 2. 从配置中读取相机内参
        cv::Mat camera_matrix, dist_coeffs;
        if (app_cfg.armor_params.fx > 0 && app_cfg.armor_params.fy > 0) {
            camera_matrix = (cv::Mat_<double>(3,3) <<
                app_cfg.armor_params.fx, 0, app_cfg.armor_params.cx,
                0, app_cfg.armor_params.fy, app_cfg.armor_params.cy,
                0, 0, 1);
            dist_coeffs = (cv::Mat_<double>(1,5) <<
                app_cfg.armor_params.k1, app_cfg.armor_params.k2,
                app_cfg.armor_params.p1, app_cfg.armor_params.p2,
                app_cfg.armor_params.k3);
            std::cout << "[INFO] 使用配置文件中的相机内参" << std::endl;
        } else {
            std::cerr << "[ERROR] 配置文件中无有效相机内参" << std::endl;
            return -1;
        }
        if (camera_matrix.empty() || camera_matrix.rows != 3) {
            std::cerr << "[ERROR] 相机内参无效" << std::endl;
            return -1;
        }

        // 3. 初始化位姿解算器
        ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);

        // 4. 初始化 EKF
        ExtendedKalman tracker;
        bool tracker_inited = false;
        cv::Point3f last_valid_position;

        // 5. 打开视频源（优先视频文件，失败则摄像头）
        cv::VideoCapture cap;
        if (!cap.open("config/armor5blue.mp4")) {
            std::cerr << "[WARNING] 视频文件打开失败，尝试摄像头..." << std::endl;
            if (!cap.open(0)) {
                std::cerr << "[ERROR] 摄像头打开失败" << std::endl;
                return -1;
            }
        }

        cv::Mat frame;
        while (cap.read(frame)) {
            if (frame.empty()) break;

            // YOLO 检测
            auto detections = yolo.detect(frame);
            yolo.drawResults(frame, detections);

            // 转换格式
            std::vector<cv::Rect> boxes;
            std::vector<float> confs;
            std::vector<int> ids;
            for (auto& d : detections) {
                boxes.push_back(d.bbox);
                confs.push_back(d.confidence);
                ids.push_back(d.class_id);
            }

            // 位姿解算
            auto armors = armorsdetector.detect(frame, boxes, confs, ids);

            double timestamp = getTimestamp();
            if (!armors.empty()) {
                Armors& best = armors[0];
                bool pose_valid = !best.tvec.empty() && cv::checkRange(best.tvec) &&
                                  std::isfinite(best.yaw) && best.distance > 0;
                if (pose_valid) {
                    cv::Point3f meas_pos(best.tvec.at<double>(0), best.tvec.at<double>(1), best.tvec.at<double>(2));
                    double meas_yaw_rad = best.yaw * CV_PI / 180.0;
                    if (!tracker_inited) {
                        tracker.init(meas_pos, meas_yaw_rad, timestamp);
                        tracker_inited = true;
                        last_valid_position = meas_pos;
                    } else {
                        tracker.predict(timestamp);
                        tracker.update(meas_pos, meas_yaw_rad, timestamp);
                        last_valid_position = meas_pos;
                    }
                    cv::Point3f pred_pos = tracker.getEstimatedPosition();
                    double pred_yaw_deg = tracker.getPredictedYawDeg();
                    drawTrack(frame, best, pred_pos, pred_yaw_deg, camera_matrix, dist_coeffs);
                } else if (tracker_inited) {
                    tracker.predict(timestamp);
                    cv::Point3f pred_pos = tracker.getEstimatedPosition();
                    double pred_yaw_deg = tracker.getPredictedYawDeg();
                    drawTrack(frame, best, pred_pos, pred_yaw_deg, camera_matrix, dist_coeffs);
                }
            } else if (tracker_inited) {
                tracker.predict(timestamp);
                cv::Point3f pred_pos = tracker.getEstimatedPosition();
                double pred_yaw_deg = tracker.getPredictedYawDeg();
                drawTrack(frame, Armors(), pred_pos, pred_yaw_deg, camera_matrix, dist_coeffs);
            }

            cv::imshow("Armor Tracker (YOLO+EKF)", frame);
            if (cv::waitKey(30) == 27) break;
        }

        cap.release();
        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return -1;
    }
    return 0;
}