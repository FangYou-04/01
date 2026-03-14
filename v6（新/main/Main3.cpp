#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include "Kalman.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>

void drawTrack(cv::Mat& img, const Armors& detected_armor, 
               const cv::Point3f& predict_position, double predict_yaw,
               const cv::Mat& last_valid_rvec) {
    
    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;
    
    cv::FileStorage fs("src/calib_result.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "[ERROR] 无法打开calib_result.yml" << std::endl;
        return;
    }
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;
    fs.release();
    
    // 定义装甲板三维参考点
    const float armorWidth = 0.141f;
    const float armorHeight = 0.125f;
    std::vector<cv::Point3f> armor_3d_points;
    armor_3d_points.emplace_back(-armorWidth/2, armorHeight/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth/2, armorHeight/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth/2, -armorHeight/2, 0.0f);
    armor_3d_points.emplace_back(-armorWidth/2, -armorHeight/2, 0.0f);
    
    // ========== 绘制检测轮廓（红色）==========
    if (!detected_armor.rvec.empty() && !detected_armor.tvec.empty()) {
        std::vector<cv::Point2f> detected_points;
        cv::projectPoints(armor_3d_points, detected_armor.rvec, detected_armor.tvec, 
                         camera_matrix, dist_coeffs, detected_points);
        
        std::vector<cv::Point> detected_points_int;
        for (const auto& p : detected_points) {
            detected_points_int.emplace_back(cvRound(p.x), cvRound(p.y));
        }
        
        if (detected_points_int.size() == 4) {
            cv::polylines(img, detected_points_int, true, cv::Scalar(0, 0, 255), 2);
        }
    }
    
    // ========== 绘制预测轮廓（绿色）==========
    
    // 选择 rvec（优先用检测的姿态）
    cv::Mat rvec;
    if (!detected_armor.rvec.empty()) {
        rvec = detected_armor.rvec.clone();
    } else if (!last_valid_rvec.empty()) {
        rvec = last_valid_rvec.clone();
    } else {
        // 无任何历史：用预测的 yaw 构造 rvec
        cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);
        rot_mat.at<double>(0, 0) = cos(predict_yaw);
        rot_mat.at<double>(0, 1) = -sin(predict_yaw);
        rot_mat.at<double>(1, 0) = sin(predict_yaw);
        rot_mat.at<double>(1, 1) = cos(predict_yaw);
        rot_mat.at<double>(2, 2) = 1.0;
        cv::Rodrigues(rot_mat, rvec);
    }
    
    // 强制使用预测的 tvec
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 
                   static_cast<double>(predict_position.x),
                   static_cast<double>(predict_position.y),
                   static_cast<double>(predict_position.z));
    
    // 兜底：限制深度
    if (tvec.at<double>(2) < 0.5) {
        tvec.at<double>(2) = 0.5;
    }
    
    // 投影预测的3D点到2D
    std::vector<cv::Point2f> predict_image_points;
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix, dist_coeffs, predict_image_points);
    
    std::vector<cv::Point> predict_image_points_int;
    for (const auto& p : predict_image_points) {
        predict_image_points_int.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    
    if (predict_image_points_int.size() == 4) {
        cv::polylines(img, predict_image_points_int, true, cv::Scalar(0, 255, 0), 2);
    }
    
    // 绘制预测位置中心点（青色十字）
    std::vector<cv::Point2f> center_projected_vec;
    std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
    cv::projectPoints(center_3d, rvec, tvec, camera_matrix, dist_coeffs, center_projected_vec);

    if (!center_projected_vec.empty()) {
        cv::Point center_int(cvRound(center_projected_vec[0].x), cvRound(center_projected_vec[0].y));
        cv::drawMarker(img, center_int, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
    }
    
    // 显示滤波后坐标
    char text1[100];
    snprintf(text1, sizeof(text1), "Predict: (%.2f, %.2f, %.2f)", 
             predict_position.x, predict_position.y, predict_position.z);
    cv::putText(img, text1, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 2);
    
    char text2[100];
    snprintf(text2, sizeof(text2), "Predict Yaw: %.2f deg", predict_yaw);
    cv::putText(img, text2, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 2);
}



int main()
{
    // 定义相机内参和畸变系数
    cv::Mat camera_matrix, dist_coeffs;

    //读取文件
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

    ArmorsDetector armorsdetector(camera_matrix, dist_coeffs);

    // 保留yaw_rate的标准Kalman初始化
    Kalman tracker(0.033, 7.33);
    bool inited = false;
    cv::Point3f last_valid_position;
    double last_valid_yaw;
    cv::Mat last_valid_rvec; // 缓存最后一次有效完整rvec

    cv::VideoCapture cap("src/red1.mp4");
    if (!cap.isOpened())
    {
        std::cout << "视频打开失败！"; 
        return -1;
    }
    
    cv::Mat frame;

    while (cap.read(frame))
    {
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

        std::vector<Armors> armors = armorsdetector.detect(frame);

        if (!armors.empty()) {
            // 取第一个装甲板
            Armors best_armor = armors[0];

            // 位姿有效，更新历史缓存
            cv::Point3f detect_point(
                best_armor.tvec.at<double>(0), 
                best_armor.tvec.at<double>(1),
                best_armor.tvec.at<double>(2)
            );
            double yaw = best_armor.yaw;
            last_valid_position = detect_point;
            last_valid_yaw = yaw;
            last_valid_rvec = best_armor.rvec.clone(); // 缓存有效rvec

            if (!inited) {
                tracker.init(detect_point, yaw);
                inited = true;
            } else {
                tracker.predict();
                tracker.update(detect_point, yaw);
            }

            // 输出平滑3D坐标
            cv::Point3f predict_point = tracker.getPosition();
            double predict_yaw = tracker.getYaw();

            // 修复：调用drawTrack时传入last_valid_rvec
            drawTrack(frame, best_armor, predict_point, predict_yaw, last_valid_rvec);
        } else {
            // 没有检测到装甲板，若已初始化，仅执行预测并可视化
            if (inited) {
                tracker.predict();
                cv::Point3f predict_point = tracker.getPosition();
                double predict_yaw = tracker.getYaw();
                // 修复：调用drawTrack时传入last_valid_rvec
                drawTrack(frame, Armors(), predict_point, predict_yaw, last_valid_rvec);
            }
        }

        imshow("Armor Tracker", frame);
        if (cv::waitKey(50) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
