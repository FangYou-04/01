#include "PoseSlove.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

ArmorsDetector::ArmorsDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : config_(Config::getInstance()->getConfig())
{
    cameraMatrix_ = camera_matrix.clone();
    distCoeffs_ = dist_coeffs.clone();
    config_ptr_ = Config::getInstance();
}

bool ArmorsDetector::solveArmorPose(Armors& armor)
{
    // 2. 定义三维模型点
    std::vector<cv::Point3f> obj_pts = {
        {-static_cast<float>(armorWidth_/2),  static_cast<float>(armorHeight_/2), 0.0f},
        { static_cast<float>(armorWidth_/2),  static_cast<float>(armorHeight_/2), 0.0f},
        { static_cast<float>(armorWidth_/2), -static_cast<float>(armorHeight_/2), 0.0f},
        {-static_cast<float>(armorWidth_/2), -static_cast<float>(armorHeight_/2), 0.0f}
    };

    // 3. 获取图像角点
    std::vector<cv::Point2f> img_pts = armor.corners;
    if (img_pts.size() != 4) {
        std::cerr << "[solveArmorPose] 角点数量不为4" << std::endl;
        return false;
    }

    // 4. 角点排序
    cv::Point2f center(0, 0);
    for (const auto& pt : img_pts) center += pt;
    center *= 0.25f;

    std::vector<std::pair<float, int>> angle_idx;
    for (int i = 0; i < 4; ++i) {
        float angle = atan2(img_pts[i].y - center.y, img_pts[i].x - center.x);
        angle_idx.emplace_back(angle, i);
    }
    std::sort(angle_idx.begin(), angle_idx.end());

    // 找到左上角
    int top_left_idx = 0;
    float min_sum = img_pts[0].x + img_pts[0].y;
    for (int i = 1; i < 4; ++i) {
        float sum = img_pts[i].x + img_pts[i].y;
        if (sum < min_sum) {
            min_sum = sum;
            top_left_idx = i;
        }
    }

    // 找到起始位置
    int start_pos = 0;
    for (int i = 0; i < 4; ++i) {
        if (angle_idx[i].second == top_left_idx) {
            start_pos = i;
            break;
        }
    }

    // 按顺时针顺序排列
    std::vector<cv::Point2f> ordered_pts(4);
    ordered_pts[0] = img_pts[angle_idx[start_pos].second];               // 左上
    ordered_pts[1] = img_pts[angle_idx[(start_pos + 3) % 4].second];    // 右上
    ordered_pts[2] = img_pts[angle_idx[(start_pos + 2) % 4].second];    // 右下
    ordered_pts[3] = img_pts[angle_idx[(start_pos + 1) % 4].second];    // 左下

    // 5. IPPE求解
    std::vector<cv::Mat> rvecs, tvecs;
    bool success = cv::solvePnPGeneric(obj_pts, ordered_pts, cameraMatrix_, distCoeffs_,
                                       rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    
    if (!success || rvecs.size() != 2) {
        std::cerr << "[solveArmorPose] IPPE求解失败" << std::endl;
        return false;
    }

    // 10. 计算两个解的重投影误差
    double errors[2] = {0.0, 0.0};
    std::vector<cv::Point2f> proj_pts[2];
    
    for (int i = 0; i < 2; ++i) {
        cv::projectPoints(obj_pts, rvecs[i], tvecs[i], cameraMatrix_, distCoeffs_, proj_pts[i]);
        for (int j = 0; j < 4; ++j) {
            errors[i] += cv::norm(proj_pts[i][j] - ordered_pts[j]);
        }
        errors[i] /= 4.0;  // 平均误差
    }

    // 11. 选择最佳解
    int best_idx = 0;
    if (errors[1] < errors[0]) best_idx = 1;
    
    std::cout << "解0误差: " << errors[0] << " 像素, 解1误差: " << errors[1] << " 像素" << std::endl;
    std::cout << "选择解 " << best_idx << ", 误差: " << errors[best_idx] << " 像素" << std::endl;

    // 12. 误差阈值检查
    if (errors[best_idx] > 7.0) {
        std::cerr << "[solveArmorPose] 重投影误差过大: " << errors[best_idx] << " 像素" << std::endl;
        return false;
    }

    // 13. 保存初始解
    cv::Mat best_rvec = rvecs[best_idx].clone();
    cv::Mat best_tvec = tvecs[best_idx].clone();

    // 14. 迭代优化（可选）
    cv::Mat rvec_refined = best_rvec.clone();
    cv::Mat tvec_refined = best_tvec.clone();
    
    // 使用迭代法进一步优化
    cv::solvePnP(obj_pts, ordered_pts, cameraMatrix_, distCoeffs_,
                 rvec_refined, tvec_refined, true, cv::SOLVEPNP_ITERATIVE);
    
    // 计算优化后的重投影误差
    std::vector<cv::Point2f> proj_refined;
    cv::projectPoints(obj_pts, rvec_refined, tvec_refined, cameraMatrix_, distCoeffs_, proj_refined);
    double refined_error = 0.0;
    for (int j = 0; j < 4; ++j) {
        refined_error += cv::norm(proj_refined[j] - ordered_pts[j]);
    }
    refined_error /= 4.0;

    // 15. 选择误差更小的结果
    if (refined_error < errors[best_idx]) {
        armor.rvec = rvec_refined.clone();
        armor.tvec = tvec_refined.clone();
        std::cout << "优化后误差: " << refined_error << " 像素" << std::endl;
    } else {
        armor.rvec = best_rvec;
        armor.tvec = best_tvec;
    }

    // 16. 计算距离
    armor.distance = cv::norm(armor.tvec);

    // 17. 计算欧拉角
    cv::Mat R;
    cv::Rodrigues(armor.rvec, R);

    double pitch = atan2(-R.at<double>(2, 0), 
                         std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + 
                                   R.at<double>(1, 0) * R.at<double>(1, 0))) * 180.0 / CV_PI;
    double yaw   = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;
    double roll  = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;

    armor.pitch = pitch;
    armor.yaw   = yaw;
    armor.roll  = roll;

    // 18. 输出最终结果
    std::cout << "最终结果: 距离=" << armor.distance << "m, "
              << "pitch=" << pitch << "°, yaw=" << yaw << "°, roll=" << roll << "°" << std::endl;

    return true;
}


std::vector<Armors> ArmorsDetector::detect(const cv::Mat frame)
{
    std::vector<Armors> armors;

    cv::Mat mask = preprocessImage(frame);
    std::vector<Light> lights = detectLights(mask);
    std::vector<Armors> matchedArmors = matchArmors(lights);

    for (auto& Armor : matchedArmors)
    {
        if(solveArmorPose(Armor))
        {
            armors.push_back(Armor);
        }
    }
    return armors;
}