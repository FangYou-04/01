#include "PoseSlove.hpp"
#include <opencv4/opencv2/opencv.hpp>
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
    // 1. 三维参考点（单位：米）
    std::vector<cv::Point3f> objectPoints = {
        {-armorWidth_/2, -armorHeight_/2, 0.0f}, // 左上
        { armorWidth_/2, -armorHeight_/2, 0.0f}, // 右上
        { armorWidth_/2,  armorHeight_/2, 0.0f}, // 右下
        {-armorWidth_/2,  armorHeight_/2, 0.0f}  // 左下
    };

    // 2. 图像角点（必须与三维点严格对应）
    std::vector<cv::Point2f> imagePoints(4);
    imagePoints[0] = armor.left.center  + cv::Point2f(0, -armor.left.rect.size.height/2);  // 左上
    imagePoints[1] = armor.right.center + cv::Point2f(0, -armor.right.rect.size.height/2); // 右上
    imagePoints[2] = armor.right.center + cv::Point2f(0,  armor.right.rect.size.height/2); // 右下
    imagePoints[3] = armor.left.center  + cv::Point2f(0,  armor.left.rect.size.height/2);  // 左下

    // 3. 使用 solvePnPGeneric 获取 IPPE 双解
    std::vector<cv::Mat> rvecs, tvecs;
    cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix_, distCoeffs_,
                        rvecs, tvecs, false, cv::SOLVEPNP_IPPE);

    if (rvecs.empty() || tvecs.empty())
    {
        std::cerr << "[solveArmorPose] solvePnPGeneric 未返回任何解" << std::endl;
        return false;
    }

    // 4. 选择物体在相机前方 (Z > 0) 的解，并用迭代法精化
    cv::Mat rvec, tvec;
    bool found = false;
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        if (rvecs[i].empty() || tvecs[i].empty()) continue;

        // 检查 Z 分量是否为正（相机前方）
        if (tvecs[i].at<double>(2) > 0)
        {
            // 用迭代法精化该解
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_,
                         rvecs[i], tvecs[i], true, cv::SOLVEPNP_ITERATIVE);
            rvec = rvecs[i];
            tvec = tvecs[i];
            found = true;
            break;
        }
    }

    if (!found)
    {
        std::cerr << "[solveArmorPose] 未找到有效的IPPE解(Z>0)" << std::endl;
        return false;
    }

    // 5. 有效性检查
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec))
    {
        std::cerr << "[solveArmorPose] rvec 或 tvec 含 NaN/Inf" << std::endl;
        return false;
    }

    double distance = cv::norm(tvec);
    if (distance <= 0 || distance > 10.0)
    {
        std::cerr << "[solveArmorPose] 距离异常: " << distance << std::endl;
        return false;
    }

    // 6. 计算旋转矩阵与欧拉角
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    if (!cv::checkRange(R))
    {
        std::cerr << "[solveArmorPose] 旋转矩阵含 NaN/Inf" << std::endl;
        return false;
    }

    // pitch轴（俯仰角）【y】
    double pitch =  asin(-R.at<double>(2, 0)) * 180.0 / CV_PI;
    // yaw轴（偏航角）【z】
    double yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;
    // roll轴（翻滚角）【x】
    double roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;


    if (!std::isfinite(pitch) || !std::isfinite(yaw) || !std::isfinite(roll))
    {
        std::cerr << "[solveArmorPose] 欧拉角含 NaN/Inf" << std::endl;
        return false;
    }

    // 7. 重投影误差校验（过滤错误解）
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix_, distCoeffs_, projected);
    double reprojErr = cv::norm(imagePoints, projected, cv::NORM_L2) / 4.0;
    if (reprojErr > 5.0)  // 阈值根据图像分辨率调整
    {
        std::cerr << "[solveArmorPose] 重投影误差过大: " << reprojErr << std::endl;
        return false;
    }

    // 8. 角度归一化到 [-180, 180)
    auto normalizeAngle = [](double angle) -> double {
        angle = std::fmod(angle, 360.0);
        if (angle < -180.0) angle += 360.0;
        if (angle >= 180.0) angle -= 360.0;
        return angle;
    };

    armor.pitch = normalizeAngle(pitch);
    armor.yaw   = normalizeAngle(yaw);
    armor.roll  = normalizeAngle(roll);
    armor.distance = distance;
    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();

    // 调试输出
    std::cout << "最终结果: 距离=" << distance << "m, "
              << "pitch=" << armor.pitch << "°, yaw=" << armor.yaw << "°, roll=" << armor.roll << "°" << std::endl;

    return true;
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat frame)
{
    std::vector<Armors> armors;

    cv::Mat mask = preprocessImage(frame);
    std::vector<Light> lights = detectLights(mask);
    std::vector<Armors> matchedArmors = matchArmors(lights);

    for (const auto& matched : matchedArmors)
    {
        Armors info = matched;

        if(solveArmorPose(info))
        {
            armors.push_back(info);
        }
    }
    return armors;
}