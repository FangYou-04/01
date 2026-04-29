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

    // 2. 位姿估计
    if (armor.corners.empty()) {
        std::cerr << "[ERROR] 装甲板角点为空" << std::endl;
        return false;
    }

    try {
        cv::solvePnP(objectPoints, armor.corners, cameraMatrix_, distCoeffs_,
                     armor.rvec, armor.tvec, false, cv::SOLVEPNP_IPPE);
        
        // 计算距离和角度
        armor.distance = cv::norm(armor.tvec);
        
        double yaw_rad = atan2(armor.tvec.at<double>(0), armor.tvec.at<double>(2));
        double pitch_rad = atan2(armor.tvec.at<double>(1), armor.tvec.at<double>(2));
        
        armor.yaw = yaw_rad * 180.0 / CV_PI;
        armor.pitch = pitch_rad * 180.0 / CV_PI;
        
        // 从旋转向量恢复欧拉角 (Roll, Pitch, Yaw)
        cv::Mat rotationMatrix;
        cv::Rodrigues(armor.rvec, rotationMatrix);
        
        // 提取欧拉角（ZYX顺序：Yaw-Pitch-Roll）
        double sy = sqrt(rotationMatrix.at<double>(0,0) * rotationMatrix.at<double>(0,0) +
                         rotationMatrix.at<double>(1,0) * rotationMatrix.at<double>(1,0));
        bool singular = sy < 1e-6;
        
        double roll, pitch, yaw;
        if (!singular) {
            roll = atan2(rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2));
            pitch = atan2(-rotationMatrix.at<double>(2,0), sy);
            yaw = atan2(rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(0,0));
        } else {
            roll = atan2(-rotationMatrix.at<double>(1,2), rotationMatrix.at<double>(1,1));
            pitch = atan2(-rotationMatrix.at<double>(2,0), sy);
            yaw = 0;
        }
        
        armor.roll = roll * 180.0 / CV_PI;

        std::cout << "最终结果: 距离=" << armor.distance << "m, "
              << "pitch=" << pitch << "°, yaw=" << yaw << "°, roll=" << roll << "°" << std::endl;

        
        return true;
    } 
    catch (const cv::Exception& e) {
        std::cerr << "[ERROR] PnP求解失败: " << e.what() << std::endl;
        return false;
    }
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

        if (!armors.empty())
        {
            // 使用 lambda 按距离升序排序
            std::sort(armors.begin(), armors.end(),
                    [](const Armors& a, const Armors& b) {
                        return a.distance < b.distance;
                    });
            // 只保留第一个（距离最小）
            armors.erase(armors.begin() + 1, armors.end());
        }
    }
    return armors;
}
