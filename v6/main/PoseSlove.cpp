#include "PoseSlove.hpp"
#include <opencv4/opencv2/opencv.hpp>

ArmorsDetector::ArmorsDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
{
    cameraMatrix_ = camera_matrix.clone();
    distCoeffs_ = dist_coeffs.clone(); 
}

bool ArmorsDetector::sloveArmorPose(Armors& armor)
{
    // 定义装甲板三维参考点
    std::vector<cv::Point3f> armor_3d_points;
    // 构造三维点集
    armor_3d_points.emplace_back(-armorWidth_/2,  armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth_/2,  armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth_/2, -armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back(-armorWidth_/2, -armorHeight_/2, 0.0f);

    // pnp解集
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(armor_3d_points, armor.corners, cameraMatrix_, 
                                distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_EPNP);
    if (!success) 
    {
        return false;
    }

    // 计算欧拉角（Z～Y顺序）
    cv::Mat R;
    cv::Rodrigues(rvec, R); // 旋转向量转旋转矩阵
    // pitch轴（俯仰角）【y】
    armor.pitch = atan2(-R.at<double>(2,0), sqrt(pow(R.at<double>(0,0), 2) + pow(R.at<double>(1,0),2))) * (180.0 / CV_PI);
    // yaw轴（偏航角）【z】
    armor.yaw = atan2(R.at<double>(1,0), R.at<double>(0,0)) * (180.0 / CV_PI);
    // roll轴（翻滚角）【x】
    armor.roll = atan2(R.at<double>(2,1), R.at<double>(2,2)) * (180.0 / CV_PI);

    // 角度归一化（限制在+-180）
    armor.pitch = fmod(armor.pitch + 180.0, 360.0) - 180.0;
    armor.yaw = fmod(armor.yaw + 180.0, 360.0) - 180.0;
    armor.roll = fmod(armor.roll + 180.0, 360.0) - 180.0;

    // 计算相机到装甲板距离
    armor.distance = sqrt(pow(tvec.at<double>(0),2) + pow(tvec.at<double>(1),2) + pow(tvec.at<double>(2),2));
    
    // 保存旋转和平移向量
    armor.rvec = rvec;
    armor.tvec = tvec;

    return true;
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat frame)
{
    std::vector<Armors> armors;

    cv::Mat mask = preprocessImage(frame);
    std::vector<Light> lights = detectLights(mask);
    std::vector<Armors> matchedArmors = matchArmors(lights);

    for (const auto& Armor : matchedArmors)
    {
        Armors info;
        info.boundingRect = Armor.boundingRect;
        info.boundingRect.center = Armor.center;
        
        cv::Point2f corners[4];
        Armor.boundingRect.points(corners);
        info.corners.assign(corners, corners + 4);

        if(sloveArmorPose(info))
        {
            armors.push_back(info);
        }
    }
    return armors;
}