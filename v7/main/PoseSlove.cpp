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

// 新增：计算装甲板二维roll角（用于对比验证）
float get2DRoll(const cv::RotatedRect& ArmorRect) {
    float angle = ArmorRect.angle;
    // 规范化角度到[-90, 90)
    while (angle >= 90.0f) angle -= 180.0f;
    while (angle < -90.0f) angle += 180.0f;
    // 确保长边垂直时roll接近0°
    return (ArmorRect.size.width > ArmorRect.size.height) ? angle + 90.0f : angle;
}

bool ArmorsDetector::solveArmorPose(Armors& armor)
{
    // 定义装甲板三维参考点
    std::vector<cv::Point3f> armor_3d_points;
    // 构造三维点集（单位：米）
    armor_3d_points.emplace_back(-armorWidth_/2,  armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth_/2,  armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back( armorWidth_/2, -armorHeight_/2, 0.0f);
    armor_3d_points.emplace_back(-armorWidth_/2, -armorHeight_/2, 0.0f);

    // 重新排列角点
    std::vector<cv::Point2f> image_points = armor.corners;
    if (image_points.size() != 4)
    {
        std::cerr << "[solveArmorPose] 角点数量不是4个,跳过" << std::endl;
        return false;
    }

    // 顺时针排列 
    // 找到左上角点
    int top_left = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x + image_points[i].y <
            image_points[top_left].x + image_points[top_left].y)
        {
            top_left = i;
        }
    }

    // 找到右上角点
    int top_right = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x - image_points[i].y >
            image_points[top_right].x - image_points[top_right].y)
        {
            top_right = i;
        }
    }

    // 找到右下角点
    int bottom_right = 0;
    for (int i = 1; i < 4; i++)
    {
        if (image_points[i].x + image_points[i].y >
            image_points[bottom_right].x + image_points[bottom_right].y)
        {
            bottom_right = i;
        }
    }
    
    // 找到左下角点
    int bottom_left = 6 - top_left - top_right - bottom_right;

    // 排列
    std::vector<cv::Point2f> ordered_points(4);
    ordered_points[0] = image_points[top_left];
    ordered_points[1] = image_points[top_right];
    ordered_points[2] = image_points[bottom_right];
    ordered_points[3] = image_points[bottom_left];

    // pnp解集
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(armor_3d_points, ordered_points, cameraMatrix_, 
                                distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    if (!success) 
    {
        std::cerr << "[solveArmorPose] PnP求解失败" << std::endl;
        return false;
    }
    // debug输出——监视输入和输出
    std::cout << "ordered pts: "
              << ordered_points[0] << "," << ordered_points[1] << ","
              << ordered_points[2] << "," << ordered_points[3] << std::endl;
    std::cout << "rvec " << rvec.t() << " tvec " << tvec.t() << std::endl;

    // --- 新增：检查 rvec 和 tvec 是否有效 ---
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec)) {
        std::cerr << "[solveArmorPose] rvec 或 tvec 含 NaN/Inf,跳过" << std::endl;
        return false;
    }

    // 检查深度是否合理（物体在相机前方，z > 0）
    double z = tvec.at<double>(2);
    if (z <= 0 || z > 10.0) { // 假设最大检测距离为10米
        std::cerr << "[solveArmorPose] 深度 z 异常: " << z << "，跳过" << std::endl;
        return false;
    }

    // 计算欧拉角（Z～Y顺序）
    cv::Mat R;
    cv::Rodrigues(rvec, R); // 旋转向量转旋转矩阵

    // --- 新增：检查旋转矩阵 R 是否有效 ---
    if (!cv::checkRange(R)) {
        std::cerr << "[solveArmorPose] 旋转矩阵 R 含 NaN/Inf,跳过" << std::endl;
        return false;
    }

    // pitch轴（俯仰角）【y】
    double pitch = atan2(-R.at<double>(2,0), sqrt(pow(R.at<double>(0,0), 2) + pow(R.at<double>(1,0),2))) * (180.0 / CV_PI);
    // yaw轴（偏航角）【z】
    double yaw = atan2(R.at<double>(1,0), R.at<double>(0,0)) * (180.0 / CV_PI);
    // roll轴（翻滚角）【x】
    double roll = atan2(R.at<double>(2,1), R.at<double>(2,2)) * (180.0 / CV_PI);

    // 对比验证：计算二维roll角
    float roll_2d = get2DRoll(armor.boundingRect);
    std::cout << "roll3D: " << roll << " roll2D: " << roll_2d << std::endl;

    if (std::fabs(roll - roll_2d) > 60.0)
    {
        roll = -roll;
        rvec = -rvec;
        tvec = -tvec;
        std::cout << "修正后 roll3D: " << roll << std::endl;
    }
    

    // --- 新增：检查欧拉角是否有效 ---
    if (!std::isfinite(pitch) || !std::isfinite(yaw) || !std::isfinite(roll)) {
        std::cerr << "[solveArmorPose] 欧拉角含 NaN/Inf,跳过" << std::endl;
        return false;
    }

    // 角度归一化（限制在+-180）
    armor.pitch = fmod(pitch + 180.0, 360.0) - 180.0;
    armor.yaw = fmod(yaw + 180.0, 360.0) - 180.0;
    armor.roll = fmod(roll + 180.0, 360.0) - 180.0;

    if (armor.roll > 90.0)
    {
        armor.roll -= 180.0;
    }
    else if (armor.roll < -90.0)
    {
        armor.roll += 180.0;
    }
    
    // 计算相机到装甲板距离
    double distance = cv::norm(tvec);
    if (!std::isfinite(distance) || distance <= 0 || distance > 10.0) {
        std::cerr << "[solveArmorPose] 距离异常: " << distance << "，跳过" << std::endl;
        return false;
    }
    armor.distance = distance;
    
    // 保存旋转和平移向量
    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();

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
        cv::Point2f tempcorners[4];
        Armor.boundingRect.points(tempcorners);
        info.corners.assign(tempcorners, tempcorners + 4);

        if(solveArmorPose(info))
        {
            armors.push_back(info);
        }
    }
    return armors;
}