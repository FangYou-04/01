#include "Congfig.hpp"
#include "Struct.hpp"
#include "PoseSlove.hpp"
#include <algorithm>
#include <cmath>
#include <cfloat>


// 图像预处理
cv::Mat ArmorsDetector::preprocessImage(const cv::Mat &img) 
{
    
    if(img.empty() || img.data == nullptr || img.rows == 0 ||
         img.cols == 0 || img.type() != CV_8UC3)
    {
        std::cout << "[ERROR]预处理图像无效" << std::endl;
        return cv::Mat();
    }

    cv::Mat img_continuous = img.isContinuous() ? img : img.clone();

    // 亮度调整
    cv::Mat img_L;
    int beta = -90; // 如果要测试不同亮度请修改此处
    img_continuous.convertTo(img_L, -1, 0.7, beta);  // 降低对比度

   cv::Mat gray, blur, binary;
    cv::cvtColor(img_L, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(3, 3), 0); // 在灰度图阶段模糊，减少噪声
    // cv::threshold(blur, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // 大津阈值
    cv::adaptiveThreshold(blur, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 15, -5); // 自适应阈值

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(config_.morph_config.kernel_size, config_.morph_config.kernel_size));
    cv::Mat img_N;
    cv::morphologyEx(binary, img_N, cv::MORPH_CLOSE, kernel); // 直接对二值图形态学操作
    cv::imshow("二值化", img_N);

    return img_N;
}


// 灯条检测
std::vector<Light> ArmorsDetector::detectLights(const cv::Mat &mask) 
{
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 筛选灯条
    std::vector<Light> lights;
    for (size_t i = 0; i < contours.size(); i++) 
    {
        // if (contours[i].size() < 5) continue; // 拟合椭圆至少需要5个点

        double area = cv::contourArea(contours[i]);
        if (area < config_.light_config.area) continue;

        // 获取最小外接矩形
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);

        // cv::RotatedRect rect;
        // try 
        // {
        //     rect = cv::fitEllipse(contours[i]);
        // } 
        // catch (...) 
        // {
        //     rect = cv::minAreaRect(contours[i]); // 椭圆拟合失败时降级
        // }

        // 规范化宽高
        float width = rect.size.width;
        float height = rect.size.height;
        float angle = rect.angle;
        if (width > height)
        {
            std::swap(width, height);
            angle += 90.0f; // 旋转角度调整
            if (angle > 90.0f)
            {
                angle -= 180.0f; // 保持角度在 [-90, 90] 范围内
            }
            
        }

        // 计算比例和角度
        float ratio = 0.0f; // 【必做】变量显式初始化
        if (width > 0.0f) // 避免除0
        {
            ratio = height / width;
        }

        // 筛选符合比例和角度要求的灯条
        if (ratio > config_.light_config.ratio_max || ratio < config_.light_config.ratio_min)
            continue;

        if (std::fabs(angle) > config_.light_config.angle)
            continue;

        Light light;
        light.rect = rect;
        light.rat = ratio;
        light.AbsAngle = angle;
        light.center = rect.center;

        cv::Point2f pts[4];
        light.rect.points(pts);

        const int vertexCount = sizeof(light.vertices)/sizeof(light.vertices[0]);
        
        for (int k = 0; k < 4; k++) 
        {
            light.vertices[k] = pts[k];
        }
        lights.push_back(light);
        
    }
    // 调试：打印检测到的灯条数量
    std::cout << "检测到灯条数量：" << lights.size() << std::endl;

    return lights;
}

// 装甲板配对
std::vector<Armors> ArmorsDetector::matchArmors(const std::vector<Light> &lights) 
{
    std::vector<Armors> armors;
    if (lights.size() < 2) return armors;

    // 两两配对灯条
    for (size_t i = 0; i < lights.size(); i++) 
    {
        for (size_t j = i + 1; j < lights.size(); j++) 
        {
            const Light &bar1 = lights[i];
            const Light &bar2 = lights[j];

            const Light &leftBar = bar1.center.x < bar2.center.x ? bar1 : bar2;
            const Light &rightBar = bar1.center.x < bar2.center.x ? bar2 : bar1;

            float distance = cv::norm(leftBar.center - rightBar.center);
            float heightAvg = (leftBar.rect.size.height + rightBar.rect.size.height) / 2;

            if (heightAvg < 0.0f) continue; // 避免除0

            // 距离筛选
            float distMin = heightAvg * config_.armor_config.distance_min;
            float distMax = heightAvg * config_.armor_config.distance_max;
            if (distance < distMin || distance > distMax) {
                std::cout << "[不满足] 距离不在范围内: dist=" << distance << ", range=[" << distMin << ", " << distMax << "]" << std::endl;
                continue;
            }

            float angle = fabs(atan2(rightBar.center.y - leftBar.center.y,
                                     rightBar.center.x - leftBar.center.x) * 180 / CV_PI);
            if (angle > config_.armor_config.armor_angle) {
                std::cout << "[不满足] 装甲板角度过大: angle=" << angle << ", limit=" << config_.armor_config.armor_angle << std::endl;
                continue;
            }

            float heightDiff = fabs(leftBar.rect.size.height - rightBar.rect.size.height)
                               / std::max(leftBar.rect.size.height, rightBar.rect.size.height);
            if (heightDiff > config_.armor_config.height_diff) {
                std::cout << "[不满足] 高度差过大: heightDiff=" << heightDiff << ", limit=" << config_.armor_config.height_diff << std::endl;
                continue;
            }

            float angleDiff = fabs(leftBar.AbsAngle - rightBar.AbsAngle);
            if (angleDiff > config_.armor_config.angle_diff) {
                std::cout << "[不满足] 灯条角度差过大: angleDiff=" << angleDiff << ", limit=" << config_.armor_config.angle_diff << std::endl;
                continue;
            }

            float armorRatio = distance / heightAvg;
            if (armorRatio < config_.armor_config.armor_ratio_min || armorRatio > config_.armor_config.armor_ratio_max) {
                std::cout << "[不满足] 装甲板比例不在范围内: ratio=" << armorRatio << ", range=[" << config_.armor_config.armor_ratio_min << ", " << config_.armor_config.armor_ratio_max << "]" << std::endl;
                continue;
            }

            // 垂直偏差筛选（放宽以适应斜视角切入）
            float yDiff = std::fabs(leftBar.center.y - rightBar.center.y);
            if (yDiff > heightAvg * 3.0f) {
                // 调试：打印哪个条件不满足
                std::cout << "[不满足] 垂直偏差过大: yDiff=" << yDiff << ", limit=" << heightAvg * 3.0f << std::endl;
                continue;
            }

            Armors armor;
            armor.left = leftBar;
            armor.right = rightBar;
            armor.center = (leftBar.center + rightBar.center) * 0.5f;

            cv::Point2f armorPts[4];
            armorPts[0] = leftBar.vertices[1].y > leftBar.vertices[3].y ? leftBar.vertices[1] : leftBar.vertices[3];
            armorPts[1] = rightBar.vertices[0].y > rightBar.vertices[2].y ? rightBar.vertices[0] : rightBar.vertices[2];
            armorPts[2] = rightBar.vertices[0].y < rightBar.vertices[2].y ? rightBar.vertices[0] : rightBar.vertices[2];
            armorPts[3] = leftBar.vertices[1].y < leftBar.vertices[3].y ? leftBar.vertices[1] : leftBar.vertices[3];

            armor.corners.resize(4);  // 
            std::copy(std::begin(armorPts), std::end(armorPts), std::begin(armor.corners));

            std::vector<cv::Point2f> pts_vec(armorPts, armorPts + 4);
            armor.boundingRect = cv::minAreaRect(pts_vec);
            
            armors.push_back(armor);

            // 调试：打印角点坐标
            std::cout << "装甲板角点坐标: " << armor.corners[0] << ", " << armor.corners[1] << ", " << armor.corners[2] << ", " << armor.corners[3] << std::endl;

        }
    }
    // 调试：打印装甲板数量
    std::cout << "检测到装甲板数量：" << armors.size() << std::endl;

    return armors;
}
