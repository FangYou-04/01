#include "Config.hpp"
#include "Struct.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

// 调整旋转矩形角度，使其长边接近垂直方向
void adjustRotatedRect(cv::RotatedRect& rect, const float ANGLE_TO_UP = 90.0f)
{
    // 旋转矩形的angle角度范围为[-90, 0)，要转换为[0,180]的范围，方便后续处理
    float angle = rect.angle;
    if( angle < 0 ) angle += 180.0f;

    // 调整角度，使矩形长边接近垂直方向
    if (std::abs(angle - ANGLE_TO_UP) > 45.0f)
    {
        std::swap(rect.size.width, rect.size.height);
        rect.angle = angle - 90.0f;
    }
}

// 检测轮廓并返回符合条件的灯条列表
std::vector<LightBar>detectContours(const cv::Mat& mask)
{
    std::vector<LightBar> lightBars;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours)
    {
        if (cv::contourArea(contour) < MIN_LIGHTBAR_AREA) // 过滤掉面积过小的轮廓
            continue;
            
        cv::RotatedRect rect = cv::fitEllipse(contour); // 拟合椭圆以获得旋转矩形 
        adjustRotatedRect(rect, ANGLE_TO_UP); // 调整角度使其接近垂直

        LightBar lb(rect); // 使用自定义结构体存储灯条信息

        // 规范化宽高
        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) cv::swap(width, height);

        lb.length = height;
        lb.width = width;
        
        float ratio = (lb.width == 0) ? 0 : lb.length / lb.width;
        if (ratio >= MIN_LIGHTBAR_RATIO && ratio <= MAX_LIGHTBAR_RATIO)
        {
           lightBars.push_back(lb); // 过滤掉不符合长宽比的灯条
        }
    }
    
    std::sort(lightBars.begin(), lightBars.end(), [](const LightBar& a, const LightBar& b) {
        return a.center.y < b.center.y; // 按中心点的y坐标排序
    });

    return lightBars;
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
        if (contours[i].size() < 5) continue; // 拟合椭圆至少需要5个点

        double area = cv::contourArea(contours[i]);
        if (area < AREA) continue;

        // 获取最小外接矩形
        // cv::RotatedRect rect = cv::minAreaRect(contours[i]);

        cv::RotatedRect rect;
        try 
        {
            rect = cv::fitEllipse(contours[i]);
        } 
        catch (...) 
        {
            rect = cv::minAreaRect(contours[i]); // 椭圆拟合失败时降级
        }
        // adjustRotatedRect(rect, ANGLE_TO_UP); // 调整角度使其接近垂直

        // 规范化宽高
        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) cv::swap(width, height);

        // 计算比例和角度
        float ratio = 0.0f; // 【必做】变量显式初始化
        const float eps = 1e-6; // 浮点精度容错值，避免除0
        if (width > eps) {
            ratio = height / width;
        }

        float angle = fabs(rect.angle);

        // 计算轮廓的圆形度（灯条为细长矩形，圆形度接近0；反光点为圆形，圆形度接近1）
        float perimeter = cv::arcLength(contours[i], true);
        float circularity = 0.0f; // 【必做】变量显式初始化
        if (perimeter > eps) {
            circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        }
        
        bool isNoise = (circularity > 0.8f) && (ratio < 1.8f); // 根据经验值判断是否为噪声
        if (isNoise) continue;

        // 筛选符合比例和角度要求的灯条
        if (ratio > RATIO_MIN && ratio < RATIO_MAX && angle < ANGLE) {
            Light light;
            light.rect = rect;
            light.rat = ratio;
            light.AbsAngle = angle;
            light.center = rect.center;
            memset(light.vertices, 0, sizeof(light.vertices));

            cv::Point2f pts[4];
            light.rect.points(pts);

            const int vertexCount = sizeof(light.vertices)/sizeof(light.vertices[0]);
            
            for (int k = 0; k < 4; k++) 
            {
                light.vertices[k] = pts[k];
            }
            lights.push_back(light);
        }
    }
    // 调试：打印检测到的灯条数量
    std::cout << "检测到灯条数量：" << lights.size() << std::endl;

    return lights;
}