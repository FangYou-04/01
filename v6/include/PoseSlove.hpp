#ifndef POSE_SLOVE_H
#define POSE_SLOVE_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"

struct Armors
{
    Light left;
    Light right;
    cv::RotatedRect boundingRect;
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
    cv::Mat rvec;
    cv::Mat tvec;
    float distance = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f; 
    float roll = 0.0f;
};

class ArmorsDetector
{
public:
// 构造函数，初始化相机内参和畸变参数
    ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
// 核心接口：输入图像中的装甲板，输出其位姿信息
    std::vector<Armors> detect(const cv::Mat frame);
// 友元
    friend void drawFriend(ArmorsDetector& detector, cv::Mat& img, const std::vector<Light> &lights, const std::vector<Armors> &armors);
private:
    // 图像预处理
    cv::Mat preprocessImage(const cv::Mat &frame);
    // 灯条检测
    std::vector<Light> detectLights(const cv::Mat &mask);
    // 装甲板配对
    std::vector<Armors> matchArmors(const std::vector<Light> &lights);
    // 位姿估计
    bool sloveArmorPose(Armors &armor);
    // 绘制
    void draw(cv::Mat &img, const std::vector<Light> &lights, const std::vector<Armors> &armors);

    // 相机内参和畸变参数
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    // 装甲板固定物理尺寸（单位：毫米）
    const float armorWidth_ = 141.0f;
    const float armorHeight_ = 125.0f;
};

#endif // POSE_SLOVE_H