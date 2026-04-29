#ifndef POSE_SLOVE_H
#define POSE_SLOVE_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"
#include "Congfig.hpp"

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
    
// ROI 设置接口
    void setROI(const cv::Rect& roi);
    void setROI(const cv::Point2f& center, float width, float height);
    void clearROI();
    cv::Rect getROI() const { return m_roi; }
    bool hasROI() const { return m_hasROI; }
    
private:
    // 图像预处理
    cv::Mat preprocessImage(const cv::Mat &frame);
    // 灯条检测
    std::vector<Light> detectLights(const cv::Mat &mask);
    // 装甲板配对
    std::vector<Armors> matchArmors(const std::vector<Light> &lights);
    // 位姿估计
    bool solveArmorPose(Armors &armor);

    // 相机内参和畸变参数
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    // 配置
    Config* config_ptr_;
    const AppConfig& config_;
    // 装甲板固定物理尺寸（单位：米）
    const float armorWidth_ = 0.141f;
    const float armorHeight_ = 0.125f;
    // ROI 区域
    cv::Rect m_roi;
    bool m_hasROI;
    
    // 内部辅助函数
    cv::Rect clampROI(const cv::Mat& img) const;
};

#endif // POSE_SLOVE_H