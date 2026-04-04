#ifndef POSE_SOLVE_H
#define POSE_SOLVE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "Struct.hpp"
#include "Config.hpp"

class ArmorsDetector {
public:
    ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    std::vector<Armors> detect(const cv::Mat& frame,
                               const std::vector<cv::Rect>& yoloBoxes,
                               const std::vector<float>& confidences,
                               const std::vector<int>& classIds);
    // 为了兼容，保留空实现
    std::vector<Armors> detect(const cv::Mat frame) { return {}; }

private:
    bool solveArmorPose(Armors& armor);
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    Config* config_ptr_;
    const AppConfig* config_;
    float armorWidth_, armorHeight_;
};

#endif