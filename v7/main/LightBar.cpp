#include "Config.hpp"
#include "Struct.hpp"
#include "Contours.cpp"
#include "PreProcess.cpp"
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<LightBar> detectLightBars (const cv::Mat& img, ArmorColor color)
{
    cv::Mat mask = PreProcess(img, color);
    return detectLightBars(mask, color);
}

