#include "PoseSlove.hpp"
#include "Congfig.hpp"
#include "Struct.hpp"
#include <string>

// 绘制识别结果
void ArmorsDetector::draw(cv::Mat &mask, const std::vector<Light> &lights, const std::vector<Armors> &armors) {
    
    // 检测无灯条时反馈
    if (lights.empty() && armors.empty()) 
    {
        cv::putText(mask, "No Lights/Armors Detected", cv::Point2f(20, 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        return;
    }
    
    // 绘制灯条
    // for (size_t i = 0; i < lights.size(); i++) 
    // {
    //     const auto &light = lights[i];
    //     cv::Point2f pts[4];
    //     light.rect.points(pts);
    //     for (int k = 0; k < 4; k++) 
    //     {
    //         cv::line(mask, pts[k], pts[(k+1)%4], cv::Scalar(0,255,0), 3);
    //     }
    //     cv::circle(mask, light.center, 3, cv::Scalar(255,0,0), -1);

    //     // 添加灯条编号
    //     cv::putText(mask, std::to_string(i + 1), light.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    // }

    for (size_t i = 0; i < armors.size(); i++) 
    {
        const auto &armor = armors[i];

        cv::Point2f center = armor.boundingRect.center;

        // 绘制装甲板  
        cv::Point2f armor_pts[4];
        armor.boundingRect.points(armor_pts);
        for (int k = 0; k < 4; k++)
        {
            cv::line(mask, armor_pts[k], armor_pts[(k+1)%4], cv::Scalar(0,255,0), 3);
        }
        cv::circle(mask, center, 2, cv::Scalar(0,255,0), -1);

        // 绘制tvec与rvec
        std::string tvecText = "T: x " + std::to_string(static_cast<int>(armor.tvec.at<double>(0))) + 
                                "  y " + std::to_string(static_cast<int>(armor.tvec.at<double>(1))) + 
                                "  z " + std::to_string(static_cast<int>(armor.tvec.at<double>(2))); 
        std::string rvecText = "R: x " + std::to_string(static_cast<int>(armor.rvec.at<double>(0))) + 
                                "  y " + std::to_string(static_cast<int>(armor.rvec.at<double>(1))) +
                                "  z " + std::to_string(static_cast<int>(armor.rvec.at<double>(2))) +
                                "  length " + std::to_string(cv::norm(armor.rvec));           ;
        cv::putText(mask, tvecText, cv::Point2f(10, 50 + i * 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        cv::putText(mask, rvecText, cv::Point2f(10, 30 + i * 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

        // 绘制yaw pitch roll
        std::string eulerText = "euler angles: yaw " + std::to_string(static_cast<int>(armor.yaw)) + 
                                " pitch " + std::to_string(static_cast<int>(armor.pitch)) + 
                                " roll " + std::to_string(static_cast<int>(armor.roll));
        cv::putText(mask, eulerText, cv::Point2f(10, 70 + i * 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

        
    }
}


