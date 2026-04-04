#include "PoseSolve.hpp"
#include <iostream>
#include <cmath>

ArmorsDetector::ArmorsDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
    : cameraMatrix_(cameraMatrix.clone()), distCoeffs_(distCoeffs.clone()) {
    config_ptr_ = Config::getInstance();
    if (!config_ptr_) throw std::runtime_error("Config singleton failed");
    config_ = &(config_ptr_->getConfig());
    armorWidth_ = config_->armor_size.width;
    armorHeight_ = config_->armor_size.height;
}

bool ArmorsDetector::solveArmorPose(Armors& armor) {
    std::vector<cv::Point3f> obj_pts = {
        {-armorWidth_/2,  armorHeight_/2, 0},
        { armorWidth_/2,  armorHeight_/2, 0},
        { armorWidth_/2, -armorHeight_/2, 0},
        {-armorWidth_/2, -armorHeight_/2, 0}
    };
    if (armor.corners.size() != 4) return false;

    // 顺时针排序（左上、右上、右下、左下）
    std::vector<cv::Point2f> pts = armor.corners;
    int top_left = 0;
    for (int i=1;i<4;++i) if (pts[i].x+pts[i].y < pts[top_left].x+pts[top_left].y) top_left=i;
    int top_right = 0;
    for (int i=1;i<4;++i) if (pts[i].x-pts[i].y > pts[top_right].x-pts[top_right].y) top_right=i;
    int bottom_right = 0;
    for (int i=1;i<4;++i) if (pts[i].x+pts[i].y > pts[bottom_right].x+pts[bottom_right].y) bottom_right=i;
    int bottom_left = 6 - top_left - top_right - bottom_right;
    std::vector<cv::Point2f> ordered = {pts[top_left], pts[top_right], pts[bottom_right], pts[bottom_left]};

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(obj_pts, ordered, cameraMatrix_, distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    if (!success) return false;
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec)) return false;
    double z = tvec.at<double>(2);
    if (z <= 0 || z > 10.0) return false;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double pitch = atan2(-R.at<double>(2,0), sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0))) * 180.0/CV_PI;
    double yaw   = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0/CV_PI;
    double roll  = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180.0/CV_PI;

    armor.rvec = rvec.clone();
    armor.tvec = tvec.clone();
    armor.distance = cv::norm(tvec);
    armor.pitch = pitch;
    armor.yaw   = yaw;
    armor.roll  = roll;
    return true;
}

std::vector<Armors> ArmorsDetector::detect(const cv::Mat& frame,
                                           const std::vector<cv::Rect>& boxes,
                                           const std::vector<float>& confs,
                                           const std::vector<int>& ids) {
    std::vector<Armors> armors;
    if (frame.empty() || boxes.empty()) return armors;
    float conf_blue = config_->yolo_config.conf_thred_blue;
    float conf_red  = config_->yolo_config.conf_thred_red;
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (i >= confs.size() || i >= ids.size()) continue;
        int cls = ids[i];
        float conf = confs[i];
        if ((cls == 0 && conf < conf_blue) || (cls == 1 && conf < conf_red)) continue;
        Armors armor;
        armor.classId = cls;
        armor.confidence = conf;
        const cv::Rect& box = boxes[i];
        armor.center = cv::Point2f(box.x + box.width/2.0f, box.y + box.height/2.0f);
        armor.boundingRect = cv::RotatedRect(armor.center, cv::Size2f(box.width, box.height), 0);
        armor.corners = {
            cv::Point2f(box.x, box.y),
            cv::Point2f(box.x+box.width, box.y),
            cv::Point2f(box.x+box.width, box.y+box.height),
            cv::Point2f(box.x, box.y+box.height)
        };
        if (solveArmorPose(armor))
            armors.push_back(armor);
    }
    return armors;
}