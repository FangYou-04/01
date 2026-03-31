#ifndef DRAWTRACK_ONE_HPP
#define DRAWTRACK_ONE_HPP

#include <opencv2/opencv.hpp>
#include "Struct.hpp"
#include "Congfig.hpp"
#include "PoseSlove.hpp"
#include "KalmanTracker.hpp"

void drawTrack(cv::Mat& img, 
            const Armors& detected_armor, 
            const cv::Point3f& predict_position, 
            double predict_yaw,
            const cv::Mat& last_valid_rvec,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs);

#endif 