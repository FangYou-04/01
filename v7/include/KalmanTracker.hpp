#ifndef KALMAN_TRACKER_hpp
#define KALMAN_TRACKER_hpp

#include <opencv2/opencv.hpp>
#include <vector>

// 六维卡尔曼
class KalmanTracker
{
public:
    KalmanTracker();

    void init(const cv::Point3f& position, double timeStamp);
    cv::Point3f predicted(double timeStamp);
    cv::Point3f update(const cv::Point3f measuredPos, double timeStamp);

    cv::Point3f getEstimatedPosition() const;

private:
    void loadParamInConfig();
    void setTransitionMatrix(double dt);

private:
    cv::KalmanFilter m_kf;

    cv::Mat m_state;
    cv::Mat m_measured;

    double m_lastTime;
    double m_dt;
    double m_initialized;

    cv::Point3f m_predictedPose;
};

#endif