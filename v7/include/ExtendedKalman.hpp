#ifndef EXTENDED_KALMAN
#define EXTENDED_KALMAM

#include <opencv2/opencv.hpp>

/*
    六维卡尔曼（拓展卡尔曼）
    状态向量： x, y, z, vx, vy, vz
    观测： x, y, z
    非线性： yaw = atan2(y, x)
*/

class ExtendedKalman
{
public:
    ExtendedKalman();

    void init(const cv::Point3f& position, double timeStamp);
    cv::Point3f predict(double timeStamp);
    cv::Point3f update(const cv:Point3f measuredPos, double timeStamp);

    cv::Point3f getEstimatedPosition() const;
    float getPredictedYaw() const;
    float getPredictedYawDeg() const;

private:
    void loadParamInConfig();
    void setTransitionMatrix(double dt);
    float clacYawFromXY(float x, float y) const;

private:
    cv::KalmanFilter  m_kf;
    cv::Mat m_state;
    cv::Mat m_measure;
    
    double m_lastTime;
    double m_dt;
    double m_initialized;

    cv::Point3f m_predictedPose;
    float m_predicetedYaw;
};

#endif


/*















*/