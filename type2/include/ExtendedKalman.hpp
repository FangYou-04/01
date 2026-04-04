#ifndef EXTENDED_KALMAN_HPP
#define EXTENDED_KALMAN_HPP

#include <opencv2/opencv.hpp>

class ExtendedKalman {
public:
    ExtendedKalman();
    void init(const cv::Point3f& position, double yaw_rad, double timeStamp);
    cv::Point3f predict(double timeStamp);
    cv::Point3f update(const cv::Point3f measuredPos, double measuredYawRad, double timeStamp);
    cv::Point3f getEstimatedPosition() const;
    float getPredictedYaw() const;      // 弧度
    float getPredictedYawDeg() const;

private:
    void loadParamInConfig();
    void setTransitionMatrix(double dt);
    float calcYawFromXY(float x, float y) const;
    void predictState();
    void predictCovariance();
    void computeJacobianH(const cv::Mat& state, cv::Mat& H) const;
    void update(const cv::Mat& z);

    cv::Mat m_state;   // 6x1: x,y,z, vx,vy,vz
    cv::Mat m_cov;     // 6x6
    cv::Mat m_F;       // 6x6
    cv::Mat m_Q;       // 6x6
    cv::Mat m_R;       // 4x4
    double m_lastTime;
    double m_dt;
    bool m_initialized;
    cv::Point3f m_predictedPose;
    float m_predictedYaw; // 弧度
};

#endif