#include "ExtendedKalman.hpp"
#include "Congfig.hpp"
#include <cmath>

ExtendedKalman::ExtendedKalman()
    : m_kf(6, 3, 0),
    m_lastTime(0),
    m_dt(0),
    m_initialized(false)
    m_predictedYaw(0.0f)
{
    // 观测矩阵H
    m_kf.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
    m_kf.measurementMatrix.at<float>(0, 0) = 1.0f;
    m_kf.measurementMatrix.at<float>(1, 1) = 1.0f;
    m_kf.measurementMatrix.at<float>(2, 2) = 1.0f;
    
    loadParamInConfig();
}

void ExtendedKalman::loadParamInConfig()
{
    const AppConfig& app_cfg = Config::getInstance() ->getConfig();
    const KalmanConfig& c = app_cfg.kalman;

    // 过程噪声Q
    cv::setIdentity(m_kf.processNoiseCov, cv::Scalar(0));
    m_kf.processNoiseCov.at<float>(0,0) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(1,1) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(2,2) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(3,3) = c.processNoiseVel;
    m_kf.processNoiseCov.at<float>(4,4) = c.processNoiseVel;
    m_kf.processNoiseCov.at<float>(5,5) = c.processNoiseVel;

    // 测量噪声R
    cv::setIdentity(m_kf.measurementNoiseCov, cv::Scalar(c.measurementNoisePos));

    // 最小后验协方差
    cv::setIdentity(m_kf.errorCovPost, cv::Scalar(c.initialErrorCov));
}

void ExtendedKalman::setTransitionMatrix(double dt)
{
    // EKF 线性化转移矩阵(匀速7.33)
    cv::Mat F = m_kf.transitionMatrix;
    cv::setIdentity(F);

    F.at<float>(0, 3) = dt;
    F.at<float>(1, 4) = dt;
    F.at<float>(2, 5) = dt;
}

// 初始化滤波器
void ExtendedKalman::init(const cv::Point3f& position, double timeStamp)
{
    m_state = cv::Mat::zeros(6, 1, CV_32F);
    m_state.at<float>(0) = position.x;
    m_state.at<float>(1) = position.y;
    m_state.at<float>(2) = position.z;

    m_kf.statePost = m_state.clone();

    m_lastTime = timeStamp;
    m_predictedPose = position;
    m_predicetedYaw = clacYawFromXY(position.x, position.y);
    m_initialized = true;
}

// EKF Yaw
float ExtendedKalman::clacYawFromXY(const x, const y);
{
    return atan2(y, x);
}

// 预测函数
cv::Point3f ExtendedKalman::predict(double timeStmap)
{
    if (!m_initialized)
    {
        return cv::Point3f(0, 0, 0);
    }

    // 计算时间差
    if (m_lastTime > 0)
    {
        m_dt = timeStmap - m_lastTime;
        if (m_dt < 0.001)
        {
            m_dt = 0.033;
        }
        if (m_dt > 0.1)
        {
            m_dt = 0.033;
        }
    }
    else
    {
        m_dt = 0.033
    }

    setTransitionMatrix(m_dt);
    m_state = m_kf.predict();

    // 提取预测位置
    m_predictedPose.x = m_state.at<float>(0);
    m_predictedPose.y = m_state.at<float>(1);
    m_predictedPose.z = m_state.at<float>(2);
    
    // EKF 预测步
    // EKF 非线性预测
    yaw = atan2(y, x);
    m_predicetedYaw = clacYawFromXY(m_predictedPose.x, m_predictedPose.y);

    return m_predictedPose;
}

// 更新， 预测后校正
cv::Point3f ExtendedKalman::update(const cv::Point3f measuredPos, double timeStamp)
{
    // 初始化
    if (!m_initialized)
    {
        init(measuredPos, timeStamp);
        return measuredPos;
    }
    
    // 预测步
    predicted(timeStamp);

    // 构造观测向量
    m_measure = cv::Mat::zeros(3, 1, CV_32F);
    m_measure.at<float>(0) = measuredPos.x;
    m_measure.at<float>(1) = measuredPos.y;
    m_measure.at<float>(2) = measuredPos.z;

    // 校正步
    m_state = m_kf.correct(m_measure);

    // 输出滤波后位置
    cv::Point3f estPos(
        m_state.at<float>(0),
        m_state.at<float>(1),
        m_state.at<float>(2)
    );

    m_lastTime = timeStamp;
    return estPos;
}

// 获取最优估计
cv::Point3f ExtendedKalman::getEstimatedPosition() const
{
    if (!m_initialized)
    {
        return cv::Point3f(0, 0, 0);
    }

    return cv::Point3f(
        m_kf.statePost.at<float>(0),
        m_kf.statePost.at<float>(1),
        m_kf.statePost.at<float>(2)
    );
}

// 获取预测Yaw
float ExtendedKalman::getPredictedYaw() const
{
    return m_predicetedYaw;
}

// Yaw转角度
float ExtendedKalman::getPredictedYawDeg() const
{
    return m_predicetedYaw * 180.0f/CV_PI;
}