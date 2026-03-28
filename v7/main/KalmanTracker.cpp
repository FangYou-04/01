#include "KalmanTracker.hpp"
#include "Congfig.hpp"
#include <cmath>

// 构造函数
KalmanTracker::KalmanTracker()
    : m_kf(6, 3, 0),
    m_lastTime(0),
    m_dt(0),
    m_initialized(false)
{
    // 观测矩阵 H(3 * 6)
    // 仅观测位置而不直接观测速度
    m_kf.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
    m_kf.measurementMatrix.at<float>(0, 0) = 1.0f;
    m_kf.measurementMatrix.at<float>(1, 1) = 1.0f;
    m_kf.measurementMatrix.at<float>(2, 2) = 1.0f;

    loadParamInConfig();
}

// 从Config中加载卡尔曼噪声参数
void KalmanTracker::loadParamInConfig()
{
    const AppConfig& app_cfg = Config::getInstance()->getConfig();
    const KalmanConfig& c = app_cfg.kalman;

    cv::setIdentity(m_kf.processNoiseCov, cv::Scalar(0));
    m_kf.processNoiseCov.at<float>(0,0) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(1,1) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(2,2) = c.processNoisePos;
    m_kf.processNoiseCov.at<float>(3,3) = c.processNoiseVel;
    m_kf.processNoiseCov.at<float>(4,4) = c.processNoiseVel;
    m_kf.processNoiseCov.at<float>(5,5) = c.processNoiseVel;

    cv::setIdentity(m_kf.measurementNoiseCov, cv::Scalar(c.measurementNoisePos));
    cv::setIdentity(m_kf.errorCovPost, cv::Scalar(c.initialErrorCov));
}

// 卡尔曼匀速模型形态转移矩阵
void KalmanTracker::setTransitionMatrix(double dt)
{
    cv::Mat& F = m_kf.transitionMatrix;
    cv::setIdentity(F);

    // x = x_prev + vx*dt
    F.at<float>(0, 3) = dt;
    F.at<float>(1, 4) = dt;
    F.at<float>(2, 5) = dt;
}

// 初始化滤波器
void KalmanTracker::init(const cv::Point3f& position, double timeStamp)
{
    m_state = cv::Mat::zeros(6, 1, CV_32F);
    m_state.at<float>(0) = position.x;
    m_state.at<float>(1) = position.y;
    m_state.at<float>(2) = position.z;

    m_kf.statePost = m_state.clone();

    m_lastTime = timeStamp;
    m_predictedPose = position;
    m_initialized = true;
}

// 预测函数
cv::Point3f KalmanTracker::predicted(double timeStamp)
{
    if (!m_initialized)
    {
        return cv::Point3f(0, 0, 0);
    }

    // 计算时间差并限制区间
    if (m_lastTime > 0)
    {
        m_dt = timeStamp - m_lastTime;
        if(m_dt > 0.1) m_dt = 0.033;
        if(m_dt < 0.001) m_dt = 0.033;
    }
    else
    {
        m_dt = 0.033;   // 默认30fps
    }
    
    setTransitionMatrix(m_dt);
    m_state = m_kf.predict();

    // 提取预测位置
    m_predictedPose.x = m_state.at<float>(0);
    m_predictedPose.y = m_state.at<float>(1);
    m_predictedPose.z = m_state.at<float>(2);

    return m_predictedPose;
}

// 更新，预测后校正
cv::Point3f KalmanTracker::update(const cv::Point3f measuredPos, double timeStamp)
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
    m_measured = cv::Mat::zeros(3, 1, CV_32F);
    m_measured.at<float>(0) = measuredPos.x;
    m_measured.at<float>(1) = measuredPos.y;
    m_measured.at<float>(2) = measuredPos.z;

    // 校正步
    m_state = m_kf.correct(m_measured);

    // 输出滤波后位置
    cv::Point3f estPos(
        m_state.at<float>(0),
        m_state.at<float>(1),
        m_state.at<float>(2)
    );

    m_lastTime = timeStamp;
    return estPos;
}

// 外部获取最优估计
cv::Point3f KalmanTracker::getEstimatedPosition() const
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