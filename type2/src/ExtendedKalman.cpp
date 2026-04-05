#include "ExtendedKalman.hpp"
#include "Config.hpp"
#include <cmath>
#include <iostream>

ExtendedKalman::ExtendedKalman()
    : m_lastTime(0), m_dt(0), m_initialized(false), m_predictedYaw(0.0f) {

    m_state = cv::Mat::zeros(6, 1, CV_32F);
    m_cov = cv::Mat::eye(6, 6, CV_32F);
    m_F = cv::Mat::eye(6, 6, CV_32F);
    m_Q = cv::Mat::zeros(6, 6, CV_32F);
    m_R = cv::Mat::zeros(4, 4, CV_32F);

    loadParamInConfig();
}

void ExtendedKalman::loadParamInConfig() {
    Config* cfg = Config::getInstance();
    if (!cfg) {
        std::cerr << "[EKF] ERROR: Config instance is null, using default parameters" << std::endl;
        // 使用默认值（已在构造函数中设置 m_Q, m_R 等为默认值，这里直接返回）
        return;
    }
    const AppConfig& app_cfg = cfg->getConfig();
    const KalmanConfig& k = app_cfg.kalman;
    m_Q.at<float>(0,0) = m_Q.at<float>(1,1) = m_Q.at<float>(2,2) = k.processNoisePos;
    m_Q.at<float>(3,3) = m_Q.at<float>(4,4) = m_Q.at<float>(5,5) = k.processNoiseVel;
    m_R.at<float>(0,0) = m_R.at<float>(1,1) = m_R.at<float>(2,2) = k.measurementNoisePos;
    m_R.at<float>(3,3) = k.measurementNoiseYaw;
    float initCov = k.initialErrorCov;
    cv::setIdentity(m_cov, cv::Scalar(initCov));
}

void ExtendedKalman::setTransitionMatrix(double dt) {
    cv::setIdentity(m_F);
    m_F.at<float>(0,3) = dt;
    m_F.at<float>(1,4) = dt;
    m_F.at<float>(2,5) = dt;
}

void ExtendedKalman::init(const cv::Point3f& pos, double yaw_rad, double ts) {
    m_state.at<float>(0) = pos.x;
    m_state.at<float>(1) = pos.y;
    m_state.at<float>(2) = pos.z;
    m_state.at<float>(3) = 0.0f;
    m_state.at<float>(4) = 0.0f;
    m_state.at<float>(5) = 0.0f;
    m_lastTime = ts;
    m_predictedPose = pos;
    m_predictedYaw = static_cast<float>(yaw_rad);
    m_initialized = true;
}

float ExtendedKalman::calcYawFromXY(float x, float y) const {
    return atan2(y, x);
}

void ExtendedKalman::predictState() {
    m_state = m_F * m_state;
}

void ExtendedKalman::predictCovariance() {
    m_cov = m_F * m_cov * m_F.t() + m_Q;
}

cv::Point3f ExtendedKalman::predict(double ts) {
    if (!m_initialized) return cv::Point3f(0,0,0);
    double dt = ts - m_lastTime;
    if (dt > 0.1) dt = 0.033;
    if (dt < 0.001) dt = 0.033;
    m_dt = dt;
    setTransitionMatrix(m_dt);
    predictState();
    predictCovariance();
    m_predictedPose.x = m_state.at<float>(0);
    m_predictedPose.y = m_state.at<float>(1);
    m_predictedPose.z = m_state.at<float>(2);
    m_predictedYaw = calcYawFromXY(m_predictedPose.x, m_predictedPose.y);
    m_lastTime = ts;
    return m_predictedPose;
}

void ExtendedKalman::computeJacobianH(const cv::Mat& state, cv::Mat& H) const {
    H = cv::Mat::zeros(4, 6, CV_32F);
    H.at<float>(0,0) = 1.0f;
    H.at<float>(1,1) = 1.0f;
    H.at<float>(2,2) = 1.0f;
    float x = state.at<float>(0);
    float y = state.at<float>(1);
    float r2 = x*x + y*y;
    if (r2 < 1e-6f) r2 = 1e-6f;
    H.at<float>(3,0) = -y / r2;
    H.at<float>(3,1) =  x / r2;
}

void ExtendedKalman::update(const cv::Mat& z) {
    cv::Mat H;
    computeJacobianH(m_state, H);
    cv::Mat S = H * m_cov * H.t() + m_R;
    cv::Mat K = m_cov * H.t() * S.inv();
    cv::Mat z_pred(4,1,CV_32F);
    z_pred.at<float>(0) = m_state.at<float>(0);
    z_pred.at<float>(1) = m_state.at<float>(1);
    z_pred.at<float>(2) = m_state.at<float>(2);
    z_pred.at<float>(3) = calcYawFromXY(m_state.at<float>(0), m_state.at<float>(1));
    cv::Mat y = z - z_pred;
    if (y.at<float>(3) > CV_PI) y.at<float>(3) -= 2*CV_PI;
    if (y.at<float>(3) < -CV_PI) y.at<float>(3) += 2*CV_PI;
    m_state = m_state + K * y;
    cv::Mat I = cv::Mat::eye(6,6,CV_32F);
    m_cov = (I - K * H) * m_cov * (I - K * H).t() + K * m_R * K.t();
}

cv::Point3f ExtendedKalman::update(const cv::Point3f measPos, double measYawRad, double ts) {
    if (!m_initialized) {
        init(measPos, measYawRad, ts);
        return measPos;
    }
    predict(ts);
    cv::Mat z(4,1,CV_32F);
    z.at<float>(0) = measPos.x;
    z.at<float>(1) = measPos.y;
    z.at<float>(2) = measPos.z;
    z.at<float>(3) = static_cast<float>(measYawRad);
    update(z);
    cv::Point3f est(m_state.at<float>(0), m_state.at<float>(1), m_state.at<float>(2));
    m_predictedPose = est;
    m_predictedYaw = calcYawFromXY(est.x, est.y);
    return est;
}

cv::Point3f ExtendedKalman::getEstimatedPosition() const {
    if (!m_initialized) return cv::Point3f(0,0,0);
    return cv::Point3f(m_state.at<float>(0), m_state.at<float>(1), m_state.at<float>(2));
}

float ExtendedKalman::getPredictedYaw() const { return m_predictedYaw; }
float ExtendedKalman::getPredictedYawDeg() const { return m_predictedYaw * 180.0f / CV_PI; }