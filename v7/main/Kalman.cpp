#include "Kalman.hpp"

// 构造函数
Kalman::Kalman(double dt, double angular_velocity) : dt_(dt), angular_velocity_(angular_velocity) 
{
    // 状态维度为8（x, y, z, vx, vy, vz, yaw, yaw_rate）
    // 测量维度为4（x, y, z, yaw）
    // 控制维度为0（无控制输入）
    // 匀速模型
    kf_ = cv::KalmanFilter(8, 4, 0, CV_64F);
    
    // 初始化状态向量和测量向量
    state_ = cv::Mat::zeros(8, 1, CV_64F);
    measurement_ = cv::Mat::zeros(4, 1, CV_64F);
    
    // 状态转移矩阵A
    cv::Mat A = cv::Mat::eye(8, 8, CV_64F);
    A.at<double>(0, 3) = dt_;  // x' = x + vx*dt
    A.at<double>(1, 4) = dt_;  // y' = y + vy*dt
    A.at<double>(2, 5) = dt_;  // z' = z + vz*dt
    A.at<double>(6, 7) = dt_;  // yaw' = yaw + yaw_rate*dt
    kf_.transitionMatrix = A;
    
    // 测量矩阵H
    cv::Mat H = cv::Mat::zeros(4, 8, CV_64F);
    H.at<double>(0, 0) = 1;  // 测量x = 1 * x
    H.at<double>(1, 1) = 1;  // 测量y = 1 * y
    H.at<double>(2, 2) = 1;  // 测量z = 1 * z
    H.at<double>(3, 6) = 1;  // 测量yaw = 1 * yaw
    kf_.measurementMatrix = H;
    
    // 差异化噪声参数
    cv::Mat Q = cv::Mat::zeros(8, 8, CV_64F);
    Q.at<double>(0, 0) = 1e-2;   // x 位置噪声
    Q.at<double>(1, 1) = 1e-2;   // y 位置噪声
    Q.at<double>(2, 2) = 5e-3;   // z 位置噪声（高度更稳定）
    Q.at<double>(3, 3) = 1e-2;   // vx 速度噪声
    Q.at<double>(4, 4) = 1e-2;   // vy 速度噪声
    Q.at<double>(5, 5) = 5e-3;   // vz 速度噪声
    Q.at<double>(6, 6) = 5e-2;   // yaw 角度噪声
    Q.at<double>(7, 7) = 5e-2;   // yaw_rate 角速度噪声（变化较大）
    kf_.processNoiseCov = Q;
    
    cv::Mat R = cv::Mat::zeros(4, 4, CV_64F);
    R.at<double>(0, 0) = 1e-3;   // x 测量噪声
    R.at<double>(1, 1) = 1e-3;   // y 测量噪声
    R.at<double>(2, 2) = 5e-3;   // z 测量噪声
    R.at<double>(3, 3) = 1e-2;   // yaw 测量噪声（角度测量通常更不准）
    kf_.measurementNoiseCov = R;
    
    // 后验错误协方差矩阵P
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1e-3));
}

// 初始化滤波器状态
void Kalman::init(const cv::Point3f& position, double yaw) 
{
    state_.at<double>(0) = std::isfinite(position.x) ? position.x : 0.0;
    state_.at<double>(1) = std::isfinite(position.y) ? position.y : 0.0;
    state_.at<double>(2) = std::isfinite(position.z) ? position.z : 0.0;
    
    double yaw_norm = std::fmod(yaw + 180.0, 360.0) - 180.0;  // 将yaw规范化到[-180, 180]
    state_.at<double>(6) = yaw_norm;  // yaw
    state_.at<double>(3) = 0.0;  // vx
    state_.at<double>(4) = 0.0;  // vy
    state_.at<double>(5) = 0.0;  // vz
    state_.at<double>(7) = angular_velocity_;  // yaw_rate
    
    last_position_ = position;  // 记录初始位置
    kf_.statePost = state_.clone();  // 将初始状态设置为后验状态
}

// 预测下一帧状态
void Kalman::predict() 
{
    cv::Mat prediction = kf_.predict();  // 预测下一帧状态
    
    // 限制z轴在合理范围内
    prediction.at<double>(2) = std::clamp(prediction.at<double>(2), 0.5, 10.0);
    
    // 将yaw规范化到[-180, 180]
    prediction.at<double>(6) = std::fmod(prediction.at<double>(6) + 180.0, 360.0) - 180.0;
    
    state_ = prediction.clone();  // 更新后验状态
    last_position_ = getPosition();  // 更新上一次有效位置
}

// 更新滤波器状态
void Kalman::update(const cv::Point3f& position, double yaw) 
{
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || 
        !std::isfinite(position.z) || !std::isfinite(yaw)) 
    {
        return;  // 如果测量值无效，则跳过更新
    }
    
    // 测量值范围过滤（防止异常值）
    if (std::abs(position.x) > 15.0 || std::abs(position.y) > 10.0 || 
        position.z < 0.1 || position.z > 20.0) 
    {
        return;  // 异常测量，跳过更新
    }
    
    // 填充测量向量
    measurement_.at<double>(0) = static_cast<double>(position.x);  // 测量x
    measurement_.at<double>(1) = static_cast<double>(position.y);  // 测量y
    measurement_.at<double>(2) = static_cast<double>(position.z);  // 测量z
    measurement_.at<double>(3) = std::fmod(yaw + 180.0, 360.0) - 180.0;  // 标准化测量角度
    
    // 更新滤波器状态
    cv::Mat estimated = kf_.correct(measurement_);  // 更新状态估计
    state_ = estimated.clone();  // 更新后验状态
}

// 获取当前状态
cv::Point3f Kalman::getPosition() const 
{
    return cv::Point3f(
        static_cast<float>(state_.at<double>(0)),  // x
        static_cast<float>(state_.at<double>(1)),  // y
        static_cast<float>(state_.at<double>(2))   // z
    );
}

cv::Point3f Kalman::getVelocity() const 
{
    return cv::Point3f(
        static_cast<float>(state_.at<double>(3)),  // vx
        static_cast<float>(state_.at<double>(4)),  // vy
        static_cast<float>(state_.at<double>(5))   // vz
    );
}

double Kalman::getYaw() const 
{
    return state_.at<double>(6);  // yaw
}

double Kalman::getYawRate() const 
{
    return state_.at<double>(7);  // yaw_rate
}
