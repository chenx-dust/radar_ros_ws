
#include "KalmanFilter.h"
#include <Eigen/Dense>

using namespace pc_detector;

// 已知一辆小车的乐观估计 最高速度 5m/s 最高加速度 20m/s^2
// 点云速度 452,000p/s
// 协方差矩阵的迹的平方根是标准差
// 误差的 65% 在 1 标准差内
// 由此可以得出 Q R 的取值

// 状态变量
// {x, y, z, vx, vy, vz}

double Kf_speed_limit, cov_factor;

size_t stop_p_time;

Eigen::Matrix4d Kf_A, Kf_Q, Kf_R;

void KalmanFilter::read_params(
    double q_pos, double q_vel, double q_pv,
    double r_pos, double r_vel,
    double decay_rate,
    double speed_limit,
    double cov_factor_,
    size_t stop_p_time_)
{
    Kf_Q = Eigen::Matrix4d {
        { q_pos, 0, q_pv, 0 },
        { 0, q_pos, 0, q_pv },
        { q_pv, 0, q_vel, 0 },
        { 0, q_pv, 0, q_vel },
    };
    Kf_R = Eigen::Matrix4d {
        { r_pos, 0, 0, 0 },
        { 0, r_pos, 0, 0 },
        { 0, 0, r_vel, 0 },
        { 0, 0, 0, r_vel },
    };
    Kf_A = Eigen::Matrix4d {
        { 1, 0, 1, 0 },
        { 0, 1, 0, 1 },
        { 0, 0, 1 - decay_rate, 0 },
        { 0, 0, 0, 1 - decay_rate },
    };
    Kf_speed_limit = speed_limit;
    cov_factor = cov_factor_;
    stop_p_time = stop_p_time_;
}

KalmanFilter::KalmanFilter(const Eigen::Vector2d& X0, size_t p_times)
{
    X << X0[0], X0[1], 0, 0;
    P = Kf_Q * p_times;
    X_ = X;
    P_ = P;
    lost_time = 0;
}

void KalmanFilter::predict() {
    X_ = Kf_A * X;
    X_(Eigen::seq(2, 3)) = X_(Eigen::seq(2, 3)).cwiseMax(-Kf_speed_limit).cwiseMin(Kf_speed_limit);
    if (lost_time > stop_p_time)
        return;
    Eigen::Matrix4d real_Q = Kf_Q;
    real_Q(Eigen::seq(0, 1), Eigen::seq(0, 1)) += X_(Eigen::seq(2, 3)) * X_(Eigen::seq(2, 3)).transpose() * cov_factor;
    P_ = Kf_A * P * Kf_A.transpose() + real_Q;
}

void KalmanFilter::update() {
    predict();
    X = X_;
    P = P_;
    lost_time++;
}

void KalmanFilter::update(const Eigen::Vector2d &Z) {
    predict();
    // 利用坐标作差得到观测速度
    Eigen::Vector4d Z_v;
    Z_v << Z, (Z - X(Eigen::seq(0, 1))) / (double)(lost_time + 1);
    // 计算卡尔曼增益
    Eigen::Matrix4d K = P_ * Eigen::Inverse(P_ + Kf_R);
    // 更新状态变量
    Eigen::Vector4d d = (Z_v - X_);
    X = X_ + K * d;
    // 更新协方差矩阵
    // P = (Eigen::Matrix4d::Identity() - K) * P_ + d * d.transpose() * cov_factor / (lost_time + 1);
    P = (Eigen::Matrix4d::Identity() - K) * P_;
    // P(Eigen::seq(0, 2), Eigen::seq(0, 2)) += X(Eigen::seq(3, 5)) * X(Eigen::seq(3, 5)).transpose() * cov_factor;
    lost_time = 0;
}

