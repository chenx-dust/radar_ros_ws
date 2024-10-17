
#pragma once

#include <Eigen/Core>


namespace pc_detector {
class KalmanFilter {
// protected:
public:
    Eigen::Vector4d X, X_; // X 状态变量, X_ 先验状态变量
    Eigen::Matrix4d P, P_; // P 协方差矩阵, P_ 先验协方差矩阵
    size_t lost_time;
    void predict();

public:
    static void read_params(
        double q_pos, double q_vel, double q_pv,
        double r_pos, double r_vel,
        double decay_rate, double max_velocity, double cov_factor_,
        size_t stop_p_time_);
    /// X0 初始位置
    KalmanFilter(const Eigen::Vector2d& X0, size_t p_times=0);
    /// 无观测, 只预测
    void update();
    /// Z 观测位置
    void update(const Eigen::Vector2d& Z);

    inline Eigen::Vector2d pos() const {
        /// @brief 返回位置
        return X(Eigen::seq(0, 1));
    }
    inline Eigen::Vector2d velocity_rel() const {
        /// @brief 返回相对速度
        return X(Eigen::seq(2, 3));
    }
    inline double possibility(Eigen::Vector2d pt) const {
        /// @brief 计算概率密度
        Eigen::Matrix2d cov = P(Eigen::seq(0, 2), Eigen::seq(0, 2));
        Eigen::Vector2d d = pt - pos();
        return 1 / sqrt(pow(2 * M_PI, 2 /*维*/) * cov.determinant()) * exp(-0.5 * d.transpose() * cov.inverse() * d);
    }
    inline double mah_dis(Eigen::Vector2d pt) const {
        /// @brief 计算马氏距离 (xy平面)
        // Eigen::Matrix3d cov = P(Eigen::seq(0, 2), Eigen::seq(0, 2));
        // Eigen::Vector3d d = pt - pos();
        // return d.transpose() * cov.inverse() * d;
        Eigen::Matrix2d cov = P(Eigen::seq(0, 1), Eigen::seq(0, 1));
        Eigen::Vector2d d = pt - pos();
        return d.transpose() * cov.inverse() * d;
    }
};
}
