#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
    const Eigen::VectorXd &x_in,
    const Eigen::MatrixXd &P_in,
    const Eigen::MatrixXd &F_in,
    const Eigen::MatrixXd &H_in,
    const Eigen::MatrixXd &R_laser_in,
    const Eigen::MatrixXd &R_radar_in,
    const Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_radar_ = R_radar_in;
  R_laser_ = R_laser_in;
  Q_ = Q_in;
}

void KalmanFilter::UpdateMatrices(double dt) {
    constexpr double noise_ax = 9;
    constexpr double noise_ay = 9;
    F_(0, 2) = F_(1, 3) = dt;
    // 2. Set the process covariance matrix Q
    Q_ = Eigen::MatrixXd(4, 4);
    Q_.setZero();
    assert(Q_.rows() == 4);
    assert(Q_.cols() == 4);
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    Q_(0, 0) = dt4 * noise_ax / 4;
    Q_(1, 1) = dt4 * noise_ay / 4;
    Q_(0, 2) = Q_(2, 0) = dt3 * noise_ax / 2;
    Q_(1, 3) = Q_(3, 1) = dt3 * noise_ay / 2;
    Q_(2, 2) = dt2 * noise_ax;
    Q_(3, 3) = dt2 * noise_ay;
}

void KalmanFilter::Predict(double dt) {
    UpdateMatrices(dt);
    x_ = F_ * x_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateImpl(const Eigen::VectorXd &y, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P_ * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
    Eigen::VectorXd y = z - H_ * x_;
    UpdateImpl(y, H_, R_laser_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    auto Hj = Tools::CalculateJacobian(x_);
    auto polar = Tools::CartesianToPolar(x_);
    Eigen::VectorXd y = z - polar;
    y[1] = Tools::normalizeAngle(y[1]);
    UpdateImpl(y, Hj, R_radar_);
}
