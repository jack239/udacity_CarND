#include "tracking.h"
#include <iostream>
#include <Eigen/Dense>

Tracking::Tracking() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // create a 4D state vector, we don't know yet the values of the x state
    kf_.x_ = Eigen::VectorXd(4);

    // state covariance matrix P
    kf_.P_ = Eigen::MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;


    // measurement covariance
    kf_.R_ = Eigen::MatrixXd(2, 2);
    kf_.R_ << 0.0225, 0,
    0, 0.0225;

    // measurement matrix
    kf_.H_ = Eigen::MatrixXd(2, 4);
    kf_.H_ << 1, 0, 0, 0,
    0, 1, 0, 0;

    // the initial transition matrix F_
    kf_.F_ = Eigen::MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

    // set the acceleration noise components
    noise_ax = 5;
    noise_ay = 5;
}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        //cout << "Kalman Filter Initialization " << std::endl;

        // set the state with the initial location and zero velocity
        kf_.x_ << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1],
        0,
        0;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
  
    // TODO: YOUR CODE HERE
    // 1. Modify the F matrix so that the time is integrated
    Eigen::MatrixXd& F = kf_.F_;
    F(0, 2) = F(1, 3) = dt;
    // 2. Set the process covariance matrix Q
    Eigen::MatrixXd& Q = kf_.Q_;
    Q = Eigen::MatrixXd(4, 4);
    Q.setZero();
    assert(Q.rows() == 4);
    assert(Q.cols() == 4);
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    Q(0, 0) = dt4 * noise_ax / 4;
    Q(1, 1) = dt4 * noise_ay / 4;
    Q(0, 2) = Q(2, 0) = dt3 * noise_ax / 2;
    Q(1, 3) = Q(3, 1) = dt3 * noise_ay / 2;
    Q(2, 2) = dt2 * noise_ax;
    Q(3, 3) = dt2 * noise_ay;
    // 3. Call the Kalman Filter predict() function
    kf_.Predict();
    // 4. Call the Kalman Filter update() function
    kf_.Update(measurement_pack.raw_measurements_);
    //      with the most recent raw measurements_
  
    std::cout << "x_= " << kf_.x_ << std::endl;
    std::cout << "P_= " << kf_.P_ << std::endl;
}