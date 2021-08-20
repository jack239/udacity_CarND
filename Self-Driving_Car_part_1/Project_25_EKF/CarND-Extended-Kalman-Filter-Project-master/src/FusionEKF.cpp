#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices

  //measurement covariance matrix - laser
  R_laser_.setIdentity(2, 2);
  R_laser_ *= 0.0225;

  //measurement covariance matrix - radar
  R_radar_.setIdentity(3, 3);
  R_radar_ *= 0.09;

  H_laser_.setIdentity(2, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

Eigen::Vector2d createStartPos(const MeasurementPackage& measurement_pack) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        return Tools::PolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        return Eigen::Vector2d {
            measurement_pack.raw_measurements_.x(),
            measurement_pack.raw_measurements_.y()
        };
    } else {
        assert(false);
    }
}

void FusionEKF::InitEkf( const MeasurementPackage& measurement_pack) {
    // first measurement
    cout << "EKF: " << endl;

    VectorXd x = VectorXd(4);
    auto startPos = createStartPos(measurement_pack);
    x << startPos[0], startPos[1], 0, 0;
    MatrixXd P;
    P.setIdentity(4, 4);
    P(2,2) = P(3, 3) = 1000;

    MatrixXd F;
    F.setIdentity(4, 4);

    ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, {});
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
      InitEkf(measurement_pack);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict(dt);

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
