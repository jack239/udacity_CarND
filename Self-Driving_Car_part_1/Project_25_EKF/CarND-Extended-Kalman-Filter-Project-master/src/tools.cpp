#include "tools.h"
#include <iostream>

namespace Tools {
Eigen::VectorXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth
    ) {

  Eigen::VectorXd rmse(4);
  rmse.setZero();

  for (int i=0; i < estimations.size(); ++i) {
      Eigen::VectorXd delta = estimations[i] - ground_truth[i];
      delta = delta.array() * delta.array();
      rmse += delta;
  }
  rmse /= estimations.size();

  return rmse.array().sqrt();
}

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) {
    Eigen::MatrixXd Hj(3,4);
    Hj.setZero();
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float p_norm = std::hypot(px, py);
    float p_norm2 = p_norm * p_norm;
    float p_norm3 = p_norm * p_norm2;
    if (p_norm > 1e-5) {
        Hj(0, 0) = Hj(2, 2) = px / p_norm;
        Hj(0, 1) = Hj(2, 3) = py / p_norm;

        Hj(1, 0) = - py / p_norm2;
        Hj(1, 1) = px / p_norm2;

        float hj3 = (py * vx - px * vy) / p_norm3;
        Hj(2, 0) = py * hj3;
        Hj(2, 1) = -px * hj3;
    }

    return Hj;

}

Eigen::Vector2d PolarToCartesian(const Eigen::VectorXd& polar) {
    return Eigen::Vector2d{
        std::cos(polar[1]),
        std::sin(polar[1])
    } * polar[0];
}

Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& cart) {
    double norm = Eigen::Vector2d{cart[0], cart[1]}.norm();
    Eigen::VectorXd result;
    result = Eigen::VectorXd(3);
    result.setZero(3);
    if (norm > 1e-6) {
        double r3 = (cart[0] * cart[2] + cart[1] * cart[3]) / norm;
        double angle = std::atan2(cart[1], cart[0]);
        result << norm, angle, r3;
    }
    return result;
}

double normalizeAngle(double angle) {
    double result = std::fmod(angle, 2 * M_PI);
    if (result > M_PI) {
        result -= 2 * M_PI;
    } else if (result < -M_PI) {
        result += 2 * M_PI;
    }
    return result;
}

}
