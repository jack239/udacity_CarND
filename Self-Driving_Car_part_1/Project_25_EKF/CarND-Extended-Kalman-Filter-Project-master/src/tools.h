#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

namespace Tools {
  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd> &estimations,
      const std::vector<Eigen::VectorXd> &ground_truth
      );

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  Eigen::Vector2d PolarToCartesian(const Eigen::VectorXd& polar);
  Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& cart);
  double normalizeAngle(double angle);
}

#endif  // TOOLS_H_
