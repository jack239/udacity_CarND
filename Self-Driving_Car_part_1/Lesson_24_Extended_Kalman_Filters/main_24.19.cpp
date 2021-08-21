#include <iostream>
#include <vector>
#include <Eigen/Dense>

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

int main() {
    /**
     * Compute the Jacobian Matrix
     */

    // predicted state example
    // px = 1, py = 2, vx = 0.2, vy = 0.4
    Eigen::VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    Eigen::MatrixXd Hj = CalculateJacobian(x_predicted);

    std::cout << "Hj:" << std::endl << Hj << std::endl;

    return 0;
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


    // TODO: YOUR CODE HERE 

    // check division by zero
  
    // compute the Jacobian matrix

    return Hj;
}