#include <iostream>
#include <Eigen/Core>

Eigen::MatrixXd createTransform(double x, double y, double theta) {
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    Eigen::MatrixXd transform(2, 3);
    transform <<
        cos, -sin, x,
        sin, cos, y;
    return transform;
}

int getClosest(const Eigen::Vector2d& obs, const std::vector<Eigen::Vector2d>& landmarks) {
    std::vector<double> distances(landmarks.size());
    std::transform(
        landmarks.begin(),
        landmarks.end(),
        distances.begin(),
        [&obs](const Eigen::Vector2d& lm) {
            return (lm - obs).squaredNorm();
        }
    );
    return std::min_element(distances.begin(), distances.end()) - distances.begin();
}

double getDensity(const Eigen::Vector2d& p, const Eigen::Vector2d& mu, double sigma_x, double sigma_y) {
    Eigen::Vector2d diff = p - mu;
    diff.x() /= sigma_x;
    diff.y() /= sigma_y;
    double pow = - (diff.squaredNorm())/ 2;
    return std::exp(pow) / (2 * M_PI * sigma_x * sigma_y);
}

int main() {
    std::vector<Eigen::Vector2d> landmarks {
        {5, 3},
        {2, 1},
        {6, 1},
        {7, 4},
        {4, 7}
    };
    std::vector<Eigen::Vector2d> observations {
        {2, 2},
        {3, -2},
        {0, -4}
    };

    auto transform = createTransform(4, 5, -M_PI_2);
    auto toLocal = [&transform](double x, double y) {
        Eigen::VectorXd p(3);
        p << x, y, 1;
        Eigen::VectorXd res = transform * p;
        return res;
    };

    double totalP = 1;
    for (const auto obs: observations) {
        auto loc = toLocal(obs.x(), obs.y());
        int closest = getClosest(loc, landmarks);
        double dens = getDensity(loc, landmarks.at(closest), 0.3, 0.3);
        totalP *= dens;

        std::cout << "obs " << obs.x() << " " << obs.y() << std::endl;
        std::cout << "\t local " << loc.x() << " " << loc.y() << std::endl;
        std::cout << "\t closest " << closest << std::endl;
        std::cout << "\t prob " << dens << std::endl;
    }
    std::cout << "total " << totalP << std::endl;

    return 0;
}
