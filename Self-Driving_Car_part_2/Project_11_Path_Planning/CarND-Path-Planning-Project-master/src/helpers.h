#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

// for convenience
using std::string;
using std::vector;

using Trajectory = std::vector<Eigen::Vector2d>;

static constexpr double MPH2MS = 1609. / 3600.;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
inline double distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return (p1 - p2).norm();
}

inline double get_angle(const Eigen::Vector2d& p) {
    return std::atan2(p.y(), p.x());
}

inline Eigen::Vector2d from_angle(double angle) {
    return Eigen::Vector2d{std::cos(angle), std::sin(angle)};
}

int NextWaypoint(
    const Eigen::Vector2d& p,
    double theta,
    const Trajectory& points
    );
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Eigen::Vector2d getFrenet(
    const Eigen::Vector2d& p,
    double theta,
    const std::vector<Eigen::Vector2d>& points
);

// Transform from Frenet s,d coordinates to Cartesian x,y
Eigen::Vector2d getXY(
    const Eigen::Vector2d& frenet,
    const std::vector<double>& maps_s,
    const std::vector<Eigen::Vector2d>& points
    );


inline double extract_x(const Eigen::Vector2d& p) {
    return p.x();
}

inline double extract_y(const Eigen::Vector2d& p) {
    return p.y();
}

std::vector<double> extract(const Trajectory& trajectory, std::function<double(const Eigen::Vector2d&)> transform);

#endif  // HELPERS_H