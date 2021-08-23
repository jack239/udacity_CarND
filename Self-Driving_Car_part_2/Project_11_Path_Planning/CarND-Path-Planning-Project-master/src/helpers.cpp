#include "helpers.h"

namespace {
// Calculate closest waypoint to current x, y position
int ClosestWaypoint(const Eigen::Vector2d& p, const Trajectory& points) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < points.size(); ++i) {
        double dist = distance(p, points[i]);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

}

// Returns next waypoint of the closest waypoint
int NextWaypoint(
    const Eigen::Vector2d& p,
    double theta,
    const Trajectory& points
    ) {
    int closestWaypoint = ClosestWaypoint(p, points);

    auto closest = points[closestWaypoint];

    double heading = get_angle(closest - p);

    double angle = fabs(theta-heading);
    angle = std::min(2*pi() - angle, angle);

    if (angle > pi()/2) {
        ++closestWaypoint;
        if (closestWaypoint == points.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

Eigen::Vector2d getFrenet(
    const Eigen::Vector2d& p,
    double theta,
    const std::vector<Eigen::Vector2d>& points
    ) {
    int next_wp = NextWaypoint(p, theta, points);

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = points.size()-1;
    }

    Eigen::Vector2d dir_n = points[next_wp] - points[prev_wp];
    Eigen::Vector2d dir = p - points[prev_wp];

    // find the projection of x onto n
    double proj_norm = (dir.x() * dir_n.x() + dir.y() * dir_n.y()) / dir_n.squaredNorm();
    Eigen::Vector2d proj = proj_norm * dir_n;

    double frenet_d = distance(dir, proj);

    //see if d value is positive or negative by comparing it to a center point
    Eigen::Vector2d center = Eigen::Vector2d{1000, 2000} - points[prev_wp];
    double centerToPos = distance(center, dir);
    double centerToRef = distance(center, proj);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += distance(points[i], points[i + 1]);
    }

    frenet_s += proj.norm();

    return {frenet_s,frenet_d};
}

Eigen::Vector2d getXY(
    const Eigen::Vector2d& frenet,
    const std::vector<double>& maps_s,
    const std::vector<Eigen::Vector2d>& points
    ) {
    double s = frenet.x();
    double d = frenet.y();
    int prev_wp = -1;

    while (s > maps_s[prev_wp+1] && (prev_wp < (int)(points.size()-1))) {
        ++prev_wp;
    }

    int wp2 = (prev_wp+1)%points.size();

    double heading = get_angle((points[wp2] - points[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    Eigen::Vector2d seg = points[prev_wp] + seg_s * from_angle(heading);

    double perp_heading = heading-pi()/2;

    return seg + d * from_angle(perp_heading);
}

std::vector<double> extract(const Trajectory& trajectory, std::function<double(const Eigen::Vector2d&)> transform) {
    std::vector<double> result(trajectory.size());
    std::transform(trajectory.begin(), trajectory.end(), result.begin(), transform);
    return result;
}

