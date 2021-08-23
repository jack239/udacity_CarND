#include "trajectory.h"
#include "spline.h"

namespace {

constexpr double time = 0.2; // in seconds

double get_current_speed(
    double start_speed,
    double target_speed) {
    const double max_acc = 5 * time * time;
    if (start_speed < max_acc) {
        return max_acc;
    }
    if (start_speed < target_speed) {
        return std::min(start_speed + max_acc, target_speed);
    }
    if (start_speed > target_speed) {
        return std::max(start_speed - max_acc, target_speed);
    }
    return target_speed;
}

std::vector<double> createSampling(
    double start_sample,
    double target_speed,
    double init_sample,
    int points_num
) {
    std::vector<double> samplings;
    samplings.reserve(points_num);
    double start_speed = start_sample / time * 10;

    double speed = get_current_speed(start_speed, target_speed);

    double sampling = time * speed * 0.1;

    while (samplings.size() < points_num) {
        samplings.emplace_back(init_sample + samplings.size() * sampling);
    }
    return samplings;
}

Trajectory convertToEgoLocal(Trajectory trajectory, const VehiclePos& ego) {
    auto ego_dir = from_angle(ego.yaw);
    Eigen::MatrixXd rotation(2, 2);
    rotation <<
    ego_dir[0], ego_dir[1],
    -ego_dir[1], ego_dir[0];
    for (auto& point : trajectory) {
        point = rotation * (point - ego.pos);
    }
    return trajectory;
}

Trajectory convertFromEgoLocal(Trajectory trajectory, const VehiclePos& ego) {
    auto ego_dir = from_angle(ego.yaw);
    Eigen::MatrixXd rotation(2, 2);
    rotation <<
    ego_dir[0], -ego_dir[1],
    ego_dir[1], ego_dir[0];
    for (auto& point : trajectory) {
        point = rotation * point + ego.pos;
    }
    return trajectory;
}

}

double get_target_s(const VehicleState& vehicle_state) {
    constexpr double follow_target_s = 50;
    constexpr double default_target_s = 20;
    if (vehicle_state.state == State::change_lane) {
        return follow_target_s;
    }
    return default_target_s;
}

TargetPoints create_trajectory(
    const std::vector<Eigen::Vector2d>& map_waypoints,
    const std::vector<double>& map_waypoints_s,
    const VehiclePos& pos,
    const VehicleState& vehicle_state,
    const Trajectory& previous_path
) {
    double target_s = get_target_s(vehicle_state);
    Trajectory startPoints;
    Eigen::Vector2d targetf{pos.frenet.x() + target_s, vehicle_state.target_d};
    Eigen::Vector2d target = getXY(targetf, map_waypoints_s, map_waypoints);
    size_t start_size = 3;
    if (previous_path.empty()) {
        auto ego_dir = from_angle(pos.yaw);
        constexpr double start_sampling = 0.1;
        for (size_t i = 0; i < start_size; ++i) {
            double offset = start_sampling * i;
            startPoints.emplace_back(pos.pos + offset * ego_dir);
        }
    } else {
        int start = NextWaypoint(pos.pos, pos.yaw, previous_path);
        startPoints.assign(previous_path.begin() + start, previous_path.begin() + start + start_size);
    }
    return {startPoints, target};
}

Trajectory calculate_velocity(
    const TargetPoints& target_points,
    const VehiclePos& ego,
    const VehicleState& vehicle_state,
    size_t points_num
)  {
    Trajectory result;
    result.emplace_back(target_points.start.front());
    double start_sample = (target_points.start.at(0) - target_points.start.at(1)).norm();

    std::vector<Eigen::Vector2d> spline_points{target_points.start.begin() + 1, target_points.start.end()};
    spline_points.emplace_back(target_points.target);

    const Trajectory localTargetPoints = convertToEgoLocal(spline_points, ego);

    tk::spline s(
        extract(localTargetPoints, extract_x),
        extract(localTargetPoints, extract_y));

    Trajectory localResult; // new part in local coordinate
    result.reserve(points_num);
    double init_sample = localTargetPoints.front().x();
    auto sampling = createSampling(start_sample, vehicle_state.target_speed, init_sample, points_num);
    for (double sample : sampling) {
        localResult.emplace_back(sample, s(sample));
    }

    auto new_spline = convertFromEgoLocal(localResult, ego); // new part in global
    assert((new_spline.front() - target_points.start.at(1)).norm() < 1e-1);

    result.insert(result.end(), new_spline.begin(), new_spline.end()); // concatinate
    return result;
}