#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include "state.h"
#include "helpers.h"

struct TargetPoints {
    Trajectory start;
    Eigen::Vector2d target;
};

TargetPoints create_trajectory(
    const std::vector<Eigen::Vector2d>& map_waypoints,
    const std::vector<double>& map_waypoints_s,
    const VehiclePos& pos,
    const VehicleState& vehicle_state,
    const Trajectory& previous_path
);

Trajectory calculate_velocity(
    const TargetPoints& target_points,
    const VehiclePos& ego,
    const VehicleState&  vehicle_state,
    size_t points_num
);

#endif //TRAJECTORY_H_
