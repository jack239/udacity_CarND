#ifndef COST_H
#define COST_H

#include "vehicle.h"

using HelperData = std::map<std::string, float>;

float calculate_cost(
    const Vehicle& vehicle,
    const Prediction& predictions,
    const Trajectory& trajectory);

float goal_distance_cost(
    const Vehicle& vehicle,
    const Trajectory& trajectory,
    const Prediction& predictions,
    HelperData& data);

float inefficiency_cost(
    const Vehicle& vehicle,
    const Trajectory& trajectory,
    const Prediction& predictions,
    HelperData& data);

float lane_speed(const Prediction& predictions, int lane);

HelperData get_helper_data(
    const Vehicle& vehicle,
    const Trajectory& trajectory,
    const Prediction& predictions
    );

#endif  // COST_H