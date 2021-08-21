#include "cost.h"

double inefficiency_cost(
    int target_speed,
    int intended_lane,
    int final_lane,
    const std::vector<int>& lane_speeds
    ) {
    double speed_intended = lane_speeds[intended_lane];
    double speed_final = lane_speeds[final_lane];
    double cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;
    return cost;
}