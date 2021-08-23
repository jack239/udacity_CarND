#ifndef STATE_H_
#define STATE_H_

#include "Eigen-3.3/Eigen/Core"

enum class State {
    not_init,
    keep_lane,
    following,
    change_lane
};

struct VehicleState {
    State state = State::not_init;
    double target_d = 0;
    double target_speed = 0;
    int target_lane = 1;
    int current_lane = 1;
};

struct VehiclePos {
    Eigen::Vector2d pos;
    Eigen::Vector2d frenet;
    double speed;
    double yaw;
    int id;
};

#endif //STATE_H_
