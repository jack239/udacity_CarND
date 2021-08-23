#include "behavior_planer.h"
#include "helpers.h"
#include <iostream>

namespace {

static constexpr double laneWidth = 4;
constexpr double max_speed = 49 * MPH2MS;

double get_lane_d(int lane_id) {
    return laneWidth * (lane_id + 0.5);
}

void set_lane(
    VehicleState& state,
    int lane_id
    ) {
    state.target_d = get_lane_d(lane_id);
    state.target_lane = lane_id;
}

void start_following(VehicleState& state) {
    state.state = State::following;
    std::cerr << "start following " << std::endl;
}

void keep_lane(VehicleState& state, int lane_id) {
    std::cerr << "keep lane " << std::endl;
    state.state = State::keep_lane;
    state.current_lane = lane_id;
    set_lane(state, lane_id);
}

void change_lane(VehicleState& state, int lane_id) {
    std::cerr << "change lane " << lane_id << std::endl;
    state.state = State::change_lane;
    set_lane(state, lane_id);
}

std::vector<int> get_objects_in_lane(
    const std::vector<VehiclePos>& objects,
    double d,
    double min_s,
    double max_s) {
    constexpr double d_range = 0.5 * laneWidth;

    std::vector<int> result;
    for (size_t i = 0; i < objects.size(); ++i) {
        const auto& frenet = objects.at(i).frenet;
        if (
            (std::abs(frenet.y() - d) < d_range) &&
            (frenet.x() > min_s) &&
            (frenet.x() < max_s)
            ) {
            result.emplace_back(i);
        }
    }
    return result;
}

int get_object_for_following(
    const std::vector<VehiclePos>& objects,
    const Eigen::Vector2d& ego_pos,
    double max_s
    ) {
    double ego_s = ego_pos.x();
    auto candidates = get_objects_in_lane(objects, ego_pos.y(), ego_s, ego_s + max_s);
    if (candidates.empty()) {
        return -1;
    }
    auto cmp_s = [&objects](int i, int j) {
        return objects.at(i).frenet.x() < objects.at(j).frenet.x();
    };
    return *std::min_element(candidates.begin(), candidates.end(), cmp_s);
}

bool lane_is_free(int lane_id, const std::vector<VehiclePos>& objects, double ego_s) {
    if (lane_id < 0 || lane_id > 2) {
        return false;
    }
    constexpr double backward_offset = -20;
    constexpr double forward_offset = 50;
    double lane_d = get_lane_d(lane_id);
    auto objects_in_lane = get_objects_in_lane(objects, lane_d, ego_s + backward_offset, ego_s + forward_offset);
    return objects_in_lane.empty();
}

int check_lanes(
    VehicleState& state,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
    ) {
    for (int lane_id : {state.current_lane - 1, state.current_lane + 1 }) {
        if (lane_is_free(lane_id, objects, ego.frenet.x())) {
            return lane_id;
        }
    }
    return -1;
}

double max_speed_in_lane(
    int lane_id,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
    )  {
    constexpr double backward_offset = 0;
    constexpr double forward_offset = 50;
    double lane_d = get_lane_d(lane_id);
    double ego_s = ego.frenet.x();
    auto objects_in_lane = get_objects_in_lane(objects, lane_d, ego_s + backward_offset, ego_s + forward_offset);
    double result = max_speed;
    for (int object_id : objects_in_lane) {
        result = std::min(result, objects.at(object_id).speed);
    }
    return result;
}

double get_target_speed(
    const VehicleState& state,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
    ) {
    if (state.state == State::change_lane) {
        return std::min(
            max_speed_in_lane(state.current_lane, ego, objects),
            max_speed_in_lane(state.target_lane, ego, objects)
            );
    }
    if (state.state == State::following) {
        return max_speed_in_lane(state.current_lane, ego, objects);
    }
    return max_speed;
}

void update_state_impl(
    VehicleState& state,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
) {
    constexpr double start_s_following = 20;
    constexpr double stop_s_following = 40;

    auto get_target_object = [&objects, &ego](double min_s) -> int{
        return get_object_for_following(objects, ego.frenet, min_s);
    };

    if (state.state == State::not_init) {
        keep_lane(state, 1);
    } else if (state.state == State::keep_lane) {
        int object_id = get_target_object(start_s_following);
        if (object_id >= 0) {
            start_following(state);
        }
    } else if (state.state == State::following) {
        int target_lane = check_lanes(state, ego, objects);
        if (target_lane >= 0) {
            change_lane(state, target_lane);
            return;
        }

        int object_id = get_target_object(stop_s_following);
        if (object_id < 0) {
            keep_lane(state, state.current_lane);
        }
    } else if (state.state == State::change_lane) {
        if (std::abs(ego.frenet.y() - state.target_d) < laneWidth * 0.5) {
            keep_lane(state, state.target_lane);
        }
    }
}
}

void update_state(
    VehicleState& state,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
) {
    update_state_impl(state, ego, objects);
    state.target_speed = get_target_speed(state, ego, objects);
}