#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

class Vehicle;

using Trajectory = std::vector<Vehicle>;
using Prediction = std::map<int, Trajectory>;

class Vehicle {
public:
    enum class State : int {
        CS = 0,
        KL,
        PLCL,
        PLCR,
        LCL,
        LCR,
    };
public:
    // Constructors
    Vehicle();
    Vehicle(int lane, float s, float v, float a, State state=State::CS);

    // Destructor
    virtual ~Vehicle();

    // Vehicle functions
    Trajectory choose_next_state(Prediction& predictions);

    std::vector<State> successor_states();

    Trajectory generate_trajectory(
        State state,
        Prediction& predictions);

    std::vector<float> get_kinematics(Prediction& predictions, int lane);

    Trajectory constant_speed_trajectory();

    Trajectory keep_lane_trajectory(Prediction& predictions);

    Trajectory lane_change_trajectory(
        State state,
        Prediction& predictions);

    Trajectory prep_lane_change_trajectory(
        State state,
        Prediction& predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_vehicle_behind(
        Prediction& predictions,
        int lane,
        Vehicle& rVehicle);

    bool get_vehicle_ahead(
        Prediction& predictions,
        int lane,
        Vehicle& rVehicle);

    Trajectory generate_predictions(int horizon=2);

    void realize_next_state(Trajectory& trajectory);

    void configure(std::vector<int> &road_data);

    // public Vehicle variables
    struct collider{
        bool collision; // is there a collision?
        int  time; // time collision happens
    };

    std::map<State, int> lane_direction = {
        {State::PLCL, 1},
        {State::LCL, 1},
        {State::LCR, -1},
        {State::PLCR, -1}
    };

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane, s, goal_lane, goal_s, lanes_available;

    float v, target_speed, a, max_acceleration;

    State state;
};


#endif  // VEHICLE_H