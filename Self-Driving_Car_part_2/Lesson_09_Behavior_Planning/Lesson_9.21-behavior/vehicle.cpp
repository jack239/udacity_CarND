#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, State state) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
}

Vehicle::~Vehicle() {}

Trajectory Vehicle::choose_next_state(Prediction& predictions) {
    /**
     * Here you can implement the transition_function code from the Behavior
     *   Planning Pseudocode classroom concept.
     *
     * @param A predictions map. This is a map of vehicle id keys with predicted
     *   vehicle trajectories as values. Trajectories are a vector of Vehicle
     *   objects representing the vehicle at the current timestep and one timestep
     *   in the future.
     * @output The best (lowest cost) trajectory corresponding to the next ego
     *   vehicle state.
     *
     * Functions that will be useful:
     * 1. successor_states - Uses the current state to return a vector of possible
     *    successor states for the finite state machine.
     * 2. generate_trajectory - Returns a vector of Vehicle objects representing
     *    a vehicle trajectory, given a state and predictions. Note that
     *    trajectory vectors might have size 0 if no possible trajectory exists
     *    for the state.
     * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
     *
     * TODO: Your solution here.
     */
    auto states = successor_states();

    std::vector<Trajectory> trajectories;
    std::vector<float> costs;
    for (auto st : states) {
        trajectories.emplace_back(generate_trajectory(st, predictions));
        auto cost = calculate_cost(*this, predictions, trajectories.back());
        costs.emplace_back(cost);
    }

    auto minIt = std::min_element(costs.begin(), costs.end());
    return trajectories.at(minIt - costs.begin());
}

std::vector<Vehicle::State> Vehicle::successor_states() {
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    std::vector<State> states;
    states.push_back(State::KL);
    State state = this->state;
    if(state == State::KL) {
        states.push_back(State::PLCL);
        states.push_back(State::PLCR);
    } else if (state == State::PLCL) {
        if (lane != lanes_available - 1) {
            states.push_back(State::PLCL);
            states.push_back(State::LCL);
        }
    } else if (state == State::PLCR) {
        if (lane != 0) {
            states.push_back(State::PLCR);
            states.push_back(State::LCR);
        }
    }
    
    // If state is State::LCL or State::LCR, then just return State::KL
    return states;
}

Trajectory Vehicle::generate_trajectory(
    State state,
    Prediction& predictions) {
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    Trajectory trajectory;
    if (state == State::CS) {
        trajectory = constant_speed_trajectory();
    } else if (state == State::KL) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state == State::LCL || state == State::LCR) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state == State::PLCL || state == State::PLCR) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }

    return trajectory;
}

std::vector<float> Vehicle::get_kinematics(
    Prediction& predictions,
    int lane
) {
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = vehicle_ahead.v;
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s
                - this->preferred_buffer) + vehicle_ahead.v
                    - 0.5 * (this->a);
            new_velocity = std::min(std::min(max_velocity_in_front,
                                             max_velocity_accel_limit),
                                    this->target_speed);
        }
    } else {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    
    return{new_position, new_velocity, new_accel};
}

Trajectory Vehicle::constant_speed_trajectory() {
    // Generate a constant speed trajectory.
    float next_pos = position_at(1);
    Trajectory trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state),
                                  Vehicle(this->lane,next_pos,this->v,0,this->state)};
    return trajectory;
}

Trajectory Vehicle::keep_lane_trajectory(Prediction& predictions) {
    // Generate a keep lane trajectory.
    Trajectory trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    std::vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, State::KL));
  
    return trajectory;
}

Trajectory Vehicle::prep_lane_change_trajectory(
    State state,
    Prediction& predictions) {
    // Generate a trajectory preparing for a lane change.
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    Trajectory trajectory = {Vehicle(this->lane, this->s, this->v, this->a,
                                  this->state)};
    std::vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    } else {
        std::vector<float> best_kinematics;
        std::vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  
    return trajectory;
}

Trajectory Vehicle::lane_change_trajectory(
    State state,
    Prediction& predictions) {
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    Trajectory trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (auto it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            // If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a,
                                 this->state));
    std::vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
    this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(
    Prediction& predictions,
    int lane,
    Vehicle& rVehicle) {
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (auto it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s
        && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
  
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(
    Prediction& predictions,
    int lane,
    Vehicle& rVehicle
) {
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (auto it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s
        && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
  
    return found_vehicle;
}

Trajectory Vehicle::generate_predictions(int horizon) {
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    Trajectory predictions;
    for(int i = 0; i < horizon; ++i) {
        float next_s = position_at(i);
        float next_v = 0;
        if (i < horizon-1) {
            next_v = position_at(i+1) - s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
  
    return predictions;
}

void Vehicle::realize_next_state(Trajectory& trajectory) {
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(std::vector<int>& road_data) {
    // Called by simulator before simulation begins. Sets various parameters which
    //   will impact the ego vehicle.
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}