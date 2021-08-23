#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

//default parameters was taken from observe.
// it was 0.5, 0, 10
//

using Params = std::array<double, 3>;
static constexpr Params init_params{0.25, 0.005, 4};
static constexpr Params init_delta{1e-2, 1e-4, 1e-1};
// static constexpr Params init_params{0.61, 0.041, 11};
// static constexpr Params init_delta{1e-2, 1e-3, 1e-1};
// static constexpr Params init_params{0.25, 1e-3, 10};
// static constexpr Params init_delta{1e-2, 1e-4, 1e-1};
// static constexpr Params init_delta{0.1, 0.1, 0.1};
// static constexpr Params init_delta{0.1, 0, 0.1};

class PIDTRainer {
private:
    static constexpr int update_count = 1000;
    enum class State {
        Init,
        Inc,
        Dec
    };
public:
    PIDTRainer(PID& pid_)
    : pid(pid_),
    state(State::Init),
    best_params{init_params},
    delta{init_delta}
    {
        current_params = best_params;
        set_params();
    }
public:
    void update_cte(double cte) {
        sum_cte += cte * cte;
        count_cte += 1;
        if (count_cte == update_count){
            update_state();
        }
    }
private:
    void set_params() {
        //        std::cout << "set_params " << nlohmann::json(current_params) <<  std::endl;
        pid.Init(current_params[0], current_params[1], current_params[2]);
        sum_cte = 0;
        count_cte = 0;
    }

    void update_state() {
        //        std::cout << "update_state current "<< sum_cte << "best " << best_sum_cte << std::endl;
        if (state == State::Init) {
            param_id = 0;
            best_sum_cte = sum_cte;
            try_params(State::Inc);
        } else {
            if (best_sum_cte > sum_cte) {
                std::cout << "new best cte "<< (sum_cte / count_cte) << " params " << nlohmann::json(current_params) <<  std::endl;
                delta[param_id] *= 1.1;
                best_params = current_params;
                best_sum_cte = sum_cte;
                try_params(State::Inc);
            } else {
                if (state == State::Inc) {
                    try_params(State::Dec);
                } else {
                    delta[param_id] *= 0.9;
                    //                     delta[param_id] *= 0.1;
                    try_params(State::Inc);
                }
            }
        }
    }

    void try_params(State new_state) {
        current_params = best_params;
        state = new_state;
        if (state == State::Inc) {
            param_id = (param_id + 1) % 3;
            current_params[param_id] += delta[param_id];
        }
        set_params();
    }

private:
    PID& pid;
    State state;
    size_t param_id;
    Params best_params;
    Params current_params;
    Params delta;
    double sum_cte;
    double best_sum_cte;
    int count_cte;
};

int main() {
    uWS::Hub h;

    PID pid;
    PIDTRainer trainer(pid);
    /**
     * TODO: Initialize the pid variable.
     */

    h.onMessage([&pid, &trainer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
        uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    trainer.update_cte(cte);
                    //          double speed = std::stod(j[1]["speed"].get<string>());
                    //          double angle = std::stod(j[1]["steering_angle"].get<string>());
                    pid.UpdateError(cte);
                    double steer_value = pid.TotalError();

                    // DEBUG
                    //          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    //                    << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //           std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}