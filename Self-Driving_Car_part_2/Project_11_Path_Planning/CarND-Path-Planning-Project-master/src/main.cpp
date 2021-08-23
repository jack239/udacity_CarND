#include "trajectory.h"
#include "behavior_planer.h"

#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

struct MapData {
    std::vector<Eigen::Vector2d> map_waypoints;
    std::vector<Eigen::Vector2d> map_d_waypoints;
    std::vector<double> map_waypoints_s;
};

// Load up map values for waypoint's x,y,s and d normalized normal vectors
MapData readMap(const std::string& map_file) {
    MapData result;
    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        result.map_waypoints.emplace_back(Eigen::Vector2d{x, y});
        result.map_d_waypoints.emplace_back(Eigen::Vector2d{d_x, d_y});
        result.map_waypoints_s.push_back(s);
    }
    return result;
}

VehiclePos getPos(const nlohmann::json& js) {
    VehiclePos result;
    result.pos = Eigen::Vector2d{js["x"], js["y"]};
    result.frenet = Eigen::Vector2d{js["s"], js["d"]};
    result.yaw = deg2rad(js["yaw"]);
    result.speed = js["speed"];
    result.speed *= MPH2MS;
    result.id = -1;
    return result;
}

std::vector<VehiclePos> getOthers(const nlohmann::json& js) {
    std::vector<VehiclePos> result;
    result.reserve(js.size());
    for (const auto& car_js : js) {
        result.emplace_back();
        result.back().id = car_js[0];
        result.back().pos = Eigen::Vector2d{car_js[1], car_js[2]};
        Eigen::Vector2d velocity{car_js[3], car_js[4]};
        result.back().speed = velocity.norm();
        result.back().yaw = get_angle(velocity);
        result.back().frenet = Eigen::Vector2d{car_js[5], car_js[6]};
    }
    return result;
}

Trajectory getPreviousPath(const nlohmann::json& x, const nlohmann::json& y) {
    Trajectory path;
    size_t path_size = std::min(x.size(), y.size());
    path.reserve(path_size);
    for (size_t i = 0; i < path_size; ++i) {
        path.emplace_back(x[i], y[i]);
    }
    return path;
}


void process_position(
    const MapData& map,
    VehicleState& vehicle_state,
    uWS::WebSocket<uWS::SERVER> ws,
    char* data,
    size_t length,
    uWS::OpCode
    ) {
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(data);

        if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
                // j[1] is the data JSON object

                // Main car's localization Data
                auto ego_pos = getPos(j[1]);

                // Previous path data given to the Planner
                auto previous_path = getPreviousPath(j[1]["previous_path_x"], j[1]["previous_path_y"]);
                // Previous path's end s and d values
//                double end_path_s = j[1]["end_path_s"];
//                double end_path_d = j[1]["end_path_d"];

                // Sensor Fusion Data, a list of all other cars on the same side
                //   of the road.
                auto objects = getOthers(j[1]["sensor_fusion"]);

                json msgJson;

                update_state(vehicle_state, ego_pos, objects);

                auto target_points = create_trajectory(map.map_waypoints, map.map_waypoints_s, ego_pos, vehicle_state, previous_path);
                auto trajectory = calculate_velocity(target_points, ego_pos, vehicle_state, 50);

                msgJson["next_x"] = extract(trajectory, extract_x);
                msgJson["next_y"] = extract(trajectory, extract_y);

                auto msg = "42[\"control\","+ msgJson.dump()+"]";

                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
        } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    }  // end websocket if
}


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
//  double max_s = 6945.554;

  const auto mapData = readMap(map_file_);

  VehicleState vehicle_state;

  h.onMessage([&mapData, &vehicle_state] (
                  uWS::WebSocket<uWS::SERVER> ws,
                  char* data,
                  size_t length,
                  uWS::OpCode opCode
                  ) {
      process_position(mapData, vehicle_state, ws, data, length, opCode);
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> , uWS::HttpRequest) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int ,
                         char *, size_t ) {
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