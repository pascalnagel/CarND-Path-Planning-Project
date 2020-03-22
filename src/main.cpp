#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> wps, wpx, wpy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
    wps.push_back(s);
    wpx.push_back(x);
    wpy.push_back(y);
  }
  wps.push_back(max_s);
  wpx.push_back(map_waypoints_x[0]);
  wpy.push_back(map_waypoints_y[0]);

  tk::spline x_spl, y_spl;
  x_spl.set_points(wps, wpx);
  y_spl.set_points(wps, wpy);
 
  double prev_end_path_s = 0;
  double prev_end_path_d = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &prev_end_path_s, &prev_end_path_d,
               &x_spl, &y_spl, max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          // Comment: For some reason these don't give me the s and d coordinates, 
          //          which I passed to the simulator last. Workaround with global
          //          vars prev_end_path{s,d}.
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<vehicle> vehicles;
          for (int i=0; i<sensor_fusion.size(); i++){
            vehicle v;
            v.id = i;
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            v.v = sqrt(vx*vx + vy*vy);
            v.s = sensor_fusion[i][5];
            v.d = sensor_fusion[i][6];
            vehicles.push_back(v);
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          int path_size = previous_path_x.size();
          for (int i=0; i<path_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double last_x, last_y, nlast_x, nlast_y, last_angle, last_speed;
          if (path_size < 2){
            last_x = car_x;
            last_y = car_y;
            last_angle = deg2rad(car_yaw);
            last_speed = car_speed;
            prev_end_path_s = car_s;
            prev_end_path_d = car_d;
          } else {
            last_x = previous_path_x[path_size-1];
            last_y = previous_path_y[path_size-1];
            nlast_x = previous_path_x[path_size-2];
            nlast_y = previous_path_y[path_size-2];
            last_angle = atan2(last_y-nlast_y,last_x-nlast_x);
            last_speed = distance(last_x, last_y, nlast_x, nlast_y) / 0.02;
          }

          if (path_size < 30){
            int next_waypoint_ind = NextWaypoint(last_x, last_y, rad2deg(last_angle), map_waypoints_x, map_waypoints_y);
            std::cout << "Current car s: " << car_s << std::endl;
            std::cout << "Current waypoint id: " << next_waypoint_ind << std::endl;

            double begin_s = prev_end_path_s;
            double begin_d = prev_end_path_d;
            double begin_speed = last_speed;

            int current_lane = round((begin_d - 2.0)/4.0);
            // Possible lanes are {0,1,2} = {left, center, right}
            int tar_lane;
            double tar_speed, tar_d;
            
            // Start: Behavior planner
            vector<string> states = {"KL", "LCL", "LCR", "DLCL", "DLCR"};
            vector<double> speed_states = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19}; // [m/s]
            vector<string> possible_next_states = get_possible_next_states(current_lane);

            string best_state;
            double best_speed;
            vector<vector<double>> best_trajectory;
            double cost = 9999;
            for (int i=0; i<possible_next_states.size(); i++){
              for (int s=0; s<speed_states.size(); s++){
                string state = possible_next_states[i];           
                if (state == "KL"){
                  tar_lane = current_lane;
                } else if (state == "LCL"){
                  tar_lane = current_lane - 1;
                } else if (state == "LCR"){
                  tar_lane = current_lane + 1;
                } else if (state == "DLCL"){
                  tar_lane = current_lane - 2;
                } else if (state == "DLCR"){
                  tar_lane = current_lane + 2;
                }
                tar_speed = speed_states[s];
                tar_d = 2.0 + 4.0*tar_lane-0.2;

                // travel time of the generated trajectory in seconds
                double T;
                if (tar_lane == current_lane){
                  // longer maneuver time for larger speed differences, at least 0.5 sec
                  T = std::max(fabs(tar_speed - begin_speed)/5.5, 0.5);
                } else if ((tar_lane == current_lane + 1) || (tar_lane == current_lane - 1)){
                  // minimum of 2.5 sec for a single lane change maneuver
                  T = std::max(fabs(tar_speed - begin_speed)/5.5, 2.5);
                } else {
                  // minimum of 4 sec for a double lane change maneuver
                  T = std::max(fabs(tar_speed - begin_speed)/5.5, 4.0);
                }
                double tar_s = begin_s + T*(tar_speed + begin_speed)/2.0;
                
                // Generate trajectory
                vector<vector<double>> trajectory = generate_trajectory(begin_s, begin_d, begin_speed, tar_s, tar_d, 
                                                              tar_speed, map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                                              x_spl, y_spl, T);
                
                // Calculate cost
                // Weights: {efficiency, inlane safety, lane change safety, lane switch penalty, double lane change safety}
                vector<double> weights = {1, 2, 100, 0.01, 100};
                double state_cost = eval_cost(trajectory, vehicles, weights, T);
                std::cout << "Cost of state " << state << " with speed " << tar_speed << "m/s: " << state_cost << std::endl;
                if (state_cost < cost){
                  cost = state_cost;
                  best_state = state;
                  best_speed = tar_speed;
                  prev_end_path_s = fmod(tar_s, max_s);
                  prev_end_path_d = tar_d;
                  best_trajectory = trajectory;
                }
              }
            }
            std::cout << "Planner output state: " << best_state << std::endl;
            std::cout << "Planner output speed: " << best_speed << "m/s" << std::endl;
            std::cout << "------------------------------" << std::endl;
            // End: Behavior planner

            // Append best trajectory to the path
            for (int i=0; i<best_trajectory[0].size(); ++i){
              next_x_vals.push_back(best_trajectory[0][i]);
              next_y_vals.push_back(best_trajectory[1][i]);
            }
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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