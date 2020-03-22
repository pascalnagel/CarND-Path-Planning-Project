#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include <algorithm>

// for convenience
using std::string;
using std::vector;
using Eigen::Matrix3f;
using Eigen::Vector3f;

// The max s value before wrapping around the track back to 0
double max_s = 6945.554;

vector<double> JMT(vector<double> &start, vector<double> &end, double T);
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, const vector<double> &maps_y,
                     tk::spline x_spl, tk::spline y_spl);


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


struct vehicle {
  int id;
  double s;
  double d;
  double v;
};


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Determine possible lane changes
vector<string> get_possible_next_states(int current_lane){
  vector<string> possible_next_states;
  possible_next_states.push_back("KL");
  if (current_lane == 0){
    possible_next_states.push_back("LCR");
    possible_next_states.push_back("DLCR");
  } else if (current_lane == 2){
    possible_next_states.push_back("LCL");
    possible_next_states.push_back("DLCL");
  } else {
    possible_next_states.push_back("LCR");
    possible_next_states.push_back("LCL");
  }
  return possible_next_states;
}


// Rewards larger average velocities during a maneuver
double efficiency_cost(vector<vector<double>> trajectory, double T){
  double begin_s = trajectory[2][0];
  double end_s = trajectory[2][trajectory[2].size()-1];
  double cost = 1.0 - 1.0 / (1.0 + exp(-(end_s - begin_s)/T/10.0));
  //std::cout << "Efficiency cost: " << cost << std::endl;
  return cost;
}


// Penalizes planning to drive with too little safety distance
double inlane_safety_cost(vector<vector<double>> trajectory, vector<vehicle> vehicles, double T){
  double start_s = trajectory[2][0];
  double end_s = trajectory[2][trajectory[2].size()-1];
  double start_d = trajectory[3][0];
  double end_d = trajectory[3][trajectory[3].size()-1];
  vehicle closest_vehicle_tar_lane;
  double dist_tar_lane = 9999;
  for (int i=0; i<vehicles.size(); ++i){
    // Check if a vehicle is in our target lane
    if ((fabs(vehicles[i].d - end_d) < 1.0)){
      // Check if the vehicle will be in front of us in time T
      if (vehicles[i].s + vehicles[i].v*T > end_s){
        double dist = vehicles[i].s + vehicles[i].v*T - end_s;
        if (dist < dist_tar_lane){
          closest_vehicle_tar_lane = vehicles[i];
          dist_tar_lane = dist;
          //std::cout << "Distance to vehicle in front: " << dist_tar_lane << std::endl;
        }
      }
    }
  }
  double cost = 1.0 - 1.0 / (1.0 + exp(-(dist_tar_lane/10.0)));
  //std::cout << "Inlane safety_cost: " << cost << std::endl;
  return cost;
}


// Penalizes changing lanes, when the target lane is occupied
double lanechange_safety_cost(vector<vector<double>> trajectory, vector<vehicle> vehicles, double T){
  double start_s = trajectory[2][0];
  double end_s = trajectory[2][trajectory[2].size()-1];
  double start_d = trajectory[3][0];
  double end_d = trajectory[3][trajectory[3].size()-1];
  double cost = 0;

  if (fabs(end_d - start_d) > 1.0){ // if lane is changed in the trajectory
    for (int i=0; i<vehicles.size(); ++i){
      //check for vehicles on the target lane
      if (fabs(vehicles[i].d - end_d) < 1.0){
        // Check if a vehicle will be within [-25m, +12m] of our target s-position on the target lane 
        // at the end of the maneuver
        if ((vehicles[i].s + vehicles[i].v*T > end_s - 25.0) && (vehicles[i].s + vehicles[i].v*T < end_s + 12.0)){
          cost = 1;
        }
      }
    }
  }
  //std::cout << "Lane change safety_cost: " << cost << std::endl;
  return cost;
}


// Penalizes double lane changes, when the middle lane is occupied
double doublelanechange_safety_cost(vector<vector<double>> trajectory, vector<vehicle> vehicles, double T){
  double start_s = trajectory[2][0];
  double end_s = trajectory[2][trajectory[2].size()-1];
  double start_d = trajectory[3][0];
  double end_d = trajectory[3][trajectory[3].size()-1];
  double cost = 0;

  if (fabs(end_d - start_d) > 6.0){ // if double lane change is executed in the trajectory
    for (int i=0; i<vehicles.size(); ++i){
      //check for vehicles on the middle lane
      if (fabs(vehicles[i].d - 6.0) < 1.0){
        // Check if a vehicle will be within [-50m, +50m] of our s-position when crossing the middle lane
        double crossing_s = (end_s + start_s)/2.0;
        if ((vehicles[i].s + vehicles[i].v*T > crossing_s - 50.0) && (vehicles[i].s + vehicles[i].v*T < crossing_s + 50.0)){
          cost = 1;
        }
      }
    }
  }
  //std::cout << "Double lane change safety_cost: " << cost << std::endl;
  return cost;
}


// Penalizes unnecessary lane changes
double lanechange_cost(vector<vector<double>> trajectory){
  double start_d = trajectory[3][0];
  double end_d = trajectory[3][trajectory[3].size()-1];
  double cost;
  // if lane is changed in the trajectory
  if (fabs(end_d - start_d) > 1.0){
    cost = 1;
  } else {
    cost = 0;
  }
  //std::cout << "Lane change cost: " << cost << std::endl;
  return cost;
}


double eval_cost(vector<vector<double>> trajectory, vector<vehicle> vehicles, vector<double> weights, double T){
  return weights[0]*efficiency_cost(trajectory, T)
       + weights[1]*inlane_safety_cost(trajectory, vehicles, T)
       + weights[2]*lanechange_safety_cost(trajectory, vehicles, T)
       + weights[3]*lanechange_cost(trajectory)
       + weights[4]*doublelanechange_safety_cost(trajectory, vehicles, T);
}


// Generates a jerk-minimizing trajectory (JMT)
vector<vector<double>> generate_trajectory(double begin_s, double begin_d, double begin_speed, double tar_s, double tar_d, double tar_speed,
                              vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                              tk::spline x_spl, tk::spline y_spl, double T) {
  vector<double> start_s = {begin_s, begin_speed, 0.0};
  vector<double> end_s = {tar_s, tar_speed, 0.0};
  vector<double> start_d = {begin_d, 0.0, 0.0};
  vector<double> end_d = {tar_d, 0.0, 0.0};
  /*
  std::cout << "Generating trajectory..." << std::endl;
  std::cout << "Start s: " << begin_s << std::endl;
  std::cout << "Target s: " << tar_s << std::endl;
  std::cout << "Start d: " << begin_d << std::endl;
  std::cout << "Target d: " << tar_d << std::endl;
  std::cout << "Start speed: " << begin_speed << std::endl;
  std::cout << "Target speed: " << tar_speed << std::endl;
  //std::cout << "----------------------" << std::endl;
  */
  vector<double> JMTs_coeffs = JMT(start_s, end_s, T);
  vector<double> JMTd_coeffs = JMT(start_d, end_d, T);
  int NT = int(floor(T*50));
  double prev_s;
  vector<double> nx_vals, ny_vals, s_vals, d_vals;
  for (int i=1; i<=NT; ++i){
    double dt = float(i)*T/float(NT);
    double JMTs = JMTs_coeffs[0] + JMTs_coeffs[1]*dt + JMTs_coeffs[2]*dt*dt + 
                  JMTs_coeffs[3]*dt*dt*dt + JMTs_coeffs[4]*dt*dt*dt*dt + JMTs_coeffs[5]*dt*dt*dt*dt*dt;
    double JMTd = JMTd_coeffs[0] + JMTd_coeffs[1]*dt + JMTd_coeffs[2]*dt*dt + 
                   JMTd_coeffs[3]*dt*dt*dt + JMTd_coeffs[4]*dt*dt*dt*dt + JMTd_coeffs[5]*dt*dt*dt*dt*dt;
    vector<double> xy = getXY(fmod(JMTs, max_s), JMTd, map_waypoints_s, map_waypoints_x, map_waypoints_y, x_spl, y_spl);
    nx_vals.push_back(xy[0]);
    ny_vals.push_back(xy[1]);
    s_vals.push_back(JMTs);
    d_vals.push_back(JMTd);
  }
  return {nx_vals, ny_vals, s_vals, d_vals};
}


vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y,
                     tk::spline x_spl, tk::spline y_spl) {
  /**
   * Transforms from Frenet s,d coordinates to Cartesian x,y
   * Replaced provided linear interpolation with splines to be able to
   * convert a JMT in Frenet to Cartesian without getting discontinuities
   * from the convertion.
   */
  double seg_x = x_spl(s);
  double seg_y = y_spl(s);
  // finite differencing on the spline to get the heading
  double car_heading = atan2(y_spl(s+0.01) - y_spl(s-0.01), x_spl(s+0.01) - x_spl(s-0.01));
  double perp_heading = car_heading-pi()/2;
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  return {x,y};
}


vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculates the Jerk Minimizing Trajectory coefficients that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   
   double si = start[0];
   double sidot = start[1];
   double sidotdot = start[2];
   double sf = end[0];
   double sfdot = end[1];
   double sfdotdot = end[2];
   
   double a0 = si;
   double a1 = sidot;
   double a2 = sidotdot/2.;
   
   Vector3f b;
   b << sf - (si + sidot*T + sidotdot*T*T/2.),
        sfdot - (sidot+sidotdot*T),
        sfdotdot - sidotdot;
   Matrix3f A;
   A << T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T, 5*T*T*T*T,
        6*T, 12*T*T, 20*T*T*T;
        
   Vector3f x = A.colPivHouseholderQr().solve(b);
   double a3 = x[0];
   double a4 = x[1];
   double a5 = x[2];
  return {a0,a1,a2,a3,a4,a5};
}


// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}


// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

#endif  // HELPERS_H