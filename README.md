# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. One is provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## External code

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

---

# Model Documentation

## Behavior Planner

The behavior planner has the following lane actions to choose from:

 - "KL": Keep Lane
 - "LCL": Change Lane Left
 - "LCR": Change Lane Right
 - "DLCL": Double Lane Change Left
 - "DLCR": Double Lane Change Right

Each of these actions is paired with a set of possible target velocities {10, 11, 12, 13, 14, 15, 16, 17, 18, 19} m/s. This yields a maximum action space size of 50.

Every time the number of remaining waypoints falls below 30, the behavior planner is executed. The planner then figures out, which of the 50 actions are possible. A "LCL" whilst driving on the left-most lane would not be allowed for instance. It then generates trajectories for each of the possible lane actions, combined with each target speed.

The trajectories are generated as Jerk Minimizing Trajectories (JMT) with a fixed execution time T, which is chosen depending on the type of maneuver. Staying in a lane with only minor or no velocity changes can be executed over a small time T, without generating accelerations and jerks in excess of the provided bounds. On the other hand a lane change or even a double lane change, or a large change in velocity is executed over a larger time span. Jerk Minimizing Trajectories for given initial and target boundary conditions (position, speed, acceleration) for both Frenet s and d coordinates of the car, are generated by fitting a polynomial of degree 5 to these boundary conditions. The target boundary conditions are provided by the planner depending on the action, for instance a "LCL" would have a target Frenet d of (d_curr - 4.0).

Each trajectory is then assigned a cost, which is a weighted sum of 5 cost functions:

1. Efficiency Cost: Reward the largest average velocity along the trajectory.
2. Inlane Safety Cost: Yields a larger cost the closer the trajectory brings our car to the rear of another car.
3. Lane Change Safety Cost: If the trajectory involves a lane change, this cost function checks for vehicles in the target lane and yields a large cost if the lane is occupied by another car. This is done by predicting where all cars in the target lane will be at the end of the trajectory execution and yielding a large cost if our target position with a safety margin overlaps with another cars position.
4. Double Lane Change Safety Cost: If the trajectory involves a double lane change, the middle lane, which is crossed, is checked for vehicles. If a vehicle in the middle lane is predicted to be at the crossing point with some margin, a large cost is returned.
5. Lane Change Cost: A small penalty of changing lanes, to penalize unnecessary lane changes, e.g. lane changes that are made way too early.

Cost functions 1 and 2 combined make a lane change favorable, when there is a slow car in our car's lane. Cost functions 3 and 4 make sure, that a lane change is safe to execute.

The trajectory with lowest cost is then converted from Frenet s,d to Cartesian x,y coordinates, appended to the still unprocessed waypoints and passed to the simulator. The convertion to Cartesian coordinates is done by fitting two sets of cubic splines f(s) and g(s) to the map data, f(s) fitting the map x-values and g(s) fitting the map y-values over the map s-values. The d-value is then introduced by determining the slope of the spline at the desired s-value by finite differencing and adding the d-value orthogonally to the slope direction.

The planner has been tested in the simulation for 20 miles without incident.