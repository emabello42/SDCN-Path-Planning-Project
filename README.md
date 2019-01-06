# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[image1]: ./images/components.png "Components"
[image2]: ./images/fsm.png "FSM"
### Demo
[![DEMO](https://img.youtube.com/vi/8fCUIcyx_GY/0.jpg)](https://www.youtube.com/watch?v=8fCUIcyx_GY)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided by the simulator, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. Also the car does not experience total acceleration over 10 m/s^2 or jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Run script build.sh to compile the code
3. Run script run.sh to start the path planner

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

## Tips

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
## Path Generation

The main components for the solution are the following:

![alt text][image1]

### Trajectory Generator
 (see method `generate(...)` in `src/trajectory_generator.cpp`): it takes the input from the simulator (localization, sensor data) and the map waypoints to generate the trajectory composed by 50 points. To generate a smooth trajectory, a spline is used: http://kluge.in-chemnitz.de/opensource/spline/

In order to generate such a spline, a set of 5 (x, y ) points is generated. The first two points correspond to the previous path followed (if any, otherwise we use the current position of the car and estimate the previous position using the current heading angle of the car), allowing us to make a smooth transition between each trajectory generated. The other 3 points are separated by 30 meters along the S Frenet coordinate, whose D coordinate is calculated based on the decision taken by the Behavior Planner, allowing to turn left, right or just keeping the car in the same lane.

Every of this 5 points are (x,y) coordinates are in the global map, but we transform these to local coordinates (i.e. regard current car (x, y) position and heading angle)  to make the following calculations easier:

    //ptsx and ptsy contain the list of 5 points in global coordinates
    for(int i=0; i < ptsx.size(); i++)
    {
	    //shift car reference angle to 0 degrees
	    double dx = ptsx[i] - refCar.x; //refCar.x is the current x potision of the car
	    double dy = ptsy[i] - refCar.y; //refCar.y is the current y potision of the car
	    //refCar.yaw is the heading angle of the car
	    //the following to equations correspond to a basis transformation, that rotates and shift the map
	    // so that the car is in the (0,0) position and with a yaw angle = 0 in the new coordinate system
	    ptsx[i] = dx * cos(0 - refCar.yaw) - dy * sin(0 - refCar.yaw);
	    ptsy[i] = dx * sin(0 - refCar.yaw) + dy * cos(0 - refCar.yaw);
    }

 Using the local coordinates a spline is generated, from which we take 50 points separated according to the desired speed. These points are generated in a loop where the desired speed is decremented or incremented according to the decision taken by the Behavior Planner. To calculate the separation between each point, so that the car drives at the desired speed, we know that if we want to drive a distance of 30 meters, and considering that every point is going to be visited every 0.02 seconds, then:
 `N*0.02*desiredSpeed = target_dist`
where N is the number of points along the target distance.

Then these points are transformed back to global coordinates and returned to the simulator.
 
### Behavior Planner
(see method `getNextAction(...)` in `src/behavior_planner.cpp`): based on sensor data, it estimates the position of the other cars around the ego car and analyses which is the best action to take.
The Behavior planner first analyses the sensor data and, predicting the future speed and position of every car around the ego car, estimates the following:
* The cars ahead and behind the ego car for every lane, and their corresponding speed and distance to the ego car.
* Detects the possibility of collision for a given lane, considering the (future) distance to the ego car.
* Counts the number of cars ahead and the traffic speed (this information is used in the cost functions)

After the preceding analysis, only if it was detected that the ego car is too close to the car in front of it (less than 30 meters) in the same lane, the following Finite State Machine is used to take a decision:

![alt text][image2]

Where:
* KL (Keep lane): keeps the same lane.
* PLCL/PLCR: these states are used to prepare the car for a lane change (e.g. increasing the speed of the car so that it goes faster other cars behind in the intended lane).
* LCL/LCR: turn left or right.

The transitions depicted in the FSM are also restricted by the feasibility of the corresponding action, i.e. if the car is not going to collide with other cars given the current speed and distance to other cars.

Given more than one possible transition for the current state in the FSM, it evaluates the best one using two cost functions (see method `calculateCost(..)` in `src/behavior_cost.cpp`):
* `inefficiencyCost()`
* `trafficDensityCost()`

Both are considered with the same weight to calculate the final cost of a transition.

