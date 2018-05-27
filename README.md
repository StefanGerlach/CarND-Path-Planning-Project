# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./images/lane_changes.png "Lane classification"
[image2]: ./images/lane_changes_2.png "Lane classification 2"
[image3]: ./images/yt.png "Youtube video"

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Reflection

In the reflection section I am going to explain how the final submission code is working. The code is based on the original repository and additional elements from the course walkthrough video material. The main points to achieve are 

* Drive around the track without incident
* Keep speed but do not exceed speed limit
* Max acceleration and jerk may not be exceeded
* Change lanes in a sensefull way
* Do not collide

### How the path is computed from the simulator waypoints

The driving path is based on sparse waypoints that are given by the simulator data. To drive in a smooth manner, the [Cubic Spline Interpolation Library](http://kluge.in-chemnitz.de/opensource/spline/) is used. By using this library, the path is generated with a spline, going through all the given anchor points.

To adjust the car speed, the distances of future points on the spline are computed by linearization of the spline (the way Aaron did it in the walkthrough video). 

To not exceed the speed limit of 50 mph, the reference velocity is set to 49.5.
```cpp
  // Target velocity
  double v_ref_cruise = 49.5;
```

### Acceleration, Deceleration and Jerk

To stay in the given ranges of acceleration, deceleration and jerk, the reference velocity is de- or incremented in small steps. This ensures a steady behavior of the car. But one might think abount, that if another car with low speed switches lane and is suddenly in front of our car, the deceleration may not be sufficient! Jerk is also reduced by using the spline library, that creates smooth paths.

### Change the lane if it is necessary

The car should be able to change the lane to efficiently go over the track. To establish this mechanism, I do the following additions to the code:

* Add a vector for the lanes, encoding if they are safe to change to
```cpp
   // For all lanes, define a boolean for 'this is safe to switch onto'
   vector<bool> lane_safe_switch(3, true);
```

* Define some ranges in meters, where no car may drive (for the lane to be safe)
```cpp
   // Gap length in front of us in meters for safe lane switch
  double gap_length_front = 15.0;
  // Gap length behind us in meters for safe lane switch
  double gap_length_back = 15.0;
```

* Define a range to the rear of the car, where no car may be, that drives significantly faster
```cpp
   // Gap length behind us in meters were we look for other vehicles traveling with high speed
  double gap_speed_sensoring_length = 30.0;
  // Maximum speed gap of a car that drives behind us on a lane where we want to switch to.
  double gap_max_speed_diff = 20.0;
```

* Use the sensor_fusion data structure to analyse, if cars are in the lanes next to our car. These computations are based on the Frenet coordinates of our car and the sensored cars. To check all lanes, I created some helper functions:
```cpp
   float getGlobalLane(const float local_lane) {
     return 2.0 + 4.0 * local_lane;
   }

   bool isOnLocalLane(const float d, const int local_lane, const float eps = 0.25) {
     if(d < getGlobalLane((float)local_lane + eps) && d > getGlobalLane((float)local_lane - eps))
       return true;
     return false;
   }

   bool isInCorridor(const float s, const float s_ref, const float gap_forward, const float gap_backwards) {
     return ((s < (s_ref + gap_forward)) && (s > (s_ref - gap_backwards)));
   }
```

* With the help of these functions, 'block' lanes if there is a car in the restricted area. First of all, check if the current car is on a specific lane. If yes, check if this car is in a specific corridor (next to our car). Where *d* is the sensored cars d-coordinate, *check_cars_s* the sensored cars s-coordinate and *car_s* the s-coordinate of our car.

```cpp 

   // Get the d-coordinate of the car
   float d = sensor_fusion[i][6];
   
   for(int lane_id = 0; lane_id <=2; lane_id++) {
   
      // Check if this car is on the current lane to be checked
      bool on_curr_lane = isOnLocalLane(d, lane_id, lane_offset);
      if(on_curr_lane) {

         // If yes, check if this car is in the zone that blocks the switch
         bool in_corridor = isInCorridor(check_car_s, car_s, gap_length_front, gap_length_back);

         // If this car is in near range corridor, block this lane
         if(in_corridor) {
           lane_safe_switch[lane_id] = false;
      }
      ...
```

* Maybe we are really slow and want to change the lane, but there is a car (behind us) driving on the lane where we want to switch to. If this car is significantly faster then we are, then do not switch to this lane (block the lane).

```cpp 
   // Check if this car is in the speed-sensoring corridor
   // And if this car is in this gap and travelling with high speed, block lane
   bool in_speed_check_corridor = isInCorridor(check_car_s, car_s, 0.0, gap_speed_sensoring_length);
   bool vehicle_with_high_speed = check_speed > (v_ref + gap_max_speed_diff);
   if(in_speed_check_corridor && vehicle_with_high_speed) {
      lane_safe_switch[lane_id] = false;
      // std::cout << "Car speed blocks lane " << lane_id << std::endl;
   }
```

This results in either safe or unsafe lanes because of cars next to us:
![Safe or unsafe lanes 1][image1]

(Image based on an image from course material of trajectory generation)

And in safe or unsafe lanes because of other cars with high velocity:
![Safe or unsafe lanes 2][image2]

(Image based on an image from course material of trajectory generation)


* The switchLanes() function is called if we have to decelerate because of a car in front of us and if we are driving on the current reference lane *local_lane* (to avoid lane bouncing).

```cpp 
  if(too_close) {
    // Switch lane only if we drive on the reference lane
    bool driving_on_local_lane = isOnLocalLane(car_d, local_lane);
    if(driving_on_local_lane)
      local_lane = switchLane(local_lane, lane_safe_switch);

    std::cout << "DriveLane: " << local_lane << std::endl;
  }
```

* In the switchLanes() function, a safe lane next to us is selected. Otherwise the current lane is returned:
```cpp 
  int switchLane(const int current_lane, const vector<bool>& safe_lanes) {
    if(safe_lanes.size() != 3)
      return current_lane;

    bool all_blocked = true;
    for(const auto & c_safe : safe_lanes)
      all_blocked &= !c_safe;

    if(all_blocked)
      return current_lane;

    vector<int> possible_lane_switches;
    if(current_lane - 1 >= 0)
      possible_lane_switches.push_back(current_lane - 1);

    if(current_lane + 1 <= 2)
      possible_lane_switches.push_back(current_lane + 1);

    for(const auto & lane_switch : possible_lane_switches) {
      if(safe_lanes[lane_switch] == true)
        return lane_switch;
    }

    return current_lane;
  }
```

### Pros and cons

There might be a lot more points to think of. What if some other car suddenly changes its lane, and our trajectory falls into its way (collision!). Additionally, there could be a module that checks if a lane change makes sense - because if there is a much slower car in front of us after the lange change, this maneuver was senseless. 

### Video

Please click on the following image to open the Youtube-video:
![Youtube video][image3]



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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

