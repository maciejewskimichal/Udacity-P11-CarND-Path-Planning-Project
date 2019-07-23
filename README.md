# CarND-Path-Planning-Project

[Self-Driving Car Engineer Nanodegree Program Term 3 Project 11](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)
   
## Project description

![Running application demo](https://github.com/maciejewskimichal/Udacity-P11-CarND-Path-Planning-Project/blob/master/images/running_demo_view.jpg)

### Goals
In this project the goal is to:
* safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
* there is provided:
   * the car's localization 
   * sensor fusion data - other vehicles position
   * a sparse map - list of waypoints around the highway 
* The car should:
   * try to go as close as possible to the 50 MPH speed limit
   * pass slower traffic when possible, note that other cars will try to change lanes too. 
   * avoid hitting other cars at all cost 
   * drive inside of the marked road lanes at all times, unless going from one lane to another
   * be able to make one complete loop around the 6946m highway (around 5 minutes)
   * not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3

### Project workflow

There is created a new class which is prediciting car trajectory: 

```cpp
class CarPathEstimation
{
public:
    CarPathEstimation();
    // Calculate car XY path 
    void carPathCalculate(json input_parameters); 
    // Get car path x position vector
    vector<double> getCarPathX() const;           
    // get car path y position vector
    vector<double> getCarPathY() const;           

private:
    
    // Calculate current car output lane and output velocity out of: 
    // other car location, car s position, car path size, 
    // current car lane, current car velocity
    void carPathLaneSpeedStateMachine(vector<vector<double>> &sensor_fusion, 
                                      double car_s, int car_path_prev_size, 
                                      int &output_lane, double &output_vel) const;

    typedef struct
    {
        vector<double> x;
        vector<double> y;
        vector<double> s;
        vector<double> dx;
        vector<double> dy;
    } Map_waypoints; // structure to store highway points

    Map_waypoints map_waypoints;

    bool readMapWaypoints(); // read highway waypoints from csv file

    vector<double> car_path_x;
    vector<double> car_path_y;

    double vel_max    ; // car maximum speed
    double lane_size_m; // one lane size in m 
    int    lane_N     ; // number of lanes
    double cars_gap_m ; // minimum gap between cars
    double cars_gap_back_m; // minimum gap between cars to back

    double lenght_trajectory_m; // one trajectory prediction size, trajectory size = 3*this size

    int    car_lane; // current car lane
    double ref_vel;  // current car velocity
};

```

This class is initialized with some values:

```cpp
CarPathEstimation::CarPathEstimation() 
{
    // startup parameters
    vel_max     = 49.5; // car maximum speed
    lane_size_m = 4;    // one lane size in m  
    lane_N      = 3;    // number of lanes
    cars_gap_m  = 30;   // minimum gap between cars 
    cars_gap_back_m = 5;

    lenght_trajectory_m = 30; // one trajectory prediction size, trajectory size = 3*this size

    // startup car parameters
    car_lane   = 1; // this should be intialized automatically for more general cases
    ref_vel    = 0;

    readMapWaypoints();
}
```
Main loop for finiding new trajectory is running inside main *onMessage* function:

```cpp
CarPathEstimation car_path_estimation;

/// ...

if (event == "telemetry") 
{
    // Define the actual (x,y) points we will use for the planner
    car_path_estimation.carPathCalculate(input_parameters);

    json msgJsonOutput;

    msgJsonOutput["next_x"] = car_path_estimation.getCarPathX();
    msgJsonOutput["next_y"] = car_path_estimation.getCarPathY();
    auto msg = "42[\"control\","+ msgJsonOutput.dump()+"]";

    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}  // end "telemetry" if
```

Firstly there is calculated the car best line and velocity in the function in according to 
following state machine:

![Car lane and velocity update](https://github.com/maciejewskimichal/Udacity-P11-CarND-Path-Planning-Project/blob/master/images/state_machine.jpg)

Function responsible for above state machine calculation:

```cpp
void carPathLaneSpeedStateMachine(vector<vector<double>> &sensor_fusion, 
                                  double car_s, int car_path_prev_size, 
                                  int &output_lane, double &output_vel) const
{
        bool too_close_to_car_in_front = false;
    bool lane_right_free           = true;
    bool lane_left_free            = true;

    // this loop is going over all cars (sensor_fusion data)
    // for each other car there is estimated: 
    // * its current line :
    //    * other_car_lane
    //    * it line compared to main car lane
    // * out of it: it is known if there is
    //    * some car too close: too_close_to_car_in_front = true
    //    * possible to change a line to the left : lane_left_free  = true
    //    * possible to change a line to the right: lane_right_free = true
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float other_car_d = sensor_fusion[i][6];

        int other_car_lane = 0;
        for (int i = 0; i < lane_N; i++)
        {
            if (other_car_d >= lane_size_m * i && other_car_d < lane_size_m*(i + 1))
            {
                other_car_lane = i;
                break;
            }
        }

        double other_car_s     = sensor_fusion[i][5];
        double other_car_vx    = sensor_fusion[i][3];
        double other_car_vy    = sensor_fusion[i][4];
        double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy * other_car_vy);

        // predict other car s in the future 
        other_car_s += static_cast<double>(car_path_prev_size)*0.02*other_car_speed;

        // check for other car in the range
        bool other_car_too_close_to_front = (other_car_s >  car_s) && (other_car_s - car_s) < cars_gap_m;         
        bool other_car_too_close_to_back  = (other_car_s <= car_s) && (car_s - other_car_s) < cars_gap_back_m; 
        bool other_car_too_close_gap      = other_car_too_close_to_front || other_car_too_close_to_back;
      
        int car_lane_diff = car_lane - other_car_lane;
        
        if (car_lane_diff == 0 && other_car_too_close_to_front) { too_close_to_car_in_front = true; }
        if (car_lane_diff == 1 && other_car_too_close_gap)      { lane_left_free            = false;} // car on the left 
        if (car_lane_diff ==-1 && other_car_too_close_gap)      { lane_right_free           = false;} // car on the right
    }

    // very simple state machine with 4 states:
    // 1) it there is car too close in the front:
    //    a) go to the left  if there is enough space: change final lane 
    //    b) go to the right if there is enough space: change final lane
    //    c) stay in the current lane and slow down to keep distance 
    //       to following car when 1a) and 1b) is not possbile
    // 2) if can go by the same lane: 
    //    a) continue and speed up the car to maximum possible speed
    output_vel  = ref_vel;
    output_lane = car_lane;
    
    if (too_close_to_car_in_front)
    {
        if (lane_right_free && car_lane < lane_N-1) { output_lane++; }
        else 
        if (lane_left_free  && car_lane > 0       ) { output_lane--; }
        else
        {
            output_vel -= .224; // slow by a little bit beloow 5 m/s
        }
    }
    else if (output_vel < vel_max)
    {
        output_vel += .224;
    }
}
```

output of this function is a new car lane: *output_lane* and a car velocity: *output_vel*

Main function for calculating car path, explanation in comments:

```cpp
void CarPathEstimation::carPathCalculate(json input_parameters)
{
    // Initial settings

    // initialize new car path 
    car_path_x.clear();
    car_path_y.clear();

    // ------------------------------------------------------------------------
    // >> READ ALL MAIN DATA 
    // ------------------------------------------------------------------------
    // j[1] is the data JSON object

    // Main car's localization Data
    double car_x         = input_parameters[1]["x"              ];
    double car_y         = input_parameters[1]["y"              ];
    double car_s         = input_parameters[1]["s"              ];
    double car_d         = input_parameters[1]["d"              ];
    double car_yaw       = input_parameters[1]["yaw"            ];
    double car_speed     = input_parameters[1]["speed"          ];

    // Previous path data given to the Planner
    auto car_path_prev_x = input_parameters[1]["previous_path_x"];
    auto car_path_prev_y = input_parameters[1]["previous_path_y"];
    // Previous path's end s and d values 
    double end_path_s    = input_parameters[1]["end_path_s"     ];
    double end_path_d    = input_parameters[1]["end_path_d"     ];
                                                                

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vector<vector<double>> sensor_fusion = input_parameters[1]["sensor_fusion"    ];

    // ------------------------------------------------------------------------
    // << READ ALL MAIN DATA 
    // ------------------------------------------------------------------------

    int car_path_prev_size = car_path_prev_x.size();

    if (car_path_prev_size > 0)
    {
        car_s = end_path_s;
    }

    int    car_final_lane;
    double car_final_vel;
    
    // predict car lane and velocity for the best position 
    carPathLaneSpeedStateMachine(sensor_fusion, car_s, car_path_prev_size, car_final_lane, car_final_vel);

    car_lane = car_final_lane;
    ref_vel  = car_final_vel;

    // ------------------------------------------------------------------------
    // >> Create 3 points to fit spline 
    // ------------------------------------------------------------------------
    
    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    // Reference x, y, yaw states
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // If previous size is almost empty, use the car as starting reference
    if (car_path_prev_size < 2)
    {
        // Use two points that make the path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        // Use the previous path's endpoint as starting ref
        // Redefine reference state as previous path end point

        // Last point
        ref_x = car_path_prev_x[car_path_prev_size - 1];
        ref_y = car_path_prev_y[car_path_prev_size - 1];

        // 2nd-to-last point
        double ref_x_prev = car_path_prev_x[car_path_prev_size - 2];
        double ref_y_prev = car_path_prev_y[car_path_prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use two points that make the path tangent to the path's previous endpoint
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Using Frenet, add 30 m evenly spaced points ahead of the starting reference

    vector<double> next_wp0 = getXY(car_s + lenght_trajectory_m  , lane_size_m * (car_lane+0.5), 
                                    map_waypoints.s, map_waypoints.x, map_waypoints.y);
    vector<double> next_wp1 = getXY(car_s + lenght_trajectory_m*2, lane_size_m * (car_lane+0.5), 
                                    map_waypoints.s, map_waypoints.x, map_waypoints.y);
    vector<double> next_wp2 = getXY(car_s + lenght_trajectory_m*3, lane_size_m * (car_lane+0.5), 
                                    map_waypoints.s, map_waypoints.x, map_waypoints.y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        // Shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // ------------------------------------------------------------------------
    // << Create 3 points to fit spline 
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // >> Fit spline to created 3 points along car trajectory
    // ------------------------------------------------------------------------

    // Create a spline called s
    tk::spline s;

    // Set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Start with all the previous path points from last time
    for (int i = 0; i < car_path_prev_size; i++)
    {
        car_path_x.push_back(car_path_prev_x[i]);
        car_path_y.push_back(car_path_prev_y[i]);
    }

    // Compute how to break up spline points so we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

    double x_add_on = 0;

    // Fill up the rest of the path planner to always output 50 points
    for (int i = 1; i <= 50 - car_path_prev_size; i++) 
    {
        double N = (target_dist / (.02*ref_vel / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        car_path_x.push_back(x_point);
        car_path_y.push_back(y_point);
    }

    // ------------------------------------------------------------------------
    // << Fit spline to created 3 points along car trajectory
    // ------------------------------------------------------------------------

}
```

### Results

With following code car was going around highway with good speed up to 49.5 mph per at least 5.5 miles:

![Running application demo_full_loop](https://github.com/maciejewskimichal/Udacity-P11-CarND-Path-Planning-Project/blob/master/images/running_demo_view_full_loop.jpg)

# Project setup

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

