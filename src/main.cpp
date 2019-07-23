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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class CarPathEstimation
{
public:
    CarPathEstimation();
    // Calculate car XY path 
    void carPathCalculate(json input_parameters);
    // Get car path x position vector
    vector<double> getCarPathX() const;           
    // Get car path y position vector
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

    // Read highway waypoints from csv file
    bool readMapWaypoints(); 

    vector<double> car_path_x;
    vector<double> car_path_y;

    double vel_max    ; // car maximum speed
    double lane_size_m; // one lane size in m 
    int    lane_N     ; // number of lanes
    double cars_gap_m ; // minimum gap between cars 

    double lenght_trajectory_m; // one trajectory prediction size, trajectory size = 3*this size

    int    car_lane; // current car lane
    double ref_vel;  // current car velocity
};

CarPathEstimation::CarPathEstimation() 
{
    // startup parameters
    vel_max     = 49.5; // car maximum speed
    lane_size_m = 4;    // one lane size in m  
    lane_N      = 3;    // number of lanes
    cars_gap_m  = 30;   // minimum gap between cars 

    lenght_trajectory_m = 30; // one trajectory prediction size, trajectory size = 3*this size

    // startup car parameters
    car_lane   = 1; // this should be intialized automatically for more general cases
    ref_vel    = 0;

    readMapWaypoints();
}

vector<double> CarPathEstimation::getCarPathX() const { return car_path_x; };
vector<double> CarPathEstimation::getCarPathY() const { return car_path_y; };

// Load up map values for waypoint's x,y,s and d normalized normal vectors
bool CarPathEstimation::readMapWaypoints()
{
    bool correct = false;
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;

    double x;
    double y;
    float  s;
    float d_x;
    float d_y;

    while (getline(in_map_, line))
    {
        correct = true;
        std::istringstream iss(line);
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints.x.push_back( x  );
        map_waypoints.y.push_back( y  );
        map_waypoints.s.push_back( s  );
        map_waypoints.dx.push_back(d_x);
        map_waypoints.dy.push_back(d_y);
    }
}

void CarPathEstimation::carPathLaneSpeedStateMachine(vector<vector<double>> &sensor_fusion, double car_s, int car_path_prev_size, int &output_lane, double &output_vel) const
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
        double cars_gap_back_m = 5;
        bool other_car_too_close_to_back  = (other_car_s <= car_s) && (car_s - other_car_s) < cars_gap_back_m; 
        bool other_car_too_close_gap      = other_car_too_close_to_front || other_car_too_close_to_back;
      
        int car_lane_diff = car_lane - other_car_lane;
        
        if (car_lane_diff == 0 && other_car_too_close_to_front) { too_close_to_car_in_front = true; std::cout << "F: " << i << " "; }
        if (car_lane_diff == 1 && other_car_too_close_gap) { lane_left_free  = false; std::cout << "L: " << i << " " << car_s-other_car_s << " ";} // car on the right 
        if (car_lane_diff ==-1 && other_car_too_close_gap) { lane_right_free = false; std::cout << "R: " << i << " " << car_s-other_car_s << " " ;} // car on the left
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
    
    std::cout << "CL: " << car_lane << " ";

    if (too_close_to_car_in_front)
    {
        if (lane_right_free && car_lane < lane_N-1) { output_lane++; std::cout << " CTR " << output_lane << " ";}
        else 
        if (lane_left_free  && car_lane > 0       ) { output_lane--; std::cout << " CTL " << output_lane << " ";}
        else
        {
            output_vel -= .224; // slow by a little bit beloow 5 m/s
            std::cout << " SLOW " << " ";
        }
    }
    else if (output_vel < vel_max)
    {
        output_vel += .224;
        std::cout << " UP " << " ";
    }
    
    std::cout << std::endl;

}

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

    vector<double> next_wp0 = getXY(car_s + lenght_trajectory_m  , lane_size_m * (car_lane+0.5), map_waypoints.s, map_waypoints.x, map_waypoints.y);
    vector<double> next_wp1 = getXY(car_s + lenght_trajectory_m*2, lane_size_m * (car_lane+0.5), map_waypoints.s, map_waypoints.x, map_waypoints.y);
    vector<double> next_wp2 = getXY(car_s + lenght_trajectory_m*3, lane_size_m * (car_lane+0.5), map_waypoints.s, map_waypoints.x, map_waypoints.y);

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


int main() 
{
  uWS::Hub h;

  CarPathEstimation car_path_estimation;

  h.onMessage([&car_path_estimation](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(data);
      if (s != "") 
      {
        json input_parameters = json::parse(s);
        
        string event = input_parameters[0].get<string>();
        
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
      } 
      else 
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h]( uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
  {
    std::cout << "Connected!!!" << std::endl;
  }
  );

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  }
  );

  int port = 4567;
  if (h.listen(port)) 
  {
    std::cout << "Listening to port " << port << std::endl;
  } 
  else 
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}