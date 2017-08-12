#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "path_planner.h"
#include "constants.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string& s) {
  const auto found_null = s.find("null");
  const auto b1 = s.find_first_of("[");
  const auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  PathPlanner::WpMap map;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
    map.x.push_back(x);
    map.y.push_back(y);
    map.s.push_back(s);
    map.dx.push_back(d_x);
    map.dy.push_back(d_y);
  }

  /* Init path planner */
  PathPlanner pp = PathPlanner(map);

  h.onMessage([&pp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {

        const auto s = hasData(data);

        if (s != "") {
          const auto j = json::parse(s);

          const string event = j[0].get<string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            // Main car's localization Data
            PathPlanner::VehicleState egoState;
            egoState.x         = j[1]["x"];
            egoState.y         = j[1]["y"];
            egoState.d         = j[1]["d"];
            egoState.yawDeg    = j[1]["yaw"];
            egoState.endS      = j[1]["end_path_s"];
            egoState.endD      = j[1]["end_path_d"];
            egoState.speedMph  = j[1]["speed"];
            egoState.s         = j[1]["s"];

            // Previous path data given to the Planner
            const std::vector<double> previous_path_x = j[1]["previous_path_x"];
            const std::vector<double> previous_path_y = j[1]["previous_path_y"];

            // Previous path's end s and d values 
            egoState.endS = j[1]["end_path_s"];
            egoState.endD = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            const auto sensor_fusion = j[1]["sensor_fusion"];

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            pp.calcPath(egoState, previous_path_x, previous_path_y, sensor_fusion, next_x_vals, next_y_vals);

            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;  

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
