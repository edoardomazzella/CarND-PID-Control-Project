#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#define TWIDDLE false

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
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  /**
   * TODO: Initialize the pid object.
   */
  PID angle_pid(0.12, 0.00, 0.99);

#if (TWIDDLE == true)
  PID speed_pid;
  int i = 0;
  int n = 100;
  double cte_acc, cte_speed_acc;
#else
  PID speed_pid(-0.27, -0.01, -0.48);//(0.1, 0.002, 0.0);
#endif

  double ref_speed = 30.0;

  h.onMessage([&angle_pid, &speed_pid,
              #if (TWIDDLE == true)
              &i, n, &cte_acc, &cte_speed_acc,
              #endif
              ref_speed](uWS::WebSocket<uWS::SERVER> ws,
                     char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          // cte is the distance from the center of the trajectory
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double cte_speed = ref_speed - speed;
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value; // for controlling angle
          double throttle_value; // for controlling speed

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
#if (TWIDDLE == true)
          //if(i == 0 || angle_pid.GetTwiddleState() == PID::TwiddleState::kInitialization)
          //{
            //angle_pid.TwiddleStep(cte);
          //}

          // Speed PID tuning
          cte_speed_acc += cte_speed;
          if (speed_pid.GetTwiddleState() == PID::TwiddleState::kStop)
          {
            /* Intentionally left empty */
          }
          else
          {
            if(i == 0)
            {
              speed_pid.TwiddleStep(cte_speed / n);
            }
            i = (i + 1) % n;
          }
#endif
          steer_value = angle_pid.ComputeControlVariable(cte);
          steer_value = (steer_value <= 1.0)? steer_value : 1.0;
          steer_value = (steer_value >= -1.0)? steer_value : -1.0;

          throttle_value = speed_pid.ComputeControlVariable(cte_speed);
          throttle_value = (throttle_value <= 1.0)? throttle_value : 1.0;
          throttle_value = (throttle_value >= 0.0)? throttle_value : 0.0;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // DEBUG
          // std::cout << msgJson.dump() << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
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