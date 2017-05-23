#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <ctime>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

#define DEBUG 0

// for convenience
using json = nlohmann::json;

// used to calculate delta t
std::clock_t t_start, t_end;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    uWS::Hub h;

#if DEBUG == 1    
    t_start = std::clock();
#endif

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            string sdata = string(data).substr(0, length);
            //cout << sdata << endl;
            if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
            auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event == "telemetry") {

#if DEBUG == 1
            cout << sdata << endl;
#endif

            // j[1] is the data JSON object
            Eigen::VectorXd ptsx(6);
            Eigen::VectorXd ptsy(6);
            ptsx << j[1]["ptsx"][0], j[1]["ptsx"][1], j[1]["ptsx"][2], j[1]["ptsx"][3], j[1]["ptsx"][4], j[1]["ptsx"][5];
            ptsy << j[1]["ptsy"][0], j[1]["ptsy"][1], j[1]["ptsy"][2], j[1]["ptsy"][3], j[1]["ptsy"][4], j[1]["ptsy"][5];
            double px = j[1]["x"];
            double py = j[1]["y"];
            double psi = j[1]["psi"];
            double v = j[1]["speed"];
            double steer = j[1]["steering_angle"];
            //
            double cos_psi = cos(psi);
            double sin_psi = sin(psi);
            // Rotation matrix to convert from map to car frame
            //
            for(int i=0; i<ptsx.size(); i++){
                double x_tmp = (cos_psi * ptsx[i]) + (sin_psi * ptsy[i]) - (cos_psi * px) - (sin_psi * py);
                double y_tmp = (-sin_psi * ptsx[i]) + (cos_psi * ptsy[i]) + (sin_psi * px) - (cos_psi * py); 
                ptsx[i] = x_tmp;
                ptsy[i] = y_tmp;
            }
            /* 
               x_tmp = (cos_psi * ptsx[1]) + (sin_psi * ptsy[1]) - (cos_psi * px) - (sin_psi * py);
               y_tmp = (-sin_psi * ptsx[1]) + (cos_psi * ptsy[1]) + (sin_psi * px) - (cos_psi * py); 
               ptsx[1] = x_tmp;
               ptsy[1] = y_tmp;

               x_tmp = (cos_psi * ptsx[2]) + (sin_psi * ptsy[2]) - (cos_psi * px) - (sin_psi * py);
               y_tmp = (-sin_psi * ptsx[2]) + (cos_psi * ptsy[2]) + (sin_psi * px) - (cos_psi * py); 
               ptsx[2] = x_tmp;
               ptsy[2] = y_tmp;

               x_tmp = (cos_psi * ptsx[3]) + (sin_psi * ptsy[3]) - (cos_psi * px) - (sin_psi * py);
               y_tmp = (-sin_psi * ptsx[3]) + (cos_psi * ptsy[3]) + (sin_psi * px) - (cos_psi * py); 
               ptsx[3] = x_tmp;
               ptsy[3] = y_tmp;

               x_tmp = (cos_psi * ptsx[4]) + (sin_psi * ptsy[4]) - (cos_psi * px) - (sin_psi * py);
               y_tmp = (-sin_psi * ptsx[4]) + (cos_psi * ptsy[4]) + (sin_psi * px) - (cos_psi * py); 
               ptsx[4] = x_tmp;
               ptsy[4] = y_tmp;

               x_tmp = (cos_psi * ptsx[5]) + (sin_psi * ptsy[5]) - (cos_psi * px) - (sin_psi * py);
               y_tmp = (-sin_psi * ptsx[5]) + (cos_psi * ptsy[5]) + (sin_psi * px) - (cos_psi * py); 
               ptsx[5] = x_tmp;
               ptsy[5] = y_tmp;
               */
            /*
             * Calculate steeering angle and throttle using MPC.
             *
             * Both are in between [-1, 1].[3]
             *
             */
            // Fit Polynomial in Car frame 
            auto coeffs = polyfit(ptsx, ptsy, 3);

            // Evaluate polynomial at x=0, y=0, so error will be the result of the function
            double cte = polyeval(coeffs, 0);

            /* Calculate error in orientation "epsi" */
            // desired orientation at x=0
            double psi_des = atan(coeffs[1]);

            Eigen::VectorXd state(6);
            //state << px, py, psi, v, cte, epsi;
            state << 0, 0, 0, v, cte, -psi_des;

            vector<double> xmpc, ympc;
            // Solve the minimization problem
            auto vars = mpc.Solve(state, coeffs, xmpc, ympc);

            //-- send actuate commands to car --
            double steer_value = -vars[6];
            double throttle_value = vars[7];

            // create json message
            json msgJson;

            vector <double> next_x(ptsx.size());
            vector <double> next_y(ptsy.size());

            for(int i = 0; i < ptsx.size(); i++){
                next_x[i] = ptsx[i];
                next_y[i] = ptsy[i];
            }
            // Draw waypoints in yellow
            msgJson["next_x"] = next_x; 
            msgJson["next_y"] = next_y;

            //Display the MPC predicted trajectory 
            // the points in the simulator are connected by a Green line
            msgJson["mpc_x"] = xmpc;
            msgJson["mpc_y"] = ympc;


            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;

            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does actuate the commands instantly.
            //
            // Feel free to play around with this value but should be to drive
            // around the track with 100ms latency.
            //
            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
            // SUBMITTING.
            this_thread::sleep_for(chrono::milliseconds(100));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#if DEBUG == 1
            t_end = std::clock();

            // calculate delta t
            double dt = double(t_end - t_start)/CLOCKS_PER_SEC;
            std::cout << "elapsed time: " << dt << std::endl;
#endif


            }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
#if DEBUG == 1
            t_start = std::clock();
#endif            
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
