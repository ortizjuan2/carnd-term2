#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <ctime>
#include <vector>
#include <cassert>

#define Debug 1
#define Twiddle 0


#if Twiddle == 1
std::vector<double> pd(3);
std::vector<double *> p(3);
bool twiddle_init = false;
double best_error = 0;
int twiddle_indx = 0;
int twiddle_level = 0;
int it = 0;
int max_it = 1000;
double max_cte = 5.;
std::vector<double> twiddle_error;
#endif




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
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    PID pid;
    // Initialize the pid variable.
    // if Twiddle is active initialize using default values
    // if not then use tuned parameters
#if Twiddle == 1
    //pd[0]: 0.0778083 pd[1]: 0.000473514 pd[2]: 5.78739e-05
//    p[0]: 0.75079 p[1]: 0.00228351 p[2]: 0.000254954

    pid.Init(0.3, 0.001, 0.0001);
    p = {&pid.Kp_, &pid.Kd_, &pid.Ki_};
    pd = {0.1, 0.001, 0.0001};
#else
    //tuned parameters
    //pid.Init(0.08, 0.000051, 0.000105474);
    pid.Init(0.3, 0.000228351, 0.000254954);

#endif

    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event

            if (length && length > 2 && data[0] == '4' && data[1] == '2')
            {
            t_end = std::clock();

            // calculate delta t
            double dt = double(t_end - t_start)/CLOCKS_PER_SEC;

#if Debug == 1
            std::cout << "elapsed time: " << dt << std::endl;
#endif

            auto s = hasData(std::string(data));
            if (s != "") {
            auto j = json::parse(s);
            std::string event = j[0].get<std::string>();
            if (event == "telemetry") {
                // j[1] is the data JSON object
                double cte = std::stod(j[1]["cte"].get<std::string>());
                double speed = std::stod(j[1]["speed"].get<std::string>());
                double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                double steer_value;
                /*
                 * TODO: Calcuate steering value here, remember the steering value is
                 * [-1, 1].
                 * NOTE: Feel free to play around with the throttle and speed. Maybe use
                 * another PID controller to control the speed!
                 */

#if Twiddle == 1
                if((pd[0] + pd[1] + pd[2]) > 1e-3){
                    std::cout << "*** Entering Twiddle\n";
                    if(!twiddle_init){

                        //best_error = cte;
                        //pid.d_error = cte;

                        it++;
                        //if(it >= max_it || fabs(cte) >= max_cte){
                        if(it >= max_it){
                            twiddle_init = true;
                            double tmp_error = 0;
                            for(int i=0; i<twiddle_error.size(); i++)
                                tmp_error += twiddle_error[i];
                            best_error = tmp_error/twiddle_error.size();
                            twiddle_error.clear(); // Remove all elements to start over
                            it = 0;
                            std::cout << "Best Error at init: " << best_error << std::endl;
                            pid.p_error = 0;
                            pid.d_error = 0;
                            pid.i_error = 0;
                            std::string msg = "42[\"reset\"]";
                            ws.send(msg.c_str(), 11, uWS::OpCode::TEXT);
                            //assert(0);

                        }else twiddle_error.push_back(cte);

                    }else{

                        switch(twiddle_level){
                            case 0:
                                std::cout << "*** Entering case 0\n";
                                (*p[twiddle_indx]) += pd[twiddle_indx];
                                //it++;
                                twiddle_level = 1;
                                break;
                            case 1:
                                std::cout << "*** Entering case 1\n";
                                it++;
                                //twiddle_error.push_back(cte);
                                //if(it >= max_it || fabs(cte) >= max_cte){
                                if(it >= max_it){
                                    double tmp_error = 0;
                                    for(int i=0; i<twiddle_error.size(); i++)
                                        tmp_error += twiddle_error[i];
                                    tmp_error /= twiddle_error.size();

                                    if(tmp_error < best_error){
                                        best_error = tmp_error;
                                        pd[twiddle_indx] *= 1.1;
                                        twiddle_indx = (twiddle_indx + 1) % 3;
                                        twiddle_level = 0;
                                        it = 0;
                                        twiddle_error.clear();
                                        std::cout << "Best Error at Level 1: " << best_error << std::endl;
                                        std::cout << "\tpd[0]: " << pd[0] << " pd[1]: " << pd[1] << " pd[2]: " << pd[2] << std::endl;
                                        std::cout << "\tp[0]: " << (*p[0]) << " p[1]: " << (*p[1]) << " p[2]: " << (*p[2]) << std::endl;
                                        std::cout << "goto level " << twiddle_level << std::endl;
                                        //assert(0);
                                        pid.p_error = 0;
                                        pid.d_error = 0;
                                        pid.i_error = 0;
                                        std::cout << "Best Error at init: " << best_error << std::endl;
                                        std::string msg = "42[\"reset\"]";
                                        ws.send(msg.c_str(), 11, uWS::OpCode::TEXT);

                                    }else{
                                        (*p[twiddle_indx]) -= 2 * pd[twiddle_indx];
                                        twiddle_level = 2;
                                        it = 0;
                                        twiddle_error.clear();
                                        std::cout << "\tpd[0]: " << pd[0] << " pd[1]: " << pd[1] << " pd[2]: " << pd[2] << std::endl;
                                        std::cout << "\tp[0]: " << (*p[0]) << " p[1]: " << (*p[1]) << " p[2]: " << (*p[2]) << std::endl;
                                        std::cout << "goto level " << twiddle_level << std::endl;
                                        //assert(0);
                                        pid.p_error = 0;
                                        pid.d_error = 0;
                                        pid.i_error = 0;
                                        std::cout << "Best Error at init: " << best_error << std::endl;
                                        std::string msg = "42[\"reset\"]";
                                        ws.send(msg.c_str(), 11, uWS::OpCode::TEXT);
                                    }
                                }else twiddle_error.push_back(cte);

                                break;
                            case 2:
                                std::cout << "*** Entering case 2\n";
                                it++;
                                //if(it >= max_it || fabs(cte) >= max_cte){
                                if(it >= max_it){
                                    double tmp_error = 0;
                                    for(int i=0; i<twiddle_error.size(); i++)
                                        tmp_error += twiddle_error[i];
                                    tmp_error /= twiddle_error.size();

                                    if(tmp_error < best_error){
                                        best_error = tmp_error;
                                        pd[twiddle_indx] *= 1.1;
                                        twiddle_indx = (twiddle_indx + 1) % 3;
                                        twiddle_level = 0;
                                        it = 0;
                                        twiddle_error.clear();
                                        pid.p_error = 0;
                                        pid.d_error = 0;
                                        pid.i_error = 0;
                                        std::cout << "Best Error at init: " << best_error << std::endl;
                                        std::string msg = "42[\"reset\"]";
                                        ws.send(msg.c_str(), 11, uWS::OpCode::TEXT);
                                    }else{
                                        (*p[twiddle_indx]) += pd[twiddle_indx];
                                        pd[twiddle_indx] *= 0.9;
                                        twiddle_level = 0;
                                        twiddle_indx = (twiddle_indx + 1) % 3;
                                        it = 0;
                                        twiddle_error.clear();
                                        pid.p_error = 0;
                                        pid.d_error = 0;
                                        pid.i_error = 0;
                                        std::cout << "Best Error at init: " << best_error << std::endl;
                                        std::string msg = "42[\"reset\"]";
                                        ws.send(msg.c_str(), 11, uWS::OpCode::TEXT);
                                    }
                                }

                        }


                    }
                }else{
                    std::cout << "@@@@@@@@@@@@@ END OF TWIDDLE @@@@@@@@@@@@@\n";

                }
#endif
                // calculate differential error
                pid.d_error = (cte - pid.d_error)/dt;
                // calculate itegral error
                pid.i_error += cte;
                // calculate steering value
                steer_value = - pid.Kp_*cte - pid.Kd_*pid.d_error - pid.Ki_*pid.i_error;

#if Debug == 1
                std::cout << "angle: " << angle << " steer_value deg: " << rad2deg(steer_value) << " final steering angle: " << steer_value << std::endl;
#endif

#if Twiddle == 1
                std::cout << "\tKp: " << pid.Kp_ << " Kd: " << pid.Kd_ << " Ki: " << pid.Ki_ << std::endl;
                std::cout << "\tpd[0]: " << pd[0] << " pd[1]: " << pd[1] << " pd[2]: " << pd[2] << std::endl;
                std::cout << "\tp[0]: " << (*p[0]) << " p[1]: " << (*p[1]) << " p[2]: " << (*p[2]) << std::endl;
#endif
                // store last cte
                pid.d_error = cte;
                // normalize steering value
                steer_value = steer_value > 1.0 ? 1.0 : steer_value;
                steer_value = steer_value < -1.0 ? -1.0 : steer_value;

#if Debug == 1
                std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " speed: " << speed << std::endl;
#endif
                // send values to simulator
                json msgJson;
                msgJson["steering_angle"] = steer_value;
               // if((cte > fabs(0.6)) && speed > 10.0)
               //     msgJson["throttle"] = 0.0;
               // else
                    msgJson["throttle"] = .30;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
#if Debug == 1
                std::cout << msg << std::endl;
#endif
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

            t_start = std::clock();

            }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
            const std::string s = "<h1>Hello world!</h1>";
            if (req.getUrl().valueLength == 1)
            {
            res->end(s.data(), s.length());
            }
            else
            {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
            }
            });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
            std::cout << "Connected!!!" << std::endl;
            });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
            });

    int port = 4567;
    if (h.listen(port))
    {
        t_start = std::clock();

        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
