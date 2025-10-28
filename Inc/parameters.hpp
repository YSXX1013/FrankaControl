#ifndef _PARAMETERS_HPP_
#define _PARAMETERS_HPP_

#define INITIAL_ANGLE_STATE {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}


#include<cmath>
#include<cstdint>



namespace CSIR{

namespace {

const int DOF=7;

const double Kp = 1.0;
const double Ki = 0.0;
const double Kd = 0.0;

const int64_t time_update_direvalue = 20;
const int64_t time_control_interval = 1;

const int control_update_max=300;
const int control_update_min=50;

const double security_factor=0.8;

const double max_q[7]={2.6*security_factor,
                       1.6*security_factor,
                       2.6*security_factor,
                       -0.1*security_factor,
                       2.6*security_factor,
                       3.5*security_factor,
                       2.6*security_factor};

const double min_q[7]={-2.6*security_factor,
                       -1.6*security_factor,
                       -2.6*security_factor,
                       -3.0*security_factor,
                       -2.6*security_factor,
                       -0.1*security_factor,
                       -2.6*security_factor};

const double max_q_d[7]={2.1*security_factor,
                         2.17*security_factor,
                         2.1*security_factor,
                         2.1*security_factor,
                         2.6*security_factor,
                         2.6*security_factor,
                         2.6*security_factor};

const double max_q_dd[7]={14.9*security_factor,
                          7.3*security_factor,
                          9.9*security_factor,
                          12.4*security_factor,
                          14.9*security_factor,
                          19.9*security_factor,
                          19.9*security_factor};

const double max_q_ddd[7]={7000*security_factor,
                           3700*security_factor,
                           4500*security_factor,
                           5800*security_factor,
                           7000*security_factor,
                           9000*security_factor,
                           9000*security_factor};

const double max_q_dddd[7]={max_q_ddd[0]*2*1000,
                            max_q_ddd[1]*2*1000,
                            max_q_ddd[2]*2*1000,
                            max_q_ddd[3]*2*1000,
                            max_q_ddd[4]*2*1000,
                            max_q_ddd[5]*2*1000,
                            max_q_ddd[6]*2*1000};

const int udp_port = 2233;
const char* robot_ip = "172.16.0.2";
// Websocket
const int websocket_port = 8080;

};

};
#endif