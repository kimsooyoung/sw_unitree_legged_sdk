/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "matplotlibcpp.h"

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <vector>

using namespace UNITREE_LEGGED_SDK;
namespace plt = matplotlibcpp;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    std::vector<double> FR_0, FR_1, FR_2;
    std::vector<double> FL_0, FL_1, FL_2;
    std::vector<double> RR_0, RR_1, RR_2;
    std::vector<double> RL_0, RL_1, RL_2;
};

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d\n", motiontime);
    // gravity compensation
    // cmd.motorCmd[FR_0].tau = -0.65f;
    // cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    if( motiontime >= 500){
        // float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
        // if(torque > 5.0f) torque = 5.0f;
        // if(torque < -5.0f) torque = -5.0f;
        std::system("clear");

        if(FR_0.size() > 20){
            FR_0.erase(FR_0.begin());FR_1.erase(FR_1.begin());FR_2.erase(FR_2.begin());
            FL_0.erase(FL_0.begin());FL_1.erase(FL_1.begin());FL_2.erase(FL_2.begin());
            RR_0.erase(RR_0.begin());RR_1.erase(RR_1.begin());RR_2.erase(RR_2.begin());
            RL_0.erase(RL_0.begin());RL_1.erase(RL_1.begin());RL_2.erase(RL_2.begin());
        }

        FR_0.push_back(state.motorState[0].dq);FR_1.push_back(state.motorState[1].dq);FR_2.push_back(state.motorState[2].dq);
        FL_0.push_back(state.motorState[3].dq);FL_1.push_back(state.motorState[4].dq);FL_2.push_back(state.motorState[5].dq);
        RR_0.push_back(state.motorState[6].dq);RR_1.push_back(state.motorState[7].dq);RR_2.push_back(state.motorState[8].dq);
        RL_0.push_back(state.motorState[9].dq);RL_1.push_back(state.motorState[10].dq);RL_2.push_back(state.motorState[11].dq);

        // Clear previous plot
        plt::clf();

        plt::subplot(4, 1, 1);
        plt::title("Front Right");
        plt::plot(FR_1,"r");plt::plot(FR_1,"g");plt::plot(FR_2,"b");
        plt::subplot(4, 1, 2);
        plt::title("Front Left");
        plt::plot(FL_0,"r");plt::plot(FL_1,"g");plt::plot(FL_2,"b");
        plt::subplot(4, 1, 3);
        plt::title("Rear Right");
        plt::plot(RR_0,"r");plt::plot(RR_1,"g");plt::plot(RR_2,"b");
        plt::subplot(4, 1, 4);
        plt::title("Rear Left");
        plt::plot(RL_0,"r");plt::plot(RL_1,"g");plt::plot(RL_2,"b");

        plt::pause(0.002);
    }
    safe.PowerProtect(cmd, state, 1);
    // safe.PowerProtect(cmd, state, 5);
    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
