/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace UNITREE_LEGGED_SDK;

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

        std::cout << "state.motorState[FR_0].q : " << state.motorState[0].q << std::endl;
        std::cout << "state.motorState[FR_1].q : " << state.motorState[1].q << std::endl;
        std::cout << "state.motorState[FR_2].q : " << state.motorState[2].q << std::endl << std::endl;

        std::cout << "state.motorState[FL_0].q : " << state.motorState[3].q << std::endl;
        std::cout << "state.motorState[FL_1].q : " << state.motorState[4].q << std::endl;
        std::cout << "state.motorState[FL_2].q : " << state.motorState[5].q << std::endl << std::endl;

        std::cout << "state.motorState[RR_0].q : " << state.motorState[6].q << std::endl;
        std::cout << "state.motorState[RR_1].q : " << state.motorState[7].q << std::endl;
        std::cout << "state.motorState[RR_2].q : " << state.motorState[8].q << std::endl << std::endl;

        std::cout << "state.motorState[RL_0].q : " << state.motorState[9].q << std::endl;
        std::cout << "state.motorState[RL_1].q : " << state.motorState[10].q << std::endl;
        std::cout << "state.motorState[RL_2].q : " << state.motorState[11].q << std::endl << std::endl;
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
