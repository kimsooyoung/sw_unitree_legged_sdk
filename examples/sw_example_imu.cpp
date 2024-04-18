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
    float dt = 0.001;     // 0.001~0.01
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
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

    if( motiontime >= 500){
        // std::system("clear");

        std::cout << "state.imu.quaternion[0] : " << state.imu.quaternion[0] << std::endl;
        std::cout << "state.imu.quaternion[1] : " << state.imu.quaternion[1] << std::endl;
        std::cout << "state.imu.quaternion[2] : " << state.imu.quaternion[2] << std::endl;
        std::cout << "state.imu.quaternion[3] : " << state.imu.quaternion[3] << std::endl;
        std::cout << std::endl;
    }
    // safe.PowerProtect(cmd, state, 1);
    // safe.PowerProtect(cmd, state, 5);
    udp.SetSend(cmd);

    // SetSend를 안하면 => 0만 나온다.
    // 서있는 상태에서 SetSend를 하면 => 삐삐 거린다.
    // 
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
