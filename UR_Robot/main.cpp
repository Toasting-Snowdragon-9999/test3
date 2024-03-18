#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
    RTDEControlInterface rtde_control("192.168.1.54", RTDEControlInterface::FLAG_NO_WAIT | RTDEControlInterface::FLAG_USE_EXT_UR_CAP);
    RTDEReceiveInterface rtde_receive("192.168.1.54", RTDEControlInterface::FLAG_NO_WAIT | RTDEControlInterface::FLAG_USE_EXT_UR_CAP);

    //Function to set up robot and make a plane parallel with table
    rtde_control.moveJ({-1, -1.57, -1.57, -1.57, 1.57, 0}, 1, 0.1);
    std::vector<double> target = {0.0573883,
                                  -0.359131,
                                  0.219987,
                                  -0.730455,
                                  3.04761,
                                  0.0107012};
    rtde_control.moveL(target, 0.1, 0.1);

    rtde_control.speedL({0, 0,-0.01, 0, 0, 0});

    while(rtde_receive.getActualTCPForce()[2] < 30){
        std::cout << rtde_receive.getActualTCPForce()[2] << std::endl;
    }

    std::cout << rtde_receive.getActualTCPForce()[2] << std::endl;
    rtde_control.speedStop();

    /*while(1){
        rtde_control.teachMode();
        std::cin.get();

        std::vector<double> orego = {0, 0, 0, 0, 0, 0};
        std::vector<double> boardPlane = rtde_receive.getActualTCPPose();

        rtde_control.endTeachMode();

        rtde_control.startContactDetection();

        while(rtde_control.toolContact({0,0,0})){
            std::cout << rtde_control.toolContact({0,0,0});
            rtde_control.moveL(boardPlane, 0.1, 0.1);
            boardPlane[2] -= 0.001;
        }

        std::cout << rtde_control.toolContact({0,0,0});
        std::cout << "x: " << boardPlane[0] << std::endl;
        std::cout << "y: " << boardPlane[1] << std::endl;
        std::cout << "z: " << boardPlane[2] << std::endl;
        std::cout << "rx: " << boardPlane[3] << std::endl;
        std::cout << "ry: " << boardPlane[4] << std::endl;
        std::cout << "rz: " << boardPlane[5] << std::endl;

        std::cout << std::endl;
    }*/

    return 0;
}
