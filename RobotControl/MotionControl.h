/*
*  Class implementation of ur_rtde library @ https://sdurobotics.gitlab.io/ur_rtde/index.html
*  Created October 4th, 2022
*  Authored by Thomas Therkelsen @ SDU
*/

#ifndef ROBOTCONTROL_MOTIONCONTROL_H
#define ROBOTCONTROL_MOTIONCONTROL_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include <chrono>
#include <thread>
#include <string>
//#include <Eigen/Dense>

class MotionControl {
public:
    MotionControl(const std::string ipAddress, const std::string gripIP);
    MotionControl();
    void connectToRobot(const std::string ip);
    std::array<double, 3> getTCP();
    void moveHome();
    void moveDown(double amount);
    void moveUp(double amount);
    void stop(double deceleration);
    void grip();
    void release(double T);

    // Gripper logic
    void printStatus(int status);
//    void gripperConnect(std::string ip);

    // Program logic
    void showcase();

private:
    // 6 dimensional vector with starting position and orientation of TCP
    //std::vector<double> homePos{0.150, -0.230, -0.269, 0.137, -3.2, -0.05};  homePos{0.07, -0.500, 0.06, 1.8, -2.5, 0.01}
    std::vector<double> homePos{0.01329, -0.43659, 0.109789, 0.0484993, 3.10821, -0.268578};

    double radConv = acos(-1)/180;
    std::string robotIP = "";
    std::string gripperIP = "";
    double speed = 1;
    double accel = 1;
    bool async = false;

    // Control Interface Object
    ur_rtde::RTDEControlInterface rc;
    //Receive interface
    ur_rtde::RTDEReceiveInterface rr;
    // Gripper
    ur_rtde::RobotiqGripper gc;
};


#endif //ROBOTCONTROL_MOTIONCONTROL_H
